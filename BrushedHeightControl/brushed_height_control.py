import queue
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from udp_position_receiver import UdpPositionReceiver


URI_BRUSHED = "radio://0/80/2M/E7E7E7E7E7"
OBJECT_ID = 1001

CONTROL_HZ = 50.0
DT = 1.0 / CONTROL_HZ

MIN_THRUST = 45000
HOVER_THRUST = 47000
MAX_THRUST = 65535

# Position hold (X/Z), uses external UDP pose.
POS_KP = 15.0 #8.0
POS_KD = 10.0   #5.0 1.5
MAX_TILT_DEG = 10.0
POS_DEADBAND_M = 0.01
ROLL_SIGN = -1.0
PITCH_SIGN = -1.0

# Yaw hold using UDP orientation (oz as yaw-like signal).
USE_YAW_HOLD = True
YAW_KP = 60.0
MAX_YAWRATE = 90.0
YAW_SIGN = -1.0

KP = 7500.0
KI = 1500
KD = 8000.0
INT_LIM = 0.5

THRUST_SLEW = 20000.0
POSE_TIMEOUT_S = 0.5
#POSE_STALE_S = 9999999999999999999999
POSE_EPS = 0.0005

LANDING_RATE_MPS = 0.3
LANDING_CUTOFF_M = 0.05

DEFAULT_TARGET_HEIGHT_M = 0.0

try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False


def clamp(value, vmin, vmax):
    return max(vmin, min(vmax, value))


def _input_loop(cmd_queue: queue.Queue) -> None:
    while True:
        try:
            line = input("> ")
        except EOFError:
            cmd_queue.put("quit")
            break
        cmd_queue.put(line.strip())


class _CommandReader:
    def __init__(self) -> None:
        self._buffer = ""
        self._queue: queue.Queue = queue.Queue()
        self._threaded = not HAS_MSVCRT
        if self._threaded:
            threading.Thread(target=_input_loop, args=(self._queue,), daemon=True).start()

    def poll(self):
        if self._threaded:
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                return None
        return self._poll_msvcrt()

    def _poll_msvcrt(self):
        line = None
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                continue
            if ch in ("\r", "\n"):
                print("")
                line = self._buffer.strip()
                self._buffer = ""
                return line
            if ch == "\b":
                if self._buffer:
                    self._buffer = self._buffer[:-1]
                    print("\b \b", end="", flush=True)
                continue
            if ch.isprintable():
                self._buffer += ch
                print(ch, end="", flush=True)
        return line


def _print_help() -> None:
    print("Commands:")
    print("  start                start height control")
    print("  stop                 smooth landing and stop")
    print("  height <meters>      set target height")
    print("  status               print current status")
    print("  quit                 land (if needed) and exit")
    print("Note: start locks X/Z position (and yaw if enabled) at current pose.")


def main() -> None:
    cmd_reader = _CommandReader()
    if HAS_MSVCRT:
        print("Type a command and press Enter.")

    receiver = UdpPositionReceiver(object_id=OBJECT_ID)
    receiver.start()

    target_height_cmd = DEFAULT_TARGET_HEIGHT_M
    active_target = DEFAULT_TARGET_HEIGHT_M
    target_x = None
    target_z = None
    target_yaw = None
    pending_start = False
    active = False
    landing = False
    shutdown = False

    integral = 0.0
    last_pose_time = 0.0
    last_height = 0.0
    last_x = 0.0
    last_z = 0.0
    last_yaw = 0.0
    vel_x = 0.0
    vel_y = 0.0
    vel_z = 0.0
    prev_pose_time = 0.0
    prev_x = 0.0
    prev_y = 0.0
    prev_z = 0.0
    last_thrust = 0.0
    landing_height = 0.0
    last_pose_change_time = 0.0
    last_change_x = 0.0
    last_change_y = 0.0
    last_change_z = 0.0
    last_change_yaw = 0.0
    emergency = False

    _print_help()

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI_BRUSHED, cf=Crazyflie(rw_cache="./cache")) as scf:
        try:
            next_tick = time.monotonic()
            while True:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick += DT

                while True:
                    line = cmd_reader.poll()
                    if line is None:
                        break
                    if not line:
                        continue
                    parts = line.lower().split()
                    cmd = parts[0]
                    if cmd == "start":
                        if emergency:
                            emergency = False
                            print("Emergency cleared.")
                        if last_pose_time > 0.0:
                            target_x = last_x
                            target_z = last_z
                            target_yaw = last_yaw
                            active = True
                            pending_start = False
                            landing = False
                            integral = 0.0
                            print("Control started. Position locked.")
                        else:
                            pending_start = True
                            active = False
                            landing = False
                            print("Waiting for pose to start...")
                    elif cmd == "stop":
                        landing = True
                        active = True
                        pending_start = False
                        integral = 0.0
                        if last_pose_time > 0.0:
                            landing_height = last_height
                        else:
                            landing_height = 0.0
                        active_target = landing_height
                        if target_x is None and last_pose_time > 0.0:
                            target_x = last_x
                            target_z = last_z
                            target_yaw = last_yaw
                        print("Landing...")
                    elif cmd in ("height", "h", "z"):
                        if len(parts) != 2:
                            print("Usage: height <meters>")
                            continue
                        try:
                            height_value = float(parts[1].replace(",", "."))
                            target_height_cmd = max(0.0, height_value)
                            print(f"Target height set to {target_height_cmd:.2f} m")
                        except ValueError:
                            print("Invalid height value.")
                    elif cmd == "status":
                        age = now - last_pose_time if last_pose_time > 0.0 else -1.0
                        status_target = landing_height if landing else target_height_cmd
                        pos_target = target_x if target_x is not None else 0.0
                        z_target = target_z if target_z is not None else 0.0
                        print(
                            f"height={last_height:.2f} m  "
                            f"target={target_height_cmd:.2f} m  "
                            f"active_target={status_target:.2f} m  "
                            f"x={last_x:.2f} z={last_z:.2f}  "
                            f"x_t={pos_target:.2f} z_t={z_target:.2f}  "
                            f"active={active}  landing={landing}  emergency={emergency}  "
                            f"pose_age={age:.2f}s"
                        )
                    elif cmd in ("quit", "exit"):
                        shutdown = True
                        if active:
                            landing = True
                            pending_start = False
                            integral = 0.0
                            if last_pose_time > 0.0:
                                landing_height = last_height
                            else:
                                landing_height = 0.0
                            print("Landing before exit...")
                        else:
                            break
                    else:
                        _print_help()

                if shutdown and not active and not landing:
                    break

                sample = receiver.get_latest()
                if sample is not None:
                    # Height comes from Y axis in the UDP stream.
                    last_height = sample.y
                    last_x = sample.x
                    last_z = sample.z
                    last_yaw = sample.oz
                    last_pose_time = sample.t
                    if last_pose_change_time == 0.0:
                        last_pose_change_time = last_pose_time
                        last_change_x = last_x
                        last_change_y = last_height
                        last_change_z = last_z
                        last_change_yaw = last_yaw
                    else:
                        if (
                            abs(last_x - last_change_x) > POSE_EPS
                            or abs(last_height - last_change_y) > POSE_EPS
                            or abs(last_z - last_change_z) > POSE_EPS
                            or abs(last_yaw - last_change_yaw) > POSE_EPS
                        ):
                            last_pose_change_time = last_pose_time
                            last_change_x = last_x
                            last_change_y = last_height
                            last_change_z = last_z
                            last_change_yaw = last_yaw
                    if prev_pose_time > 0.0:
                        dt_pose = last_pose_time - prev_pose_time
                        if dt_pose > 1e-6:
                            vel_x = (last_x - prev_x) / dt_pose
                            vel_y = (last_height - prev_y) / dt_pose
                            vel_z = (last_z - prev_z) / dt_pose
                    prev_pose_time = last_pose_time
                    prev_x = last_x
                    prev_y = last_height
                    prev_z = last_z

                #have_pose = (now - last_pose_time) <= POSE_TIMEOUT_S

                in_flight = (
                    active
                    and (landing or target_height_cmd > LANDING_CUTOFF_M or last_height > LANDING_CUTOFF_M)
                )
                if sample.y == 0:
                    in_flight = not active
                
        #        if have_pose and last_pose_change_time > 0.0 and in_flight:
        #            if (now - last_pose_change_time) > POSE_STALE_S:
        #                if not emergency:
        #                    print("Emergency: UDP pose stalled > 1.0s.")
        #                emergency = True
        #                active = False
        #                landing = False
        #                pending_start = False
        #                integral = 0.0
        #                last_thrust = 0.0
        #                try:
        #                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
        #                    scf.cf.commander.send_stop_setpoint()
        #                except Exception:
        #                    pass

         #       if emergency:
         #           try:
         #               scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
         #           except Exception:
         #               pass
         #           continue

                if pending_start:# and have_pose:
                    target_x = last_x
                    target_z = last_z
                    target_yaw = last_yaw
                    active = True
                    pending_start = False
                    landing = False
                    integral = 0.0
                    print("Control started. Position locked.")

                if active: #and have_pose:
                    if not landing and target_height_cmd <= LANDING_CUTOFF_M:
                        if last_height > LANDING_CUTOFF_M:
                            landing = True
                            landing_height = last_height
                        else:
                            integral = 0.0
                            last_thrust = 0.0
                            scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                            continue
                    if landing:
                        landing_height = max(0.0, landing_height - LANDING_RATE_MPS * DT)
                        active_target = landing_height
                    else:
                        active_target = target_height_cmd
                    error = active_target - last_height
                    integral = clamp(integral + error * DT, -INT_LIM, INT_LIM)
                    thrust_cmd = HOVER_THRUST + KP * error + KI * integral - KD * vel_y

                    min_thrust = 0.0 if landing else MIN_THRUST
                    thrust_cmd = clamp(thrust_cmd, min_thrust, MAX_THRUST)

                    max_delta = THRUST_SLEW * DT
                    thrust_cmd = clamp(thrust_cmd, last_thrust - max_delta, last_thrust + max_delta)
                    last_thrust = thrust_cmd

                    roll_cmd = 0.0
                    pitch_cmd = 0.0
                    yawrate_cmd = 0.0
                    if target_x is not None and target_z is not None:
                        err_x = target_x - last_x
                        err_z = target_z - last_z
                        if abs(err_x) < POS_DEADBAND_M:
                            err_x = 0.0
                        if abs(err_z) < POS_DEADBAND_M:
                            err_z = 0.0
                        pitch_cmd = PITCH_SIGN * (POS_KP * err_x - POS_KD * vel_x)
                        roll_cmd = ROLL_SIGN * (POS_KP * err_z - POS_KD * vel_z)
                        roll_cmd = clamp(roll_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)
                        pitch_cmd = clamp(pitch_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)
                    if USE_YAW_HOLD and target_yaw is not None:
                        yaw_error = target_yaw - last_yaw
                        yawrate_cmd = clamp(YAW_SIGN * YAW_KP * yaw_error, -MAX_YAWRATE, MAX_YAWRATE)

                    scf.cf.commander.send_setpoint(roll_cmd, pitch_cmd, yawrate_cmd, int(thrust_cmd))

                    if landing and landing_height <= LANDING_CUTOFF_M and last_height <= LANDING_CUTOFF_M:
                        active = False
                        landing = False
                        integral = 0.0
                        last_thrust = 0.0
                        scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                        scf.cf.commander.send_stop_setpoint()
                        print("Landed.")
                #else:
                    #last_thrust = 0.0
                    #scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    #if active and not have_pose:
                    #    if not landing:
                    #        landing = True
                    #        integral = 0.0
                    #        landing_height = last_height

        except KeyboardInterrupt:
            print("\n[CTRL+C] Emergency stop ...")
        finally:
            receiver.stop()
            try:
                for _ in range(int(0.4 / DT)):
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    time.sleep(DT)
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass


if __name__ == "__main__":
    main()
