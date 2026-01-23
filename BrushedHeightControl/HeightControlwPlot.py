"""
Best-of-both Crazyflie controller:
- Height PID + XY position hold from UDP pose stream
- Manual commands like brushed_height_control.py
- Fixes "does nothing" issue: always sends setpoints (even while waiting for UDP)
- Uses monotonic time for UDP freshness (no sample.t timebase mismatch)
- Optional IMU fallback for short outages + optional plotting

Dependencies:
- cflib
- udp_position_receiver.py providing UdpPositionReceiver(object_id=...)
"""

from __future__ import annotations

import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from udp_position_receiver import UdpPositionReceiver


# -----------------------------
# User config
# -----------------------------
URI = "radio://0/80/2M/E7E7E7E7E7"
OBJECT_ID = 1001

CONTROL_HZ = 50.0
DT = 1.0 / CONTROL_HZ

# Thrust
MIN_THRUST = 45000
HOVER_THRUST = 47000
MAX_THRUST = 65535
THRUST_SLEW = 20000.0  # max thrust change per second

# Height control PID
KP_H = 7500.0
KI_H = 1500.0
KD_H = 8000.0
INT_LIM_H = 0.5

# Position hold PD (horizontal)
POS_KP = 15.0
POS_KD = 10.0
POS_DEADBAND_M = 0.01
MAX_TILT_DEG = 10.0

# Axis signs (keep same as your brushed script)
ROLL_SIGN = -1.0
PITCH_SIGN = -1.0

# Yaw hold
USE_YAW_HOLD = True
YAW_KP = 60.0
MAX_YAWRATE = 90.0
YAW_SIGN = -1.0

# UDP / IMU health
UDP_TIMEOUT_S = 0.25          # how fresh UDP must be to be "good"
IMU_TIMEOUT_S = 0.25          # how fresh IMU must be to be "good"
FAILSAFE_LAND_AFTER_S = 1.0   # if no UDP for this long while active -> land

# Takeoff/Landing shaping
CLIMB_RATE_MPS = 0.6
LANDING_RATE_MPS = 0.3
LANDING_CUTOFF_M = 0.05

DEFAULT_TARGET_HEIGHT_M = 0.0

# Plotting
PLOT_HISTORY_SEC = 10.0
PLOT_UPDATE_HZ = 10.0


# -----------------------------
# Helpers
# -----------------------------
try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def wrap_angle_deg(err: float) -> float:
    # Wrap error to [-180, 180]
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def _log_var_exists(cf, name: str) -> bool:
    group, var = name.split(".", 1)
    return group in cf.log.toc.toc and var in cf.log.toc.toc[group]


def _log_var_type(cf, name: str) -> str:
    group, var = name.split(".", 1)
    return cf.log.toc.toc[group][var].ctype


def _body_to_world(ax: float, ay: float, az: float, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
    """
    Convert body acceleration -> world acceleration using roll/pitch/yaw in radians.
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    wx = cy * cp * ax + (cy * sp * sr - sy * cr) * ay + (cy * sp * cr + sy * sr) * az
    wy = sy * cp * ax + (sy * sp * sr + cy * cr) * ay + (sy * sp * cr - cy * sr) * az
    wz = -sp * ax + cp * sr * ay + cp * cr * az
    return wx, wy, wz


def _input_loop(cmd_queue: queue.Queue) -> None:
    while True:
        try:
            line = input()
        except EOFError:
            cmd_queue.put("quit")
            break
        cmd_queue.put(line.strip())


class _CommandReader:
    """
    Cross-platform command input.
    - Windows: non-blocking via msvcrt
    - Others: background input() thread
    """
    def __init__(self) -> None:
        self._buffer = ""
        self._queue: queue.Queue[str] = queue.Queue()
        self._threaded = not HAS_MSVCRT
        if self._threaded:
            threading.Thread(target=_input_loop, args=(self._queue,), daemon=True).start()

    def poll(self) -> Optional[str]:
        if self._threaded:
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                return None

        # Windows msvcrt mode
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\r", "\n"):
                line = self._buffer.strip()
                self._buffer = ""
                if line:
                    return line
                return None
            elif ch == "\b":
                self._buffer = self._buffer[:-1]
            else:
                self._buffer += ch
        return None


def _print_help() -> None:
    print("Commands:")
    print("  start                 start control (locks position & yaw)")
    print("  stop                  smooth landing and stop control")
    print("  height <meters>       set target height (e.g. height 0.6)")
    print("  lock                  re-lock position/yaw to current pose")
    print("  status                print controller + sensor status")
    print("  quit                  land (if needed) and exit")
    print("")
    print("Notes:")
    print(" - Height is UDP sample.y (same as your scripts).")
    print(" - Horizontal hold uses UDP x and z (mapped to ctrl_x, ctrl_y).")


@dataclass
class PoseState:
    # Latest UDP pose values
    h: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    # Derived velocities from UDP
    vh: float = 0.0
    vx: float = 0.0
    vy: float = 0.0

    # Monotonic receive time
    rx_t: float = 0.0


def main(plot: bool = False) -> None:
    cmd_reader = _CommandReader()
    if HAS_MSVCRT:
        print("Type a command and press Enter.")

    receiver = UdpPositionReceiver(object_id=OBJECT_ID)
    receiver.start()

    # Control setpoints / state
    target_height_cmd = DEFAULT_TARGET_HEIGHT_M
    active_target_height = DEFAULT_TARGET_HEIGHT_M

    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_yaw: Optional[float] = None

    active = False
    pending_start = False
    landing = False
    shutdown = False

    # Height PID state
    integral_h = 0.0
    last_thrust = 0.0

    # UDP pose state
    udp = PoseState()
    prev_udp_rx_t = 0.0
    prev_udp_x = 0.0
    prev_udp_y = 0.0
    prev_udp_h = 0.0

    # IMU state (optional fallback)
    imu_state = {
        "t": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    imu_enabled = False
    logconf: Optional[LogConfig] = None

    # IMU integrated estimates (very short fallback only)
    est_x = 0.0
    est_y = 0.0
    est_h = 0.0
    est_vx = 0.0
    est_vy = 0.0
    est_vh = 0.0

    def _imu_cb(_, data, __) -> None:
        imu_state["t"] = time.monotonic()
        if "acc.x" in data:
            imu_state["ax"] = float(data["acc.x"])
        if "acc.y" in data:
            imu_state["ay"] = float(data["acc.y"])
        if "acc.z" in data:
            imu_state["az"] = float(data["acc.z"])
        if "stabilizer.roll" in data:
            imu_state["roll"] = float(data["stabilizer.roll"])
        if "stabilizer.pitch" in data:
            imu_state["pitch"] = float(data["stabilizer.pitch"])
        if "stabilizer.yaw" in data:
            imu_state["yaw"] = float(data["stabilizer.yaw"])

    # Plot setup (optional)
    if plot:
        import matplotlib.pyplot as plt
        max_points = int(PLOT_HISTORY_SEC * CONTROL_HZ)
        t_hist = deque(maxlen=max_points)
        h_hist = deque(maxlen=max_points)
        thrust_hist = deque(maxlen=max_points)
        x_hist = deque(maxlen=max_points)
        y_hist = deque(maxlen=max_points)

        plt.ion()
        fig, (ax_h, ax_t, ax_p) = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
        fig.canvas.manager.set_window_title("Crazyflie Best-of-Both Hold")

        line_h, = ax_h.plot([], [], label="height (m)")
        line_t, = ax_t.plot([], [], label="thrust")
        line_x, = ax_p.plot([], [], label="x (m)")
        line_y, = ax_p.plot([], [], label="y (m)")

        ax_h.set_ylabel("Height [m]")
        ax_t.set_ylabel("Thrust")
        ax_p.set_ylabel("Pos [m]")
        ax_p.set_xlabel("Time [s]")

        ax_h.legend()
        ax_t.legend()
        ax_p.legend()

        next_plot = 0.0
    else:
        plt = None
        t_hist = h_hist = thrust_hist = x_hist = y_hist = None
        fig = ax_h = ax_t = ax_p = None
        line_h = line_t = line_x = line_y = None
        next_plot = 0.0

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        # Try enable IMU logging (optional)
        required_imu = ["acc.x", "acc.y", "acc.z"]
        if all(_log_var_exists(scf.cf, v) for v in required_imu):
            logconf = LogConfig(name="imu", period_in_ms=int(1000 / CONTROL_HZ))
            for name in required_imu:
                logconf.add_variable(name, _log_var_type(scf.cf, name))
            for name in ("stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw"):
                if _log_var_exists(scf.cf, name):
                    logconf.add_variable(name, _log_var_type(scf.cf, name))
            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(_imu_cb)
            logconf.start()
            imu_enabled = True
            print("[OK] IMU logging enabled.")
        else:
            print("[WARN] IMU logging not available (acc.* not in TOC). Running UDP-only.")

        _print_help()

        try:
            start_time = time.monotonic()
            next_tick = time.monotonic()
            last_good_udp_time = 0.0

            # Always keep link alive with neutral setpoints during startup
            for _ in range(int(0.2 / DT)):
                scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                time.sleep(DT)

            while True:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick += DT

                # -----------------------------
                # Commands
                # -----------------------------
                cmd = cmd_reader.poll()
                if cmd:
                    parts = cmd.split()
                    key = parts[0].lower()

                    if key == "help":
                        _print_help()

                    elif key == "status":
                        udp_age = (now - udp.rx_t) if udp.rx_t > 0.0 else float("inf")
                        imu_age = (now - imu_state["t"]) if imu_state["t"] > 0.0 else float("inf")
                        print(
                            f"[STATUS] active={active} pending_start={pending_start} landing={landing} "
                            f"target_h={target_height_cmd:.2f} active_h={active_target_height:.2f} "
                            f"udp_age={udp_age:.3f}s imu_age={imu_age:.3f}s"
                        )

                    elif key == "height" or key in ("h", "z"):
                        if len(parts) != 2:
                            print("Usage: height <meters>")
                        else:
                            try:
                                target_height_cmd = max(0.0, float(parts[1].replace(",", ".")))
                                print(f"[OK] Target height set to {target_height_cmd:.2f} m")
                            except ValueError:
                                print("[ERR] Invalid height value.")

                    elif key == "lock":
                        if udp.rx_t > 0.0:
                            target_x = udp.x
                            target_y = udp.y
                            target_yaw = udp.yaw
                            print("[OK] Position re-locked to current UDP pose.")
                        else:
                            print("[WARN] No UDP pose yet, cannot lock.")

                    elif key == "start":
                        if udp.rx_t > 0.0:
                            target_x = udp.x
                            target_y = udp.y
                            target_yaw = udp.yaw
                            integral_h = 0.0
                            active = True
                            pending_start = False
                            landing = False
                            last_thrust = 0.0
                            active_target_height = clamp(udp.h, 0.0, target_height_cmd)
                            print("[OK] Control started. Position locked.")
                        else:
                            # FIX: do not "do nothing" - we keep sending neutral setpoints while waiting
                            pending_start = True
                            active = False
                            landing = False
                            integral_h = 0.0
                            last_thrust = 0.0
                            print("[WAIT] No UDP pose yet. Waiting for pose to start...")

                    elif key == "stop":
                        if active or pending_start:
                            pending_start = False
                            landing = True
                            active = True  # ensure landing loop runs
                            integral_h = 0.0
                            print("[OK] Landing...")

                    elif key == "quit":
                        shutdown = True
                        if active or landing or pending_start:
                            pending_start = False
                            landing = True
                            active = True
                            integral_h = 0.0
                            print("[OK] Landing before exit...")
                        else:
                            break

                    else:
                        print("[ERR] Unknown command. Type 'help'.")

                # -----------------------------
                # Read UDP pose
                # -----------------------------
                sample = receiver.get_latest()
                if sample is not None:
                    # Map same as your UDP controller:
                    # height = sample.y, x = sample.x, y = sample.z, yaw = sample.oz
                    rx_t = now  # IMPORTANT: use monotonic receive time for freshness
                    h = float(sample.y)
                    x = float(sample.x)
                    y = float(sample.z)
                    yaw = float(sample.oz)

                    # Velocity estimate using monotonic dt (robust, no sample.t mismatch)
                    if prev_udp_rx_t > 0.0:
                        dtp = rx_t - prev_udp_rx_t
                        if dtp > 1e-4:
                            udp.vx = (x - prev_udp_x) / dtp
                            udp.vy = (y - prev_udp_y) / dtp
                            udp.vh = (h - prev_udp_h) / dtp

                    udp.h, udp.x, udp.y, udp.yaw = h, x, y, yaw
                    udp.rx_t = rx_t

                    prev_udp_rx_t = rx_t
                    prev_udp_x, prev_udp_y, prev_udp_h = x, y, h

                    last_good_udp_time = now

                    # If we were waiting for pose to start, start now
                    if pending_start and not active:
                        target_x = udp.x
                        target_y = udp.y
                        target_yaw = udp.yaw
                        active = True
                        pending_start = False
                        landing = False
                        integral_h = 0.0
                        last_thrust = 0.0
                        active_target_height = clamp(udp.h, 0.0, target_height_cmd)
                        print("[OK] Pose received. Control started. Position locked.")

                # -----------------------------
                # Freshness checks
                # -----------------------------
                udp_fresh = (udp.rx_t > 0.0) and ((now - udp.rx_t) < UDP_TIMEOUT_S)
                imu_fresh = imu_enabled and (imu_state["t"] > 0.0) and ((now - imu_state["t"]) < IMU_TIMEOUT_S)

                # -----------------------------
                # IMU short fallback integration (only meaningful for short gaps)
                # -----------------------------
                if imu_fresh:
                    # integrate world acceleration -> velocity -> position
                    # NOTE: this will drift; used only briefly when UDP drops
                    roll = math.radians(imu_state["roll"])
                    pitch = math.radians(imu_state["pitch"])
                    yaw_r = math.radians(imu_state["yaw"])

                    wx, wy, wz = _body_to_world(imu_state["ax"], imu_state["ay"], imu_state["az"], roll, pitch, yaw_r)

                    # crude gravity compensation: assume world Z is "up"
                    # (matches your UDP script approach)
                    G = 9.81
                    wz -= G

                    est_vx += wx * DT
                    est_vy += wy * DT
                    est_vh += wz * DT

                    est_x += est_vx * DT
                    est_y += est_vy * DT
                    est_h += est_vh * DT

                # Sync estimator to UDP whenever UDP is good
                if udp_fresh:
                    est_x = udp.x
                    est_y = udp.y
                    est_h = udp.h
                    est_vx = udp.vx
                    est_vy = udp.vy
                    est_vh = udp.vh

                # -----------------------------
                # Idle behavior: ALWAYS send setpoints
                # (This is the critical "best-of-both" fix.)
                # -----------------------------
                if not active and not pending_start:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    if shutdown:
                        break
                    continue

                # -----------------------------
                # Failsafe: if active but UDP lost for too long -> land
                # -----------------------------
                if active and not landing:
                    if last_good_udp_time > 0.0 and (now - last_good_udp_time) > FAILSAFE_LAND_AFTER_S:
                        landing = True
                        integral_h = 0.0
                        print("[FAILSAFE] UDP lost too long -> landing.")

                # -----------------------------
                # Pick control inputs (UDP preferred, then IMU estimate)
                # -----------------------------
                if udp_fresh:
                    ctrl_h = udp.h
                    ctrl_vh = udp.vh
                    ctrl_x = udp.x
                    ctrl_y = udp.y
                    ctrl_vx = udp.vx
                    ctrl_vy = udp.vy
                    ctrl_yaw = udp.yaw
                elif imu_fresh:
                    ctrl_h = est_h
                    ctrl_vh = est_vh
                    ctrl_x = est_x
                    ctrl_y = est_y
                    ctrl_vx = est_vx
                    ctrl_vy = est_vy
                    ctrl_yaw = imu_state["yaw"]
                else:
                    # No usable sensors: safest is cut thrust
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    continue

                # -----------------------------
                # Height target shaping (climb/land ramps)
                # -----------------------------
                if landing:
                    active_target_height = max(0.0, active_target_height - LANDING_RATE_MPS * DT)
                else:
                    # move toward commanded height smoothly
                    delta = target_height_cmd - active_target_height
                    step = CLIMB_RATE_MPS * DT
                    if abs(delta) <= step:
                        active_target_height = target_height_cmd
                    else:
                        active_target_height += step if delta > 0 else -step

                    # If user commanded essentially zero, auto-land behavior
                    if target_height_cmd <= LANDING_CUTOFF_M:
                        landing = True
                        integral_h = 0.0

                # -----------------------------
                # Height PID
                # -----------------------------
                err_h = active_target_height - ctrl_h
                integral_h = clamp(integral_h + err_h * DT, -INT_LIM_H, INT_LIM_H)

                thrust_cmd = HOVER_THRUST + KP_H * err_h + KI_H * integral_h - KD_H * ctrl_vh
                min_thrust = 0.0 if landing else MIN_THRUST
                thrust_cmd = clamp(thrust_cmd, min_thrust, MAX_THRUST)

                # Thrust slew limit (avoid spikes)
                max_step = THRUST_SLEW * DT
                thrust_cmd = clamp(thrust_cmd, last_thrust - max_step, last_thrust + max_step)
                last_thrust = thrust_cmd

                # -----------------------------
                # Position hold (XY)
                # -----------------------------
                roll_cmd = 0.0
                pitch_cmd = 0.0

                if target_x is not None and target_y is not None:
                    err_x = target_x - ctrl_x
                    err_y = target_y - ctrl_y

                    if abs(err_x) < POS_DEADBAND_M:
                        err_x = 0.0
                    if abs(err_y) < POS_DEADBAND_M:
                        err_y = 0.0

                    pitch_cmd = PITCH_SIGN * (POS_KP * err_x - POS_KD * ctrl_vx)
                    roll_cmd = ROLL_SIGN * (POS_KP * err_y - POS_KD * ctrl_vy)

                    roll_cmd = clamp(roll_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)
                    pitch_cmd = clamp(pitch_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)

                # -----------------------------
                # Yaw hold
                # -----------------------------
                yawrate_cmd = 0.0
                if USE_YAW_HOLD and target_yaw is not None:
                    yaw_error = wrap_angle_deg(target_yaw - ctrl_yaw)
                    yawrate_cmd = clamp(YAW_SIGN * YAW_KP * yaw_error, -MAX_YAWRATE, MAX_YAWRATE)

                # -----------------------------
                # Send setpoint (always!)
                # -----------------------------
                scf.cf.commander.send_setpoint(roll_cmd, pitch_cmd, yawrate_cmd, int(thrust_cmd))

                # -----------------------------
                # Landing completion
                # -----------------------------
                if landing and active_target_height <= LANDING_CUTOFF_M and ctrl_h <= LANDING_CUTOFF_M:
                    active = False
                    landing = False
                    pending_start = False
                    integral_h = 0.0
                    last_thrust = 0.0
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    scf.cf.commander.send_stop_setpoint()
                    print("[OK] Landed. Control stopped.")
                    if shutdown:
                        break

                # -----------------------------
                # Plot update
                # -----------------------------
                if plot and plt is not None:
                    if now >= next_plot:
                        next_plot = now + (1.0 / PLOT_UPDATE_HZ)

                        t_hist.append(now - start_time)
                        h_hist.append(ctrl_h)
                        thrust_hist.append(thrust_cmd)
                        x_hist.append(ctrl_x)
                        y_hist.append(ctrl_y)

                        line_h.set_data(t_hist, h_hist)
                        line_t.set_data(t_hist, thrust_hist)
                        line_x.set_data(t_hist, x_hist)
                        line_y.set_data(t_hist, y_hist)

                        ax_h.relim(); ax_h.autoscale_view()
                        ax_t.relim(); ax_t.autoscale_view()
                        ax_p.relim(); ax_p.autoscale_view()

                        plt.pause(0.001)

        except KeyboardInterrupt:
            print("\n[CTRL+C] Emergency stop...")
        finally:
            receiver.stop()
            try:
                if logconf is not None:
                    logconf.stop()
            except Exception:
                pass

            try:
                # send a short burst of zero setpoints for safety
                for _ in range(int(0.4 / DT)):
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    time.sleep(DT)
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--plot", action="store_true", help="enable live plotting")
    args = parser.parse_args()

    main(plot=args.plot)
