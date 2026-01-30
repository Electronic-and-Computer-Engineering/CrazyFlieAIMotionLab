# Usage:
#   python cf21_brushed_udp1001_controller.py --mode free --arm --height 0.4
#   python cf21_brushed_udp1001_controller.py --mode tune --arm --rate 50

import argparse
import json
import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from udp_pose_receiver import UdpPoseReceiver, Pose


URI_DEFAULT = "radio://0/80/2M/E7E7E7E7E7"
DRONE_ID_DEFAULT = 1001
BIND_IP_DEFAULT = "0.0.0.0"
BIND_PORT_DEFAULT = 40001
RATE_DEFAULT_HZ = 50.0
HEIGHT_DEFAULT_M = 0.4

# Thrust and control limits (matching brushed scripts)
MIN_THRUST = 54000
HOVER_THRUST = 56000
MAX_THRUST = 65535
THRUST_SLEW = 20000.0

# Height PID
KP_Z = 2000.0
KI_Z = 750.0
KD_Z = 8000.0
INT_LIM_Z = 0.5

# Horizontal PD
POS_KP = 30 #15.0
POS_KD = 20 #10.0
MAX_TILT_DEG = 5 #10.0
POS_DEADBAND_M = 0.01
ROLL_SIGN = -1.0
PITCH_SIGN = -1.0

# Yaw hold (optional, uses pose.yaw)
USE_YAW_HOLD = True
YAW_KP = 60.0
MAX_YAWRATE = 90.0
YAW_SIGN = -1.0

# Pose freshness / safety
POSE_TIMEOUT_S = 0.2
FAILSAFE_LAND_AFTER_S = 1.0

# Takeoff / landing shaping
CLIMB_RATE_MPS = 0.6
LANDING_RATE_MPS = 0.3
LANDING_CUTOFF_M = 0.05


try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False


@dataclass
class ControlGains:
    kp_z: float = KP_Z
    ki_z: float = KI_Z
    kd_z: float = KD_Z
    pos_kp: float = POS_KP
    pos_kd: float = POS_KD
    yaw_kp: float = YAW_KP


@dataclass
class ControlState:
    integral_z: float = 0.0
    last_thrust: float = 0.0
    last_pose_t: float = 0.0
    last_x: float = 0.0
    last_y: float = 0.0
    last_z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    pose: Optional[Pose] = None


@dataclass
class Target:
    x: float
    y: float
    z: float
    yaw: Optional[float] = None


@dataclass
class ControlOutput:
    roll: float
    pitch: float
    yawrate: float
    thrust: float


@dataclass
class StepMetrics:
    overshoot: float
    settle_time: float
    steady_error: float
    peak: float
    target: float
    step: float


class RateLimiter:
    def __init__(self, interval_s: float) -> None:
        self._interval_s = interval_s
        self._next_time = 0.0

    def ready(self, now: Optional[float] = None) -> bool:
        if now is None:
            now = time.monotonic()
        if now >= self._next_time:
            self._next_time = now + self._interval_s
            return True
        return False


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, value))


def wrap_angle_deg(err: float) -> float:
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err

def update_pose_state(state: ControlState, pose: Pose) -> None:
    if state.last_pose_t > 0.0 and pose.t > state.last_pose_t:
        dt_pose = pose.t - state.last_pose_t
        if dt_pose > 1e-6:
            state.vx = (pose.x - state.last_x) / dt_pose
            state.vy = (pose.y - state.last_y) / dt_pose
            state.vz = (pose.z - state.last_z) / dt_pose
    state.last_pose_t = pose.t
    state.last_x = pose.x
    state.last_y = pose.y
    state.last_z = pose.z
    state.pose = pose


def update_active_target_z(
    active_target_z: float,
    target_z_cmd: float,
    landing: bool,
    dt: float,
    *,
    ramp: bool,
) -> float:
    if not ramp:
        return target_z_cmd
    if landing:
        return max(0.0, active_target_z - LANDING_RATE_MPS * dt)
    delta = target_z_cmd - active_target_z
    step = CLIMB_RATE_MPS * dt
    if abs(delta) <= step:
        return target_z_cmd
    return active_target_z + step if delta > 0.0 else active_target_z - step


def compute_setpoint(
    pose: Pose,
    target: Target,
    state: ControlState,
    gains: ControlGains,
    dt: float,
    *,
    landing: bool,
    allow_xy: bool,
    allow_yaw: bool,
    integrate: bool,
) -> ControlOutput:
    err_z = target.z - pose.z
    if integrate:
        state.integral_z = clamp(state.integral_z + err_z * dt, -INT_LIM_Z, INT_LIM_Z)

    thrust_cmd = HOVER_THRUST + gains.kp_z * err_z + gains.ki_z * state.integral_z - gains.kd_z * state.vz
    min_thrust = 0.0 if landing else MIN_THRUST
    thrust_cmd = clamp(thrust_cmd, min_thrust, MAX_THRUST)

    max_step = THRUST_SLEW * dt
    thrust_cmd = clamp(thrust_cmd, state.last_thrust - max_step, state.last_thrust + max_step)
    state.last_thrust = thrust_cmd

    roll_cmd = 0.0
    pitch_cmd = 0.0
    if allow_xy:
        err_x = target.x - pose.x
        err_y = target.y - pose.y
        if abs(err_x) < POS_DEADBAND_M:
            err_x = 0.0
        if abs(err_y) < POS_DEADBAND_M:
            err_y = 0.0
        pitch_cmd = PITCH_SIGN * (gains.pos_kp * err_x - gains.pos_kd * state.vx)
        roll_cmd = ROLL_SIGN * (gains.pos_kp * err_y - gains.pos_kd * state.vy)
        roll_cmd = clamp(roll_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)
        pitch_cmd = clamp(pitch_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)

    yawrate_cmd = 0.0
    if USE_YAW_HOLD and allow_yaw and target.yaw is not None and pose.yaw is not None:
        yaw_error = wrap_angle_deg(target.yaw - pose.yaw)
        yawrate_cmd = clamp(YAW_SIGN * gains.yaw_kp * yaw_error, -MAX_YAWRATE, MAX_YAWRATE)

    return ControlOutput(roll=roll_cmd, pitch=pitch_cmd, yawrate=yawrate_cmd, thrust=thrust_cmd)


def send_setpoint(
    scf: SyncCrazyflie,
    output: ControlOutput,
    *,
    armed: bool,
    dry_run: bool,
) -> None:
    if dry_run:
        return
    roll = output.roll
    pitch = output.pitch
    yawrate = output.yawrate
    thrust = int(output.thrust)
    if not armed:
        roll = 0.0
        pitch = 0.0
        yawrate = 0.0
        thrust = 0
    scf.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)


def send_neutral(scf: SyncCrazyflie, *, dry_run: bool) -> None:
    if dry_run:
        return
    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)


def parse_axis_delta(parts: List[str]) -> Optional[float]:
    if len(parts) == 2:
        try:
            return float(parts[1].replace(",", "."))
        except ValueError:
            return None
    if len(parts) == 3 and parts[1] in ("+", "-"):
        try:
            return float(parts[1] + parts[2].replace(",", "."))
        except ValueError:
            return None
    return None


def _input_loop(cmd_queue: queue.Queue) -> None:
    while True:
        try:
            line = input()
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

    def poll(self) -> Optional[str]:
        if self._threaded:
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                return None
        return self._poll_msvcrt()

    def _poll_msvcrt(self) -> Optional[str]:
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


def _print_free_help() -> None:
    print("Commands:")
    print("  start                 start takeoff + control")
    print("  x +/- <m>             increment X target")
    print("  y +/- <m>             increment Y target")
    print("  z +/- <m>             increment Z target")
    print("  goto <x> <y> <z>       set absolute target")
    print("  hold                  keep current target")
    print("  status                show pose + target + errors")
    print("  land                  land and exit")
    print("  quit                  land (if needed) and exit")


def _print_tune_help() -> None:
    print("Commands:")
    print("  start                 begin auto-tuning")
    print("  status                show pose + status")
    print("  quit                  land (if needed) and exit")

def analyze_step(samples: List[Tuple[float, float]], target: float, step: float, settle_band: float) -> StepMetrics:
    if not samples:
        return StepMetrics(
            overshoot=0.0,
            settle_time=float("inf"),
            steady_error=0.0,
            peak=target,
            target=target,
            step=step,
        )
    values = [v for _, v in samples]
    peak = max(values) if step >= 0.0 else min(values)
    denom = abs(step) if abs(step) > 1e-6 else 1.0
    overshoot = (peak - target) / denom

    settle_time = samples[-1][0] - samples[0][0]
    for i in range(len(samples)):
        if all(abs(v - target) <= settle_band for _, v in samples[i:]):
            settle_time = samples[i][0] - samples[0][0]
            break

    tail = values[int(0.8 * len(values)):] or values
    steady_error = sum(tail) / len(tail) - target

    return StepMetrics(
        overshoot=overshoot,
        settle_time=settle_time,
        steady_error=steady_error,
        peak=peak,
        target=target,
        step=step,
    )


def clamp_gains(gains: ControlGains) -> None:
    gains.kp_z = clamp(gains.kp_z, 2000.0, 15000.0)
    gains.ki_z = clamp(gains.ki_z, 0.0, 3000.0)
    gains.kd_z = clamp(gains.kd_z, 2000.0, 15000.0)
    gains.pos_kp = clamp(gains.pos_kp, 5.0, 30.0)
    gains.pos_kd = clamp(gains.pos_kd, 0.0, 20.0)


def tune_height_gains(gains: ControlGains, metrics: StepMetrics) -> None:
    overshoot = abs(metrics.overshoot)
    if overshoot > 0.25:
        gains.kd_z *= 1.2
        gains.kp_z *= 0.9
    elif overshoot < 0.05 and metrics.settle_time > 1.0:
        gains.kp_z *= 1.1
    if abs(metrics.steady_error) > 0.01:
        gains.ki_z *= 1.1
    clamp_gains(gains)


def tune_pos_gains(gains: ControlGains, metrics: StepMetrics) -> None:
    overshoot = abs(metrics.overshoot)
    if overshoot > 0.25:
        gains.pos_kd *= 1.2
        gains.pos_kp *= 0.9
    elif overshoot < 0.05 and metrics.settle_time > 1.0:
        gains.pos_kp *= 1.1
    clamp_gains(gains)


def run_hold_for(
    scf: SyncCrazyflie,
    receiver: UdpPoseReceiver,
    state: ControlState,
    gains: ControlGains,
    target: Target,
    active_target_z: float,
    dt: float,
    *,
    armed: bool,
    dry_run: bool,
    duration_s: float,
    landing: bool,
    allow_xy: bool,
    allow_yaw: bool,
    ramp_target: bool,
    last_good_pose_time: float,
    sample_axis: Optional[str] = None,
) -> Tuple[List[Tuple[float, float]], float, float, bool]:
    samples: List[Tuple[float, float]] = []
    start = time.monotonic()
    next_tick = time.monotonic()
    aborted = False

    while True:
        now = time.monotonic()
        if now - start >= duration_s:
            break
        if now < next_tick:
            time.sleep(next_tick - now)
        next_tick += dt

        pose = receiver.get_latest()
        if pose is not None and (state.pose is None or pose.t > state.last_pose_t):
            update_pose_state(state, pose)

        if ramp_target:
            active_target_z = update_active_target_z(active_target_z, target.z, landing, dt, ramp=True)
        else:
            active_target_z = target.z

        pose = state.pose
        pose_age = pose.age(now) if pose is not None else float("inf")
        pose_fresh = pose is not None and pose_age < POSE_TIMEOUT_S
        if pose_fresh:
            last_good_pose_time = now
        else:
            if last_good_pose_time > 0.0 and (now - last_good_pose_time) > FAILSAFE_LAND_AFTER_S:
                aborted = True

        if pose is None:
            send_neutral(scf, dry_run=dry_run)
        else:
            allow_xy_now = allow_xy and pose_fresh and not landing
            allow_yaw_now = allow_yaw and pose_fresh and not landing
            integrate = pose_fresh
            current_target = Target(target.x, target.y, active_target_z, target.yaw)
            output = compute_setpoint(
                pose,
                current_target,
                state,
                gains,
                dt,
                landing=landing,
                allow_xy=allow_xy_now,
                allow_yaw=allow_yaw_now,
                integrate=integrate,
            )
            send_setpoint(scf, output, armed=armed, dry_run=dry_run)

            if sample_axis is not None:
                if hasattr(pose, sample_axis):
                    samples.append((now - start, float(getattr(pose, sample_axis))))

        if aborted:
            break

    return samples, active_target_z, last_good_pose_time, aborted

def run_free_mode(
    scf: SyncCrazyflie,
    receiver: UdpPoseReceiver,
    *,
    rate_hz: float,
    height_m: float,
    armed: bool,
    dry_run: bool,
) -> None:
    dt = 1.0 / rate_hz
    cmd_reader = _CommandReader()
    _print_free_help()

    gains = ControlGains()
    state = ControlState()
    target = Target(0.0, 0.0, max(0.0, height_m), None)
    active_target_z = 0.0

    active = False
    pending_start = False
    landing = False
    exit_after_landing = False
    last_good_pose_time = 0.0

    setpoint_log = RateLimiter(1.0)
    stale_log = RateLimiter(1.0)

    next_tick = time.monotonic()
    while True:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        next_tick += dt

        pose = receiver.get_latest()
        if pose is not None and (state.pose is None or pose.t > state.last_pose_t):
            update_pose_state(state, pose)

        pose = state.pose
        pose_age = pose.age(now) if pose is not None else float("inf")
        pose_fresh = pose is not None and pose_age < POSE_TIMEOUT_S
        if pose_fresh:
            last_good_pose_time = now

        cmd = cmd_reader.poll()
        if cmd:
            parts = cmd.strip().split()
            if not parts:
                continue
            key = parts[0].lower()

            if key in ("help", "?"):
                _print_free_help()
            elif key == "start":
                if pose_fresh:
                    target.x = pose.x
                    target.y = pose.y
                    target.yaw = pose.yaw
                    active_target_z = pose.z
                    active = True
                    pending_start = False
                    landing = False
                    exit_after_landing = False
                    state.integral_z = 0.0
                    state.last_thrust = 0.0
                    print("[OK] Control started. Position locked.")
                else:
                    pending_start = True
                    active = False
                    landing = False
                    print("[WAIT] No fresh pose yet. Waiting...")
            elif key in ("hold",):
                print("[OK] Holding current target.")
            elif key == "status":
                if pose is None:
                    print("[STATUS] No pose yet.")
                else:
                    err_x = target.x - pose.x
                    err_y = target.y - pose.y
                    err_z = target.z - pose.z
                    print(
                        f"[STATUS] pose=({pose.x:+.2f},{pose.y:+.2f},{pose.z:+.2f}) "
                        f"target=({target.x:+.2f},{target.y:+.2f},{target.z:+.2f}) "
                        f"err=({err_x:+.2f},{err_y:+.2f},{err_z:+.2f}) age={pose_age:.3f}s"
                    )
            elif key == "land":
                if active or pending_start:
                    pending_start = False
                    active = True
                    landing = True
                    exit_after_landing = True
                    target.z = 0.0
                    print("[OK] Landing...")
                else:
                    print("[OK] Already idle.")
            elif key in ("quit", "exit"):
                if active or landing or pending_start:
                    pending_start = False
                    active = True
                    landing = True
                    exit_after_landing = True
                    target.z = 0.0
                    print("[OK] Landing before exit...")
                else:
                    break
            elif key == "goto" and len(parts) == 4:
                try:
                    target.x = float(parts[1].replace(",", "."))
                    target.y = float(parts[2].replace(",", "."))
                    target.z = max(0.0, float(parts[3].replace(",", ".")))
                    print(f"[OK] Target set to {target.x:.2f} {target.y:.2f} {target.z:.2f}")
                except ValueError:
                    print("[ERR] Invalid goto values.")
            elif key in ("x", "y", "z"):
                delta = parse_axis_delta(parts)
                if delta is None:
                    print("[ERR] Usage: x +/- <m>")
                else:
                    if key == "x":
                        target.x += delta
                    elif key == "y":
                        target.y += delta
                    else:
                        target.z = max(0.0, target.z + delta)
                    print(f"[OK] Target now {target.x:.2f} {target.y:.2f} {target.z:.2f}")
            else:
                print("[ERR] Unknown command. Type 'help'.")

        if pending_start and pose_fresh:
            target.x = pose.x
            target.y = pose.y
            target.yaw = pose.yaw
            active_target_z = pose.z
            active = True
            pending_start = False
            landing = False
            exit_after_landing = False
            state.integral_z = 0.0
            state.last_thrust = 0.0
            print("[OK] Pose received. Control started.")

        if not active and not pending_start:
            state.integral_z = 0.0
            state.last_thrust = 0.0
            send_neutral(scf, dry_run=dry_run)
            continue

        if active and not landing:
            if not pose_fresh:
                landing = True
                target.z = 0.0
                exit_after_landing = True
                if stale_log.ready(now):
                    print(f"[FAILSAFE] Pose stale (age={pose_age:.3f}s). Landing.")
            elif last_good_pose_time > 0.0 and (now - last_good_pose_time) > FAILSAFE_LAND_AFTER_S:
                landing = True
                target.z = 0.0
                exit_after_landing = True
                print("[FAILSAFE] Pose stale too long. Landing.")

        active_target_z = update_active_target_z(active_target_z, target.z, landing, dt, ramp=True)

        if pose is None:
            send_neutral(scf, dry_run=dry_run)
            continue

        allow_xy = pose_fresh and not landing
        allow_yaw = pose_fresh and not landing
        integrate = pose_fresh

        current_target = Target(target.x, target.y, active_target_z, target.yaw)
        output = compute_setpoint(
            pose,
            current_target,
            state,
            gains,
            dt,
            landing=landing,
            allow_xy=allow_xy,
            allow_yaw=allow_yaw,
            integrate=integrate,
        )
        send_setpoint(scf, output, armed=armed, dry_run=dry_run)

        if setpoint_log.ready(now):
            print(
                f"[SP] roll={output.roll:+.2f} pitch={output.pitch:+.2f} "
                f"now_z={pose.z:.2f} age={pose_age:.3f}s"
                f"yawrate={output.yawrate:+.2f} thrust={int(output.thrust)} "
                f"target_z={current_target.z:.2f} age={pose_age:.3f}s"
            )

        if landing and active_target_z <= LANDING_CUTOFF_M and pose.z <= LANDING_CUTOFF_M:
            active = False
            landing = False
            state.integral_z = 0.0
            state.last_thrust = 0.0
            send_neutral(scf, dry_run=dry_run)
            if not dry_run:
                scf.cf.commander.send_stop_setpoint()
            print("[OK] Landed.")
            if exit_after_landing:
                break

def run_tune_mode(
    scf: SyncCrazyflie,
    receiver: UdpPoseReceiver,
    *,
    rate_hz: float,
    height_m: float,
    armed: bool,
    dry_run: bool,
    drone_id: int,
    uri: str,
) -> None:
    dt = 1.0 / rate_hz
    cmd_reader = _CommandReader()
    _print_tune_help()

    gains = ControlGains()
    state = ControlState()
    target = Target(0.0, 0.0, max(0.0, height_m), None)
    active_target_z = 0.0
    last_good_pose_time = 0.0
    pending_start = False

    next_tick = time.monotonic()
    while True:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        next_tick += dt

        pose = receiver.get_latest()
        if pose is not None and (state.pose is None or pose.t > state.last_pose_t):
            update_pose_state(state, pose)

        pose = state.pose
        pose_age = pose.age(now) if pose is not None else float("inf")
        pose_fresh = pose is not None and pose_age < POSE_TIMEOUT_S
        if pose_fresh:
            last_good_pose_time = now

        cmd = cmd_reader.poll()
        if cmd:
            parts = cmd.strip().split()
            if not parts:
                continue
            key = parts[0].lower()
            if key in ("help", "?"):
                _print_tune_help()
            elif key == "status":
                if pose is None:
                    print("[STATUS] No pose yet.")
                else:
                    print(
                        f"[STATUS] pose=({pose.x:+.2f},{pose.y:+.2f},{pose.z:+.2f}) "
                        f"age={pose_age:.3f}s gains=KP_Z:{gains.kp_z:.0f} KI_Z:{gains.ki_z:.0f} KD_Z:{gains.kd_z:.0f} "
                        f"POS_KP:{gains.pos_kp:.2f} POS_KD:{gains.pos_kd:.2f}"
                    )
            elif key in ("quit", "exit"):
                print("[OK] Exit requested.")
                return
            elif key == "start":
                if pose_fresh:
                    print("[OK] Starting auto-tune.")
                    break
                pending_start = True
                print("[WAIT] No fresh pose yet. Waiting...")
            else:
                print("[ERR] Unknown command. Type 'help'.")

        if pending_start and pose_fresh:
            print("[OK] Pose received. Starting auto-tune.")
            break

        send_neutral(scf, dry_run=dry_run)

    if pose is None:
        print("[ERR] No pose available, aborting tune.")
        return

    base_x = pose.x
    base_y = pose.y
    base_z = max(0.0, height_m)
    target.x = base_x
    target.y = base_y
    target.yaw = pose.yaw
    target.z = base_z
    active_target_z = pose.z

    print(f"[TUNE] Takeoff to {base_z:.2f} m...")
    _, active_target_z, last_good_pose_time, aborted = run_hold_for(
        scf,
        receiver,
        state,
        gains,
        target,
        active_target_z,
        dt,
        armed=armed,
        dry_run=dry_run,
        duration_s=2.0,
        landing=False,
        allow_xy=True,
        allow_yaw=True,
        ramp_target=True,
        last_good_pose_time=last_good_pose_time,
    )
    if aborted:
        print("[FAILSAFE] Pose lost during takeoff. Aborting tune.")
        return

    iterations = 2
    step_z = 0.05
    step_xy = 0.05
    hold_s = 2.0
    settle_band = 0.01

    for idx in range(iterations):
        print(f"[TUNE] Iteration {idx + 1}/{iterations}")

        # Height step up
        target.z = base_z + step_z
        samples, active_target_z, last_good_pose_time, aborted = run_hold_for(
            scf,
            receiver,
            state,
            gains,
            target,
            active_target_z,
            dt,
            armed=armed,
            dry_run=dry_run,
            duration_s=hold_s,
            landing=False,
            allow_xy=True,
            allow_yaw=True,
            ramp_target=False,
            last_good_pose_time=last_good_pose_time,
            sample_axis="z",
        )
        if aborted:
            print("[FAILSAFE] Pose lost during Z+ step. Aborting tune.")
            return
        metrics_up = analyze_step(samples, target.z, step_z, settle_band)

        # Height step down
        target.z = base_z - step_z
        samples, active_target_z, last_good_pose_time, aborted = run_hold_for(
            scf,
            receiver,
            state,
            gains,
            target,
            active_target_z,
            dt,
            armed=armed,
            dry_run=dry_run,
            duration_s=hold_s,
            landing=False,
            allow_xy=True,
            allow_yaw=True,
            ramp_target=False,
            last_good_pose_time=last_good_pose_time,
            sample_axis="z",
        )
        if aborted:
            print("[FAILSAFE] Pose lost during Z- step. Aborting tune.")
            return
        metrics_down = analyze_step(samples, target.z, -step_z, settle_band)

        avg_metrics = StepMetrics(
            overshoot=0.5 * (metrics_up.overshoot + metrics_down.overshoot),
            settle_time=0.5 * (metrics_up.settle_time + metrics_down.settle_time),
            steady_error=0.5 * (metrics_up.steady_error + metrics_down.steady_error),
            peak=metrics_up.peak,
            target=base_z,
            step=step_z,
        )
        tune_height_gains(gains, avg_metrics)
        print(
            f"[TUNE] Height metrics: overshoot={avg_metrics.overshoot:+.2f} "
            f"settle={avg_metrics.settle_time:.2f}s steady_err={avg_metrics.steady_error:+.2f}"
        )
        print(f"[TUNE] Height gains: KP={gains.kp_z:.0f} KI={gains.ki_z:.0f} KD={gains.kd_z:.0f}")

        # Return to base height
        target.z = base_z
        _, active_target_z, last_good_pose_time, aborted = run_hold_for(
            scf,
            receiver,
            state,
            gains,
            target,
            active_target_z,
            dt,
            armed=armed,
            dry_run=dry_run,
            duration_s=1.0,
            landing=False,
            allow_xy=True,
            allow_yaw=True,
            ramp_target=False,
            last_good_pose_time=last_good_pose_time,
        )
        if aborted:
            print("[FAILSAFE] Pose lost during settle. Aborting tune.")
            return

        # X step
        target.x = base_x + step_xy
        samples, active_target_z, last_good_pose_time, aborted = run_hold_for(
            scf,
            receiver,
            state,
            gains,
            target,
            active_target_z,
            dt,
            armed=armed,
            dry_run=dry_run,
            duration_s=hold_s,
            landing=False,
            allow_xy=True,
            allow_yaw=True,
            ramp_target=False,
            last_good_pose_time=last_good_pose_time,
            sample_axis="x",
        )
        if aborted:
            print("[FAILSAFE] Pose lost during X step. Aborting tune.")
            return
        metrics_x = analyze_step(samples, target.x, step_xy, settle_band)

        # Y step
        target.x = base_x
        target.y = base_y + step_xy
        samples, active_target_z, last_good_pose_time, aborted = run_hold_for(
            scf,
            receiver,
            state,
            gains,
            target,
            active_target_z,
            dt,
            armed=armed,
            dry_run=dry_run,
            duration_s=hold_s,
            landing=False,
            allow_xy=True,
            allow_yaw=True,
            ramp_target=False,
            last_good_pose_time=last_good_pose_time,
            sample_axis="y",
        )
        if aborted:
            print("[FAILSAFE] Pose lost during Y step. Aborting tune.")
            return
        metrics_y = analyze_step(samples, target.y, step_xy, settle_band)

        avg_pos = StepMetrics(
            overshoot=0.5 * (metrics_x.overshoot + metrics_y.overshoot),
            settle_time=0.5 * (metrics_x.settle_time + metrics_y.settle_time),
            steady_error=0.0,
            peak=metrics_x.peak,
            target=0.0,
            step=step_xy,
        )
        tune_pos_gains(gains, avg_pos)
        print(
            f"[TUNE] Pos metrics: overshoot={avg_pos.overshoot:+.2f} settle={avg_pos.settle_time:.2f}s"
        )
        print(f"[TUNE] Pos gains: KP={gains.pos_kp:.2f} KD={gains.pos_kd:.2f}")

        # Return to base
        target.x = base_x
        target.y = base_y
        _, active_target_z, last_good_pose_time, aborted = run_hold_for(
            scf,
            receiver,
            state,
            gains,
            target,
            active_target_z,
            dt,
            armed=armed,
            dry_run=dry_run,
            duration_s=1.0,
            landing=False,
            allow_xy=True,
            allow_yaw=True,
            ramp_target=False,
            last_good_pose_time=last_good_pose_time,
        )
        if aborted:
            print("[FAILSAFE] Pose lost during settle. Aborting tune.")
            return

    print("[TUNE] Final gains:")
    print(f"  Height: KP={gains.kp_z:.0f} KI={gains.ki_z:.0f} KD={gains.kd_z:.0f}")
    print(f"  Position: KP={gains.pos_kp:.2f} KD={gains.pos_kd:.2f}")

    out_path = os.path.join(os.path.dirname(__file__), f"pid_tuning_udp{drone_id}.json")
    payload = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "drone_id": drone_id,
        "uri": uri,
        "height_pid": {"kp": gains.kp_z, "ki": gains.ki_z, "kd": gains.kd_z},
        "pos_pd": {"kp": gains.pos_kp, "kd": gains.pos_kd},
    }
    try:
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"[TUNE] Saved gains to {out_path}")
    except OSError as exc:
        print(f"[WARN] Could not save gains: {exc}")

    print("[TUNE] Landing...")
    target.z = 0.0
    landing = True
    _, active_target_z, last_good_pose_time, _ = run_hold_for(
        scf,
        receiver,
        state,
        gains,
        target,
        active_target_z,
        dt,
        armed=armed,
        dry_run=dry_run,
        duration_s=4.0,
        landing=landing,
        allow_xy=False,
        allow_yaw=False,
        ramp_target=True,
        last_good_pose_time=last_good_pose_time,
    )
    send_neutral(scf, dry_run=dry_run)
    if not dry_run:
        scf.cf.commander.send_stop_setpoint()

def main() -> None:
    parser = argparse.ArgumentParser(description="Crazyflie 2.1 brushed UDP controller (ID 1001)")
    parser.add_argument("--mode", choices=["tune", "free"], required=True, help="Control mode")
    parser.add_argument("--uri", default=URI_DEFAULT, help="Crazyflie URI")
    parser.add_argument("--id", dest="drone_id", type=int, default=DRONE_ID_DEFAULT, help="UDP drone ID")
    parser.add_argument("--bind-ip", default=BIND_IP_DEFAULT, help="UDP bind IP")
    parser.add_argument("--bind-port", action="append", type=int, default=None, help="UDP bind port (repeatable)")
    parser.add_argument("--rate", type=float, default=RATE_DEFAULT_HZ, help="Control rate (Hz)")
    parser.add_argument("--height", type=float, default=HEIGHT_DEFAULT_M, help="Takeoff / tune height (m)")
    parser.add_argument("--arm", action="store_true", help="Enable motor commands")
    parser.add_argument("--dry-run", action="store_true", help="Run without sending commands")
    args = parser.parse_args()

    bind_ports = args.bind_port if args.bind_port else BIND_PORT_DEFAULT

    receiver = UdpPoseReceiver(bind_ip=args.bind_ip, bind_port=bind_ports, drone_id=args.drone_id, swap_yz=True)
    receiver.start()

    cflib.crtp.init_drivers(enable_debug_driver=False)
    print(f"[CF] Connecting to {args.uri}...")

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("[CF] Connected.")
        try:
            if args.mode == "free":
                run_free_mode(
                    scf,
                    receiver,
                    rate_hz=args.rate,
                    height_m=args.height,
                    armed=args.arm,
                    dry_run=args.dry_run,
                )
            else:
                run_tune_mode(
                    scf,
                    receiver,
                    rate_hz=args.rate,
                    height_m=args.height,
                    armed=args.arm,
                    dry_run=args.dry_run,
                    drone_id=args.drone_id,
                    uri=args.uri,
                )
        except KeyboardInterrupt:
            print("\n[CTRL+C] Landing...")
            try:
                send_neutral(scf, dry_run=args.dry_run)
                if not args.dry_run:
                    scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass
        finally:
            receiver.stop()
            try:
                if not args.dry_run:
                    for _ in range(int(0.4 / (1.0 / args.rate))):
                        scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                        time.sleep(1.0 / args.rate)
                    scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass


if __name__ == "__main__":
    main()
