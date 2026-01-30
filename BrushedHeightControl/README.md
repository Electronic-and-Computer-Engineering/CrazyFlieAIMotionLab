# BrushedHeightControl

Short, practical tools for Crazyflie 2.1 (brushed) using external UDP pose data.

## Motion System

**Important:** To use the AI Motion lab system, take at least 4 markers on the drone, so the system works acurratly.

More markers means better stability as the system is less prone to loosing pitch, roll, etc. when it looses one marker.

## Files

- `udp_pose_receiver.py`
  - Reusable UDP listener for pose packets.
  - Filters by drone/object ID, corrects axis swap (incoming Y/Z are swapped), and exposes a `Pose` dataclass.
  - Uses a background thread + `select` and provides `get_latest(timeout=None)` plus optional `subscribe()`.
  - **Packet format (from repo convention):** 14 bytes, `<7h` = `id, x, y, z, ox, oy, oz`
    - Position scale: `x/y/z = int16 / 1000.0` (meters)
    - Orientation scale: `ox/oy/oz = int16 / 32767.0` (unitless)

- `cf21_brushed_udp1001_controller.py`
  - CLI controller for Crazyflie 2.1 at `radio://0/80/2M/E7E7E7E7E7`.
  - Uses `udp_pose_receiver.py` (already swaps Y/Z), so **height = pose.z**.
  - Two modes:
    - `--mode free` : interactive hold & incremental setpoint control.
    - `--mode tune` : automatic PID/PD tuning routine, saves JSON.
  - Safety:
    - `--arm` is required to spin motors.
    - `--dry-run` runs the loop without sending any setpoints.
    - Pose stale (>0.2s) triggers landing failsafe.

- `brushed_init_ramp.py`
  - Simple motor spin-up + thrust ramp test (no external pose required).
  - Useful for verifying link and motor response.

## Quick start

### 1) Check UDP pose stream
```bash
python udp_pose_receiver.py --bind 0.0.0.0 --port 40001 --id 1001
```

### 2) Free mode (interactive hold)
```bash
python cf21_brushed_udp1001_controller.py --mode free --arm --height 0.4
```
Commands:
- `start` ? lock current pose as target (hold X/Y and yaw if available)
- `x + 0.1`, `y - 0.2`, `z + 0.05` ? incremental target changes
- `goto x y z` ? absolute target
- `status` ? show pose/target/errors
- `land` / `quit` ? controlled landing

### 3) Auto tuning mode
```bash
python cf21_brushed_udp1001_controller.py --mode tune --arm --rate 50
```
- Runs small step tests in Z, then X/Y.
- Prints final gains and saves `pid_tuning_udp1001.json` in this folder.

### 4) Motor ramp test (no UDP)
```bash
python brushed_init_ramp.py
```

## Safety notes
- Use `--dry-run` for logic testing without motors.
- Use `--arm` only when you are ready to fly.
- Keep clear space; the controller will land if pose becomes stale.

## Common options
- `--uri` override the Crazyflie URI
- `--id` target UDP drone/object ID
- `--bind-ip`, `--bind-port` UDP listen settings
- `--rate` control loop rate (Hz)

