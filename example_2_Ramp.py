import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import cfMRB

# Open-loop thrust ramp using low-level RPYT setpoints (send_setpoint).
# - BRUSHED: works open-loop (no sensors), but height is NOT stable.
# - BRUSHLESS: same ramp, but we add an "arm/disarm" step via parameters
#   (matching your working hover test behavior).

URI_BRUSHED   = "radio://0/80/2M/E7E7E7E7E7"  # brushed (AA)
URI_BRUSHLESS = "radio://0/80/2M/E7E7E7E7AA"  # brushless (E7)

IS_BRUSHLESS = True
URI = URI_BRUSHLESS if IS_BRUSHLESS else URI_BRUSHED

RATE_HZ = 50
DT = 1.0 / RATE_HZ

THRUST_START = 8000
THRUST_STEP  = 2000
THRUST_MAX   = 22000
HOLD_TIME    = 1.0


def send_rpyt_for(scf, roll, pitch, yawrate, thrust, seconds):
    """Send the same RPYT setpoint repeatedly for 'seconds'."""
    n = int(seconds * RATE_HZ)
    for _ in range(n):
        scf.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(DT)


def open_loop_ramp_test(scf):
    """
    Open-loop ramp test:
    - Slowly increases thrust in steps
    - Holds each level for HOLD_TIME
    - Then ramps down and stops
    """
    print("Open-loop RPYT ramp test starting.")
    print("Keep roll/pitch/yawrate at 0. This is thrust-only.")

    # Small pre-stream (helps some setups avoid immediate timeout behavior)
    send_rpyt_for(scf, 0.0, 0.0, 0.0, 0, 0.2)

    # Ramp up
    thrust = THRUST_START
    while thrust <= THRUST_MAX:
        print(f"Thrust = {thrust}")
        send_rpyt_for(scf, 0.0, 0.0, 0.0, thrust, HOLD_TIME)
        thrust += THRUST_STEP

    # Ramp down
    thrust -= THRUST_STEP
    while thrust >= THRUST_START:
        print(f"Thrust = {thrust}")
        send_rpyt_for(scf, 0.0, 0.0, 0.0, thrust, HOLD_TIME)
        thrust -= THRUST_STEP

    # Stop cleanly
    print("Stopping (thrust=0, stop setpoint).")
    send_rpyt_for(scf, 0.0, 0.0, 0.0, 0, 0.3)
    scf.cf.commander.send_stop_setpoint()
    time.sleep(0.1)


if __name__ == "__main__":
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        try:
            if IS_BRUSHLESS:
                cfMRB.start_motors(scf,6000)

            open_loop_ramp_test(scf)

        except KeyboardInterrupt:
            print("\n[CTRL+C] Emergency stop ...")
            # Best-effort stop
            try:
                send_rpyt_for(scf, 0.0, 0.0, 0.0, 0, 0.2)
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass

        finally:
            # Always stop setpoints
            try:
                send_rpyt_for(scf, 0.0, 0.0, 0.0, 0, 0.2)
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass

            if IS_BRUSHLESS:
                try:
                    cfMRB.stop_motors(scf)
                except Exception:
                    pass
