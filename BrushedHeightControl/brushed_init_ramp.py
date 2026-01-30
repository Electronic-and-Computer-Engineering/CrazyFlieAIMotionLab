import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Brushed drone URI (adjust if needed, based on README example)
URI_BRUSHED = "radio://0/80/2M/E7E7E7E7E7"

RATE_HZ = 50
DT = 1.0 / RATE_HZ

SPIN_THRUST = 10000
SPIN_TIME_S = 1.5

THRUST_START = 40000
THRUST_STEP = 2000
THRUST_MAX = 50000
HOLD_TIME_S = 0.8


def send_rpyt_for(scf, roll, pitch, yawrate, thrust, seconds):
    n = int(seconds * RATE_HZ)
    for _ in range(n):
        scf.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(DT)


def slow_spinup(scf):
    print("Slow motor spin-up...")
    send_rpyt_for(scf, 0.0, 0.0, 0.0, 0, 0.2)
    steps = 5
    for i in range(1, steps + 1):
        thrust = int(SPIN_THRUST * i / steps)
        send_rpyt_for(scf, 0.0, 0.0, 0.0, thrust, SPIN_TIME_S / steps)


def thrust_ramp(scf):
    print("Thrust ramp up...")
    thrust = THRUST_START
    while thrust <= THRUST_MAX:
        print(f"Thrust = {thrust}")
        send_rpyt_for(scf, 0.0, 0.0, 0.0, thrust, HOLD_TIME_S)
        thrust += THRUST_STEP

    print("Thrust ramp down...")
    thrust -= THRUST_STEP
    while thrust >= THRUST_START:
        print(f"Thrust = {thrust}")
        send_rpyt_for(scf, 0.0, 0.0, 0.0, thrust, HOLD_TIME_S)
        thrust -= THRUST_STEP


def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI_BRUSHED, cf=Crazyflie(rw_cache="./cache")) as scf:
        try:
            slow_spinup(scf)
            thrust_ramp(scf)
        except KeyboardInterrupt:
            print("\n[CTRL+C] Emergency stop ...")
        finally:
            send_rpyt_for(scf, 0.0, 0.0, 0.0, 0, 0.4)
            scf.cf.commander.send_stop_setpoint()
            time.sleep(0.1)


if __name__ == "__main__":
    main()
