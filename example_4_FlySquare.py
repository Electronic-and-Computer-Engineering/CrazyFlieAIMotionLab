import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.platformservice import PlatformService

import cfMRB

URI = "radio://0/33/2M/E7E7E7E7AA"  # Brushless URI
duration = 10

RADIUS = 1.0          # Radius in Metern
PERIOD = 1.5         # Zeit für eine volle Umdrehung in Sekunden
HEIGHT = 1.0         # Zielhöhe in Metern
speed_vx = 0.5
speed_vy = 0.5

def generate_square():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:

        t0 = time.time()
        # fly forward
        while time.time() - t0 < PERIOD:
            # Befehl senden: vx, vy, yawrate=0, zDistance=HEIGHT
            scf.cf.commander.send_hover_setpoint(speed_vx, 0, 0.0, HEIGHT)

        t0 = time.time()
        # fly forward
        while time.time() - t0 < PERIOD:
            # Befehl senden: vx, vy, yawrate=0, zDistance=HEIGHT
            scf.cf.commander.send_hover_setpoint(0, speed_vy, 0.0, HEIGHT)

        t0 = time.time()
        # fly forward
        while time.time() - t0 < PERIOD:
            # Befehl senden: vx, vy, yawrate=0, zDistance=HEIGHT
            scf.cf.commander.send_hover_setpoint(-speed_vx, 0, 0.0, HEIGHT)

        t0 = time.time()
        # fly forward
        while time.time() - t0 < PERIOD:
            # Befehl senden: vx, vy, yawrate=0, zDistance=HEIGHT
            scf.cf.commander.send_hover_setpoint(0, -speed_vy, 0.0, HEIGHT)


def arm_drone():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        """
        Sends the arming request to the Crazyflie.
        """
        ps = PlatformService(scf.cf)
        
        # Send the arm command (True to arm, False to disarm)
        ps.send_arming_request(True)
        time.sleep(0.2)

def disarm_drone():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        """
        Sends the arming request to the Crazyflie.
        """
        ps = PlatformService(scf.cf)
        ps.send_arming_request(False)
        time.sleep(0.2)

def land():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:

        t0 = time.time()

        while time.time() - t0 < 2.0:
            scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.5)
        
        scf.cf.commander.send_stop_setpoint()


def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        
        t0 = time.time()

        while time.time() - t0 < 10.0:
            scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, HEIGHT)
            time.sleep(0.05)  # 20 Hz


if __name__ == '__main__':
    arm_drone()
    main()
    generate_square()
    land()
    disarm_drone()
