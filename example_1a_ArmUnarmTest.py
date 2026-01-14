import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import cfMRB

URI = "radio://0/33/2M/E7E7E7E7AA"  # Brushless URI
duration = 10

def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Connected to drone.")
        cfMRB.start_motors(scf)
        time.sleep(duration)
        cfMRB.stop_motors(scf)
    print("Disconnected.")

if __name__ == '__main__':
    main()
