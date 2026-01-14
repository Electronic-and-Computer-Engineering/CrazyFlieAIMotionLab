from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
import time

# Brushed drone URI (adjust if needed)
URI = "radio://0/33/2M/E7E7E7E7AA"

def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("Connected")
        time.sleep(5)
    print("Disconnected")

if __name__ == '__main__':
    main()