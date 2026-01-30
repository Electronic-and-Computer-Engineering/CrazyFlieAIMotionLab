import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.platformservice import PlatformService
 
import cfMRB
 
URI = "radio://0/33/2M/E7E7E7E7AA"  # Brushless URI
duration = 10
 
def arm_drone():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        """
        Sends the arming request to the Crazyflie.
        """
        ps = PlatformService(scf.cf)
        # Send the arm command (True to arm, False to disarm)
        ps.send_arming_request(True)
        time.sleep(duration)
        ps.send_arming_request(False)
 
if __name__ == '__main__':
    arm_drone()