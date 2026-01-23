import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.platformservice import PlatformService
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = "radio://0/33/2M/E7E7E7E7AA"  # Brushless URI


def detect_spin_toss(scf):
    """
    Detects when drone is tossed while spinning by monitoring:
    - Vertical velocity (vz) for upward motion
    - Gyro data for rotation
    Returns True when spin-toss is detected.
    """
    # Setup logging for vertical velocity and gyro
    logconf = LogConfig(name="spin_toss_detect", period_in_ms=50)  # 20 Hz
    logconf.add_variable("stateEstimate.vz", "float")
    logconf.add_variable("gyro.x", "float")
    logconf.add_variable("gyro.y", "float")
    logconf.add_variable("gyro.z", "float")
    
    print("Ready to detect spin-toss...")
    print("Toss the drone upward while spinning it now!")
    
    detection_timeout = time.time() + 30.0
    
    with SyncLogger(scf, logconf) as logger:
        for entry in logger:
            ts, data, _ = entry
            vz = data["stateEstimate.vz"]
            gx = data["gyro.x"]
            gy = data["gyro.y"]
            gz = data["gyro.z"]
            
            # Calculate total angular velocity magnitude
            angular_velocity = (gx**2 + gy**2 + gz**2) ** 0.5
            
            # Detect spin-toss: upward velocity AND significant rotation
            if vz > 1 and angular_velocity > 200.0:  # 0.5 m/s upward, 200 deg/s rotation
                print(f"[SPIN-TOSS DETECTED!]")
                print(f"  Vertical velocity: {vz:.2f} m/s")
                print(f"  Angular velocity: {angular_velocity:.1f} deg/s")
                print(f"  Gyro (x,y,z): ({gx:.1f}, {gy:.1f}, {gz:.1f}) deg/s")
                return True
            
            if time.time() > detection_timeout:
                print("Spin-toss detection timeout.")
                return False


def main():
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        ps = PlatformService(scf.cf)
        ps.send_arming_request(True)
        time.sleep(0.2)
        scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 5000)
        
        try:
            # Detect the spin-toss
            if not detect_spin_toss(scf):
                print("No spin-toss detected. Landing...")
                scf.cf.commander.send_stop_setpoint()
                time.sleep(2)
                return
            
            # Give it a moment to stabilize after toss detection
            time.sleep(0.5)
            
            print("Starting hover at 0.5m...")
            hover_start = time.time()
            while time.time() - hover_start < 15.0:  # Hover for 15 seconds
                scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.5)
                time.sleep(0.05)  # 20 Hz
            
            # Land
            print("Landing...")
            scf.cf.commander.send_stop_setpoint()
            time.sleep(2)
            
        except KeyboardInterrupt:
            print("\n[CTRL+C] Emergency stop!")
            scf.cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    main()

