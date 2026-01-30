import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.platformservice import PlatformService
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = "radio://0/33/2M/E7E7E7E7AA"  # Brushless URI


def detect_toss(scf):
    """
    Detects when drone is tossed by monitoring vertical velocity.
    Spins motors at low thrust to prepare for flight.
    Returns True when toss is detected.
    """
    # Setup logging for vertical velocity
    logconf = LogConfig(name="toss_detect", period_in_ms=50)  # 20 Hz
    logconf.add_variable("stateEstimate.vz", "float")
    
    print("Ready to detect toss... Throw the drone upward now!")
    
    detection_timeout = time.time() + 30.0
    entry_count = 0
    low_thrust = 5000  # Low thrust to spin motors without lifting off
    
    try:
        with SyncLogger(scf, logconf) as logger:
            for entry in logger:
                ts, data, _ = entry
                entry_count += 1
                
                if entry_count == 1:
                    print(f"[Logging started] First data received: {data}")
                    print(f"[Motors spinning at low thrust: {low_thrust}]")
                
                vz = data["stateEstimate.vz"]
                print(f"vz: {vz:.3f} m/s", end="\r")
                
                # Spin motors at low thrust (roll, pitch, yaw_rate, thrust)
                scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, low_thrust)
                
                # Detect toss: upward velocity (positive vz)
                if vz > 3:  # 0.5 m/s threshold for upward motion
                    print(f"\n[TOSS DETECTED!] Vertical velocity: {vz:.2f} m/s")
                    print(f"[Switching to hover mode]")
                    return True
                
                if time.time() > detection_timeout:
                    print("\nToss detection timeout.")
                    return False
                
                time.sleep(0.05)  # 20 Hz rate
    
    except Exception as e:
        print(f"[ERROR in detect_toss] {type(e).__name__}: {e}")
        return False


def main():
    cflib.crtp.init_drivers()
    
    print("Connecting to drone...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("[Connected]")
        
        ps = PlatformService(scf.cf)
        ps.send_arming_request(True)
        print("[Arming request sent]")
        time.sleep(0.2)
        
        try:
            # Wait for IMU/estimator to stabilize - KEEP DRONE COMPLETELY STILL
            print("\n[IMPORTANT] Keep the drone COMPLETELY STILL for calibration!")
            print("Waiting 1 seconds for IMU/estimator to stabilize...")
            time.sleep(3)
            print("Calibration complete. Ready for toss detection.         ")
            
            # Detect the toss
            print("[Starting toss detection...]")
            if not detect_toss(scf):
                print("[No toss detected]")
                return
            
            print("Starting hover at 1.0m...")
            hover_start = time.time()
            while time.time() - hover_start < 10.0:  # Hover for 15 seconds
                scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 2.0)
                time.sleep(0.05)  # 20 Hz
            
            # Soft landing - gradually reduce height smoothly
            print("Landing softly...")
            land_start = time.time()
            landing_duration = 3.0  # 3 seconds for smooth landing
            hover_height = 2.0
            
            while time.time() - land_start < landing_duration:
                progress = (time.time() - land_start) / landing_duration
                # Smooth descent: quadratic curve for gentler landing
                current_height = hover_height * (1 - progress * progress)
                scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, max(current_height, 0.0))
                time.sleep(0.05)
            
            scf.cf.commander.send_stop_setpoint()
            time.sleep(1)
            
        except KeyboardInterrupt:
            print("\n[CTRL+C] Emergency stop!")
            scf.cf.commander.send_stop_setpoint()
        except Exception as e:
            print(f"\n[ERROR] {type(e).__name__}: {e}")
            scf.cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    main()
