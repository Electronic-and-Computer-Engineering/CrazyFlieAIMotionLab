
import logging
import time
import sys
import threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.platformservice import PlatformService


# URI to the Crazyflie to connect to
URI = "radio://0/33/2M/E7E7E7E7AA"


class PushDemo:
    def __init__(self, link_uri):
        self._cf = Crazyflie(rw_cache="./cache")

        # Initialize  drivers
        cflib.crtp.init_drivers()

        self._scf = SyncCrazyflie(link_uri, cf=self._cf)

        self._is_connected = False
        self._running = True 
        
        # Current sensor readings
        self._front = 0
        self._back = 0
        self._up = 0
        self._left = 0
        self._right = 0
        self._zrange = 0
        self._target_height = 0.5

        # Control thread
        self._control_thread = threading.Thread(target=self._control_loop)

    def connect(self):
        # Connect to the Crazyflie
        self._scf.open_link()
        self._is_connected = True
        print(f"Connected to {URI}")

        # Add log config
        self._log_conf = LogConfig(name='Multiranger', period_in_ms=100)
        self._log_conf.add_variable('range.front', 'uint16_t')
        self._log_conf.add_variable('range.back', 'uint16_t')
        self._log_conf.add_variable('range.up', 'uint16_t')
        self._log_conf.add_variable('range.left', 'uint16_t')
        self._log_conf.add_variable('range.right', 'uint16_t')
        self._log_conf.add_variable('range.zrange', 'uint16_t') # Down sensor

        self._cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self._data_callback)
        self._log_conf.start()

    def _data_callback(self, timestamp, data, logconf):
        self._front = data['range.front']
        self._back = data['range.back']
        self._up = data['range.up']
        self._left = data['range.left']
        self._right = data['range.right']
        self._zrange = data['range.zrange']

    def close(self):
        self._running = False
        if self._control_thread.is_alive():
            self._control_thread.join()
            
        if self._is_connected:
            self._log_conf.stop()
            self._scf.close_link()
            self._is_connected = False
            print("Disconnected")

    def start(self):
        self.connect()
        self._control_thread.start()
        # Blocking call to show plot - must be in main thread
        self._start_plot()

    def _control_loop(self):
        # Configure Estimator for Flowdeck 
        print("Configuring estimator (Kalman)...")
        self._cf.param.set_value('stabilizer.estimator', '2') # 2=Kalman
        self._cf.param.set_value('stabilizer.controller', '1') # 1=PID
        
        # Reset estimator to ensure good initial state
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        print("Estimator reset complete.")
       

        # Arm motors
        print("Arming brushless motors...")
        ps = PlatformService(self._scf.cf)
        ps.send_arming_request(True)
        time.sleep(1)
        
        # Release manual override to let High Level Commander take over
        # self._cf.param.set_value('motorPowerSet.enable', '0')

        print("Taking off...")
        commander = self._cf.high_level_commander
        
        # Take off to 0.5m (Standard Flowdeck Logic)
        commander.takeoff(0.5, 2.0)
        time.sleep(3)

        print("Push mode active! Keep hands away to hover.")
        
        # We start with the target height reached by takeoff
        self._target_height = 0.5
        
        try:
            while self._running:
                # Base velocity
                vx = 0.0
                vy = 0.0
                
                # Thresholds in mm
                MIN_DIST = 300 # 30cm
                GAIN = 0.002   # Velocity factor

                # If distance < MIN_DIST, fly away proportional to penetration
                
                # Front/Back (X-axis)
                if self._front > 0 and self._front < MIN_DIST:
                    vx -= (MIN_DIST - self._front) * GAIN
                if self._back > 0 and self._back < MIN_DIST:
                    vx += (MIN_DIST - self._back) * GAIN

                # Left/Right (Y-axis)
                if self._left > 0 and self._left < MIN_DIST:
                    vy -= (MIN_DIST - self._left) * GAIN
                if self._right > 0 and self._right < MIN_DIST:
                    vy += (MIN_DIST - self._right) * GAIN

                # Up/Down (Z-axis) - Modify Target Height instead of Velocity
                if self._up > 0 and self._up < MIN_DIST:
                    # Sensor above detects hand -> go down
                    self._target_height -= (MIN_DIST - self._up) * GAIN * 0.1 # scaled for position
                
                if self._zrange > 0 and self._zrange < 300: # Increase floor limit slightly
                   # Too close to floor -> go up
                   self._target_height += (300 - self._zrange) * GAIN * 0.1

                # sqeeze logic (If hands come from both sides the drone should go up to dodge the attack)
                is_squeezed_lr = (self._left > 0 and self._left < MIN_DIST) and (self._right > 0 and self._right < MIN_DIST)
                is_squeezed_fb = (self._front > 0 and self._front < MIN_DIST) and (self._back > 0 and self._back < MIN_DIST)

                if is_squeezed_lr or is_squeezed_fb:
                    self._target_height += 0.05 
                

                # Clamp target height to safe range
                self._target_height = max(0.2, min(self._target_height, 1.5))

                # Send hover setpoint (Hardware uses Z-ranger to hold this height)
                self._cf.commander.send_hover_setpoint(vx, vy, 0, self._target_height)
                
                time.sleep(0.1)

        except Exception as e:
            print(f"Error in control loop: {e}")
        
        finally:
            print("Landing...")
            commander.land(0.0, 2.0)
            time.sleep(2)
            commander.stop()
            
            # Stop motors
            print("Stopping motors...")
            ps.arming_request(False)
            

    def _start_plot(self):
        # Set up plot
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        
        # plot 4 bars: Front, Back, Left, Right (Up/Down ignored for simplicity or added later)
        sensors = ['Front', 'Back', 'Left', 'Right', 'Up', 'Down']
        x_pos = [i for i in range(len(sensors))]
        
        bar_colors = ['red', 'green', 'blue', 'orange', 'purple', 'brown']

        def animate(i):
            ax.clear()
            values = [self._front, self._back, self._left, self._right, self._up, self._zrange]
            
            bars = ax.bar(x_pos, values, color=bar_colors)
            ax.set_xticks(x_pos)
            ax.set_xticklabels(sensors)
            ax.set_ylim(0, 2000) # 2 meters max usually
            ax.set_ylabel('Distance (mm)')
            ax.set_title('Multiranger Live Feed')
            
            # Add threshold line
            ax.axhline(y=300, color='r', linestyle='--', label='Push Threshold')
            ax.legend()



            # Check if running
            if not self._running:
                plt.close(fig)

        ani = animation.FuncAnimation(fig, animate, interval=100)
        print("Starting plot window...")
        plt.show() # Blocks until window closed
        
        print("Plot window closed.")
        self._running = False
        self.close()

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    
    # Run the demo
    demo = PushDemo(URI)
    demo.start()
