[HIT THE CHEAT SHEET](cfCmdCheatSheets.MD)

# Crazyflie Python Lab – Brushed & Brushless

This is a structured hands-on lab for **controlling Crazyflie drones** using **Python** and **Crazyradio v2**.  
Both Brushed and Brushless versions of the Crazyflie 2.1 are supported.

---

## Table of Contents

- [Crazyflie Python Lab – Brushed & Brushless](#crazyflie-python-lab--brushed--brushless)
  - [1. Setup & Installation](#1-setup--installation)
    - [1.1 Required Software](#11-required-software)
    - [1.2 Install CFClient](#12-install-cfclient)
    - [1.3 Python Environment](#13-python-environment)
  - [2. Flashing & URI Configuration (Using CFClient)](#2-flashing--uri-configuration-using-cfclient)
    - [2.1 Flash Firmware](#21-flash-firmware)
    - [2.2 Set Radio URI](#22-set-radio-uri)
  - [3. Python Control Basics](#3-python-control-basics)
  - [4. Python Examples (Standalone Scripts)](#4-python-examples-standalone-scripts)
    - [4.1 Connect Only](#41-connect-only)
    - [4.2 (Might needed for the brushless Drone) Arm and Start Motors](#42-might-needed-for-the-brushless-drone-arm-and-start-motors)
    - [4.2 Take Off and Land (Ramp using THRUST)](#42-take-off-and-land-ramp-using-thrust)
    - [4.3 Hover and Move Forward](#43-hover-and-move-forward)
    - [4.4 Rotate Yaw in Air](#44-rotate-yaw-in-air)
    - [4.5 (Additionally) LOG DATA from the drone `estimator`](#45-additionally-log-data-from-the-drone-estimator)
  - [Official references (Bitcraze)](#official-references-bitcraze)
  - [Official software downloads / installation](#official-software-downloads--installation)
  - [Source repositories (used in practice)](#source-repositories-used-in-practice)
  - [Examples (good “known working” patterns)](#examples-good-known-working-patterns)

## 1. Setup & Installation

### 1.1 Required Software

Install the following tools:

- Python 3.10 or newer  
- Visual Studio Code  
- Crazyflie Client (`cfclient`)  
- Crazyradio PA v2 USB dongle

### 1.2 Install CFClient

Get the installer from:

> https://www.bitcraze.io/downloads/

Install and verify that CFClient opens correctly.

### 1.3 Python Environment

In VS Code or terminal:

```bash
mkdir crazyflie-lab
cd crazyflie-lab
# Optitonal: Creat Virtual Environment
python -m venv venv
source venv/bin/activate   
# Windows: venv\Scripts\activate
pip install --upgrade pip
pip install cflib
```

---

## 2. Flashing & URI Configuration (Using CFClient)

### 2.1 Flash Firmware

1. Connect drone via USB  
2. Open CFClient → "Bootloader"  
3. Select correct firmware:
   - **Brushed**: Crazyflie 2.1
   - **Brushless**: Crazyflie 2.1 Brushless  
4. Flash and reboot

### 2.2 Set Radio URI

1. CFClient → Connect via USB  
2. Go to "Configure 2.x"  
3. Set:

- Channel: `80`  
- Datarate: `2M`  
- Address:

```text
Brushed:   E7E7E7E7E7
Brushless: E7E7E7E7AA
```

4. Write config and reboot  
5. Disconnect USB and use radio

---

## 3. Python Control Basics

Use the following constants in all examples:

```python
URI_BRUSHED   = "radio://0/80/2M/E7E7E7E7AA"
URI_BRUSHLESS = "radio://0/33/2M/E7E7E7E7E7"
``` 
Never run CFClient and Python scripts simultaneously.

This is a link to our command cheat-sheet:
[Our MRB CF Cheat Sheet](cfCmdCheatSheets.md)

---

## 4. Python Examples (Standalone Scripts)

### 4.1 Connect Only
Here, we just test if your python setup is ready and allows to connect to the drone.

```python
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
```
---
### 4.2 (Might needed for the brushless Drone) Arm and Start Motors

For the brushless crazyDrone, bitcraze disarmed the controlling of the motors for security reason. With this small "hack" we circumvent that for further testing:

```python
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
````
Where the additional functions, that actually start and stop the propellers are located in the `cfMRB` folder. 
We have the possibility to move user-created lib functions to a separated folder. 

```python
import time

def start_motors(scf, pwm):
    """
    Start the brushless motors at a given PWM value.

    :param scf: SyncCrazyflie instance
    :param pwm: PWM value between 0 and 65535
    """
    print(f"Starting motors with PWM = {pwm}")
    scf.cf.param.set_value("motorPowerSet.m1", str(pwm))
    scf.cf.param.set_value("motorPowerSet.m2", str(pwm))
    scf.cf.param.set_value("motorPowerSet.m3", str(pwm))
    scf.cf.param.set_value("motorPowerSet.m4", str(pwm))
    time.sleep(0.5)
    scf.cf.param.set_value("motorPowerSet.enable", "1")

def stop_motors(scf):
    """
    Stop all motors safely.
    
    :param scf: SyncCrazyflie instance
    """
    print("Stopping motors...")
    scf.cf.param.set_value("motorPowerSet.m1", "0")
    scf.cf.param.set_value("motorPowerSet.m2", "0")
    scf.cf.param.set_value("motorPowerSet.m3", "0")
    scf.cf.param.set_value("motorPowerSet.m4", "0")
    time.sleep(0.5)
    scf.cf.param.set_value("motorPowerSet.enable", "0")
```

### 4.2 Take Off and Land (Ramp using THRUST)

```python
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import cfMRB

# Open-loop thrust ramp using low-level RPYT setpoints (send_setpoint).
# - BRUSHED: works open-loop (no sensors), but height is NOT stable.
# - BRUSHLESS: same ramp, but we add an "arm/disarm" step via parameters
#   (matching your working hover test behavior).

URI_BRUSHED   = "radio://0/80/2M/E7E7E7E7E7"  
URI_BRUSHLESS = "radio://0/33/2M/E7E7E7E7AA"  

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

```
* Find a way to let the drone hover at a certain height position (just using THRUST for the crazyflie without sensors)
    
    - For the brushless drone you could use `scf.cf.commander.send_hover_setpoint(0, 0, 0, 0.5)` to hover. Where the last input is the height you want to achieve. But therefore, sensors are needed

* You can also use the `aiMotion` Camera system in order to get the information of the height, measure the height of the drone objects and use the `THRUST` to stabilize the height.

Hovering however, will be our most powerfull move, if this works in a stable way. We are fine for every single upcoming task we want to implement. 
---

### 4.3 Hover and Move Forward

- [ ] Observe **Phase 1 (Hover)**: drone should try to reach/hold **1 m or more** for **~2 s**.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, THRUST_HOVER)`
- [ ] Observe **Phase 2 (Forward)**: drone should move forward for **~2 s** (open-loop tilt) while keeping roughly similar thrust.  
  - `scf.cf.commander.send_setpoint(0.0, PITCH_FWD_DEG, 0.0, THRUST_HOVER)`
- [ ] Observe **Phase 3 (Stop/Level)**: stop forward tilt for **~2 s**.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, THRUST_HOVER)`
- [ ] Observe **Phase 4 (Ramp Down)**: stop forward tilt for **~2 s**.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, THRUST_REDUCTION)`
- [ ] Finish: cut thrust and stop setpoints.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)`  
  - `scf.cf.commander.send_stop_setpoint()`

---

### 4.4 Rotate Yaw in Air

```python
### 4.4 Rotate Yaw in Air

- [ ] Observe **Phase 1 (Hover / Stabilize)**: drone should try to reach/hold **1 m or more** for **~2 s**.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, THRUST_HOVER)`
- [ ] Observe **Phase 2 (Rotate CW)**: drone should rotate for **~2 s** at constant thrust.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, YAWRATE_CW_DEG_S, THRUST_HOVER)`
- [ ] Observe **Phase 3 (Rotate CCW)**: drone should rotate in the opposite direction for **~2 s**.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, YAWRATE_CCW_DEG_S, THRUST_HOVER)`
- [ ] Observe **Phase 4 (Stop Rotation / Level)**: stop yaw rotation for **~2 s**.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, THRUST_HOVER)`
- [ ] Observe **Phase 5 (Ramp Down)**: reduce thrust for **~2 s** to descend.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, THRUST_REDUCTION)`
- [ ] Finish: cut thrust and stop setpoints.  
  - `scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)`  
  - `scf.cf.commander.send_stop_setpoint()`
```
---

### 4.5 (Additionally) LOG DATA from the drone `estimator`

```python
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI_BRUSHED   = "radio://0/80/2M/E7E7E7E7E7E7"
URI_BRUSHLESS = "radio://0/33/2M/E7E7E7E7E7AA"

IS_BRUSHLESS = True
URI = URI_BRUSHLESS if IS_BRUSHLESS else URI_BRUSHED

ESTIMATOR_VALUE  = "2"  # firmware-dependent (often Kalman)
CONTROLLER_VALUE = "1"  # firmware-dependent (often PID)

def safe_set_param(scf, name, value):
    try:
        scf.cf.param.set_value(name, value)
        return True
    except Exception:
        return False

def configure_estimator(scf):
    safe_set_param(scf, "stabilizer.estimator", ESTIMATOR_VALUE)
    safe_set_param(scf, "stabilizer.controller", CONTROLLER_VALUE)
    time.sleep(0.5)

def read_estimator_into_variables(scf, duration_s=5.0, period_ms=50):
    """
    Cyclically reads stateEstimate.* for duration_s and writes into variables.
    Prints the variables each cycle.
    Returns the last values after the loop.
    """
    logconf = LogConfig(name="state", period_in_ms=period_ms)
    logconf.add_variable("stateEstimate.x", "float")
    logconf.add_variable("stateEstimate.y", "float")
    logconf.add_variable("stateEstimate.z", "float")
    logconf.add_variable("stateEstimate.yaw", "float")

    est_x = 0.0
    est_y = 0.0
    est_z = 0.0
    est_yaw = 0.0

    t0 = time.time()
    with SyncLogger(scf, logconf) as logger:
        for _, data, _ in logger:
            est_x = float(data["stateEstimate.x"])
            est_y = float(data["stateEstimate.y"])
            est_z = float(data["stateEstimate.z"])
            est_yaw = float(data["stateEstimate.yaw"])

            print(f"x={est_x:.3f}  y={est_y:.3f}  z={est_z:.3f}  yaw={est_yaw:.1f}")

            if time.time() - t0 >= duration_s:
                break

    return est_x, est_y, est_z, est_yaw

if __name__ == "__main__":
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        configure_estimator(scf)
        x, y, z, yaw = read_estimator_into_variables(scf, duration_s=5.0, period_ms=50)

        print("LAST VALUES:")
        print(f"x={x:.3f}  y={y:.3f}  z={z:.3f}  yaw={yaw:.1f}")
```

## Official references (Bitcraze)

- cflib documentation (home): https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/  
- cflib API reference (auto-generated): https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/  
- cflib API: `cflib.crazyflie.commander` (send_setpoint / send_hover_setpoint etc.): https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/commander/ 
- cflib API: `motion_commander` class reference: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/positioning/motion_commander/
- User guides overview: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/
- Step-by-Step: Connect + Logging + Parameters: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/ 
- Step-by-Step: Motion Commander: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_motion_commander/
- “Python API explanation” (high-level overview): https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/ 

## Official software downloads / installation

- Downloads & Install (Bitcraze): https://www.bitcraze.io/support/downloads/
- cfclient installation guide: https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/
- cfclient user guide (GUI overview): https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/

## Source repositories (used in practice)

- cflib repo (Crazyflie python library): https://github.com/bitcraze/crazyflie-lib-python  
- cfclient repo (PC client & tools): https://github.com/bitcraze/crazyflie-clients-python 

## Examples (good “known working” patterns)

- MotionCommander demo script (repo example): https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/autonomy/motion_commander_demo.py

[HIT THE CHEAT SHEET](cfCmdCheatSheets.MD)