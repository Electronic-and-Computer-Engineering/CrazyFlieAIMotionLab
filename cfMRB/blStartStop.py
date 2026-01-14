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