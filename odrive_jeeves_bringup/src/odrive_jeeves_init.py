import odrive
import time
from odrive.enums import *

odrv0 = odrive.find_any()
# AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    print("Axis0 Calibrating..")
    time.sleep(1)
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Axis0 calibration done")
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while odrv0.axis1.current_state != AXIS_STATE_IDLE:
    print("Axis1 Calibrating..")
    time.sleep(1)
print("Axis1 calibration done")
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("ODrive setup complete , Bus voltage is " + str(odrv0.vbus_voltage) + "V")
