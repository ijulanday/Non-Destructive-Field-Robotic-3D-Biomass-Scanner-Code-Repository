import odrive
from odrive.utils import dump_errors
from odrive.enums import *
import time
import math

odrv0 = odrive.find_any()

# configure pole pairs for standard hoverboard hub motor 
odrv0.axis0.motor.config.pole_pairs = 15

# set higher calibration voltage and reduce current controller bandwidth
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis0.motor.config.current_control_bandwidth = 100

# set encoder to hall mode
odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90

# configure ODrive velocity tracking for hall feedback
odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.1
odrv0.axis0.controller.config.vel_limit = 1000
odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

# make sure that some of the above settings that require a reboot are applied first.
odrv0.save_configuration()
odrv0.reboot()

# Make sure the motor is free to move, then activate the motor calibration.
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION

'''TODO: check these values are set correctly`
  error = 0x0000 (int)
  phase_inductance = 0.00033594953129068017 (float)
  phase_resistance = 0.1793474406003952 (float)
'''

# let ya boi breathe
odrv0.axis0.motor.config.pre_calibrated = True

odrv0.save_configuration()
odrv0.reboot()

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
time.sleep(5)

# Set velocity to some value
#   RPM * 2*pi/60 * polepairs = vel rad/s electrical
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.vel_setpoint = 50 * 2*math.pi/60 * 15

time.sleep(5)

# Return to idle
odrv0.axis0.controller.vel_setpoint = 0
odrv0.axis0.requested_state = AXIS_STATE_IDLE


