#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math
from fibre.libfibre import ObjectLostError

print("Starting ODrive configuration.")

print("Finding ODrive connection...")
odrv0 = odrive.find_any()
#print("Found ODrive with Serial #: " + str(odrv0.serial_number))

print("Erasing current configuration...")
try:
	odrv0.erase_configuration()
except ObjectLostError:
	time.sleep(3.0)
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()


print("Configuring power supply...")
odrv0.config.dc_bus_overvoltage_trip_level = 30
odrv0.config.dc_max_positive_current = 5
odrv0.config.dc_max_negative_current = -1

print("Configuring motor...")
odrv0.axis0.config.motor.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.config.motor.pole_pairs = 7
odrv0.axis0.config.motor.torque_constant = 8.27 / 330
odrv0.axis0.config.motor.calibration_current = 3.88
odrv0.axis0.config.calibration_lockin.current = 3.88

print("Running motor calibration...")
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

print("Saving configuration...")
try:
	odrv0.save_configuration()
except ObjectLostError:
	time.sleep(3.0)
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

print("Configuring limits...")
odrv0.axis0.config.motor.current_soft_max = 7.76
odrv0.axis0.config.motor.current_hard_max = 26.4
odrv0.axis0.controller.config.vel_limit = 10

print("Configuring encoder...")
odrv0.axis0.config.load_encoder = ENCODER_ID_ONBOARD_ENCODER0
odrv0.axis0.config.commutation_encoder = ENCODER_ID_ONBOARD_ENCODER0

print("Saving configuration...")
try:
	odrv0.save_configuration()
except ObjectLostError:
	time.sleep(3.0)
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

print("Running encoder calibration...")
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)
	
print("Saving configuration...")
try:
	odrv0.save_configuration()
except ObjectLostError:
	time.sleep(3.0)
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

print("Verification: ")
print(" Current motor position = " + str(odrv0.axis0.pos_vel_mapper.pos_rel))
print(" Current motor velocity = " + str(odrv0.axis0.pos_vel_mapper.vel))

print("Testing position control:")
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
for p in range(6):
	goto_pos = p*0.20
	odrv0.axis0.controller.input_pos = goto_pos
	print(f" Setting position to {goto_pos}")
	time.sleep(0.5)
print("Testing velocity control:")
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
for v in range(6):
	goto_vel = v*2
	odrv0.axis0.controller.input_vel = goto_vel
	print(f" Setting velocity to {goto_vel}")
	time.sleep(0.5)

odrv0.axis0.requested_state = AXIS_STATE_IDLE
print("Configuration done.")	


