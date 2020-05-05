##################################
#     libraries and modules      #
##################################

# Skoltech RPLiDAR Python module:
#   https://github.com/SkoltechRobotics/rplidar
from rplidar import RPLidar

import odrive               # For motor control
from odrive.utils import dump_errors
from odrive.enums import *
import time                 # For ODrive module & general timing
import math                 # For useful constants & computations
import numpy                # For useful memes
import os                   # For easy file IO
import re                   # For file processing (regex module)

##################################
# constants and global variables #
##################################

# RPM (physical) * 2*pi/60 * polepairs = vel rad/s electrical
# motor wheel diameter = 0.165 m
# 50 rpm (physical) = 25.905 m/min = 0.432 m/s
RPM = 50                                            # physical rpm
MOTOR_VELOCITY_CONST = RPM * 2*math.pi/60 * 15      # electrical rpm
WHEEL_VELOCITY_CONST = RPM * 3.14159 * 0.165 / 60   # m/sec
SINGLE_DIRECTION_TIME_CONST = 2.315 #seconds
RECOVER_DATA_MSG = 'scan complete! please remove the USB drive'
POST_SCAN_IDLE_TIME = 0
OUTPUT_FILENAME = 'scan_data.txt'
BOUNDING_CLOUD_FILENAME = 'boundingcloud.txt'

_xyzPointCloud = []
_lidars = []
_numlidars = 3

_angularTxList = []
_lidarCenterList = [] 

##################################
#  classes                       #
##################################

class point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
    def __str__(self):
        return (str(self.x) + ' ' + str(self.y) + ' ' + str(self.z))
    def setVal(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

##################################
#  functions                     #
##################################


def polarToXYZ(distance, angle, angularTx, lidarCenter, dtime):
    # return a 3D vector given polar coordinates, an 
    #   optional rotational transformation, and XYZ
    #   coordinates representing the polar coordinate's
    #   origin in 3-space. Coordinate value in the direction
    #   of the crop row (in the dimension which the robot
    #   will move) shall be determined by time offset 
    angleRad = float(angle) * math.pi / 180.0 + angularTx
    x = float(distance)/1000 * math.cos(angleRad) + lidarCenter[0]
    y = float(distance)/1000 * math.sin(angleRad) + lidarCenter[1]
    # dtime is the time since the robot has begun to scan the crop
    z = dtime * MOTOR_VELOCITY_CONST
    # before returning the point, define the bounds where points
    #   should be ignored based on the physical design. see 
    #   "processor.py" for an idea of how to do this
    return [x, y, z]

def writeXYZFile(xyzPointArray):
    # write xyz point array to a file. Filename and 
    #   output directory to be specified by constants.
    f = open(OUTPUT_FILENAME, 'w')
    for chunk in data:
        for scan in chunk:
            f.write(str(scan[1]) + ',' + str(scan[2]) + '\n')
    f.close()

def odriveMotorConfiguration():
    # configure ODrive for hoverboard motor / hall effect
    #   feedback. Returns a boolean for config success/failure.
    #   Reference: https://docs.odriverobotics.com/hoverboard.
    #   Raises ValueError for odrive motor config failure
    odrv0 = odrive.find_any()
    odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
    odrv0.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
    odrv0.axis0.motor.config.current_control_bandwidth = 100
    odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis0.encoder.config.cpr = 90
    odrv0.axis0.encoder.config.bandwidth = 100
    odrv0.axis0.controller.config.pos_gain = 1
    odrv0.axis0.controller.config.vel_gain = 0.02
    odrv0.axis0.controller.config.vel_integrator_gain = 0.1
    odrv0.axis0.controller.config.vel_limit = 1000
    odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    odrv0.save_configuration()
    odrv0.reboot()
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    try:
        if error != 0 : raise Exception('unable to configure ODrive')
        if phase_inductance != 0.00033594953129068017 : raise Exception('unable to configure ODrive')
        if phase_resistance != 0.1793474406003952 : raise Exception('unable to configure ODrive')
    except Exception:
        return False

    odrv0.axis0.motor.config.pre_calibrated = True
    odrv0.save_configuration()
    odrv0.reboot()
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(5)
    return True

def rplidarConfiguration():
    # connect to RPLiDAR modules. Will loop through a collection
    #   of port name strings (i.e. ['/dev/ttyUSB0']) defined as 
    #   a constant and verify that RPLiDAR modules are connected.
    #   Also starts LiDAR motors and checks health status codes. 
    #   Returns a boolean for config success/failure. 
    print('powering up LiDARs...')
    try: 
        for i in range(0,numlidars):
            lidars.append(RPLidar('/dev/ttyUSB' + str(i)))
            print(lidars[i].get_info())
            print(lidars[i].get_health())
            lidars[i].set_pwm(400)
            time.sleep(3)
        time.sleep(.1)
    except ValueError:
        return False 
    return True 

def createBoundingPointCloud(pointDensity):
    # create an array of XYZ points to set bounds for point cloud.
    # formula for number of points defining hollow cube: 
    #   p = n^3 - (n-2)^3
    #   p -> total number of points
    #   n -> number of points on an edge
    #   i.e. a 2x2 cube would have 2^3 - 0^3 = 8 total points
    #   below defines a 10x10 cube
    try:
        outputFile = open(BOUNDING_CLOUD_FILENAME, "w") 
        points = [point()] * 488
        ppos = 0
        xmin, xmax = 0, 10
        ymin, ymax = -5, 5
        zmin, zmax = 0, 10
        scalemax = 10

        for z in range(zmin,zmax):
            for y in range(ymin,ymax):
                for x in range(xmin,xmax):
                    if y == ymin or y == ymax-1 or z == zmin or z == zmax-1 or x == xmin or x == xmax-1:
                        points[ppos].setVal(x/scalemax, y/scalemax, z/scalemax)
                        outputFile.write(str(points[ppos]) + '\n')
                        ppos += 1

        outputFile.close()
    except Exception:
        return False
    return True

def waitForKeyPress():
    # to reduce typing
    input("Press Enter to continue")

def deactivateLidars():
    # for properly stopping lidar modules.
    #   stops motor and data collection, 
    #   sleep times can be adjusted
    for lidar in _lidars:
        lidar.stop_motor()
        time.sleep(0.2)
        lidar.stop()
        time.sleep(0.2)
        lidar.disconnect()
        time.sleep(2.5)

##################################
#         main script            #
##################################

# perform RPLiDAR and ODRIVE setup
try:
    odriveMotorConfiguration()
    rplidarConfiguration()
except ValueError as err:
    # if there are any configuration errors with the LiDAR units or
    #   ODrive controller, then the program must exit and be debugged. 
    print(err.args)
    print('the system will now exit the scanning operation')
    waitForKeyPress()
    exit()

# use ODrive Tool to set motor velocities to motor velocity constant
odrv0.axis0.controller.vel_setpoint = MOTOR_VELOCITY_CONST
odrv0.axis1.controller.vel_setpoint = MOTOR_VELOCITY_CONST

# begin timed loop for collecting data in one direction and collect
#   RPLiDAR data. 
timeEnd = time.time() + SINGLE_DIRECTION_TIME_CONST
_startTime = time.time()
while time.time() < timeEnd:
    # loop through LiDAR units and record scan data
    for index, lidar in enumerate(_lidars):
        # send lidar SCAN_BYTE command
        lidar._send_cmd(rplidar.SCAN_BYTE)
        dsize, is_single, dtype = lidar._read_descriptor()
        rawScanData = lidar._read_response(dsize)
        
        # get angle and distance data from raw scan data.
        #   _process_scan returns [new_scan, quality, angle, distance]
        scan = rplidar._process_scan(rawScanData)
        
        # transform polar data to xyz data and add point to xyz point cloud
        xyzPoint = polarToXYZ(scan[3], scan[2], _angularTxList[index], _lidarCenterList[index], time.time() - _startTime)
        _xyzPointCloud.append(xyzPoint)

# stop lidars and motors, append bounding cloud to point cloud
#   and wait for some time 
odrv0.axis0.controller.vel_setpoint = 0
odrv0.axis1.controller.vel_setpoint = 0
createBoundingPointCloud()
deactivateLidars()

# return from crop row after scanning
odrv0.axis0.controller.vel_setpoint = -1 * MOTOR_VELOCITY_CONST
odrv0.axis1.controller.vel_setpoint = -1 * MOTOR_VELOCITY_CONST

# do nothing. alternatively, lidar motors can be stopped later
#   and more data may be collected. must change above from disabling
#   lidars to simply stopping the motors, or re-start them here.
time.sleep(SINGLE_DIRECTION_TIME_CONST)

# stop motors then save output data to a file
odrv0.axis0.controller.vel_setpoint = 0
odrv0.axis1.controller.vel_setpoint = 0
writeXYZFile(_xyzPointCloud)

# report scan finishing and prompt user to remove data, then exit
print(RECOVER_DATA_MSG)
waitForKeyPress()
exit()