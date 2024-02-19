import DragonFlyFunctions as dff
import Waypoint_Select_Optimization.WaypointSelectFunctions as waypointSelect
import IceDragonFunctions as IDF
import RPi.GPIO as GPIO #for raspberryPi
import numpy as np
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time
#import csv

# --- Initializing Optimal Waypoint (pre-launch) ---
# Use closest target from example (already given)

# --- Initializing Pixhawk Connection ---
print("Connecting to Vehicle...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)
print("Connected.")
# plug PI into telem2 port on pixhawk


# --- Initialize Servo Parameters ---
vehicle.parameters['SERVO1_FUNCTION'] = 4 # left aileron
vehicle.parameters['SERVO1_REVERSED'] = 0
print("SERVO1 Set - Left Aileron")

vehicle.parameters['SERVO2_FUNCTION'] = 4 # right aileron
vehicle.parameters['SERVO2_REVERSED'] = 1
print("SERVO2 Set - Right Aileron")

vehicle.parameters['SERVO3_FUNCTION'] = 19 # elevator
print("SERVO3 Set - Elevator")

vehicle.parameters['SERVO4_FUNCTION'] = 33 # motor 1
print("SERVO4 Set - Motor 1")

vehicle.parameters['SERVO5_FUNCTION'] = 34 # motor 2
print("SERVO5 Set - Motor 1")

#dff.initializeServos(vehicle)


# disable all control surfaces for ascent
#dff.ascentSet(vehicle)

#Upload new mission
cmds = vehicle.commands
cmds.clear()

# Set mode AUTO
vehicle.mode = VehicleMode("AUTO")


## Setting Waypoints and Geographic Fence
# LAUNCH
lat1 = 40.4445537
lon1 = -87.0325775
alt1 = 300

# TARGET
lat2 = 40.4440833
lon2 = -87.0296807
alt2 = 0

lat_wp, lon_wp, alt_wp, lat2, lon2, alt2, alt_above = IDF.set_waypoints(lat1, lon1, alt1, lat2, lon2, alt2)
print("Waypoints Generated")

# START 
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat1, lon1, alt1))
print("Launch Waypoint Added")

# WAYPOINT FUNCTION
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_wp[0], lon_wp[0], alt_wp[0]))
print("Waypoint 1 added")

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_wp[1], lon_wp[1], alt_wp[1]))
print("Waypoint 2 added")

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_wp[2], lon_wp[2], alt_wp[2]))
print("Waypoint 3 added")

# LOITER
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 0, 0, 0, 10, 0, 0, lat2, lon2, alt_above))
print("Loiter Waypoint added")

# LAND
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 25, 0, 0, lat2, lon2, alt2))
print("Landing Waypoint added")

# Sets Geographic Fence (6-sided polygon)
lat = 40.4437230
long = -87.0312885
alt = 300 # setting maximum altitude of geofence?
#cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0, 0, 6, 100, 0, 0, lat, long, alt))
#cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 0, 0, 6, 100, 0, 0, lat, long, alt))
#cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, 0, 0, 0, 0, 0, 0, lat, long, alt))
#cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT, 0, 0, 0, 0, 0, 0, lat, long, alt))

cmds.upload()

