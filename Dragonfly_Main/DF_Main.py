import DragonFlyFunctions as dff
import Waypoint_Select_Optimization.WaypointSelectFunctions as waypointSelect
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
vehicle.parameters['SERVO1_FUNCTION'] = 'AileronLeft'
vehicle.parameters['SERVO2_FUNCTION'] = 'AileronRight'
vehicle.parameters['SERVO3_FUNCTION'] = 'Elevator'
vehicle.parameters['SERVO4_FUNCTION'] = 73
vehicle.parameters['SERVO5_FUNCTION'] = 74
#dff.initializeServos(vehicle)


# disable all control surfaces for ascent
#dff.ascentSet(vehicle)

#Upload new mission
cmds = vehicle.commands
cmds.clear()

# Set mode AUTO
vehicle.mode = VehicleMode("AUTO")


## Setting Waypoints and Geographic Fence
# Waypoint 1 (Takeoff)
lat = 40.4444476
lon = -87.0299722
alt = 0
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))


# Waypoint 2
lat = 40.4434236
lon = -87.0301597
alt = 100

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

# Waypoint 3 
lat = 40.4428473
lon = -87.0308122
alt = 100 

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

# Waypoint 4 (Loiter)
lat = 40.4427637
lon = -87.0321875
alt = 50 # assume to be terrain floor

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 0, 0, 0, 10, 0, 0, lat, lon, alt))

# Waypoint 5 
lat = 40.4434291
lon = -87.032303
alt = 50 

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

# Waypoint 6 
lat = 40.4444802
lon = -87.0321680
alt = 50 

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

# Waypoint 7 (Land)
lat = 40.4444753
lon = -87.0313085
alt = 0 # assume to be terrain floor

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 25, 0, 0, lat, lon, alt))

# Sets Geographic Fence (6-sided polygon)
lat = 40.4437230
long = -87.0312885
alt = 300 # setting maximum altitude of geofence?
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0, 0, 6, 100, 0, 0, lat, lon, alt))
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 0, 0, 6, 100, 0, 0, lat, lon, alt))
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

cmds.upload()

