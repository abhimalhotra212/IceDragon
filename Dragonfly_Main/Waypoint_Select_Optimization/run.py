#!/usr/bin/env python3
# run.py

#import statement
import WaypointSelectFunctions as dff
import numpy as np
#from dronekit import connect


"""
Description: This script select the optimized waypoint as the node propagates throughout the misson, 
             while managing the MAVLINK data flow from Pixhawk Flight Controller.
Author: Kasidit Muenprasitivej, Evan Glenn, Aaron Riley
Date Created: May 23, 2023
Date Modified: July 14, 2023
Version: 1.0
Python Version: 3.9.7
Dependencies: ~
License: Purdue University
"""

#Connect to Pixhawk Flight Controller using Serial protocol
# vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)

#Choose Test Case Number
#opt = dff.getOption()
opt = "23"

#Retrieve sounding data and target coordinates provided by NASA
# opt_list = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10",
#             "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
#             "21", "22", "23", "24", "25", "26", "27", "28", "29", "30"]

useCustomWaypoints = True  #use target waypoints from "Custom_waypoints" folder
altitude, wind_velocity, waypoints = dff.getSoundingDataAndTargetCoord(opt, useCustomWaypoints)

print("all waypoints", waypoints)

#Retrive predicted trajectory of the node
predictedTrajectory = dff.getPredictedTrajectory()

#Retrive the current GPS coordinate of node from Pixhawk Flight Controller via MAVLINK
""" Here, we use a placeholder GPS coordinate to run the code"""
lat = 40.42001 #lat = vehicle.location.global_frame.lat
lon = -86.92001 #lon = vehicle.location.global_frame.lon
alt = 36000 #alt = vehicle.location.global_frame.alt
currentPositionLatLongAlt = np.array([lat, lon, alt]) #Lat (deg), Long (deg), Atl (meters)

#Simulate the node's trajectory to each targeted waypoint and select the best possible waypoint
plotTraj = False
printTraj = False
waypointSuccesses = dff.simulate(altitude, wind_velocity, predictedTrajectory, waypoints, currentPositionLatLongAlt, plotTraj, printTraj)
print("success", waypointSuccesses)
optimalWaypoint, Reachable = dff.findOptimalWaypoint(waypointSuccesses, opt, useCustomWaypoints, currentPositionLatLongAlt)
print("optimal", optimalWaypoint)
print("Reachable?", Reachable)


