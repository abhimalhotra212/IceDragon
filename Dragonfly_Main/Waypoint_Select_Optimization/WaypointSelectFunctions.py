
#!/usr/bin/env python3
# DragonFlyFunctions.py

#import statements
import numpy as np
#import matplotlib.pyplot as plt
import datetime

"""
Description: This script contains all user defined functions used in run.py
Author: Kasidit Muenprasitivej, Evan Glenn, Aaron Riley
Date Created: May 23, 2023
Date Modified: July 14, 2023
Version: 1.0
Python Version: 3.9.7
Dependencies: ~
License: Purdue University
"""

#DEFINE CONSTANTS
#Unit conversion
ft_to_m = 0.3048
kts_to_m_s = 0.514


def getOption():
    """
    getOption - ask user for an option number to determine which test case to run

    :return: option number indicating the test case number
    """ 
    opt_list = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10",
            "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
            "21", "22", "23", "24", "25", "26", "27", "28", "29", "30"]

    valid_opt = False
    while(not valid_opt):
        opt = str(input("Choose the test case by entering two digits number (from 01 - 30): "))
        if opt in opt_list:
            valid_opt = True
        else:
            print("Invalid test case. Try again.")
    return opt



def getClosest(altitudeOfInterest, altitude_array, arrayOfInterest):
    """
    getClosest - retrive the closest values from sounding data/predicted data at the altitude of interest

    :param altitudeOfInterest: Altitude of interest [meter]
    :param altitude_array: Array of altitudes with corresponding defined data
    :param arrayOfInterest: Array of the corresponding data
    :return: Closest data point in the Array of the corresponding data at given altitude of interest
    """ 
    altitude_array = np.asarray(altitude_array)
    idx = (np.abs(altitude_array - altitudeOfInterest)).argmin()
    return arrayOfInterest[idx]


def getSoundingDataAndTargetCoord(opt, useCustomWaypoints):
    """
    getSoundingDataAndTargetCoord - read and return all sounding data and target coordinate as valued arrays, given the option digit

    :param opt: option digit ranging from 01 - 30
    :param useCustomWaypoints: boolean variable indicating the program to use custom targeted waypoints
    :return: altitude_array, wind_velocity_array, waypoints_array (i.e., 6 target points)
    """ 

    #Initialize lists
    altitude=[]
    wind_direction=[]
    wind_speed=[]

    #read sounding file and extract data
 #   with open ("NASA_files/sounding.txt", "r") as f: 
    with open ("Dragonfly_Main/Waypoint_Select_Optimization/NASA_files/sounding.txt", "r") as f:
        
        # MAKE SURE FORMAT IS SAME AS EXAMPLES
        next(f)
        winds_aloft = f.read().split('\n')
        for i in winds_aloft:
            array = i.split("\t")
            #Handling NaN value in dataset
            if len(array) < 5:
                continue
            altitude.append(float(array[1]))
            wind_direction.append(float(array[2]))
            wind_speed.append(float(array[3]))

    # CHECK FOR WHICH COLUMN IS WHICH
    altitude = np.array(altitude) * ft_to_m #m
    wind_direction = np.array(wind_direction) #deg
    wind_speed = np.array(wind_speed) * kts_to_m_s #m/s

    #----------------------------

    #Initialize lists
    latitude = []
    longitude = []

    #read target coordinate and extract data
#    targetFileName = "NASA_files/targets.txt"
    targetFileName = "Dragonfly_Main/Waypoint_Select_Optimization/NASA_files/targets.txt"

    with open (targetFileName, "r") as f:

        # MAKE SURE FORMAT IS SAME AS EXAMPLES
        next(f)
        waypointsRaw = f.read().split('\n')
        for i in waypointsRaw:
            array = i.split(",")
            #Handling NaN value in dataset
            if len(array) < 4:
                continue
            latitude.append(float(array[2]))
            longitude.append(float(array[1]))

    latitude = np.array(latitude)
    longitude = np.array(longitude)

    waypoints = []
    for i in range(len(latitude)):
        waypoints.append(np.array([latitude[i], longitude[i],0]))
    waypoints = np.array(waypoints)

    #----------------------------

    #create wind velocity vector
    v_list = []
    for i in range(len(wind_speed)):
        vx = wind_speed[i] * np.sin(wind_direction[i] * np.pi/180)
        vy = wind_speed[i] * np.cos(wind_direction[i] * np.pi/180)
        v = np.array([vx, vy, 0])
        v_list.append(v)
    wind_velocity = np.array(v_list)

    return altitude, wind_velocity, waypoints



def getPredictedTrajectory():
    """
    getPredictedTrajectory - read and return predicted trajectory of the node provided from descentProfileCallScript.m (MATLAB) 

    :return: predicted trajectory of the node
    """ 

    #Initialize lists
    predictedAltitude = []
    verticalVelocity = []
    horizontalVelocity = []
    predictedTrajectory = []

    #read predicted trajectory and extract data
#    with open ("descentTrajectoryPrediction/predictedTrajectory.dat", "r") as f:
    with open ("Dragonfly_Main/Waypoint_Select_Optimization/descentTrajectoryPrediction/predictedTrajectory.dat", "r") as f:
        next(f)
        predictedTrajectoryRaw = f.read().split('\n')
        for i in predictedTrajectoryRaw:
            array = i.split(",")
            #Handling NaN value in dataset
            if len(array) < 6:
                continue
            predictedAltitude.append(float(array[0]))
            verticalVelocity.append(float(array[1]))
            horizontalVelocity.append(float(array[3]))

        predictedAltitude = np.array(predictedAltitude)
        verticalVelocity = np.array(verticalVelocity)
        horizontalVelocity = np.array(horizontalVelocity)

    for i in range(len(predictedAltitude)):
        predictedTrajectory.append(np.array([predictedAltitude[i], verticalVelocity[i], horizontalVelocity[i]]))
    predictedTrajectory = np.array(predictedTrajectory)

    return predictedTrajectory



def simulate(altitude, wind_velocity, predictedTrajectory, waypoints, initialPositionLatLongAlt, plotTraj, printTraj):
    """
    simulate - Simulate the node's trajectory (from current position) to each targeted waypoint
               in order to determine the best waypoint to navigate.

    :param altitude: Array of altitudes
    :param wind_velocity: Array of wind velocity vectors corresponding to each altitude in the altitude array
    :param predictedTrajectory: Predicted trajectory of the node including vertical velocity (z) and resultant horizontal velocity (x + y)
    :param waypoints: Array of coordinates of the targeted waypoint (A - F)
    :param initialPositionLatLongAlt: current positon of the node given from the Pixhawk Flight Controller's GPS data
    :param plotTraj: Boolean variable to show/hide the plots of predicted trajectories
    :return: describe what it returns
    """ 
    earthRadius = 6371.1370 * 1000 #m
    timeStep = 1 #s
    margin = 250 #m
    groundLevel = 1223 #m
    nFig = 0

    predictedAltitude = predictedTrajectory[:, 0]
    verticalVelocity = predictedTrajectory[:, 1]
    horizontalVelocity = predictedTrajectory[:, 2]
    position = np.array([0, 0, 0])
    targetDirection = np.array([0, 0, 0])

    waypointSuccesses = []
    
    waypointIdx = 0
    for waypoint in waypoints:
    
        positionLatLongAlt = initialPositionLatLongAlt

        #convert coordinate [lat, long, m] to xyz [m, m, m]
        position[0] = (positionLatLongAlt[1] - waypoint[1]) * earthRadius * np.pi / 180
        position[1] = (positionLatLongAlt[0] - waypoint[0]) * earthRadius * np.pi / 180
        position[2] = positionLatLongAlt[2]
        
        #Find the directional vector toward the waypoint
        targetDirection[0] = (positionLatLongAlt[1] - waypoint[1]) * earthRadius * np.pi / 180
        targetDirection[1] = (positionLatLongAlt[0] - waypoint[0]) * earthRadius * np.pi / 180
        targetDirection[2] = 0 #-positionLatLongAlt[2]
        targetDirectionUnit = targetDirection / np.linalg.norm(targetDirection)

        #Define list variable to keep track of the node's simulated position history
        xPosition = []
        yPosition = []
        zPosition = []

        soundingArray = []
        soundingArrayX = []
        soundingArrayY = []
        xPositionSounding = []
        yPositionSounding = []
        zPositionSounding = []

        #Begin Trajectory Prediction
        i = 0
        while position[2] > (groundLevel):
            i = i + 1
            currentAltitude = position[2]

            #Get closest wind velocity at current altitude
            soundingVelocity = getClosest(currentAltitude, altitude, wind_velocity)
            
            if (i % 17 == 0):
                soundingArrayX.append(soundingVelocity[0])
                soundingArrayY.append(soundingVelocity[1]) 
                xPositionSounding.append(position[0])
                yPositionSounding.append(position[1])
                zPositionSounding.append(position[2])


            #Get the node speed, predicted at the current altitude
            predictedHorizontalVelocity = getClosest(currentAltitude, predictedAltitude, horizontalVelocity) # sqrt(Vx^2 + Vy^2)
            predictedVerticalVelocity = getClosest(currentAltitude, predictedAltitude, verticalVelocity) #Vz

            #Get the node velocity vector, predicted at the current altitude
            predictedVelocityDirected = [targetDirectionUnit[0] * predictedHorizontalVelocity, targetDirectionUnit[1] * predictedHorizontalVelocity, predictedVerticalVelocity]
            
            #Vector addition 
            trueVelocity = soundingVelocity + predictedVelocityDirected

            position = position + trueVelocity * timeStep
            xPosition.append(position[0])
            yPosition.append(position[1])
            zPosition.append(position[2])

            distanceToTarget = np.linalg.norm(position)
            targetDirectionUnit[0] = (-position [0]) / distanceToTarget
            targetDirectionUnit[1] = (-position [1]) / distanceToTarget

            if printTraj:
                print("distance to Targ", np.linalg.norm(position[0:2]))
            
            reachTarg = []
            if (np.linalg.norm(position[0:2]) <= margin):
                #send waypoint to Pixhawk
                waypointSuccesses.append([int(waypointIdx), position[2]])
                reachTarg = position

                if printTraj:
                    print("Reached!", reachTarg)
                break

        waypointIdx += 1

        # comment out plotted things
        '''
        if plotTraj:
            nFig += 1
            fig = plt.figure(nFig)
            ax = plt.axes(projection='3d')
            if(len(reachTarg) == 3):
                plt.plot(reachTarg[0], reachTarg[1],reachTarg[2], 'o', color = 'lime', markersize = 10)
                print("plotted target")
            ax.scatter(xPosition, yPosition, zPosition, 'bo')
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_zlabel("Altitude [m]")
            plt.plot(0, 0, 0, 'ro')
            ax.quiver(xPositionSounding, yPositionSounding, zPositionSounding, soundingArrayX, soundingArrayY, 0, length = 500, color = 'orange')
        '''
    # comment out plotted things 
#    plt.show()

    return np.array(waypointSuccesses)




def findOptimalWaypoint(waypointSuccesses, opt, useCustomWaypoints, currentPositionLatLongAlt):
    """
    findOptimalWaypoint - find the optimal waypoint coordinate (if any) and print the result onto the commandline

    :param waypointSuccesses: List of Reachable waypoints with corresponding index
    :param opt: option digit ranging from 01 - 30
    :param useCustomWaypoints: boolean variable indicating the program to use custom targeted waypoints
    :param currentPositionLatLongAlt: current positon of the node given from the Pixhawk Flight Controller's GPS data
    :return: An optimal targeted waypoint
    """ 

    if waypointSuccesses is not None and np.size(waypointSuccesses) > 0:

        #Initialize empty array
        latitude = []
        longitude = []
        deviation = []
        labels = []

        #choose success waypoint that has highest remaining altitdue for loiter
        #waypointSucesses = [index_of_the_waypoints , altitude_when_reach_the_waypoint_radius]
        highest_alt_idx = np.where(waypointSuccesses[:,1] == max(waypointSuccesses[:,1])) 
        optimal_waypoint_idx = waypointSuccesses[highest_alt_idx, 0]
        
        #print("Before", optimal_waypoint_idx)

        #read target coordinate and extract data
        targetFileName = "NASA_files/targets.txt"
            
        with open (targetFileName, "r") as f:

            # MAKE SURE FORMAT IS SAME AS EXAMPLES
            next(f)
            waypointsRaw = f.read().split('\n')
            for i in waypointsRaw:
                array = i.split(",")

                #Handling NaN value in dataset
                if len(array) < 4:
                    continue

                labels.append(str(array[0]))
                latitude.append(float(array[2]))
                longitude.append(float(array[1]))
                deviation.append(float(array[3]))

        latitude = np.array(latitude)
        longitude = np.array(longitude)
        deviation = np.array(deviation)

        #if there are multiple optimal waypoints (with the same remainng altitude for loiter)
        #then choose the closest waypoint to the current node coordinate
        if len(optimal_waypoint_idx[0]) > 1:   
            node_lat = currentPositionLatLongAlt[0]
            node_lon = currentPositionLatLongAlt[1]
            distance_node2waypoints = []

            #find distance between each success waypoint to the node's position
            for i in optimal_waypoint_idx[0]:
                waypoint_lat = latitude[int(i)]
                waypoint_lon = longitude[int(i)]

                d = latlon2distance(node_lat, waypoint_lat, node_lon, waypoint_lon)
                distance_node2waypoints.append(d)
            
            #print("Distance", distance_node2waypoints)
    
            lowest_dist_idx = np.where(distance_node2waypoints == min(distance_node2waypoints))
            optimal_waypoint_idx = optimal_waypoint_idx[0, lowest_dist_idx]
        
        optimal_waypoint_idx = optimal_waypoint_idx[0,0]
        #print("After", optimal_waypoint_idx)

        #print result
        # print("The optimal waypoint is", labels[optimal_waypoint_idx.astype(int)])
        # print("Latitude: ", latitude[optimal_waypoint_idx.astype(int)])
        # print("Longtitude: ", longitude[optimal_waypoint_idx.astype(int)])
        # print("Deviation: ", deviation[optimal_waypoint_idx.astype(int)])
        
        optimal_waypoint = np.array([latitude[optimal_waypoint_idx.astype(int)], longitude[optimal_waypoint_idx.astype(int)]])
        return optimal_waypoint, True

    else:
        # print("No optimal waypoint found at the current node's position.")
        optimal_waypoint = np.array([1000,1000,0])
        return optimal_waypoint, False
    


def latlon2distance(lat1, lat2, lon1, lon2):
    """
    latlon2distance - find distance between two lat-lon coordinates in kilometer

    :param lat1: latitude of point 1
    :param lat2: latitude of point 2
    :param lon1: longtitude of point 1
    :param lon1: longtitude of point 2
    :return: distance between two points in km.
    """ 
     
    
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = lon1 * np.pi / 180
    lon2 = lon2 * np.pi / 180
    lat1 = lat1 * np.pi / 180
    lat2 = lat2 * np.pi / 180
      
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2
 
    c = 2 * np.arcsin(np.sqrt(a))
    
    # Radius of earth in kilometers.
    r = 6371.1370 #km
      
    # calculate the result in km
    return(c * r)

def getBalloonTrajectory():

    #initialize array
    cum_time = []
    lat = []
    long = []
    alt = []

    #read sounding file and extract data
    with open ("Sample_Balloon_Trajectory/20230724-004100Z_34.0297_255.3083_standard_profile_2023072318Z", "r") as f:
        next(f)
        dataRow = f.read().split('\n')
        for i in dataRow:
            array = i.split(",")

            #Handling NaN value in dataset
            if len(array) < 4:
                continue
            
            dateAndTime = array[0, :-1]
            dateAndTime = dateAndTime.split("T")
            date = dateAndTime[0].split("-")
            time = dateAndTime[1].split(":")


def run(currentPositionLatLongAlt):
    '''
    node_position: array([lat, lon, alt])
    '''
    #Retrive predicted trajectory of the node
    predictedTrajectory = getPredictedTrajectory()

    #Get sounding data and targeted waypoints
    opt = "23"
    useCustomWaypoints = False  #use target waypoints from "Custom_waypoints" folder
    altitude, wind_velocity, waypoints = getSoundingDataAndTargetCoord(opt, useCustomWaypoints)

    #Simulate the node's trajectory to each targeted waypoint and select the best possible waypoint
    plotTraj = False
    printTraj = False
    waypointSuccesses = simulate(altitude, wind_velocity, predictedTrajectory, waypoints, currentPositionLatLongAlt, plotTraj, printTraj)
    
    return findOptimalWaypoint(waypointSuccesses, opt, useCustomWaypoints, currentPositionLatLongAlt)




