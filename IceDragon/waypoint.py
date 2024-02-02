import numpy as np
import matplotlib.pyplot as plt
import datetime
import math

"""
Description: This script displays the path desired by the glider to reach its target
Author: Kayla Wojcik
Date Created: January 23, 2024
"""

# DEFINE CONSTANTS
# Unit Conversion
ft_to_m = 0.3048
kts_to_m_s = 0.514

def getSoundingData():
    """
    getSoundingData - reads NASA's sounding data file and returns relevant data as an array
    
    :return: alt_array, wind_drc_array, wind_speed_array
    """
    # Initialize Lists
    alt = [] # altitude from sounding file
    wind_drc = [] # wind direction from sounding file
    wind_speed = [] # wind speed from sounding file

    # Read Sounding File
    with open ("Dragonfly_Main/Waypoint_Select_Optimization/NASA_files/sounding.txt", "r") as f:
        
        # WILL NEED TO CHECK FORMAT OF NEW SOUNDING FILE!!
        next(f)
        winds_aloft = f.read().split('\n')
        for i in winds_aloft:
            array = i.split("\t")
            #Handling NaN value in dataset
            if len(array) < 5:
                continue
            alt.append(float(array[1]))
            wind_drc.append(float(array[2]))
            wind_speed.append(float(array[3]))

    # Conversion
    alt = np.array(alt) * ft_to_m # [m]
    wind_drc = np.array(wind_drc) # [deg]
    wind_speed = np.array(wind_speed) * kts_to_m_s # [m/s]

    return alt, wind_drc, wind_speed


def getTargetCoord(wind_direction, wind_speed):
    """
    getTargetCoord - reads NASA's target coordinates file and calculates wind velocity vectors
    
    :param wind_direction: direction array retreived from getSoundingData
    :param wind_speed: speed array retreived from getSoundingData
    :return target_array ([latitude, longitude]), wind_vel
    """
    # Initialize Lists
    latitude = []
    longitude = []

    # Read Target File
    with open ("Dragonfly_Main/Waypoint_Select_Optimization/NASA_files/targets.txt", "r") as f:

        # WILL NEED TO CHECK FORMAT OF NEW TARGET FILE!!
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
    targets = []
    for i in range(len(latitude)):
        targets.append(np.array([latitude[i], longitude[i],0]))
    targets = np.array(targets)

    # Create Wind Velocity Vector
    v_list = []
    for i in range(len(wind_speed)):
        vx = wind_speed[i] * np.sin(wind_direction[i] * np.pi/180)
        vy = wind_speed[i] * np.cos(wind_direction[i] * np.pi/180)
        v = np.array([vx, vy, 0])
        v_list.append(v)
    wind_velocity = np.array(v_list)

    return wind_velocity, targets


def latlon2distance(lat1, lat2, lon1, lon2):
    """
    latlon2distance - calculates distance between two lat-lon coordinates in kilometer

    :param lat1: latitude of point 1
    :param lat2: latitude of point 2
    :param lon1: longtitude of point 1
    :param lon1: longtitude of point 2
    :return: distance between two points in km.
    """      
    # Haversine Formula
    dlon = math.radians(lon2 - lon1)
    dlat = math.radians(lat2 - lat1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
 
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    # Radius of Earth in km
    r = 6371.1370 #km
      
    # Return Distance
    return(c * r)


def getClosest(altitudeOfInterest, altitude_array, arrayOfInterest):
    """
    getClosest - retrieve the closest values from sounding data/predicted data at the altitude of interest

    :param altitudeOfInterest: altitude of interest [meter]
    :param altitude_array: array of altitudes from sounding data
    :param arrayOfInterest: array of the data we are interested in retrieving
    :return arrayOfInterest: alosest data point in the array of interet at given altitude of interest
    """ 
    altitude_array = np.asarray(altitude_array)
    idx = (np.abs(altitude_array - altitudeOfInterest)).argmin()
    return arrayOfInterest[idx]


def checkHeading(velocityX, velocityY):
    """
    checkHeading - calculates current heading of the glider

    :param velocityX: current velocity in x direction (nose-pointing direction) 
    :param velocityY: current velocity in y direction (orthogonal-pointing direction from nose)
    :return heading_ang: current heading angle of glider 
    """
    heading_ang = math.atan(velocityY, velocityX)

    return heading_ang


def windCorrection(wind_speed, wind_direction, altitude_array, heading_angle, node_alt):
    """
    windCorrection - calculates the correction necessary to stay on track with target given wind conditions

    :param wind_speed: wind speed array from sounding data
    :param wind_drc: wind direction array from sounding data
    :param altitude_array: altitude array from sounding data
    :param heading_angle: current heading angle of glider calculated by checkHeading function
    :return wind_correction: correction needed to stay on course [length/time]
    """
    #nodegps = vehicle.location.global_frame
    #node_alt = 3000 #float(nodegps.alt) # [m]
    wind_drc = getClosest(node_alt, altitude_array, wind_direction)
    print("wind direction at alt:", wind_drc)
    wind_spd = getClosest(node_alt, altitude_array, wind_speed)
    print("wind speed at alt:", wind_spd)

    wind_ang = math.radians(wind_drc - heading_angle)
    wind_correction = wind_spd * math.sin(wind_ang)

    return wind_correction


def createWaypoints(target_alt, target_lon, target_lat, wind_speed, wind_direction, altitude):
    """
    createWaypoints - creates list of latitude, longitude, and altitude coordinates for glider to use as waypoints

    :param target_alt: final target altitude
    :param target_lon: final target longitude
    :param target_lat: final target latitude
    :param wind_speed: array of wind speeds from sounding file
    param wind_direction: array of wind directions from sounding file
    param altitude: array of altitudes from sounding file
    :return waypoints: waypoints for glider to travel through until reaching final destination
    """
    point_num = 3
    start_lat = 36
    start_lon = -105
    start_alt = 2000
    waypoints = []
    data_length = len(altitude)
    print("altitude:", altitude)
    altitude = np.asarray(altitude)

    for i in range(1, point_num + 1):
        ratio = i / (point_num + 1)
        lat = start_lat + ratio * (target_lat - start_lat)
        print("lat:", lat)
        lon = start_lon + ratio * (target_lon - start_lon)
        print("long:", lon)
        alt = start_alt + ratio * (target_alt - start_alt)
        print("alt:", alt)
        index = (np.abs(altitude - alt)).argmin()
        altInArray = altitude[index]
        print("new alt:", altInArray)
        print("index:", index) 


    # Apply wind correction based on altitude
        wind_correction = windCorrection(wind_speed, wind_direction, altitude, 0, altInArray)
        lon_correction = wind_correction / (60 * math.cos(math.radians(lat)))  # longitude correction in degrees
        lat_correction = wind_correction / 60  # latitude correction in degrees
        lat += lat_correction
        lon += lon_correction
        waypoints.append((lat, lon, alt))

    return waypoints


def GPScoords2velocity(lat1, lon1, alt1, time1, lat2, lon2, alt2, time2):
    #Retrieve current position from GPS module (code taken from dragonFlyMain)
    #nodegps = vehicle.location.global_frame
    #node_lat = float(nodegps.lat)
    #node_lon = float(nodegps.lon)
    #node_alt = float(nodegps.alt)
    #currentPositionLatLongAlt = np.array([node_lat, node_lon, node_alt]) 

    # Radius of Earth in km
    r = 6371.1370 #km

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    # Calculate differences in position and time
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    dalt = alt2 - alt1
    dtime = time2 - time1

    # Calculate velocities
    Vnorth = dlat / dtime * r # velocity in northward directions
    Veast = dlon / dtime * r * math.cos((lat1_rad + lat2_rad) / 2) # velocity in eastward direction
    Vup = dalt / dtime # velocity in upward direction

    return Vnorth, Veast, Vup



# TEST OF FUNCTIONS
altitude, wind_drc, wind_speed = getSoundingData() # gathers altitude, wind direction, and speed sounding data
print("altitude [m]:", altitude)
print("wind direction:", wind_drc)
print("wind speed [m/s]:", wind_speed) 

#wind_vel_vector, targets = getTargetCoord(wind_drc, wind_speed)
#print("wind vectors:", wind_vel_vector)
#print("target:", targets)

head_ang = 0 # heading angle of glider [deg]
node_alt = 2000 # starting altitude of glider [m]
wind_correct = windCorrection(wind_speed, wind_drc, altitude, head_ang, node_alt)
#print("wind correction:", wind_correct)

waypoints = createWaypoints(300, -106.7, 32.4, wind_speed, wind_drc, altitude)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i, waypoint in enumerate(waypoints):
    print(f"Waypoint {i + 1}: Latitude {waypoint[0]}, Longitude {waypoint[1]}, Altitude {waypoint[2]}")
    ax.scatter(waypoint[1], waypoint[0], waypoint[2])

ax.scatter(-106.7, 32.4, 300)
ax.scatter(-105, 36, 2000)
ax.set_xlabel("Longitude [deg]")
ax.set_ylabel("Latitude [deg]")
ax.set_zlabel("Altitude [m]")
plt.show()