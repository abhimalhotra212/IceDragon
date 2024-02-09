import numpy as np
import matplotlib.pyplot as plt
import datetime
import math

"""
Description: This script displays the path desired by the glider to reach its target altitude above the landing location
Author: Kayla Wojcik
Date Created: February 5, 2024
"""

# Functions

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
    ft_to_m = 0.3048
    kts_to_m_s = 0.514
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

    # Input Target Coordinates
    latitude = input("Enter target latitude: ")
    longitude = input("Enter target longitude: ")
    altitude = input("Enter target altitude: ")
    print("Target Location (lat, lon, alt): (" + latitude + ", " + longitude + ", " + altitude + ")")
    
    target = []
    target.append(latitude)
    target.append(longitude)
    target.append(altitude)

    # Create Wind Velocity Vector
    v_list = []
    for i in range(len(wind_speed)):
        vx = wind_speed[i] * np.sin(wind_direction[i] * np.pi/180)
        vy = wind_speed[i] * np.cos(wind_direction[i] * np.pi/180)
        v = np.array([vx, vy, 0])
        v_list.append(v)
    wind_velocity = np.array(v_list)

    return wind_velocity, target


def glideAngle(target):
    """
    glideAngle - gets glide angle input from user to determine distance to target after dive phase
    :param target: target array with lat, lon, alt
    :return alt_above, lat1, lon1, alt1 (location of node currently)
    """

    # Get glider current location 
    #nodegps = vehicle.location.global_frame
    lat1 = float(-105) #float(nodegps.lat)
    lon1 = float(36) #float(nodegps.lon)
    alt1 = float(1200) #float(nodegps.alt)
    # Target location
    lat2 = float(target[0])
    lon2 = float(target[1])
    alt2 = float(target[2])

    # User input glide angle
    glide = float(input("Enter desired glide angle: "))
    angle = glide * np.pi / 180 # convert degrees to radians

    # Calculate distance between starting and target location
    flat_dist = latlon2distance(lat1, lat2, lon1, lon2) 

    # Trig
    delta_h = flat_dist * math.tan(angle)
    alt_above = alt1 - delta_h
    print(alt_above)

    return alt_above, lat1, lon1, alt1, lat2, lon2, alt2, alt_above


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


def createWaypoints(alt_above, lat2, lon2, lat1, lon1, alt1):
    """
    createWaypoints - creates list of latitude, longitude, and altitude coordinates for glider to use as waypoints

    :param lat1: initial latitude
    :param lon1: initial longitude
    :param alt1: initial altitude
    :return waypoints: waypoints for glider to travel through until reaching location above target
    """
    print("start lat:", lat1)
    # Target location
    #lat2 = float(target[0])
    #print("target lat:", lat2)
    #lon2 = float(target[1])
    #alt2 = float(target[2])
    
    lat_wp = []
    lon_wp = []
    alt_wp = []
    point_num = 3

    for i in range(1, point_num + 1):
        ratio = i / (point_num + 1)
        lat = lat1 + ratio * (lat2 - lat1)
        print("wp lat:", lat)
        lon = lon1 + ratio * (lon2 - lon1)
        print("wp lon:", lon)
        alt = alt1 + ratio * (alt_above - alt1)
        print("wp alt:", alt)
        lat_wp.append(lat)
        lon_wp.append(lon)
        alt_wp.append(alt)

    return lat_wp, lon_wp, alt_wp

# Test of Functions
alt, wind_drc, wind_speed = getSoundingData()
wind_vel, target = getTargetCoord(wind_drc, wind_speed)
alt_above, lat1, lon1, alt1, lat2, lon2, alt2, alt_above = glideAngle(target)
#print("startlat:" + lat1 + "startlon:" + lon1 + "startalt:" + alt1)
lat_wp, lon_wp, alt_wp = createWaypoints(alt_above, lat2, lon2, lat1, lon1, alt1)
print(lat_wp)
print(lon_wp)
print(alt_wp)

# Figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# plots waypoints
ax.scatter(lon_wp[0], lat_wp[0], alt_wp[0])
ax.text(lon_wp[0], lat_wp[0], alt_wp[0], str(2))
ax.scatter(lon_wp[1], lat_wp[1], alt_wp[1])
ax.text(lon_wp[1], lat_wp[1], alt_wp[1], str(3))
ax.scatter(lon_wp[2], lat_wp[2], alt_wp[2])
ax.text(lon_wp[2], lat_wp[2], alt_wp[2], str(4))
ax.scatter(lon2, lat2, alt_above)
ax.text(lon2, lat2, alt_above, str(5))
# plots starting and target locations
ax.scatter(lon1, lat1, alt1)
ax.text(lon1, lat1, alt1, str("start"))
ax.scatter(lon2, lat2, alt2)
ax.text(lon2, lat2, alt2, str("target"))

ax.set_xlabel("Longitude [deg]")
ax.set_ylabel("Latitude [deg]")
ax.set_zlabel("Altitude [m]")
plt.show()


