import RPi.GPIO as GPIO
import math
import numpy as np
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time

def deployNode(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Deployment']), 1000,0, 0, 0, 0, 0)


def check_wind_speed():
    '''
    use sounding data file to get wind speeds at given altitude

    code is something like...
    '''
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
    alt = np.array(alt) * 0.3048 # [m]
    wind_drc = np.array(wind_drc) # [deg]
    wind_speed = np.array(wind_speed) * 0.514 # [m/s]

    return alt, wind_drc, wind_speed


def get_altitude(vehicle, bme):
    altitude_gps = 0
    altitude_bme = 0
    if (vehicle.gps_0.fix_type == 2 or vehicle.gps_0.fix_type == 3):
        altitude_gps = vehicle.location.global_frame.alt
    
    altitude_bme = get_altitude_BME()

    # averaging the altitudes for now, need to test accuracy of either sensor
    # seeing if gps altitude is within 20 meters of bme sensor
    if altitude_gps < altitude_bme - 20 & altitude_gps > altitude_bme + 20:
        altitude = (altitude_gps + altitude_bme) / 2
        
        return altitude
    else:
        return altitude_bme
    


def get_altitude_BME():
    '''
    get altitude data from bme sensor
    '''

def haversine_formula(lat1, lon1, lat2, lon2):
    '''
    calculates distance between two lat/lon coordinates using Haversine Formula
    '''
    dlon = math.radians(lon2 - lon1)
    dlat = math.radians(lat2 - lat1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    r = 6371.1370 # radius of Earth in km 
    
    return c * r * 1000 # returns distance in meters


def set_waypoints():
    '''
    kayla's waypoint algorithm used here

    returns: 3 waypoints (lat, lon, alt), target (lat, lon, alt), and altitude above target where we begin loiter
    '''

    # starting coordinates
    #nodegps = vehicle.location.global_frame
    lat1 = float(-105) #float(nodegps.lat)
    lon1 = float(36) #float(nodegps.lon)
    alt1 = float(1200) #float(nodegps.alt)
    # target coordinates
    lat2 = float(-109)
    lon2 = float(30)
    alt2 = float(300)

    # glide angle
    glide = float(30)
    angle = glide * np.pi / 180 # convert degrees to radians
    # calculate distance [m] between starting and target location with Haversine Formula
    flat_dist = haversine_formula(lat1, lon1, lat2, lon2)
    # some trig
    delta_h = flat_dist * math.tan(angle)
    alt_above = alt1 - delta_h # altitude above target coords

    # Waypoints    
    lat_wp = []
    lon_wp = []
    alt_wp = []
    point_num = 3
    for i in range(1, point_num + 1):
        ratio = i / (point_num + 1)
        lat = lat1 + ratio * (lat2 - lat1)
        lon = lon1 + ratio * (lon2 - lon1)
        alt = alt1 + ratio * (alt_above - alt1)
        lat_wp.append(lat)
        lon_wp.append(lon)
        alt_wp.append(alt)

    return lat_wp, lon_wp, alt_wp, lat2, lon2, alt2, alt_above    

def check_inside_radius(lat2, lon2, vehicle):
    '''
    check if inside designated radius of target to loiter about
    parameter: lat2 - target latitude, lon2 - target longitude
    '''

    nodegps = vehicle.location.global_frame
    current_lat = float(nodegps.lat)
    current_lon = float(nodegps.lon)
    current_alt = float(nodegps.alt)
    dist = haversine_formula(current_lat, current_lon, lat2, lon2)
    radius = 20 # radius around target [m]
    if dist <= radius:
        print("Vehicle inside radius")
    else:
        print("Vehicle outside radius")

    return

def send_loiter_mission(vehicle, lat2, lon2, alt2, loit_time):
    '''
    defines a loiter mission given a target location
    
    not sure if correct
    '''
    loiter = mavutil.mavlink.MAVLink_mission_item_message(
        1, 1,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        0,  # current
        0,  # autocontinue
        0, 0, 0, 
        loit_time,  # loiter time in seconds
        0,  # orbit count
        lat2, lon2, alt2  # latitude, longitude, altitude
    )

    vehicle.mav.send(loiter)

def check_systems():
    '''
    check heating system and gps data, 
    '''
