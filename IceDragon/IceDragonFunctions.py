import RPi.GPIO as GPIO
import math
import numpy as np
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time
import windDataObject as windData
import os
import shutil

def deployNode(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Deployment']), 1000,0, 0, 0, 0, 0)


def uploadSounding():
    """
    checks to see if NASA's sounding file is uploaded onto the Pi

    returns visible LED color cue 
    """
    # Define the GPIO pins for the red and green LEDs
    GPIO.setmode(GPIO.BCM)
    red = 27
    green = 17
    # Set up the GPIO pins
    GPIO.setup(red, GPIO.OUT)
    GPIO.setup(green, GPIO.OUT)

    file_path = "IceDragon/AntSoundingData.txt"
    GPIO.output(red, GPIO.HIGH) # turns on red LED
    time.sleep(2)

    # Check if the file exists
    if os.path.exists(file_path):
        GPIO.output(red, GPIO.LOW) # turns off red LED
        GPIO.output(green, GPIO.HIGH) # turns on green LED
    else:
        usb_drive_path = "/media/usb"  # Change to location where USB is actually mounted

        # List files in the USB drive
        usb_files = os.listdir(usb_drive_path)
        # Assuming there is only one file on the USB drive
        if usb_files:
            source_file = os.path.join(usb_drive_path, usb_files[0])
            shutil.copy(source_file, file_path)
            GPIO.output(red, GPIO.LOW) # turns off red LED
            GPIO.output(green, GPIO.HIGH) # turns on green LED

    time.sleep(5)
    GPIO.output(green, GPIO.LOW) # turns off green LED
    GPIO.cleanup()

    return

def avg_data():
    with open("IceDragon/AntSoundingData.txt", 'r') as file:
        # Read lines from the input file
        lines = file.readlines()


    # Filter lines containing only numerical data
    numerical_lines = [line.strip() for line in lines if all(char.isdigit() or char in {'-', '.', ' '} for char in line.strip())]

    with open("IceDragon/filtered_sounding.txt", 'w') as file:
        # Write the filtered numerical data back to the file
        file.write('\n'.join(numerical_lines))


def get_sounding_data(alt):
    '''
    returns sounding data at current altitude

    :param alt: current altitude of vehicle
    '''
    # Initialize Lists
    pressure = []
    height = []
    direction = []
    speed = []
    temp = []
    current_alt = alt * 3.28084 # meters to feet

    # Read Sounding File
    with open ("IceDragon/filtered_sounding.txt", "r") as f:
        
        # WILL NEED TO CHECK FORMAT OF NEW SOUNDING FILE!!
        next(f)
        winds_aloft = f.read().split('\n')
        for i in winds_aloft:
            array = i.split("\t")
            #Handling NaN value in dataset
            if len(array) < 5:
                continue
            pressure.append(float(array[0]))
            height.append(float(array[1]))
            direction.append(float(array[2]))
            speed.append(float(array[3]))
            temp.append(float(array[4]))

    # Get closest data to current altitude
    closest = min(height, key=lambda x: abs(x - current_alt))
    idx = height.index(closest)
    data = windData(pressure[idx], height[idx], direction[idx], speed[idx], temp[idx])

    return data


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

    # Create sensor object, communicating over the board's default I2C bus
    i2c = board.I2C()  # uses board.SCL and board.SDA

    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)

    # change this to match the location's pressure (hPa) at sea level
    bme680.sea_level_pressure = 982.5 # needs to be confirmed by fit of pressure

    # You will usually have to add an offset to account for the temperature of
    # the sensor. This is usually around 5 degrees but varies by use. Use a
    # separate temperature sensor to calibrate this one.
    temperature_offset = -5

    while True:
        print("\nTemperature: %0.1f C" % (bme680.temperature + temperature_offset))
        print("Gas: %d ohm" % bme680.gas)
        print("Humidity: %0.1f %%" % bme680.relative_humidity)
        print("Pressure: %0.3f hPa" % bme680.pressure)
        print("Altitude = %0.2f meters" % bme680.altitude)

    time.sleep(1)

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

    lat_wp.append(lat2)
    lon_wp.append(lon2)
    alt_wp.append(alt_above)
    return lat_wp, lon_wp, alt_wp, alt_above    

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
    radius = 30 # radius around target [m]
    if dist <= radius:
        print("Vehicle inside radius")
        return True
    else:
        print("Vehicle outside radius")
        return False

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


def initialize_servos(vehicle):
    # --- Initialize Servo Parameters ---
    vehicle.parameters['SERVO1_FUNCTION'] = 4 # left aileron
    vehicle.parameters['SERVO1_REVERSED'] = 0
    print("SERVO1 Set - Left Aileron")

    vehicle.parameters['SERVO2_FUNCTION'] = 4 # right aileron
    vehicle.parameters['SERVO2_REVERSED'] = 1
    print("SERVO2 Set - Right Aileron")

    vehicle.parameters['SERVO3_FUNCTION'] = 19 # elevator
    print("SERVO3 Set - Elevator")

    vehicle.parameters['SERVO4_FUNCTION'] = 0 # rudder (UNSURE needs to be corrected)
    print("SERVO4 Set - Rudder")

    vehicle.parameters['SERVO5_FUNCTION'] = 0 # parachute (UNSURE needs to be corrected)
    print("SERVO5 Set - Parachute")

    # Left Aileron
    vehicle.parameters['SERVO1_MIN'] = 1000
    vehicle.parameters['SERVO1_MAX'] = 2000
    vehicle.parameters['SERVO1_TRIM'] = 1500

    # Right Aileron
    vehicle.parameters['SERVO2_MIN'] = 1000
    vehicle.parameters['SERVO2_MAX'] = 2000
    vehicle.parameters['SERVO2_TRIM'] = 1500

    # Elevator
    vehicle.parameters['SERVO3_MIN'] = 1000
    vehicle.parameters['SERVO3_MAX'] = 2000
    vehicle.parameters['SERVO3_TRIM'] = 1500

    # Rudder
    vehicle.parameters['SERVO4_MIN'] = 1000
    vehicle.parameters['SERVO4_MAX'] = 2000
    vehicle.parameters['SERVO4_TRIM'] = 1500
