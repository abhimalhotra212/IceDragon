import RPi.GPIO as GPIO

from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time

C_METER = 3.28084
C_FEET = 1

def deployNode(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Deployment']), 1000,0, 0, 0, 0, 0)


def check_wind_speed():
    '''
    use sounding data file to get wind speeds at given altitude
    '''

def get_altitude(vehicle, bme):
    altitude_gps = 0
    altitude_bme = 0
    if (vehicle.gps_0.fix_type == 2 or vehicle.gps_0.fix_type == 3):
        altitude_gps = vehicle.location.global_frame.alt
    
    altitude_bme = get_altitude_BME()

    # averaging the altitudes for now, need to test accuracy of either sensor
    altitude = (altitude_gps + altitude_bme) / 2


def get_altitude_BME():
    '''
    get altitude data from bme sensor
    '''
    
def set_waypoints():
    '''
    kayla's waypoint algorithm used here
    '''

def check_systems():
    '''
    check heating system and gps data, 
    '''
