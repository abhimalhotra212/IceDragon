import RPi.GPIO as GPIO

from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time

C_METER = 3.28084
C_FEET = 1

def deployNode(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Deployment']), 1000,0, 0, 0, 0, 0)


def get_altitude(vehicle, bme):
    if (vehicle.gps_0.fix_type == 2 or vehicle.gps_0.fix_type == 3):
        altitude = vehicle.location.global_frame.alt
    else
        altitude = get_altitude_BME()


    # need to add else statement if altitude data is not available use BME Sensor

def set_waypoints():
    '''
    kayla's waypoint algorithm used here
    '''
