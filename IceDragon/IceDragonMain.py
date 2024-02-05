import RPi.GPIO as GPIO
import board
import adafruit_bme680
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time
import IceDragonFunctions as ice

# connecting to pixhawk using serial connection in telem2 port
print("Connecting to Vehicle...")
vehicle = connect('dev/serial0', wait_ready=True, baud=921600)
print("Connected.")

i2c = board.I2C()
bme = adafruit_bme680.Adafruit_BME680_I2C(i2c)

deployed = False
dive = False
mounted = False
glide = False


while mounted:
    # pseudocode for sitting on gondola
    '''
    if pin is inserted
        mounted = true
        jitter servos
        get current position
    if signal is recieved
        mounted = false
        deploy node
        get current position
    '''

while deployed:
    altitude = ice.get_altitude()
    '''
    Compare altitude to sounding data file for lower wind speeds ~ 30-40k feet
    Set mode to auto
    '''
    if check_wind_speed(altitude):
        break

while glide:
    waypoints = ice.set_waypoints()
    cmds = vehicle.commands
    cmds.clear()
    
    for i in waypoints:
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))

    vehicle.Mode = ("AUTO")
    


