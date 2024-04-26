import RPi.GPIO as GPIO
import board
import adafruit_bme680
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import windData as object
import time
import IceDragonFunctions as ice

# connecting to pixhawk using serial connection in telem2 port
print("Connecting to Vehicle...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)
print("Connected.")

i2c = board.I2C()
bme = adafruit_bme680.Adafruit_BME680_I2C(i2c)

deployed = False
dive = False
mounted = False
glide = False
loiter = False

## Check for these before script runs and store them in variables ##
groundPresure = ice.get_PT_pressure
gpsData = vehicle.location.global_frame
airspeed = ice.check_airspeed(vehicle)
groundAltitude = ice.get_altitude_GPS(vehicle)

while mounted:
    # will check airspeed and acceleration is not freefall then break
    if (ice.check_airspeed(vehicle) > 10 or ice.get_acceleration(vehicle) < -1200):
        mounted = False
        break


# NEED to drop the pixhawk and record the actual free fall accelerations values 
# Have dropped from gondola but not going to target yet (initial descent)
while deployed and glide == False:
    '''
    Compare altitude to sounding data file for lower wind speeds ~ 30-40k feet
    Set mode to auto
    '''
    altitude = ice.get_altitude() # current altitude in meters
    windObject = ice.getSoundingData(altitude) # object data from sounding file in object

    # compare with wind object from vehicle

    # Exit this loop once we have GPS data
    if vehicle.gps_0.fix_type != 0:
        glide == True
        break
    time.sleep(.1)

    # Need to check actual acceleration value
    if (ice.get_acceleration(vehicle)[2] < -1200):
            deployed = True


while glide:
    lat_wp, lon_wp, alt_wp, alt_above = ice.set_waypoints(vehicle)
    cmds = vehicle.commands
    cmds.clear()

    for i in range(0, len(lat_wp)):
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_wp[i], lon_wp[i], alt_wp[i]))

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER, 0, 0, 0, 0, 0, 0, lat_wp[i], lon_wp[i], alt_above))

    print("Uploading commands to vehicle") # delete after testing
    cmds.upload()
    print("Commands uploaded") # delete after testing

    # Setting mode to execute mission
    vehicle.mode = ("AUTO")

    # check if we have lat long data, heating system is working etc. need to work on this function
    #ice.checkSystems()

    # need to always be getting altititude and airspeed (if airspeed exceeds certain value, reduce climb angle)


    if ice.check_inside_radius(target_lat, target_lon, vehicle) == True:
        '''
        check if vehicle is within radius of waypoint
        '''
        vehicle.mode = ("LOITER")
        loiter = True
        glide = False
        break

while loiter:
    # explore guided mode; how to set value to loiter about
    # right now, Loiter means loiter around point where mode switched
    vehicle.mode = ("Loiter")

    # deploying chute if vehicle is ~250 meters above ground level
    if ice.get_altitude(vehicle) < 250 + groundAltitude:

        ice.deploy_chute(vehicle)

        print("Chute Deployed")
    
    time.sleep(2)
