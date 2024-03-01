import RPi.GPIO as GPIO
import board
import adafruit_bme680
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import windData
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
loiter = False

'''
Mounted variable needs to go True when signal is recieved
'''
# --- Mounting ---
# NEED TO HAVE MOUNTING PINS IN
GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(25, GPIO.IN)    # set GPIO25 as input (button)  
ice.nodeDeploymentTest(vehicle,1500)


# NEEDS TO BE CHANGED
while not mounted:
    if GPIO.input(25) == GPIO.HIGH: # we need to check to make sure the correct GPIO pin is receiving the signal
        dff.nodeDeploymentTest(vehicle,2000)
        mounted = True

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
    time.sleep(1)

while deployed and glide == False:
    altitude = ice.get_altitude()
    '''
    Compare altitude to sounding data file for lower wind speeds ~ 30-40k feet
    Set mode to auto
    '''
    if check_wind_speed(altitude):
        glide == True
        break
    time.sleep(.1)


while glide:
    lat_wp, lon_wp, alt_wp = ice.set_waypoints()
    cmds = vehicle.commands
    cmds.clear()

    for i in range(0, len(lat_wp)):
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_wp[i], lon_wp[i], alt_wp[i]))

    print("Uploading commands to vehicle")
    cmds.upload()
    print("Commands uploaded")

    # arming the vehicle
    vehicle.armed = True

    # Setting mode to execute mission
    vehicle.Mode = ("AUTO")

    # check if we have lat long data, heating system is working etc.
    ice.checkSystems()


    if ice.check_inside_radius(target_lat, target_lon, vehicle):
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
    vehicle.Mode = ("Loiter")

    # deploying chute if vehicle is ~3000 meters above ground level
    if ice.get_altitude(vehicle) < 3000:

        ice.deploy_chute(vehicle)

        print("Chute Deployed")
    
    time.sleep(2)
