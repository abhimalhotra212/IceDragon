import TestIceDragonFunctions as ice
from dronekit import connect, Command, mavutil

vehicle = connect('dev/serial0', wait_ready=True, baud=921600)

# assuming we have GPS data
glide = True

while glide:
    lat_wp, lon_wp, alt_wp, alt_above = ice.set_waypoints()
    for i in range(0, len(lat_wp)):
        print("Waypoint" + i + ":" + lat_wp[i] + lon_wp[i] + alt_wp[i])
    cmds = vehicle.commands
    cmds.clear()

    # upload waypoints
    for i in range(0, len(lat_wp)):
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_wp[i], lon_wp[i], alt_wp[i]))

    # begin loiter location
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER, 0, 0, 0, 0, 0, 0, lat_wp[i], lon_wp[i], alt_above))

    print("Uploading commands to vehicle") # delete after testing
    cmds.upload()
    print("Commands uploaded") # delete after testing

    # Setting mode to execute mission
    vehicle.Mode = ("AUTO")

    #if ice.check_inside_radius(target_lat, target_lon, vehicle) == True:
        #'''
        #check if vehicle is within radius of waypoint
        #'''
        #vehicle.mode = ("LOITER")
        #loiter = True
        #glide = False
        #break