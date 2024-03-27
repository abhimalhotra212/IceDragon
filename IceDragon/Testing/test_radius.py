import TestIceDragonFunctions as ice


target_lat = 4.73
target_lon = -73.93528

if ice.check_inside_radius(target_lat, target_lon) == True:
    '''
    check if vehicle is within radius of waypoint
    '''
    print("Vehicle is within target radius!")
else:
    print("Too far away!")           