import numpy as np
import matplotlib.pyplot as plt
import datetime
import math

"""
Description: This script displays the path desired by the glider to reach its target altitude above the landing location
Author: Kayla Wojcik
Date Created: February 5, 2024
"""
# Initialize Sounding File Lists
alt = [] 
# read sounding file
with open ("Dragonfly_Main/Waypoint_Select_Optimization/NASA_files/sounding.txt", "r") as f:
    # WILL NEED TO CHECK FORMAT OF NEW SOUNDING FILE!!
    next(f)
    winds_aloft = f.read().split('\n')
    for i in winds_aloft:
        array = i.split("\t")
        if len(array) < 5:
            continue
        alt.append(float(array[1]))
# conversion
alt = np.array(alt) * 0.3048 # [m]

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
# calculate distance between starting and target location with Haversine Formula
dlon = math.radians(lon2 - lon1)
dlat = math.radians(lat2 - lat1)
a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
r = 6371.1370 # radius of Earth in km 
flat_dist = c * r
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

# Figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("Longitude [deg]")
ax.set_ylabel("Latitude [deg]")
ax.set_zlabel("Altitude [m]")
# plot waypoints
for i in range(0, 3):   
    ax.scatter(lon_wp[i], lat_wp[i], alt_wp[i])
    ax.text(lon_wp[i], lat_wp[i], alt_wp[i], str(i+2))
# plot point before loiter
ax.scatter(lon2, lat2, alt_above)
ax.text(lon2, lat2, alt_above, str(5))
# plots starting and target locations
ax.scatter(lon1, lat1, alt1)
ax.text(lon1, lat1, alt1, str("start"))
ax.scatter(lon2, lat2, alt2)
ax.text(lon2, lat2, alt2, str("target"))
plt.show()


