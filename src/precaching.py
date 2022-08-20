#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.srv import GetMap
import pickle
import yaml

rospy.init_node('precaching', anonymous=True)
rospy.wait_for_service("static_map")
grid_map =rospy.ServiceProxy("static_map", GetMap)().map
data=grid_map.data
data = np.array(data, dtype='int8')
map_h=grid_map.info.height
map_w=grid_map.info.width
data = data.reshape((map_h, map_w))
res=grid_map.info.resolution
x_origin=grid_map.info.origin.position.x
y_origin=grid_map.info.origin.position.y
laser_scan_n=360
L=np.zeros((1,1,laser_scan_n))

laser_max_ang=6.28
laser_min_ang=0
laser_max_r=3.5
angles=np.linspace(laser_min_ang,laser_max_ang,laser_scan_n)
for i in range(1):
    print(i)
    x=i*res+x_origin

    for j in range(1):
        y=j*res+y_origin
        h=0
        for angle in angles:
             r=0
            
             while r<=laser_max_r+res:
                x_P=x+r*np.cos(angle)
                y_P=y+r*np.sin(angle)

                px_grid = int((y_P -y_origin) / res)
                py_grid = int((x_P -x_origin) / res)

                
                if px_grid<map_h and py_grid<map_w and data[px_grid,py_grid]==100:
                  L[i,j,h]=r
                  break
                if r>=laser_max_r:
                  L[i,j,h]=laser_max_r/2
                  break

                r=r+res
             h=h+1

L=np.reshape(L,(1,1*laser_scan_n))
np.savetxt("data.txt", L)


