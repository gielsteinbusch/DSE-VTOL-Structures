# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 10:10:12 2018

@author: giel
"""
import numpy as np
from airfoil import list_x, list_y
from loading_diagrams import lift_rotorcraft, moment_diagram
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## input values
radius = 6.
taper = 0.5
chordlength = 0.35
inc_angle = 5
twist = 5
disc_steps = 10
skin_thickness = 0.001
V_flight = 50
rpm = 150
rho = 0.5
CL = 0.5
W_aircraft = 2500


lift_list, x_list = lift_rotorcraft(2*radius, V_flight, rpm, rho, CL, disc_steps)
moment_list = moment_diagram(lift_list,x_list, W_aircraft)


x_coordinates = np.array(list_x)
z_coordinates = np.array(list_y)

#plt.plot(list_x,list_y)
length_ds = np.linspace(0, radius + radius/disc_steps, disc_steps)

taperchord = np.linspace(chordlength, taper*chordlength + taper*chordlength/disc_steps, disc_steps)

twisting = -1* np.linspace(inc_angle, inc_angle- twist + (inc_angle-twist)/disc_steps, disc_steps)
twisting = np.deg2rad(twisting)
#
profilex = []
profilez = []
profiley = []
count = 0
plt.figure()
centroids = []
for c in taperchord: 
    twiz = twisting[count]
    profilex.append((x_coordinates*c - taper*c)*np.cos(twiz) - (z_coordinates*c)           *np.sin(twiz)) #maybe remove taper*c 
    profilez.append((z_coordinates*c)          *np.cos(twiz) + (x_coordinates*c - taper*c) *np.sin(twiz))
    for i in range(len(x_coordinates)):
        profiley.append(length_ds[list(taperchord).index(c)])
    plt.plot(profilex[count],profilez[count])
    cen_x_list = []
    cen_z_list = []
    segment_list = []
    for i in range(len(x_coordinates)-1):
        segment_length = np.sqrt((profilex[count][i+1]-profilex[count][i])**2 + (profilez[count][i+1]-profilez[count][i])**2)
        segment_list.append(segment_length)
        cen_x = profilex[count][i]+(profilex[count][i+1]-profilex[count][i])/2
        cen_z = profilez[count][i]+(profilez[count][i+1]-profilez[count][i])/2
        cen_x_list.append(cen_x)
        cen_z_list.append(cen_z)
        
    centroid_x = np.sum(skin_thickness*np.array(segment_list)*np.array(cen_x_list)) / np.sum(skin_thickness*np.array(segment_list)) 
    centroid_z = np.sum(skin_thickness*np.array(segment_list)*np.array(cen_z_list)) / np.sum(skin_thickness*np.array(segment_list))
    centroids.append([centroid_x,centroid_z])
    
    ix = 0
    iz = 0
    for i in range(len(x_coordinates)-1):
        ix += (segment_list[i]*skin_thickness)*(profilex[count][i]-centroids[count][0])**2
        iz += (segment_list[i]*skin_thickness)*(profilez[count][i]-centroids[count][1])**2
    #print(ix,iz)
    count = count + 1
    
    
#print(centroids)


    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
   
ax.scatter(profilex, profilez,profiley)

#print(twisting)
