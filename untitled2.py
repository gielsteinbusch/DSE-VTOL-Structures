# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 10:10:12 2018

@author: giel
"""
import numpy as np
from airfoil import list_x, list_y
from loading_blade import lift_rotorcraft, shear_diagram, moment_diagram
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## input values
radius = 6.
taper = 0.5
chordlength = 0.35
inc_angle = 5
twist = 5
disc_steps = 10
skin_thickness = 0.003
V_flight = 50
rpm = 210
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9

## import moment distribution
lift_list, x_list, totallift = lift_rotorcraft(radius,V_flight, rpm, rho, CL, disc_steps)
totalmoment, shearforce_list = shear_diagram(totallift, lift_list, x_list)
moment_list = moment_diagram(lift_list, x_list, totallift, totalmoment)

x_coordinates = np.array(list_x)
z_coordinates = np.array(list_y)


length_ds = np.linspace(0, radius, disc_steps)
taperchord = np.linspace(chordlength, taper*chordlength , disc_steps)
twisting = -1* np.linspace(inc_angle, inc_angle- twist , disc_steps)
twisting = np.deg2rad(twisting)

profilex = []
profilez = []
profiley = []
ix_list = []
iz_list = []
count = 0
plt.figure()
centroids = []
listmaxbendstress = []
for c in taperchord: 
    # Calculate twist alang span
    twiz = twisting[count]
    profilex.append((x_coordinates*c - taper*c)*np.cos(twiz) - (z_coordinates*c)           *np.sin(twiz)) #maybe remove taper*c 
    profilez.append((z_coordinates*c)          *np.cos(twiz) + (x_coordinates*c - taper*c) *np.sin(twiz))
    
    # Applying taper ratio
    for i in range(len(x_coordinates)):
        profiley.append(length_ds[list(taperchord).index(c)])
    plt.plot(profilex[count],profilez[count])
    
    # Calculate center of gravity
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
    
    #Calculate moment of Inertia
    ix = 0
    iz = 0
    for i in range(len(x_coordinates)-1):
        ix += (segment_list[i]*skin_thickness)*(profilex[count][i]-centroids[count][0])**2
        iz += (segment_list[i]*skin_thickness)*(profilez[count][i]-centroids[count][1])**2
    ix_list.append(ix)
    iz_list.append(iz)
    
    #Calculate bending stess
    stress_list_x = []
    stress_list_z = []
    moment_x = moment_list[count]
    moment_z = moment_list[count]/LDratio
    sigmaprofile = []  #stress in each cross section
    for i in range(len(x_coordinates)-1):
        stress_x = moment_x * cen_z_list[i] / ix
        stress_list_x.append(stress_x)
        stress_z = moment_z * cen_z_list[i] /iz
        stress_list_z.append(stress_z)
        sigma = stress_x + stress_z 
        sigmaprofile.append(sigma)
    maxbend_stress = max(sigmaprofile)
    listmaxbendstress.append(maxbend_stress)
    count = count + 1
    
print(listmaxbendstress)
#print(centroids)

plt.figure()
plt.plot(length_ds, listmaxbendstress)
    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
   
ax.scatter(profilex, profilez,profiley)

#print(twisting)
