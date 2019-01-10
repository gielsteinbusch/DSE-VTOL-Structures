# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 13:01:51 2019

@author: giel
"""

import numpy as np
import matplotlib.pyplot as plt

#import values
step = 1. / 0.1 #second number is step length (m)
L_airframe = 5.8
W_airframe = 1.35
H_airframe = 1.8
E = 700*10**6
OEW = 1400.
boom_distance = 0.25
A_stringer = 0.0001
t_skin = 0.001

## forces with their distances to the nose tip
F_weight =    [-OEW*9.81          	     , 3.9]
F_mainrotor = [0.9*abs(F_weight[0])    , 3.5]
F_horstab =   [0.2*abs(F_weight[0])    , 5. ]
F_heist =     [-0.1*abs(F_weight[0])    , 5.7]

Forces = [F_weight, F_mainrotor, F_horstab, F_heist]

momentlist = []
shearlist = []

poslist = []
for pos in np.linspace(0 , L_airframe , int(L_airframe*step)+1):
    shear = 0
    moment = 0
    for i in range(len(Forces)):
        shear += Forces[i][0] * np.heaviside(pos - Forces[i][1], 0.5)
        moment += Forces[i][0] * (pos - Forces[i][1]) * np.heaviside(pos - Forces[i][1], 0.5)
    
    momentlist.append(moment)
    shearlist.append(shear)
    poslist.append(pos)

plt.plot(poslist, momentlist)
plt.plot(poslist, shearlist)

maxshear = max(shearlist)
maxmoment = max(momentlist)

a = W_airframe/2
b = H_airframe/2 + 0.2
thetalist = np.linspace(90,270,180)/180*np.pi


## creating the profile, only the left side 
x_coorlist = []
y_coorlist = []
for theta in thetalist:
    r = (a*b) / np.sqrt(b**2 * np.cos(theta)**2 + a**2 * np.sin(theta)**2)
    if r*np.sin(theta) < -H_airframe/2 + 0.2: y_coorlist.append(-H_airframe/2 + 0.2)
    else: y_coorlist.append(r*np.sin(theta))
    x_coorlist.append(r*np.cos(theta))

## adding booms, with equal spacing

x1_boomcoor = []
y1_boomcoor = []
xm_boomcoor = []
ym_boomcoor = []
dis = 0
for i in range(len(x_coorlist)-1):
    dis += np.sqrt((x_coorlist[i]-x_coorlist[i+1])**2 + (y_coorlist[i]-y_coorlist[i+1])**2)
    if dis > boom_distance or i == 0:
        x1_boomcoor.append(x_coorlist[i])
        xm_boomcoor.append(-x_coorlist[i])
        y1_boomcoor.append(y_coorlist[i])
        ym_boomcoor.append(y_coorlist[i])
        dis = 0
x1_boomcoor.reverse()
y1_boomcoor.reverse()
x_boomcoor = x1_boomcoor + xm_boomcoor[1:]
y_boomcoor = y1_boomcoor + ym_boomcoor[1:]

# calculate centroids (cen_x will obviously be 0)
cen_y = np.average(y_boomcoor)   
cen_x = np.average(x_boomcoor)

# calculating boom areas
boom_area_list = []
for i in range(len(x_boomcoor)):
    if i == 0:
        frac1 = (y_boomcoor[-1]-cen_y) / (y_boomcoor[i]-cen_y)
        frac2 = (y_boomcoor[i+1]-cen_y) / (y_boomcoor[i]-cen_y)
    elif i == len(x_boomcoor)-1:
        frac1 = (y_boomcoor[i-1]-cen_y) / (y_boomcoor[i]-cen_y)
        frac2 = (y_boomcoor[0]-cen_y) / (y_boomcoor[i]-cen_y)
    else:
        frac1 = (y_boomcoor[i-1]-cen_y) / (y_boomcoor[i]-cen_y)
        frac2 = (y_boomcoor[i+1]-cen_y) / (y_boomcoor[i]-cen_y)
    A_boom = A_stringer + t_skin*boom_distance/6 * ((2 + frac1) + (2 + frac2))
    boom_area_list.append(A_boom)
    
## calculate moment of inertia
Ixx = 0
Iyy = 0 
for i in range(len(x_boomcoor)):
    Ixx += (y_boomcoor[i]-cen_y)**2 * boom_area_list[i]
    Iyy += (x_boomcoor[i]-cen_x)**2 * boom_area_list[i]

## calculate bending stress
sigma_list = []
for i in range(len(x_boomcoor)):
    sigma_list += [maxmoment / Ixx * (y_boomcoor[i] - cen_y)]
    
## area calculation
A = 0
for i in range(len(x_boomcoor)):
    ax, az = x_boomcoor[i], y_boomcoor[i]
    if i == len(x_boomcoor)-1:
        bx, bz = x_boomcoor[0], y_boomcoor[0]
    else: 
        bx, bz = x_boomcoor[i+1], y_boomcoor[i+1]
    u = np.array([ax,az])
    v = np.array([bx,bz])
    A += 0.5*np.cross(v,u)
print(A)
    
## calculate shear stress
q_base_list = []
q_base = 0
q_moment = 0
for i in range(len(x_boomcoor)):
    q_base += -(maxshear/Ixx) * boom_area_list[i] * (y_boomcoor[i] - cen_y)
    if i == len(x_boomcoor)-1:
        q_moment += -q_base*(x_boomcoor[0]-x_boomcoor[i])*(x_boomcoor[i]-cen_x) +\
                    q_base*(y_boomcoor[0]-y_boomcoor[i])*(y_boomcoor[i]-cen_y)
    else:
        q_moment += -q_base*(x_boomcoor[i+1]-x_boomcoor[i])*(x_boomcoor[i]-cen_x) +\
                    q_base*(y_boomcoor[i+1]-y_boomcoor[i])*(y_boomcoor[i]-cen_y)
    q_base_list.append(q_base)
q_red = -q_moment / (2*A)
q_tot_list = [x+q_red for x in q_base_list]
tau_list = [x/t_skin for x in q_tot_list]

## results
#plt.axis([-1.5,1.5,-1.5,1.5])
#plt.plot(x_coorlist, y_coorlist)
#plt.scatter(x_boomcoor, y_boomcoor)
#plt.hlines(cen_y,-1,1)
print('max sigma:', max(sigma_list), ', max tau:', max(tau_list))