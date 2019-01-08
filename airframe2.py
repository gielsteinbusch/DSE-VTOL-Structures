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

#plt.plot(poslist, momentlist)
#plt.plot(poslist, shearlist)

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

plt.axis([-1.5,1.5,-1.5,1.5])
plt.plot(x_coorlist, y_coorlist)

## adding booms, with equal spacing

x_boomcoor = []
y_boomcoor = []
boom_distance = 0.3
A_boom = 100
dis = 0
for i in range(len(x_coorlist)-1):
    dis += np.sqrt((x_coorlist[i]-x_coorlist[i+1])**2 + (y_coorlist[i]-y_coorlist[i+1])**2)
    if dis > boom_distance or i == 0:
        x_boomcoor.append(x_coorlist[i])
        y_boomcoor.append(y_coorlist[i])
        dis = 0

plt.scatter(x_boomcoor, y_boomcoor)

cen_y = np.average(y_boomcoor)   