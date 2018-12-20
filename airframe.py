# -*- coding: utf-8 -*-
"""
Created on Thu Dec 20 09:12:07 2018

@author: archi
"""
import numpy as np 
import matplotlib.pyplot as plt 
from sparwars import Blade_loading
fig = plt.figure()
ax = fig.add_subplot(111, projection ='3d')

disc_cs = 50
cabin_len = 6. 
cabin_height = 1.5
cabin_width = 1.5  
door_height = 1.2 
no_step_door = int((cabin_height-door_height)/cabin_height * disc_cs)

#building the cabin as a box 
xdis = np.linspace(0,cabin_len,disc_cs)
zdis1 = []
ydis1 = []
xlist1 = []
zdis2 = []
ydis2 = []
xlist2 = []
zdis3 = []
ydis3 = []
xlist3 = []
zdis4 = []
ydis4 = []
xlist4 = []
for x in xdis: 
    y1 = np.linspace(0,cabin_width,disc_cs)
    y2 = cabin_width*np.ones(disc_cs)
    y3 = np.linspace(cabin_width,0,disc_cs)
    y4 = np.zeros(disc_cs)
    z1 = cabin_height*np.ones(disc_cs)
    z2 = np.linspace(0, cabin_height, disc_cs)
    z3 = np.zeros(disc_cs)
    z4 = np.linspace(cabin_height,0,disc_cs)
    xstep = np.ones(disc_cs)*x
    xstep4 = np.ones(disc_cs)*x
    if x >= 2  and x <= 4: 
        z5 = np.linspace(0,(cabin_height-door_height)/2,int(disc_cs/2))
        z6 = np.linspace(cabin_height - (cabin_height-door_height)/2, cabin_height, int(disc_cs/2 ))
        z4 = np.concatenate((z5,z6))
        y4 = np.zeros(disc_cs)
        xstep4 = np.ones(disc_cs)*x
        #print(len(z4), len(y4), len(xstep4))
    ydis1.append(y1)
    xlist1.append(xstep)
    zdis1.append(z1)
    ydis2.append(y2)
    xlist2.append(xstep)
    zdis2.append(z2)
    ydis3.append(y3)
    zdis3.append(z3)
    xlist3.append(xstep)
    ydis4.append(y4)
    zdis4.append(z4)
    xlist4.append(xstep4)
    
ax.scatter(xlist1,ydis1,zdis1)
ax.scatter(xlist2,ydis2,zdis2)
ax.scatter(xlist3,ydis3,zdis3)
ax.scatter(xlist4,ydis4,zdis4)
plt.show()