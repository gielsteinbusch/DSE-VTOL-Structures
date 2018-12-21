# -*- coding: utf-8 -*-
"""
Created on Fri Dec 21 10:34:49 2018

@author: archi
"""

import numpy as np 
import matplotlib.pyplot as plt 
from sparwars import Blade_loading 
from airfoil import list_x, list_z

radius = 6.
taper = 0.5
chord_length = 1
inc_angle = 10
twist = 20
skin_thickness = 0.01
V_flight = 0
rpm = 286
rho = 0.5
W_aircraft = 2500
LDratio = 9
disc_steps = 6
G = 28e9
no_blades = 3 

length_ds = np.linspace(0, radius, disc_steps)
taperchord = np.linspace(chord_length, taper*chord_length , disc_steps)
twisting = np.deg2rad(-1* np.linspace(inc_angle, inc_angle- twist , disc_steps))
CL = np.linspace(0.5,0.7,disc_steps)
x_coordinates = np.array(list_x)
z_coordinates = np.array(list_z)

def lift(length_ds, V_flight, rho,CL): 
    lift_list = []
    lift_points = []
    for t in range(disc_steps - 1): 
        seg_width = length_ds[1]-length_ds[0]
        Voutpos = 2*np.pi*(rpm/60)*(length_ds[t+1] - length_ds[0])
        L = 0.5*rho*list(CL)[t]*(Voutpos + V_flight)**2 * (np.pi*(length_ds[t+1])**2 - np.pi*(length_ds[t])**2)
        lift_list.append(L)
        lift_point = length_ds[t] + (2/3)*(seg_width) 
        lift_points.append(lift_point)
    return lift_list , lift_points

lift_list, lift_points = lift(length_ds, V_flight, rho,CL)
def moment(lift_list, lift_points): 
    ext_moment = []
    for i in range(disc_steps -1 ):
        moment = lift_list[i]*lift_points[i]
        ext_moment.append(moment)
    res_moment = sum(ext_moment)
    return ext_moment, res_moment 
print(lift_list,lift_points)
print(moment(lift_list,lift_points))
        




