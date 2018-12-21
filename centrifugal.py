# -*- coding: utf-8 -*-
"""
Created on Fri Dec 21 10:34:49 2018

@author: archi
"""

import numpy as np 
#import matplotlib.pyplot as plt 
#from sparwars import Blade_loading 
from airfoil import list_x, list_z

radius = 6.
taper = 0.5
chord_length = 1
inc_angle = 10
twist = 20
V_flight = 0
rpm = 286
rho = 0.5
W_aircraft = 2500
LDratio = 9
G = 28e9
circ = 2.04 

disc_steps = 6

## input values per segment
CL = list(np.linspace(0.5,0.7,disc_steps))
t_skin = list(np.linspace(0.02, 0.01, disc_steps))
m_seg = list(np.linspace(40, 20, disc_steps))

y_coor = np.linspace(0, radius, disc_steps+1)
taperchord = np.linspace(chord_length, taper*chord_length , disc_steps)
twisting = np.deg2rad(-1* np.linspace(inc_angle, inc_angle- twist , disc_steps))
x_coordinates = np.array(list_x)
z_coordinates = np.array(list_z)

def lift(y_coor, V_flight, rho, CL, m_seg): 
    lift_list = []
    F_cen_list = []
    S_list = []
    w_segment = y_coor[1]-y_coor[0]
    for step in range(disc_steps): 
        V_blade = 2*np.pi*(rpm/60)*(y_coor[step] + 2/3 * w_segment)
        V_total = V_blade + V_flight
        S_segment = (np.pi * y_coor[step+1]**2) - sum(S_list)
        S_list.append(S_segment)
        L = 0.5 * rho * V_total**2 * S_segment * CL[step]
        lift_list.append(L)
     
        F_cen =  m_seg[step] * V_blade**2 / (y_coor[step] + 1/2 * w_segment)
        F_cen_list.append(F_cen)
    return lift_list, F_cen_list

lift_list, F_cen_list = lift(y_coor, V_flight, rho,CL, m_seg)

def moment(lift_list, y_coor): 
    moment_list = []
    w_segment = y_coor[1]-y_coor[0]
    lift_points = y_coor + 2/3 * w_segment
    for step in range(disc_steps):
        moment = 0
        for force in range(len(lift_list)):
            moment += lift_list[force] * (lift_points[force] - y_coor[step])
            print(lift_points[force] - y_coor[step])
        moment_list.append(moment)
    return moment_list
        

print(moment(lift_list,y_coor))
        




