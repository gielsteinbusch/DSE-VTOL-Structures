# -*- coding: utf-8 -*-
"""
Created on Fri Dec 21 10:34:49 2018

@author: archi
"""

import numpy as np 
#import matplotlib.pyplot as plt 
from sparwars import Blade_loading 
from airfoil import list_x, list_z
from sympy.solvers import solve
from sympy import Symbol

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
skin_thickness = 0.01

disc_steps = 50               # MUST MATCH THE ONE IN THE SPARWARS FILE 


#material properities 
den = 2780
E = 73.1e9
Uten = 345e8

## input values per segment
CL = list(np.linspace(0.5,0.7,disc_steps))
t_skin = list(np.linspace(0.02, 0.01, disc_steps))
m_seg = list(np.linspace(40, 20, disc_steps))

y_coor = np.linspace(0, radius, disc_steps)
taperchord = np.linspace(chord_length, taper*chord_length , disc_steps)
twisting = np.deg2rad(-1* np.linspace(inc_angle, inc_angle- twist , disc_steps))
x_coordinates = np.array(list_x)
z_coordinates = np.array(list_z)

####---------------------from before, there is a possible confusion about y_coor[step] in line 52 (S_segment)
def lift(y_coor, V_flight, rho, CL, m_seg): 
    lift_list = []
    F_cen_list = []
    S_list = []
    w_segment = y_coor[1]-y_coor[0]
    for step in range(disc_steps): 
        V_blade = 2*np.pi*(rpm/60)*(y_coor[step] + 2/3 * w_segment)
        V_total = V_blade + V_flight
        S_segment = (np.pi * y_coor[step]**2) - sum(S_list)
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

#d = Symbol('d')
#w_segment = y_coor[1]-y_coor[0]
#M = F_cen_list[-1]*d - lift_list[-1]*2/3 * w_segment
#E = 700*10**6
#I = 2.0205684323392292e-05
#d = solve( (-M / (E*I)) * w_segment* 2 - d, d)
#print(d)
blade = Blade_loading(radius, chord_length, taper, skin_thickness, V_flight, rpm, rho, CL, list_x, list_z, LDratio, disc_steps)
blade.lift_distribution()
blade.shear_distribution()
blade.moment_distribution()
blade.profile()
blade.spar_coor()
blade.profile_new()
blade.twist()
blade.center_gravity()
blade.inertia()
blade.area()
#blade.bending_stress()
#blade.shear_stress()
#blade.max_bend()
#A0 = blade.area_list[0][0]    ---- NEED TO USE AREA OF CROSS SECTION (STRUCTURAL AREA)
def centrifugal_force(rpm,disc_steps,y_coor):
    CSAlist = []
    siglist = []
    mlist = []
    ylist= []
    mass = 0 
    centrifugal = []
    wrs = (rpm*2*np.pi)/60
    w_segment = y_coor[1]-y_coor[0]
    for i in range(disc_steps-1):
        Aspar = (blade.area_spar[i] + blade.area_spar[i+1])/2
        Askin = ((sum(blade.segment_list[i]) + sum(blade.segment_list[i+1]))*skin_thickness)/2 
        CSA = Aspar + Askin
        CSAlist.append(CSA)
        m = CSA*den*w_segment
        mass +=m
        #print(m)
        ypoint = (y_coor[i] + y_coor[i+1])/2
        ylist.append(ypoint)
        centri = den*CSA*((ypoint**2)/2)*wrs**2   
        centrifugal.append(centri)
        sigi = den*((ypoint**2)/2)*wrs**2        
        #sigi = Ni/CSA
        siglist.append(sigi)
    return CSA, centrifugal, siglist, mass 

CSA, centrifugal, siglist, mass = centrifugal_force(rpm,disc_steps,y_coor)

print(siglist)
print(centrifugal)

plt.plot(ylist,siglist)
plt.plot(ylist,centrifugal)
print(mass)