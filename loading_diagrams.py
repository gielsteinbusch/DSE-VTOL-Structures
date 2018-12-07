# -*- coding: utf-8 -*-
"""
Shear and Moment diagrams of the rotor and airframe

Created on Mon Dec  3 15:40:42 2018

@author: giel
"""

from math import *
import matplotlib.pyplot as plt
plt.figure(figsize=(9,9))

def lift_rotorcraft(diameter, V_flight, rpm, rho, CL, number_segments):
    x_list = []
    lift_list = []
    width_segment = diameter / number_segments
    for segment in range(number_segments):
         if segment < number_segments/2:
             distance_center = (number_segments/2 - segment)*width_segment
             V_rotor = 2*pi*(rpm/60)*distance_center
             V = V_flight - V_rotor
             x_list.append(-distance_center)
         else:
             distance_center = (segment - number_segments/2)*width_segment
             V_rotor = 2*pi*(rpm/60)*distance_center
             V = V_flight + V_rotor
             x_list.append(distance_center)
         S = pi * distance_center**2 
         L = 0.5 * rho * V**2 * S * CL
         lift_list.append(L)    
    plt.plot(x_list,lift_list)
    plt.grid(True)
    return lift_list, x_list


def drag_rotorcraft(diameter,V_flight, rpm, rho, Cd0, A, e, CL, number_segments): 
    x_list = []
    drag_list = []
    width_segment = diameter / number_segments
    for segment in range(number_segments): 
        if segment < number_segments/2.: 
            distance_center = (number_segments/2. - segment)*width_segment
            V_rotor = 2*pi*(rpm/60)*distance_center
            V = V_flight - V_rotor
            x_list.append(distance_center)
        else:
            distance_center = (segment - number_segments/2)*width_segment
            V_rotor = 2*pi*(rpm/60)*distance_center
            V = V_flight + V_rotor
            x_list.append(distance_center)
        S = pi* distance_center**2
        CD = Cd0 + (CL**2)/(pi*A*e)
        D = 0.5 * rho * V**2 * S * CD 
        drag_list.append(D)
    plt.plot(x_list, drag_list)
    plt.grid(True)
    return drag_list, x_list

def shear_diagram(lift_list, x_list, W_aircraft):
    width_segment = x_list[1]-x_list[0]
    shearforce_list = []
    lift_shearforce = 0
    for i in range(len(lift_list)):
        lift_shearforce += lift_list[i]*width_segment 
        if x_list[i] > 0 : weight_shearforce = -W_aircraft*9.80665
        else: weight_shearforce = 0
        total_shearforce = lift_shearforce + weight_shearforce
        shearforce_list.append(total_shearforce)
    plt.plot(x_list,shearforce_list)
    plt.xlabel('x along span')
    plt.ylabel('Shearforce (N)')
    return 

def moment_diagram(lift_list, x_list, W_aircraft):
    width_segment = x_list[1]-x_list[0]
    moment_list = []
    for i in range(len(lift_list)): 
        moment = 0 
        for j in range(i):
            moment += (lift_list[j]*width_segment) * (x_list[i]-x_list[j])
        if x_list[i] > 0:
            moment += -W_aircraft*9.80665 * x_list[i]
        moment_list.append(moment)
    plt.plot(x_list,moment_list)
    plt.xlabel('x along span')
    plt.ylabel('Moment (N/m)')
    return moment_list
  

####### UPPER ROTOR  
plt.subplot(331)
lift_list, x_list = lift_rotorcraft(5, 50, 150,0.405, 0.5, 1000)

plt.subplot(332)
shear_diagram(lift_list,x_list, 2500)

plt.subplot(333)
moment_diagram(lift_list, x_list, 2500)

####### LOWER ROTOR 
plt.subplot(334)
lift_list, x_list = lift_rotorcraft(5, 50, -150,0.405, 0.5, 1000)

plt.subplot(335)
shear_diagram(lift_list,x_list, 2500)

plt.subplot(336)
moment_diagram(lift_list, x_list, 2500)

###### BOTH ROTORS
plt.subplot(337)
lift_list_up, x_list = lift_rotorcraft(5, 50, 150,0.405, 0.5, 1000)
lift_list_low, x_list = lift_rotorcraft(5, 50, -150,0.405, 0.5, 1000)

plt.subplot(338)
lift_list_tot = []
for i in range(len(lift_list_up)):
    lift_list_tot.append(lift_list_up[i]+lift_list_low[i])
shear_diagram(lift_list_tot,x_list, 2500)

plt.subplot(339)
moment_diagram(lift_list_tot, x_list, 2500)
