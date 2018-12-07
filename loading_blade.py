# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 16:01:28 2018

@author: giel
"""

from math import *
import matplotlib.pyplot as plt
plt.figure(figsize=(9,9))

def lift_rotorcraft(radius, V_flight, rpm, rho, CL, number_segments):
    x_list = []
    lift_list = []
    width_segment = radius / number_segments
    totallift = 0
    for segment in range(number_segments):
        distance_center = segment*width_segment
        V_rotor = 2*pi*(rpm/60)*distance_center
        V = V_flight + V_rotor
        S = pi * distance_center**2
        L = 0.5 * rho * V**2 * S * CL
        lift_list.append(L)
        x_list.append(distance_center)
        totallift = totallift + L*width_segment
    print(totallift)
    return lift_list, x_list, totallift

lift_list, x_list, totallift = lift_rotorcraft(6,50,150,0.5,0.5,1000)

def shear_diagram(totallift, lift_list, x_list):
    width_segment = x_list[1]-x_list[0]
    shearforce_list = []
    lift_shearforce = 0
    totalmoment = 0
    for i in range(len(lift_list)):
        lift_shearforce += lift_list[i]*width_segment 
        total_shearforce = -totallift + lift_shearforce 
        shearforce_list.append(total_shearforce)
        totalmoment = totalmoment + lift_shearforce*width_segment*x_list[i]
    plt.plot(x_list,shearforce_list)
    plt.xlabel('x along span')
    plt.ylabel('Shearforce (N)')
    return totalmoment

totalmoment = shear_diagram(totallift, lift_list, x_list)

def moment_diagram(lift_list, x_list, totalmoment):
    width_segment = x_list[1]-x_list[0]
    moment_list = []
    for i in range(len(x_list)): 
        moment = -totallift 
        for j in range(i):
            moment += (lift_list[j]*width_segment) * (x_list[i]-x_list[j])
        moment_list.append(moment)
    plt.plot(x_list,moment_list)
    plt.xlabel('x along span')
    plt.ylabel('Moment (N/m)')
    return moment_list

moment_diagram(totallift, lift_list, x_list)