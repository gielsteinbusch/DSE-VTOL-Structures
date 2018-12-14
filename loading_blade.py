# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 16:01:28 2018

@author: giel
"""

from math import *
import matplotlib.pyplot as plt

def lift_rotorcraft(radius, V_flight, rpm, rho, CL, number_segments):
   x_list = []
   lift_list = []
   width_segment = radius / number_segments
   totallift = 0
   S = 0
   x = []
   for segment in range(number_segments+1):
       distance_center = segment*width_segment
       V_rotor = 2*pi*(rpm/60)*distance_center
       V = V_flight + V_rotor
       S = (pi * distance_center**2) - sum(x)
       x.append(S)
       L = 0.5 * rho * V**2 * S * CL
       lift_list.append(L)
       x_list.append(distance_center)
       totallift = totallift + L
   return lift_list, x_list, totallift

def shear_diagram(totallift, lift_list, x_list):
   shearforce_list = []
   lift_shearforce = 0
   totalmoment = 0
   for i in range(len(lift_list)):
       lift_shearforce += lift_list[i]
       total_shearforce = -totallift + lift_shearforce
       shearforce_list.append(total_shearforce)
       totalmoment = totalmoment + lift_list[i]*x_list[i]
   return totalmoment, shearforce_list


def moment_diagram(lift_list, x_list, totallift, totalmoment):
   moment_list = []
   moment = 0
   for i in range(len(x_list)):
       Mfres = totallift * x_list[i]
       Mtotm = totalmoment
       Mlif = 0
       for j in range(i):
            Mlif += lift_list[j]*(x_list[i]-x_list[j])
       moment = Mtotm - Mfres + Mlif
       moment_list.append(moment)
   return moment_list


#lift_list, x_list, totallift = lift_rotorcraft(6,50,150,0.5,0.5,1000)
#totalmoment, shearforce_list = shear_diagram(totallift, lift_list, x_list)
#moment_list = moment_diagram(lift_list, x_list, totallift, totalmoment)
#
#plt.figure(figsize=(9,9))
#plt.plot(x_list, shearforce_list)
#plt.plot(x_list, moment_list)

