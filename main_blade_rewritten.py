# -*- coding: utf-8 -*-
"""
Created on Sun Dec 16 11:56:23 2018

@author: giel
"""

from airfoil import list_x, list_y
import numpy as np

radius = 6.
taper = 0.5
chord_length = 0.35
inc_angle = 0
twist = 0
disc_steps = 50
skin_thickness = 0.005
V_flight = 0
rpm = 286
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9
shear_center = -0.5*0.5*chordlength
number_segments = 100


class Blade_loading:
    def __init__(self, radius, chord_length, V_flight, rpm, rho, CL, number_segments):
        self.radius = radius
        self.chord_length = chord_length
        self.V_flight = V_flight
    
    def lift_distribution(self):
        self.x_list = []
        self.lift_list = []
        self.totallift = 0
        x = []
        width_segment = radius / number_segments
        for segment in range(number_segments+1):
            distance_center = segment*width_segment
            V_rotor = 2*np.pi*(rpm/60)*distance_center
            V_total = V_flight + V_rotor
            S = (np.pi * distance_center**2) - sum(x)
            x.append(S)
            L = 0.5 * rho * (V_total)**2 * S * CL
            self.lift_list.append(L)
            self.x_list.append(distance_center)
            self.totallift += L
        return self.lift_list, self.x_list, self.totallift
    
    def shear_distribution(self):
        self.shearforce_list = []
        lift_shearforce = 0
        self.totalmoment = 0
        for i in range(len(self.lift_list)):
            lift_shearforce += self.lift_list[i]
            total_shearforce = -self.totallift + lift_shearforce
            self.shearforce_list.append(total_shearforce)
            self.totalmoment += self.lift_list[i]*self.x_list[i]
        return self.totalmoment, self.shearforce_list
    
    def moment_distribution(self):
        self.moment_list = []
        moment = 0
        for i in range(len(self.x_list)):
            Mfres = self.totallift * self.x_list[i]
            Mtotm = self.totalmoment
            Mlif = 0
            for j in range(i):
                Mlif += self.lift_list[j]*(self.x_list[i]-self.x_list[j])
            moment = Mtotm - Mfres + Mlif
            self.moment_list.append(moment)
        return self.moment_list   

blade = Blade_loading(radius, chord_length, V_flight, rpm, rho, CL, number_segments)
blade.lift_distribution()
blade.shear_distribution()
blade.moment_distribution()
