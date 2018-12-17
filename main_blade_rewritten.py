# -*- coding: utf-8 -*-
"""
Created on Sun Dec 16 11:56:23 2018

@author: giel
"""

from airfoil import list_x, list_y
import numpy as np
import matplotlib.pyplot as plt

radius = 6.
taper = 1
chord_length = 1
inc_angle = 0
twist = 0
skin_thickness = 0.01
V_flight = 0
rpm = 286
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9
#shear_center = -0.5*0.5*chordlength
disc_steps = 2


class Blade_loading:
    def __init__(self, radius, chord_length, taper, skin_thickness, V_flight, rpm, rho, CL, list_x, list_z, disc_steps):
        self.radius = radius
        self.chord_length = chord_length
        self.taper = taper
        self.V_flight = V_flight
        self.length_ds = np.linspace(0, radius, disc_steps)
        self.taperchord = np.linspace(chord_length, taper*chord_length , disc_steps)
        self.twisting = np.deg2rad(-1* np.linspace(inc_angle, inc_angle- twist , disc_steps))
        self.disc_steps = disc_steps
        self.x_coordinates = np.array(list_x)
        self.z_coordinates = np.array(list_z)
        
    def lift_distribution(self):
        self.x_list = []
        self.lift_list = []
        self.totallift = 0
        x = []
        width_segment = radius / disc_steps
        for segment in range(disc_steps+1):
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
    
    def twist_taper(self):
        self.profile_x = []
        self.profile_z = []
        self.profile_y = []
        for step in range(disc_steps):
            twiz = self.twisting[step]
            c = self.taperchord[step]
            self.profile_x.append((self.x_coordinates*c)*np.cos(twiz) - (self.z_coordinates*c) *np.sin(twiz)) #maybe remove taper*c 
            self.profile_z.append((self.z_coordinates*c)*np.cos(twiz) + (self.x_coordinates*c) *np.sin(twiz))
            for i in range(len(self.x_coordinates)):
                self.profile_y.append(self.length_ds[step])
        return 
    
    def center_gravity(self):
        self.centroids = []
        for step in range(disc_steps):
            self.cen_x_list = []
            self.cen_z_list = []
            self.segment_list = []
            for i in range(len(self.x_coordinates)-1):
                segment_length = np.sqrt((self.profile_x[step][i+1]-self.profile_x[step][i])**2 + (self.profile_z[step][i+1]-self.profile_z[step][i])**2)
                self.segment_list.append(segment_length)
                cen_x = self.profile_x[step][i]+(self.profile_x[step][i+1]-self.profile_x[step][i])/2
                cen_z = self.profile_z[step][i]+(self.profile_z[step][i+1]-self.profile_z[step][i])/2
                self.cen_x_list.append(cen_x) 
                self.cen_z_list.append(cen_z)
            centroid_x = np.sum(skin_thickness*np.array(self.segment_list)*np.array(self.cen_x_list)) / np.sum(skin_thickness*np.array(self.segment_list))  # np.sum not the problem 
            centroid_z = np.sum(skin_thickness*np.array(self.segment_list)*np.array(self.cen_z_list)) / np.sum(skin_thickness*np.array(self.segment_list))
            self.centroids.append([centroid_x,centroid_z])
        print(self.centroids)  
        
    def inertia(self):
        self.ix_list = []
        self.iz_list = []
        self.ixz_list = []
        for step in range(disc_steps):
            ix = 0
            iz = 0
            ixz = 0
            for i in range(len(self.x_coordinates)-1):
                iz += (self.segment_list[i]*skin_thickness)*(self.profile_x[step][i]-self.centroids[step][0])**2
                ix += (self.segment_list[i]*skin_thickness)*(self.profile_z[step][i]-self.centroids[step][1])**2
                ixz += (self.segment_list[i]*skin_thickness)*(self.profile_x[step][i]-self.centroids[step][0])*(self.profile_z[step][i]-self.centroids[step][1])
            self.ix_list.append(ix)
            self.iz_list.append(iz)
            self.ixz_list.append(ixz)
    
    

blade = Blade_loading(radius, chord_length, taper, skin_thickness, V_flight, rpm, rho, CL, list_x, list_y, disc_steps)
blade.lift_distribution()
blade.shear_distribution()
blade.moment_distribution()
blade.twist_taper()
blade.center_gravity()
#blade.inertia()

for i in range(disc_steps):
    plt.plot(blade.profile_x[i], blade.profile_z[i])
    
plt.plot(vline(0.49223247))
