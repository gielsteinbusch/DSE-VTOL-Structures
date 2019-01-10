# -*- coding: utf-8 -*-
"""
Created on Sun Dec 16 11:56:23 2018

@author: Giel
"""

from airfoil import list_x, list_z
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

radius = 6.
taper = 0.5
chord_length = 1
inc_angle = 0
twist = 0
skin_thickness = 0.005
V_flight = 0
rpm = 286
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9
disc_steps = 6

class Blade_loading:
    def __init__(self, radius, chord_length, taper, skin_thickness, V_flight, rpm, rho, CL, list_x, list_z, LDratio, disc_steps):
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
        self.LDratio = LDratio
        
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
            print(V_total)
            S = (np.pi * distance_center**2) - sum(x)
            x.append(S)
            L = 0.5 * rho * (V_total)**2 * S * CL
            self.lift_list.append(L)
            self.x_list.append(distance_center)
            self.totallift += L
    
    def shear_distribution(self):
        self.shearforce_list = []
        lift_shearforce = 0
        self.totalmoment = 0
        for i in range(len(self.lift_list)):
            lift_shearforce += self.lift_list[i]
            total_shearforce = -self.totallift + lift_shearforce
            self.shearforce_list.append(total_shearforce)
            self.totalmoment += self.lift_list[i]*self.x_list[i]
    
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
    
    def center_gravity(self):
        self.centroids = []
        self.cen_x_list = []
        self.cen_z_list = []
        self.segment_list = []
        for step in range(disc_steps):
            cen_x_list_cs = []
            cen_z_list_cs = []
            segment_list_cs = []
            for i in range(len(self.x_coordinates)-1):
                segment_length = np.sqrt((self.profile_x[step][i+1]-self.profile_x[step][i])**2 + (self.profile_z[step][i+1]-self.profile_z[step][i])**2)
                cen_x = self.profile_x[step][i]+(self.profile_x[step][i+1]-self.profile_x[step][i])/2
                cen_z = self.profile_z[step][i]+(self.profile_z[step][i+1]-self.profile_z[step][i])/2
                segment_list_cs.append(segment_length)
                cen_x_list_cs.append(cen_x) 
                cen_z_list_cs.append(cen_z)
            centroid_x = np.sum(skin_thickness*np.array(segment_list_cs)*np.array(cen_x_list_cs)) / np.sum(skin_thickness*np.array(segment_list_cs))  # np.sum not the problem 
            centroid_z = np.sum(skin_thickness*np.array(segment_list_cs)*np.array(cen_z_list_cs)) / np.sum(skin_thickness*np.array(segment_list_cs))
            self.centroids.append([centroid_x,centroid_z])  
            self.cen_x_list.append(cen_x_list_cs)
            self.cen_z_list.append(cen_z_list_cs)
            self.segment_list.append(segment_list_cs)
        
    def inertia(self):
        self.ix_list = []
        self.iz_list = []
        self.ixz_list = []
        for step in range(disc_steps):
            ix = 0
            iz = 0
            ixz = 0
            for i in range(len(self.x_coordinates)-1):
                iz += (self.segment_list[step][i]*skin_thickness)*(self.cen_x_list[step][i]-self.centroids[step][0])**2
                ix += (self.segment_list[step][i]*skin_thickness)*(self.cen_z_list[step][i]-self.centroids[step][1])**2
                ixz += (self.segment_list[step][i]*skin_thickness)*(self.cen_x_list[step][i]-self.centroids[step][0])*(self.cen_z_list[step][i]-self.centroids[step][1])
            self.ix_list.append(ix)
            self.iz_list.append(iz)
            self.ixz_list.append(ixz)
    
    def area(self):
        self.area_list = []
        for step in range(disc_steps):
            A = 0
            for i in range(len(self.x_coordinates)-1):
                ax,az = self.profile_x[step][i], self.profile_z[step][i]
                bx,bz = self.profile_x[step][i+1], self.profile_z[step][i+1]
                u = np.array([ax,az])
                v = np.array([bx,bz])
                parallelogram = np.cross(u,v)
                dA = 0.5*parallelogram
                A += dA 
            self.area_list.append(A)

    def bending_stress(self):
        self.stress_x_list = []
        self.stress_z_list = []
        self.sigma_list = []
        for step in range(disc_steps):
            stress_x_list_cs = []
            stress_z_list_cs = []
            sigma_list_cs = [1]
            moment_x = self.moment_list[step]
            moment_z = self.moment_list[step]/LDratio
            ix = self.ix_list[step]
            iz = self.iz_list[step]
            ixz = self.ixz_list[step]
            for i in range(len(self.x_coordinates)-1):
                stress_x = ((iz*moment_x - ixz*moment_z) / (ix*iz - ixz**2)) * (self.cen_z_list[step][i] - self.centroids[step][1])
                stress_z = ((ix*moment_z - ixz*moment_x) / (ix*iz - ixz**2)) * (self.cen_x_list[step][i] - self.centroids[step][0])
                sigma = stress_x + stress_z
                stress_x_list_cs.append(stress_x)
                stress_z_list_cs.append(stress_z)
                sigma_list_cs.append(sigma)
            self.stress_x_list.append(stress_x_list_cs)  
            self.stress_z_list.append(stress_z_list_cs)
            self.sigma_list.append(sigma_list_cs)
    
    def shear_stress(self):
        self.tau_list = []
        self.qb_list = []
        self.total_shear_list = []
        for step in range(disc_steps):
            ix = self.ix_list[step]
            iz = self.iz_list[step]
            ixz = self.ixz_list[step]
            Sz = self.shearforce_list[step]
            Sx = Sz / LDratio
            qx_coef = -(Sx*ix - Sz*ixz) / (ix*iz - ixz**2)
            qz_coef = -(Sz*iz - Sx*ixz) / (ix*iz - ixz**2)
            qb = 0
            qb_list_cs = [1]
            internal_moment = 0
            for i in range(len(self.x_coordinates)-1):
                qbx0 = qx_coef * skin_thickness * (self.profile_x[step][i+1] - self.centroids[step][0]) * self.segment_list[step][i]
                qbz0 = qz_coef * skin_thickness * (self.profile_z[step][i+1] - self.centroids[step][1]) * self.segment_list[step][i]
                qb += (qbx0 + qbz0)
                qb_list_cs.append(qb)
                
                qbx = ((self.profile_x[step][i+1] - self.profile_x[step][i]) / self.segment_list[step][i]) *qb
                qbz = ((self.profile_z[step][i+1] - self.profile_z[step][i]) / self.segment_list[step][i]) *qb
                moment_arm_x = self.profile_x[step][i] - 0.25*self.taperchord[step]
                moment_arm_z = self.profile_z[step][i] 
                internal_moment += (qbx * moment_arm_x + qbz * moment_arm_z) * self.segment_list[step][i]
            q0 = internal_moment / ( -2 * self.area_list[step])
            self.qb_list.append(qb_list_cs)
            total_shear_list_cs = [x + q0 for x in qb_list_cs]
            self.total_shear_list.append(total_shear_list_cs)
            tau_list_cs = [x/skin_thickness for x in total_shear_list_cs]
            self.tau_list.append(tau_list_cs)
            
    def von_mises(self):
        self.von_mises = []
        for step in range(disc_steps):
            von_mises_cs = []
            for i in range(len(self.x_coordinates)-1):
                von_mises_segment = np.sqrt(self.sigma_list[step][i]**2 + 3*(self.tau_list[step][i])**2)
                von_mises_cs.append(von_mises_segment)
            self.von_mises.append(von_mises_cs)


blade = Blade_loading(radius, chord_length, taper, skin_thickness, V_flight, rpm, rho, CL, list_x, list_z, LDratio, disc_steps)
blade.lift_distribution()
blade.shear_distribution()
blade.moment_distribution()
blade.twist_taper()
blade.center_gravity()
blade.inertia()
blade.area()
blade.bending_stress()
blade.shear_stress()
blade.von_mises()

for i in range(disc_steps):
    plt.plot(blade.profile_x[i], blade.profile_z[i])

#plotting bend stress
bend = []
for step in range(blade.disc_steps):
    for x in blade.sigma_list[step]:
        bend.append(x)
bend = np.array(bend)

fig = plt.figure()
ax = fig.add_subplot(111, projection ='3d')    
ax.scatter(blade.profile_x, blade.profile_y, blade.profile_z, c = bend, cmap=plt.jet())
plt.show()

for i in range(disc_steps):
    plt.scatter(i, max(blade.tau_list[i]),color='red')
    plt.scatter(i, max(blade.sigma_list[i]),color='blue')
    plt.scatter(i, max(blade.von_mises[i]),color='green')
