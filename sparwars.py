# -*- coding: utf-8 -*-
"""
Created on Tue Dec 18 14:15:45 2018

@author: archi
"""

from airfoil import list_x, list_z
import numpy as np
import matplotlib.pyplot as plt

radius = 6.
taper = 0.5
chord_length = 1
inc_angle = 10
twist = 20
skin_thickness = 0.01
V_flight = 0
rpm = 286
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9
disc_steps = 5
G = 28e9 #needs to be changed for the material  

#one spar at the maximum camber location 


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
        self.list_z = list_z
        self.list_x = list_x
        
        
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
    
    def profile(self):
        self.profile_x = []
        self.profile_z = []
        self.profile_y = []
        for step in range(disc_steps):
            c = self.taperchord[step]
            self.profile_x.append(self.x_coordinates*c)  
            self.profile_z.append(self.z_coordinates*c) 
            for i in range(len(self.x_coordinates)):
                self.profile_y.append(self.length_ds[step])
    
    # adding spar coordinates
    def spar_coor(self):
        self.xspar_list = []
        self.zspar_list = []
        self.len_spar = []
        self.area_spar = []
        self.cen_spar_z = []
        self.loc_spar = []
        for step in range(disc_steps):
            self.t_spar = 0.02
            loc_spar_cs = 0.3*self.taperchord[step]            
            x_spar = list((np.ones(int((len(list_x)- 1)/4)))*(loc_spar_cs))
            z_spar = list(np.linspace(max(self.profile_z[step]), min(self.profile_z[step]),int((len(self.profile_x[step])-1)/4) ))
            len_spar_cs = max(z_spar)-min(z_spar)
            area_spar_cs = len_spar_cs*self.t_spar
            cen_spar_z_cs = len_spar_cs/2
            self.xspar_list.append(x_spar)
            self.zspar_list.append(z_spar)
            self.loc_spar.append(loc_spar_cs)
            self.len_spar.append(len_spar_cs)
            self.area_spar.append(area_spar_cs)
            self.cen_spar_z.append(cen_spar_z_cs)
            
    def profile_new(self):
        self.profile1_x = []
        self.profile2_x = []
        self.profile1_z = []
        self.profile2_z = []
        self.segment1_list = []
        self.segment2_list = []
        for step in range(disc_steps):
            a = float(self.xspar_list[step][0])
            profile1_x_cs = []
            profile2_x_cs = []
            profile1_z_cs = []
            profile2_z_cs = []
            self.count_top1 = 0
            for c in range(len(self.profile_x[step])):
                if self.profile_x[step][c] > a:
                    profile1_x_cs.append(self.profile_x[step][c])
                    profile1_z_cs.append(self.profile_z[step][c])
                else: 
                    if profile1_x_cs[-1] != a:
                        profile2_x_cs.append(profile1_x_cs[-1])
                        profile2_z_cs.append(profile1_z_cs[-1])
                        self.idxtop2 = len(profile1_x_cs) -1
                        for i in range(len(self.xspar_list[step])):
                            profile1_x_cs.append(self.xspar_list[step][i])
                            profile1_z_cs.append(self.zspar_list[step][i])
                        self.idxbottom2 = len(profile1_x_cs) 
                        #print(self.idxtop2, self.idxbottom2)
                    profile2_x_cs.append(self.profile_x[step][c])
                    profile2_z_cs.append(self.profile_z[step][c])
            for i in range(1,len(self.xspar_list[step])+1):
                profile2_x_cs.append(self.xspar_list[step][-i])
                profile2_z_cs.append(self.zspar_list[step][-i])
            self.profile1_x.append(profile1_x_cs)
            self.profile2_x.append(profile2_x_cs)
            self.profile1_z.append(profile1_z_cs)
            self.profile2_z.append(profile2_z_cs)
            segment1_list_cs = []
            segment2_list_cs = []
            for i in range(len(profile1_x_cs)-1):
                segment_length1 = np.sqrt((profile1_x_cs[i+1]-profile1_x_cs[i])**2 + (profile1_z_cs[i+1]-profile1_z_cs[i])**2)
                segment1_list_cs.append(segment_length1)
            for i in range(len(profile2_x_cs)-1):
                segment_length2 = np.sqrt((profile2_x_cs[i+1]-profile2_x_cs[i])**2 + (profile2_z_cs[i+1]-profile2_z_cs[i])**2)
                segment2_list_cs.append(segment_length2)
            self.segment1_list.append(segment1_list_cs)
            self.segment2_list.append(segment2_list_cs)

    def twist(self):
        self.profile3_x = []
        self.profile4_x = []
        self.profile3_z = []
        self.profile4_z = []
        for step in range(disc_steps):
            self.profile3_x_cs = []
            self.profile4_x_cs = []
            self.profile3_z_cs = []
            self.profile4_z_cs = []
            twiz = self.twisting[step]
            for i in range(len(self.profile1_x[step])):
                self.profile3_x_cs.append(self.profile1_x[step][i]*np.cos(twiz) - self.profile1_z[step][i]*np.sin(twiz)) 
                self.profile3_z_cs.append(self.profile1_z[step][i]*np.cos(twiz) + self.profile1_x[step][i]*np.sin(twiz))
            for i in range(len(self.profile2_x[step])):
                self.profile4_x_cs.append(self.profile2_x[step][i]*np.cos(twiz) - self.profile2_z[step][i]*np.sin(twiz)) 
                self.profile4_z_cs.append(self.profile2_z[step][i]*np.cos(twiz) + self.profile2_x[step][i]*np.sin(twiz))
            self.profile3_x.append(self.profile3_x_cs)
            self.profile3_z.append(self.profile3_z_cs)
            self.profile4_x.append(self.profile4_x_cs)
            self.profile4_z.append(self.profile4_z_cs)
    
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
            centroid_x = (np.sum(skin_thickness*np.array(segment_list_cs)*np.array(cen_x_list_cs)) + (self.area_spar[step]*self.loc_spar[step]))   / (np.sum(skin_thickness*np.array(segment_list_cs)) + self.area_spar[step])  
            centroid_z = (np.sum(skin_thickness*np.array(segment_list_cs)*np.array(cen_z_list_cs)) + (self.area_spar[step]*self.cen_spar_z[step])) / (np.sum(skin_thickness*np.array(segment_list_cs)) + self.area_spar[step])
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
            twiz = self.twisting[step]
            for i in range(len(self.x_coordinates)-1):
                iz += (self.segment_list[step][i]*skin_thickness)*(self.cen_x_list[step][i]-self.centroids[step][0])**2
                ix += (self.segment_list[step][i]*skin_thickness)*(self.cen_z_list[step][i]-self.centroids[step][1])**2
                ixz += (self.segment_list[step][i]*skin_thickness)*(self.cen_x_list[step][i]-self.centroids[step][0])*(self.cen_z_list[step][i]-self.centroids[step][1])
            ix_spar = (1/12) * self.t_spar * (self.len_spar[step])**3 + self.area_spar[step] * (self.cen_spar_z[step] - self.centroids[step][1])**2
            iz_spar = (1/12) * (self.t_spar)**3 * self.len_spar[step] + self.area_spar[step] * (self.loc_spar[step] - self.centroids[step][0])**2
            ixz_spar = self.area_spar[step] * (self.cen_spar_z[step] - self.centroids[step][1]) * (self.loc_spar[step] - self.centroids[step][0])
            if twiz != 0:
                ix_spar = (ix_spar + iz_spar)/2 + ((ix_spar - iz_spar)/2)*np.cos(2*twiz) - ixz_spar*np.sin(2*twiz)
                iz_spar = (ix_spar + iz_spar)/2 - ((ix_spar - iz_spar)/2)*np.cos(2*twiz) + ixz_spar*np.sin(2*twiz)
                ixz_spar = ((ix_spar - iz_spar)/2)*np.sin(2*twiz) + ixz_spar*np.cos(2*twiz) 
            self.ix_list.append(ix + ix_spar)
            self.iz_list.append(iz + iz_spar)
            self.ixz_list.append(ixz + ixz_spar)
    
    def area(self):
        self.area_list = []
        for step in range(disc_steps):
            A3 = 0
            for i in range(len(self.profile3_x[step])-1):
                ax,az = self.profile3_x[step][i], self.profile3_z[step][i]
                bx,bz = self.profile3_x[step][i+1], self.profile3_z[step][i+1]
                u = np.array([ax-self.xspar_list[step][0],az-self.zspar_list[step][0]])
                v = np.array([bx-self.xspar_list[step][0],bz-self.zspar_list[step][0]])
                parallelogram = np.cross(u,v)
                dA = 0.5*parallelogram
                A3 += dA 
            A4 = 0
            for i in range(len(self.profile4_x[step])-1):
                ax,az = self.profile4_x[step][i], self.profile4_z[step][i]
                bx,bz = self.profile4_x[step][i+1], self.profile4_z[step][i+1]
                u = np.array([ax,az])
                v = np.array([bx,bz])
                parallelogram = np.cross(u,v)
                dA = 0.5*parallelogram
                A4 += dA 
            self.area_list.append([A3+A4,A3,A4])
        
    def bending_stress(self):
        self.stress_x_list = []
        self.stress_z_list = []
        self.sigma_list = []
        for step in range(disc_steps):
            stress_x_list_cs = []
            stress_z_list_cs = []
            sigma_list_cs = []
            moment_x = self.moment_list[step]
            moment_z = self.moment_list[step]/LDratio
            ix = self.ix_list[step]
            iz = self.iz_list[step]
            ixz = self.ixz_list[step]
            for i in range(len(self.x_coordinates)-1):
                stress_x = ((iz*moment_x - ixz*moment_z) / (ix*iz - ixz**2)) * (self.cen_z_list[step][i] - self.centroids[step][1] ) 
                stress_z = ((ix*moment_z - ixz*moment_x) / (ix*iz - ixz**2)) * (self.cen_x_list[step][i] - self.centroids[step][0] ) 
                sigma = stress_x + stress_z
                stress_x_list_cs.append(stress_x)
                stress_z_list_cs.append(stress_z)
                sigma_list_cs.append(sigma)
            for i in range(len(self.xspar_list[step])):
                stress_x_spar =  ((iz*moment_x - ixz*moment_z) / (ix*iz - ixz**2)) * (self.zspar_list[step][i] - self.centroids[step][1] )
                stress_z_spar = ((ix*moment_z - ixz*moment_x) / (ix*iz - ixz**2)) * (self.xspar_list[step][i] - self.centroids[step][0] ) 
                sigma_spar = stress_x_spar + stress_z_spar
                sigma_list_cs.append(sigma_spar)
            self.stress_x_list.append(stress_x_list_cs)  
            self.stress_z_list.append(stress_z_list_cs)
            self.sigma_list.append(sigma_list_cs)
            
    
    def shear_stress(self):
        self.tau_list = []
        self.qb_list3 = []
        self.qb_list4 = []
        self.total_shear_list = []
        self.intM3_list = []
        self.red_shear3 = []
        self.intM4_list = []
        self.red_shear4 = []
        for step in range(disc_steps):
            ix = self.ix_list[step]
            iz = self.iz_list[step]
            ixz = self.ixz_list[step]
            Sz = self.shearforce_list[step]
            Sx = Sz / LDratio
            qx_coef = -(Sx*ix - Sz*ixz) / (ix*iz - ixz**2)
            qz_coef = -(Sz*iz - Sx*ixz) / (ix*iz - ixz**2)
            qb3 = 0
            qb3_list_cs = []
            for i in range(self.idxtop2 +1 ):
                qbx0 = qx_coef * skin_thickness * (self.profile3_x[step][i+1] - self.centroids[step][0]) * self.segment1_list[step][i]
                qbz0 = qz_coef * skin_thickness * (self.profile3_z[step][i+1] - self.centroids[step][1]) * self.segment1_list[step][i]
                qb3 += (qbx0 + qbz0)
                qb3_list_cs.append(qb3)
            for i in range(self.idxtop2, self.idxtop2 +1 +len(self.xspar_list[0]) ):
                qbx0 = qx_coef * self.t_spar * (self.profile3_x[step][i+1] - self.centroids[step][0]) * self.segment1_list[step][i]
                qbz0 = qz_coef * self.t_spar * (self.profile3_z[step][i+1] - self.centroids[step][1]) * self.segment1_list[step][i]
                qb3 += (qbx0 + qbz0)
                qb3_list_cs.append(qb3)
            for i in range(self.idxbottom2 , len(self.profile3_x[0])-1):
                qbx0 = qx_coef * skin_thickness * (self.profile3_x[step][i+1] - self.centroids[step][0]) * self.segment1_list[step][i]
                qbz0 = qz_coef * skin_thickness * (self.profile3_z[step][i+1] - self.centroids[step][1]) * self.segment1_list[step][i]
                qb3 += (qbx0 + qbz0)
                qb3_list_cs.append(qb3)
            self.qb_list3.append(qb3_list_cs)
            qb4 = 0
            qb4_list_cs = []
            for i in range(len(self.profile4_x[0]) - len(self.xspar_list[0]) + 1):
                qbx0 = qx_coef * skin_thickness * (self.profile4_x[step][i+1] - self.centroids[step][0]) * self.segment2_list[step][i]
                qbz0 = qz_coef * skin_thickness * (self.profile4_z[step][i+1] - self.centroids[step][1]) * self.segment2_list[step][i]
                qb4 += (qbx0 + qbz0)
                qb4_list_cs.append(qb4)
            for i in range(len(self.profile4_x[0]) - len(self.xspar_list[0]), len(self.profile4_x[0]) -1):
                qbx0 = qx_coef * self.t_spar * (self.profile4_x[step][i+1] - self.centroids[step][0]) * self.segment2_list[step][i]
                qbz0 = qz_coef * self.t_spar * (self.profile4_z[step][i+1] - self.centroids[step][1]) * self.segment2_list[step][i]
                qb4 += (qbx0 + qbz0)
                qb4_list_cs.append(qb4)
            self.qb_list4.append(qb4_list_cs)
            Mint3 = 0 
            for i in range( len(self.profile3_x)-1):
                 qbx3 = ((self.profile3_x[step][i+1] - self.profile3_x[step][i]) / self.segment1_list[step][i]) *self.qb_list3[step][i]
                 qbz3 = ((self.profile3_z[step][i+1] - self.profile3_z[step][i]) / self.segment1_list[step][i]) *self.qb_list3[step][i]
                 m3x = qbx3 * self.segment1_list[step][i] *  (self.profile3_x[step][i] - self.profile3_x[step][self.idxbottom2])
                 m3z = qbz3 * self.segment1_list[step][i] * (self.profile3_z[step][i] - self.profile3_z[step][self.idxbottom2])
                 Mint3 += (m3x + m3z)
            self.intM3_list.append(Mint3)
            self.red_shear3.append(Mint3/(-2*self.area_list[step][1]))
            Mint4 = 0 
            for i in range(len(self.profile4_x) -1 ):
                qbx4 = ((self.profile4_x[step][i+1] - self.profile4_x[step][i]) / self.segment2_list[step][i]) *self.qb_list4[step][i]
                qbz4 = ((self.profile4_z[step][i+1] - self.profile4_z[step][i]) / self.segment2_list[step][i]) *self.qb_list4[step][i]
                m4x = qbx4 * self.segment2_list[step][i] *  (self.profile4_x[step][i] - self.profile4_x[step][len(self.profile4_x[0]) - len(self.xspar_list[0])])
                m4z = qbz4 * self.segment2_list[step][i] * (self.profile4_z[step][i] - self.profile4_z[step][len(self.profile4_x[0]) - len(self.xspar_list[0])])
                Mint4 += (m4x + m4z)
            self.intM4_list.append(Mint4)
            self.red_shear4.append(Mint4/(-2*self.area_list[step][2]))
            self.qR3 = []
            #for q in qb_list3[step]: 
                #self.qR3.append(q + red_shear3[step])
    
    def max_bend(self): 
        self.des_list = []
        for step in range(disc_steps):
            self.des_list.append([np.max(self.sigma_list[step]), np.min(self.sigma_list[step])])
#            
#    def von_mises(self):
#        self.von_mises = []
#        for step in range(disc_steps):
#            von_mises_cs = []
#            for i in range(len(self.x_coordinates)-1):
#                von_mises_segment = np.sqrt(self.sigma_list[step][i]**2 + 3*(self.tau_list[step][i])**2)
#                von_mises_cs.append(von_mises_segment)
#            self.von_mises.append(von_mises_cs)


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
blade.bending_stress()
blade.shear_stress()
blade.max_bend()
#blade.von_mises()


for i in range(disc_steps):
    plt.plot(blade.profile3_x[i], blade.profile3_z[i])
    plt.plot(blade.profile4_x[i], blade.profile4_z[i])   
    
#    plt.plot(blade.xspar_list[i],blade.zspar_list[i])

plt.figure()    
for i in range(disc_steps):
#    plt.scatter(i, max(blade.tau_list[i]),color='red')
    plt.scatter(i, max(blade.sigma_list[i]),color='blue')
#    plt.scatter(i, max(blade.von_mises[i]),color='green')
plt.show()
