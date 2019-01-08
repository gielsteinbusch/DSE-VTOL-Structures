# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 12:55:21 2019

@author: archi
"""
import numpy as np 
import matplotlib.pyplot as plt 
from airfoil import list_x, list_z
from sparwars import Blade_loading
from formula_beam import *

#input values                       (MUST MATCH THE SPARWARS VALUES)
radius = 4.46
taper = 0.5
chord_length = 1
inc_angle = 10
twist = 20
skin_thickness = 0.01
V_flight = 0
rpm = 41 * 60/(2*np.pi)
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9
disc_steps = 8

#material properities 
den = 2780
E = 73.1e9
Uten = 345e8
G = 28e9

#call the functions from the other file -----------
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
blade.centrifugal_force()
blade.area()
blade.bending_stress()
blade.shear_stress()
blade.max_bend()


#set up a class for simple beam theory 
class Simple_Beam:
    def sections(self,blade):
            self.nseg = disc_steps -1 
            self.radius = radius
            self.chord_length = chord_length
            self.taper = taper
            self.disc_steps = disc_steps
            self.V_flight = V_flight
            #making the midpoint of sections into list
            self.length_ds = np.linspace(0.6, radius, self.disc_steps)
            self.sec_points = []
            self.w_segment = self.length_ds[1] - self.length_ds[0]
            for d in self.length_ds[:-1]: 
                midpoint = d + self.w_segment/2
                self.sec_points.append(midpoint)
            #making the midpoint chords a list  
            self.taperchord = np.linspace(chord_length, taper*chord_length , self.disc_steps)
            self.change_chord = (self.taperchord[0] - self.taperchord[1])/2
            self.sec_chord = []
            for c in self.taperchord[:-1]:
                c = c - self.change_chord
                self.sec_chord.append(c)
            #making the midpoint twist a list
            self.twisting = np.deg2rad(-1* np.linspace(inc_angle, inc_angle- twist , self.disc_steps))
            self.sec_twist = []
            for t in self.twisting[:-1]: 
                change_twist = (self.twisting[0]-self.twisting[1])/2
                t = t - change_twist
                self.sec_twist.append(t)
            self.x_coordinates = np.array(list_x)
            self.z_coordinates = np.array(list_z)
            self.LDratio = LDratio
            self.list_z = list_z
            self.list_x = list_x
    #new lift distribution based on the lift at the midpoint of each segment        
    def lift_distribution(self):
        self.x_list = []
        self.lift_list = []
        self.totallift = 0
        x = []
        self.width_segment = radius / disc_steps
        for segment in range(self.nseg):
            distance_center = self.sec_points[segment]
            V_rotor = 2*np.pi*(rpm/60)*distance_center
            V_total = V_flight + V_rotor
            S = (np.pi * (self.length_ds[segment +1])**2) - sum(x)
            #print(self.length_ds[segment +1])
            x.append(S)
            L = 0.5 * rho * (V_total)**2 * S * CL
            self.lift_list.append(L)
            self.x_list.append(distance_center)
            self.totallift += L
    #new shear distribution based on the previous lift 
    def shear_distribution(self):
        self.totalmoment = 0 
        self.shear_list = []
        for s in range(self.nseg): 
            shear_mid = (blade.shearforce_list[s] + blade.shearforce_list[s+1])/2
            self.shear_list.append(shear_mid)
            self.totalmoment += self.lift_list[s]*self.x_list[s]
    #new moment on previous shear 
    def moment_distribution(self):
        self.moment_list = []
        for i in range(self.nseg):
           self.moment = (blade.moment_list[i+1] + blade.moment_list[i])/2 
           self.moment_list.append(self.moment)
    #new profiles at the midpoints 
    def profile(self):
        self.profile_x = []
        self.profile_z = []
        self.profile_y = []
        for step in range(self.nseg):
            c = self.sec_chord[step]
            self.profile_x.append(self.x_coordinates*c)  
            self.profile_z.append(self.z_coordinates*c) 
            for i in range(len(self.x_coordinates)):
                self.profile_y.append(self.sec_points[step])
    # adding spar coordinates for new cross-sections 
    def spar_coor(self):
        self.xspar_list = []
        self.zspar_list = []
        self.len_spar = []
        self.area_spar = []
        self.cen_spar_z = []
        self.loc_spar = []
        for step in range(self.nseg):
            self.t_spar = 2*skin_thickness
            loc_spar_cs = 0.3*self.sec_chord[step]            
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
#PROFILES OF THE SHEAR STUFF FOR THESE NEW CROSS SECTIONS 
    def profile_new(self):
        self.profile1_x = []
        self.profile2_x = []
        self.profile1_z = []
        self.profile2_z = []
        self.segment1_list = []
        self.segment2_list = []
        for step in range(self.nseg):
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
#twist for the cross sectional points 
    def twist(self):
        self.profile3_x = []
        self.profile4_x = []
        self.profile3_z = []
        self.profile4_z = []
        for step in range(self.nseg):
            self.profile3_x_cs = []
            self.profile4_x_cs = []
            self.profile3_z_cs = []
            self.profile4_z_cs = []
            twiz = self.sec_twist[step]
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
            #new centroid calculations for the cross sectionss !!! ------------------------------------
    def center_gravity(self):
        self.centroids = []
        self.cen_x_list = []
        self.cen_z_list = []
        self.segment_list = []
        for step in range(self.nseg):
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
    #NOW REDOING MOMENTS OF INERTIA FOR THE CROSS SECTIONSSS 
    def inertia(self):
        self.ix_list = []
        self.iz_list = []
        self.ixz_list = []
        for step in range(self.nseg):
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
    #doing the internal areas for each section!! 
    def area(self):
        self.area_list = []
        for step in range(self.nseg):
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
    #centrifugal force
    def centrifugal_force(self):
        self.CSAlist = []
        self.siglist = []
        self.mlist = []
        self.ylist= []
        mass = 0 
        self.centrifugal = []
        self.wrs = (rpm*2*np.pi)/60
        self.w_segment = self.length_ds[1]-self.length_ds[0]
        for i in range(self.nseg):
            Aspar = self.area_spar[i]
            Askin = sum(self.segment_list[i])*skin_thickness
            CSA = Aspar + Askin
            self.CSAlist.append(CSA)
            m = CSA*den*self.w_segment
            mass +=m
            #print(m)
            ypoint = self.sec_points[i]
            self.ylist.append(ypoint)
            centri = den*CSA*((ypoint**2)/2)*self.wrs**2   
            self.centrifugal.append(centri)
            sigi = den*((ypoint**2)/2)*self.wrs**2        
            #sigi = Ni/CSA
            self.siglist.append(sigi)
            
            
# BEAM THEORY -----------------------------------------------------------------------------------------------

    ## we can add and alter the stress calculations after the beam theory stuff is finalised ########################################
            
    
beam = Simple_Beam()
beam.sections(blade)
beam.lift_distribution()
beam.shear_distribution() 
beam.moment_distribution()
beam.profile()
beam.spar_coor()
beam.profile_new()
beam.twist()
beam.center_gravity()
beam.inertia()
beam.area()
beam.centrifugal_force()


for i in range(beam.nseg):
    plt.plot(beam.profile3_x[i], beam.profile3_z[i])
    plt.plot(beam.profile4_x[i], beam.profile4_z[i])   
    
    #plt.plot(beam.xspar_list[i],beam.zspar_list[i])