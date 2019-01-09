# -*- coding: utf-8 -*-
"""
Created on Wed Jan  9 13:09:05 2019

@author: archi
"""
import numpy as np 
import matplotlib.pyplot as plt
from simplebeam import Simple_Beam
from sparwars import *
from mpl_toolkits.mplot3d import Axes3D
from hubpart import * 

phi_coord = list(np.linspace(0,np.deg2rad(360),201))
#need to find the velocities of the vflight for the blades, blade 1 is just vflight 
Vflight = 50
Vflight1 = Vflight 
Vflight23 = -Vflight*np.cos(np.deg2rad(60))
#loading for blade at 0 deg inc. angle 
blade1 = Blade_loading(radius, chord_length, taper, skin_thickness, Vflight1, rpm, rho, CL, list_x, list_z, LDratio, disc_steps)
beam1 = Simple_Beam()
beam1.sections()
beam1.lift_distribution(Vflight1)
beam1.shear_distribution() 
beam1.moment_distribution()
beam1.profile()
beam1.spar_coor()
beam1.profile_new()
beam1.twist()
beam1.center_gravity()
beam1.inertia()
beam1.area()
beam1.centrifugal_force()
#loading for blade at 60 deg inc. angle 
blade23 = Blade_loading(radius, chord_length, taper, skin_thickness, Vflight23, rpm, rho, CL, list_x, list_z, LDratio, disc_steps)
beam23 = Simple_Beam()
beam23.sections()
beam23.lift_distribution(Vflight23)
beam23.shear_distribution() 
beam23.moment_distribution()

P = sum(beam1.centrifugal) # the same for all blades (only depends on rotational velocity)
Ptot = P - 2*P*np.cos(np.deg2rad(60)) # zero due to the geometry of triple blades. 

vforce1 = beam1.totallift
vforce23 = beam23.totallift
moment_applied1 = beam1.totalmoment
moment_applied23 = beam23.totalmoment

int_moment1, shear_hub1 = bend_mom(vforce1, moment_applied1)
int_moment23, shear_hub23 = bend_mom(vforce23, moment_applied23)
M1 = int_moment1[0]
M23 = int_moment23[0]
Mrot = M1 - 2*M23*np.cos(np.deg2rad(60)) #resultant moment from all the blades
#print(Mrot)


