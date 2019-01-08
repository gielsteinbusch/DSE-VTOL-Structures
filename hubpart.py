# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 16:24:54 2019

@author: archi
"""
import numpy as np 
import matplotlib.pyplot as plt 
from simplebeam import Simple_Beam
from sparwars import Blade_loading 

t2c = 0.12 
diameter = t2c
chord = 0.3 
R = 0.5*diameter*chord
t_hub = 0.001 
ds_hub = 6 # number of steps to choose for the hub itself 

phi_coord = list(np.linspace(0,np.deg2rad(360),201))
xhub = []
for phi in phi_coord: 
    xhub.append(R*np.cos(phi))
zhub = []
for phi in phi_coord: 
    zhub.append(R*np.sin(phi))
yhub = list(np.linspace(0,0.6,ds_hub)) 

#calculate the loads that are acting on the hub section!! 
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

vforce = beam.totallift
axforce = sum(beam.centrifugal)
moment_applied = beam.totalmoment

shear_hub = list(vforce*np.ones(ds_hub)) 
int_moments = []
for i in range(len(yhub)):
    Mfres = shear_hub[i] * yhub[i]
    Mtotm = moment_applied
    Mlif = 0
    moment = Mtotm - Mfres + Mlif
    int_moments.append(moment)

#THERE IS ALSO A LARGE MOMENT ACTING UPON THIS SECTION FROM THE BLADE AND THE RESULTANT

plt.plot(yhub,int_moments)
