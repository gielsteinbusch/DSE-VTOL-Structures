# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 16:31:06 2018

@author: archi
"""
import numpy as np 
from loading_blade import lift_rotorcraft, shear_diagram, moment_diagram
from main_blade import stress_analysis, total_shear, von_mises
from airfoil import list_x, list_y
from loading_diagrams import 
import matplotlib.pyplot as plt
#define the material properties for analysis (Carbon Fiber)
Emod = 70e9
Uten = 600e6
Ucomp = 570e6 
#general inputs 
radius = 4.46
taper = 1
inc_angle = 0
twist = 0
disc_steps = 20
skin_thickness = 0.001
V_flight = 0
rpm = 41.8/(2*np.pi) * 60
rho = 0.5
CL = 0.5
W_aircraft = 2500
LDratio = 9
#triple blade inputs 
chord3 = 0.304
shear_center3 = -0.5*0.5*chord3
#quad-blade inputs 
chord4 = 0.227
shear_center4 = -0.5*0.5*chord4
#define the airfoil coordinates 
x_coordinates = np.array(list_x)
z_coordinates = np.array(list_y)

## determine moment distribution (need to do for each blade)
lift_list, x_list, totallift = lift_rotorcraft(radius,V_flight, rpm, rho, CL, disc_steps) #will have to change for comparison
lift_list3, totallift3 = list(np.array(lift_list)/3), totallift/3
lift_list4, totallift4 = list(np.array(lift_list)/4), totallift/4
totalmoment3, shearforce_list3 = shear_diagram(totallift3, lift_list3, x_list) 
totalmoment4, shearforce_list4 = shear_diagram(totallift4, lift_list4, x_list)#will have to change for comparison
moment_list3 = moment_diagram(lift_list3, x_list, totallift3, totalmoment3) #will have to change for comparison
moment_list4 = moment_diagram(lift_list4, x_list, totallift4, totalmoment4)

length_ds = np.linspace(0, radius, disc_steps)
taperchord3 = np.linspace(chord3, taper*chord3 , disc_steps)
taperchord4 = np.linspace(chord4, taper*chord3 , disc_steps)
twisting = -1* np.linspace(inc_angle, inc_angle- twist , disc_steps)
twisting = np.deg2rad(twisting)

max_vm3 = Emod
max_vm4 = Emod

while max_vm3 >=  Ucomp: 
    #calculate for 3 
    profile_x3, profile_y3, profile_z3, A_list3, ix_list3, iz_list3, ixz_list3, colourstress3, base_shear3, q0_list3 = stress_analysis(taperchord3,length_ds,twisting, totalmoment3, shearforce_list3, moment_list3, skin_thickness, shear_center3)
    shear_flow3, shear_stress3 = total_shear(taperchord3, base_shear3, q0_list3)
    vonmises3, max_vm3 = von_mises(taperchord3, colourstress3, shear_stress3, x_coordinates)
    skin_thickness = skin_thickness + 0.0001
    print('triple_skin:',skin_thickness, 'max:', max_vm3)
t3 = skin_thickness
print(t3)

skin_thickness = 0.001

while max_vm4 >= Ucomp:
    #calculate for 4 
    profile_x4, profile_y4, profile_z4, A_list4, ix_list4, iz_list4, ixz_list4, colourstress4, base_shear4, q0_list4 = stress_analysis(taperchord4,length_ds,twisting, totalmoment4, shearforce_list4, moment_list4, skin_thickness, shear_center4)
    shear_flow4, shear_stress4 = total_shear(taperchord4, base_shear4, q0_list4)
    vonmises4, max_vm4 = von_mises(taperchord4, colourstress4, shear_stress4, x_coordinates)
    skin_thickness = skin_thickness + 0.0001 
    print('quad_skin:',skin_thickness, 'max:' , max_vm4)
t4 = skin_thickness
print(t4)


