# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 16:24:54 2019

@author: archi
"""
import numpy as np 
import matplotlib.pyplot as plt 
from simplebeam import Simple_Beam
from sparwars import *
from mpl_toolkits.mplot3d import Axes3D

t2c = 0.12 
diameter = t2c
chord = 1 
R = 0.5*diameter*chord
t_hub = 0.005
inc_angle = 10
ds_hub = 6 # number of steps to choose for the hub itself 
skin_hub = 0.005

#setting up some coordinates --->> NEED BLADE FOR THIS 
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
xpos_hub = blade.xspar_list[0][int((len(blade.profile_x[0])-1)/4/2)]*np.cos(np.deg2rad(inc_angle)) - blade.zspar_list[0][int((len(blade.profile_x[0])-1)/4/2)]*np.sin(np.deg2rad(inc_angle))
zpos_hub = blade.zspar_list[0][int((len(blade.profile_x[0])-1)/4/2)]*np.cos(np.deg2rad(inc_angle)) - blade.xspar_list[0][int((len(blade.profile_x[0])-1)/4/2)]*np.sin(np.deg2rad(inc_angle))

phi_coord = list(np.linspace(0,np.deg2rad(360),201))
xhub = []
for phi in phi_coord: 
    xhub.append(R*np.cos(phi) + xpos_hub) 
zhub = []
for phi in phi_coord: 
    zhub.append(R*np.sin(phi)  + zpos_hub)
yhub = list(np.linspace(0,0.6,ds_hub)) 

#calculate the loads that are acting on the hub section!! 
beam = Simple_Beam()

beam.sections()
beam.lift_distribution(V_flight)
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
moment_applied = beam.totalmoment #THERE IS ALSO A LARGE MOMENT ACTING UPON THIS SECTION FROM THE BLADE AND THE RESULTANT

#determining those bending moments in the hub
def bend_mom(vforce, moment_applied):
    shear_hub = list(vforce*np.ones(ds_hub)) 
    int_moments = []
    for i in range(len(yhub)):
        Mfres = shear_hub[i] * yhub[i]
        Mtotm = moment_applied
        Mlif = 0
        moment = Mtotm - Mfres + Mlif
        int_moments.append(moment)
    return int_moments, shear_hub
int_moments, shear_hub = bend_mom( vforce, moment_applied)

centroid_hub = [xpos_hub,zpos_hub]

segment_list = []
for i in range(len(xhub)-1):
    segment_length = np.sqrt((xhub[i+1]-xhub[i])**2 + (zhub[i+1]-zhub[i])**2)
    segment_list.append(segment_length)

def inertia(xhub,zhub, centroid_hub):    # inertia and centroid are constant at each cross section of the hub
        ix_list = []
        iz_list = []
        ixz_list = []
        for step in range(ds_hub):
            ix = 0
            iz = 0
            ixz = 0
            skin_thickness = skin_hub
            for i in range(len(xhub)-1):
                iz += (segment_list[i]*skin_thickness)*(xhub[i]-centroid_hub[0])**2
                ix += (segment_list[i]*skin_thickness)*(zhub[i]-centroid_hub[1])**2
                ixz += (segment_list[i]*skin_thickness)*(xhub[i]-centroid_hub[0])*(zhub[i]-centroid_hub[1])
            ix_list.append(ix)
            iz_list.append(iz)
            ixz_list.append(ixz)
        return ix_list, iz_list, ixz_list
ix_list, iz_list, ixz_list = inertia(xhub,zhub, centroid_hub) 

A0 = 2*np.pi*(R**2)
CSA = A0 - 2*np.pi*((R-skin_hub)**2)

def bending_stress(xhub,zhub,ix_list,iz_list,ixz_list, moment_list):
        stress_x_list = []
        stress_z_list = []
        sigma_list = []
        for step in range(ds_hub):
            stress_x_list_cs = []
            stress_z_list_cs = []
            sigma_list_cs = []
            moment_x = moment_list[step]
            moment_z = 0. #self.moment_list[step]/LDratio
            ix = ix_list[step]
            iz = iz_list[step]
            ixz = ixz_list[step]
            for i in range(len(xhub)-1):
                stress_x = ((iz*moment_x - ixz*moment_z) / (ix*iz - ixz**2)) * (zhub[i] - centroid_hub[1] ) 
                stress_z = ((ix*moment_z - ixz*moment_x) / (ix*iz - ixz**2)) * (xhub[i] - centroid_hub[0] ) 
                sigma = stress_x + stress_z
                stress_x_list_cs.append(stress_x)
                stress_z_list_cs.append(stress_z)
                sigma_list_cs.append(sigma)
            stress_x_list.append(stress_x_list_cs)  
            stress_z_list.append(stress_z_list_cs)
            sigma_list.append(sigma_list_cs)    
        return sigma_list, stress_x_list, stress_z_list
sigma_list, stress_x_list, stress_z_list =  bending_stress(xhub,zhub,ix_list,iz_list,ixz_list, int_moments)   

def tau_hub(xhub,zhub,centroid, vforce):
    tau_list = []
    qb_list = []
    Mint_list = []
    Q_list = []
    for step in range(ds_hub):
        Sz = shear_hub[step]
        Sx = 0. 
        ix = ix_list[step]
        iz = iz_list[step]
        ixz = ixz_list[step]
        qx_coeff = qx_coef = -(Sx*ix - Sz*ixz) / (ix*iz - ixz**2)
        qz_coef = -(Sz*iz - Sx*ixz) / (ix*iz - ixz**2)
        qb = 0 
        skin_thickness = skin_hub
        qb_list_cs = []
        for i in range(len(xhub)-1 ): 
            qbx0 = qx_coef * skin_thickness * (xhub[i] - centroid_hub[0]) * segment_list[i]
            qbz0 = qz_coef * skin_thickness * (zhub[i] - centroid_hub[1]) * segment_list[i]
            qb += (qbx0 + qbz0)
            qb_list_cs.append(qb)
        Mint = 0.
        Mint_cs = []
        for i in range(len(xhub)-1 ):
            Mint += qb_list_cs[i]*R*segment_list[i]
            Mint_cs.append(Mint)
        internalM = sum(Mint_cs)
        qs = internalM/(-2*A0)
        Q_cs = []
        for q in qb_list_cs: 
            Q_cs.append(q + qs )
        t_list = []
        for p in Q_cs: 
            t_list.append( p / skin_thickness)
        qb_list.append(qb_list_cs)
        Mint_list.append(Mint_cs)
        Q_list.append(Q_cs)
        tau_list.append(t_list)
    return tau_list, qb_list, Q_list
tau_list, qb_list, Q_list = tau_hub(xhub,zhub,centroid_hub, vforce) #base shear is correct, so hoping the others are fine 

#Stresses due to the axial load from the centrifugal forces 
sigax = []
for step in range(ds_hub): 
    sigax.append( axforce/CSA  )      
#now the stresses for the axial load and bending must be combined
stresses = []
for step in range(ds_hub):
    sigs = []
    for i in range(len(xhub)-1):
        sigs.append(sigma_list[step][i] + sigax[step])
    stresses.append(sigs)

#now we must find the vonmises in the section
vonmises = []
for step in range(ds_hub): 
    vm_list = []
    for i in range(len(xhub)-1): 
        vm = np.sqrt((stresses[step][i])**2 + 3*(tau_list[step][i])**2)
        vm_list.append(vm)
    vonmises.append(vm_list)
    
#plt.plot(xhub,zhub)

#plt.figure()
#plt.plot(yhub,int_moments)
count = 0
xhubplot = []
zhubplot = []
yhubplot = []
while count < ds_hub:
    xhubplot.append(xhub[:-1])
    zhubplot.append(zhub[:-1])
    yhubplot.append(np.ones(len(xhub)-1)*yhub[count])
    count += 1 
    
bend = []
for step in range(ds_hub):
    for x in vonmises[step]:
        bend.append(x)
vonmises = np.array(bend)

#fig = plt.figure()
#ax = fig.add_subplot(111, projection ='3d')    
#ax.scatter(xhubplot, yhubplot, zhubplot, c = vonmises, cmap=plt.jet())
#plt.show()

