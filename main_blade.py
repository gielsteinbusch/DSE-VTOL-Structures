# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 10:10:12 2018

@author: giel
"""
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from airfoil import list_x, list_y
from loading_blade import lift_rotorcraft, shear_diagram, moment_diagram




## input values
radius = 6.
taper = 0.5
chordlength = 0.35
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

## import moment distribution
lift_list, x_list, totallift = lift_rotorcraft(radius,V_flight, rpm, rho, CL, disc_steps)
totalmoment, shearforce_list = shear_diagram(totallift, lift_list, x_list)
moment_list = moment_diagram(lift_list, x_list, totallift, totalmoment)

x_coordinates = np.array(list_x)
z_coordinates = np.array(list_y)


length_ds = np.linspace(0, radius, disc_steps)
taperchord = np.linspace(chordlength, taper*chordlength , disc_steps)
twisting = -1* np.linspace(inc_angle, inc_angle- twist , disc_steps)
twisting = np.deg2rad(twisting)


def stress_analysis(taperchord,length_ds,twisting, totalmoment, shearforce_list, moment_list, skin_thickness, shear_center):
    profile_x = []
    profile_z = []
    profile_y = []
    A_list = []
    ix_list = []
    iz_list = []
    ixz_list = []
    count = 0
    #plt.figure()
    centroids = []
    listmaxbendstress = []
    colourstress = []
    M_int_list = []
    q0_list = []
    base_shear = []
    for c in taperchord: 
        aero_center = -0.25*c
        # Calculate twist alang span
        twiz = twisting[count]
        profile_x.append((x_coordinates*c - taper*c)*np.cos(twiz) - (z_coordinates*c)           *np.sin(twiz)) #maybe remove taper*c 
        profile_z.append((z_coordinates*c)          *np.cos(twiz) + (x_coordinates*c - taper*c) *np.sin(twiz))
        
        # Applying taper ratio
        for i in range(len(x_coordinates)):
            profile_y.append(length_ds[list(taperchord).index(c)])
        #plt.plot(profile_x[count],profile_z[count])
        
        # Calculate center of gravity
        cen_x_list = []
        cen_z_list = []
        segment_list = []
        for i in range(len(x_coordinates)-1):
            segment_length = np.sqrt((profile_x[count][i+1]-profile_x[count][i])**2 + (profile_z[count][i+1]-profile_z[count][i])**2)
            segment_list.append(segment_length)
            cen_x = profile_x[count][i]+(profile_x[count][i+1]-profile_x[count][i])/2
            cen_z = profile_z[count][i]+(profile_z[count][i+1]-profile_z[count][i])/2
            cen_x_list.append(cen_x) 
            cen_z_list.append(cen_z)
            
        centroid_x = np.sum(skin_thickness*np.array(segment_list)*np.array(cen_x_list)) / np.sum(skin_thickness*np.array(segment_list)) 
        centroid_z = np.sum(skin_thickness*np.array(segment_list)*np.array(cen_z_list)) / np.sum(skin_thickness*np.array(segment_list))
        centroids.append([centroid_x,centroid_z])
        
        #Calculate moment of Inertia
        ix = 0
        iz = 0
        ixz = 0
        for i in range(len(x_coordinates)-1):
            iz += (segment_list[i]*skin_thickness)*(profile_x[count][i]-centroids[count][0])**2
            ix += (segment_list[i]*skin_thickness)*(profile_z[count][i]-centroids[count][1])**2
            ixz += (segment_list[i]*skin_thickness)*(profile_x[count][i]-centroids[count][0])*(profile_z[count][i]-centroids[count][1])
        ################# maybe use as a verification method: take a symmetrical airfoid and see if ixy =0
        ix_list.append(ix)
        iz_list.append(iz)
        ixz_list.append(ixz)
        
        #Calculate bending stess due to assymetric bending formulae 
        stress_list_x = []
        stress_list_z = []
        moment_x = moment_list[count]                                        #in the real programme remove the moment/lift factors
        moment_z = moment_list[count]/LDratio
        sigma_list = [0]  #stress in each cross section
        for i in range(len(x_coordinates)-1):
            #stress_x = moment_x * cen_z_list[i] / ix
            stress_x = ((iz*moment_x - ixz*moment_z) / (ix*iz - ixz**2)) * cen_z_list[i]
            stress_list_x.append(stress_x)
            #stress_z = moment_z * cen_x_list[i] / iz
            stress_z = ((ix*moment_z - ixz*moment_x) / (ix*iz - ixz**2)) * cen_x_list[i]
            stress_list_z.append(stress_z)
            sigma = stress_x + stress_z 
            sigma_list.append(sigma)
        maxbend_stress = max(sigma_list)
        listmaxbendstress.append(maxbend_stress)
        colourstress.append(np.array(sigma_list))
        
        #Calculate base shear flow through integrating  qx_coeff *(t*x or z) *ds
        Sz = shearforce_list[count]
        Sx = shearforce_list[count]/LDratio                                          #REAL REMOVE THESE FACTORS
        qx_coef = -(Sx*ix - Sz*ixz) / (ix*iz - ixz**2)
        qz_coef = -(Sz*iz - Sx*ixz) / (ix*iz - ixz**2)
        qb = 0
        qb_list = []
        for i in range(len(x_coordinates)-1):
            qb_list.append(qb)
            qbx = qx_coef * skin_thickness * (profile_x[count][i+1] - centroids[count][0]) * segment_list[i]
            qbz = qz_coef * skin_thickness * (profile_z[count][i+1] - centroids[count][1]) * segment_list[i]
            qb = (qbx + qbz)
        qb_sum = np.cumsum(np.array(qb_list))
        base_shear.append(qb_sum)
        #print(Sz, qb_sum[-1],list(qb_sum).index(min(qb_sum)), min(qb_sum))
        
        #FINDING THE ENCLOSED AREA (slightly less than real life due to discretisation)
        A_0 = 0
        for i in  range(len(x_coordinates)-1):
            ax,az = profile_x[count][i], profile_z[count][i]
            bx,bz = profile_x[count][i+1], profile_z[count][i+1]
            u = np.array([ax,az])
            v = np.array([bx,bz])
            OA = np.sqrt(ax**2 + az**2) 
            OB = np.sqrt(bx**2 + bz**2)
            AB = np.sqrt((bx - ax)**2 + (bz - az)**2)
            parallelogram = np.cross(u,v)
            dA = 0.5*parallelogram
            A_0 = A_0 + dA 
    #        
        A_list.append(A_0)
        
        #NOW DO THE INTERNAL MOMENT SHIT (we choose internal moment sum at the location of cross of line of action of Sx and SZ)
        #take sum of internal shear flow moments about a single point
        internalM = 0
        for i in range(1,len(x_coordinates)-1):
            Qb = qb_sum[i]                                                         #sum or standard?? 
            axM = profile_x[count][i-1]
            azM = profile_z[count][i-1]
            bxM = profile_x[count][i]
            bzM = profile_z[count][i]
            seglen = np.sqrt((bxM-axM)**2 + (bzM - azM)**2) 
            Qxb = ((bxM - axM)/seglen)*Qb
            Qzb = ((bzM - azM)/seglen)*Qb
            Marmx = bxM - aero_center
            Marmz =  bzM    #need to change depending on where the drag acts on the z axis of the airfoil.  
            internalM = internalM + (Qxb*Marmz + Qzb*Marmx)*seglen 
        M_int_list.append(internalM) 
        q0_list.append(M_int_list[-1]/(-2.*A_0)) #this line calculates the redundant shear flow 
        
        #print(min(qb_sum), internalM/(-2.*A_0))
        
        
        count = count + 1

    return profile_x, profile_y, profile_z, A_list, ix_list, iz_list, ixz_list, colourstress, base_shear, q0_list

profile_x, profile_y, profile_z, A_list, ix_list, iz_list, ixz_list, colourstress, base_shear, q0_list = stress_analysis(taperchord,length_ds,twisting, totalmoment, shearforce_list, moment_list, skin_thickness, shear_center)
#adding the redundant shear flows to the base shear flows ------------------------------------------------------------------------------------------------------------------------------------------------------
def total_shear(taperchord, base_shear, q0_list):
    shear_flow = []
    shear_stress = []
    for j in range(len(taperchord)):
        flow = np.array(base_shear[j])
        flow = flow + q0_list[j]
        #print(q0_list[j])
        shear_flow.append(flow)
        shear_stress.append(flow/skin_thickness)
    return shear_flow, shear_stress
shear_flow, shear_stress = total_shear(taperchord, base_shear, q0_list)
#vonmises stress on the cross sections -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def von_mises(taperchord, colourstress, shear_stress, x_coordinates):
    vonmises = []
    for i in range(len(taperchord)):
        vm_list= []
        maxlist = []
        for  j in range(len(x_coordinates)-1): 
            vm = np.sqrt((colourstress[i][j])**2 + 3*(shear_stress[i][j])**2)
            vm_list.append(vm)
        vonmises.append(vm_list)
        maxlist.append(max(vm_list))
        maxlist.append(min(vm_list))
    return vonmises, max(np.abs(maxlist))
vonmises, max_vm = von_mises(taperchord, colourstress, shear_stress, x_coordinates)
print(max_vm)
##########################################################---------------------plotting 
#fig = plt.figure()
#ax = fig.add_subplot(111, projection ='3d')
#
##plotting bend stress
#bend = []
#for t in range(len(taperchord)):
#    for x in colourstress[t]:
#        bend.append(x)
#bend = np.array(bend)
##plotting shear stress
#shear = []
#for t in range(len(taperchord)):
#    for x in shear_stress[t]:
#        shear.append(x)
#    shear.append(-1)
#shear = np.array(shear)
##plotting vonmises stress 
#vm = []
#for t in range(len(taperchord)):
#    for x in vonmises[t]:
#        vm.append(x)
#    vm.append(1)
#vm = np.array(vm)
#
## UN-(#) these to see the stress distributions... 
##ax.scatter(profile_x, profile_y, profile_z, c = bend, cmap=plt.jet())
##ax.scatter(profile_x, profile_y, profile_z, c = shear, cmap=plt.jet())
#ax.scatter(profile_x, profile_y, profile_z, c=vm, cmap=plt.jet())
#
##plt.figure()
##plt.plot( np.cumsum(np.ones(len(qb_list))), qb_list)
##plt.plot(np.cumsum(np.ones(len(qb_list))), qb_sum)
##plt.plot(np.cumsum(np.ones(len(qb_list))), flow )
##plt.show()

