# -*- coding: utf-8 -*-
"""
Created on Wed Dec 12 15:47:35 2018

@author: archi
"""
import numpy as np 
import matplotlib.pyplot as plt 
from loading_blade import lift_rotorcraft, shear_diagram, moment_diagram

lift_list, x_list, totallift = lift_rotorcraft(radius,V_flight, rpm, rho, CL, disc_steps)
totalmoment, shearforce_list = shear_diagram(totallift, lift_list, x_list)
moment_list = moment_diagram(lift_list, x_list, totallift, totalmoment)