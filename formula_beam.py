# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 15:59:09 2019

@author: archi
"""
import numpy as np 

def K(E, I , P): 
    return np.sqrt(P/(E*I))

def C1(k, l):
    return np.cosh(k*l)

def C2(k,l): 
    return np.sinh(k*l)

def Ca3(k, l , a): 
    return np.cosh(k*(l-a)) -1 

def Ca4(k, l , a ): 
    return np.sinh(k*(l-a)) - k*(l-a)  #?? little query on the BODMAS here 

# for right free, left fixed (1)

def theta_A1(W, P, k, l , a): 
    return (W/P)*(Ca3(k,l,a) / C1(k,l))

def yA1(W, P, k, l, a):
    return (-W/(P*k))*((C2(k,l)*Ca3(k,l,a))/C1(k,l) - Ca4(k,l,a))

# the F functions ------------
def F1(k,x): 
    return np.cosh(k*x)

def F2(k,x):
    return np.sinh(k*x)

def F3(k,x): 
    return np.cosh(k*x) -1 

def F4(k,x): 
    return np.sinh(k*x) - k*x   # also query here on 'h' and Bodmas 

def Fa1(k,x,a):
    f = (x-a)
    if f < 0:
        f == 0
    return f*np.cosh(k*f)

def Fa2(k,x,a):
    f = (x-a)
    if f < 0:
        f == 0
    return np.sinh(f*k)

def Fa3(k,x,a): 
    f = (x-a)
    if f < 0:
        f == 0
    return f*(np.cosh(f*k) - 1)

def Fa4(k,x,a): 
    f = (x-a)
    if f < 0:
        f == 0
    return np.sinh(k*f) - k*f

#load terms -------------------------
def LTV(W, Fa1):
    return -W*Fa1

def LTM(W,k,Fa2): 
    return (-W/k)*Fa2 

def LTtheta(W, P, Fa3):
    return (-W/P)*Fa3 

def LTY(W,P, k, Fa4): 
    return (-W/(P*k))*Fa4    


