# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 09:23:40 2018

@author: giel
"""

chordlength = 1.

filename = 'airfoil2312.txt' 
fin = open(filename,'r')
line = fin.readlines()
list_z = []
list_x = []

newlist = []
for i in range(len(line)):
    row = line[i].split(' ')
    for j in row: 
        if len(list(j)) > 1: 
            newlist.append(j)

for n in newlist: 
    k = list(n)
    number = []
newlist = newlist[6:]  
for n in range(len(newlist)):
    if n%2 == 0:
        list_x.append(float(newlist[n])*chordlength)
    else:
        newlist[n] = newlist[n].replace('\n','')
        list_z.append(float(newlist[n])*chordlength)
        
