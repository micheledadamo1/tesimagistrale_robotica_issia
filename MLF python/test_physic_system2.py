# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 12:41:52 2016

@author: uso
"""

import numpy as np
from physic_system2 import physic_system_class
from data_stored import physic_system_data
import plot_methods as plot

#test di caduta libera: accelera effettivamente con acc=g=-9.8?
syst=physic_system_class()

#setto stato iniziale
r=np.array([0,0,3]) #posizione
rd=np.array([0,0,0]) #velocità
rdd=np.array([0,0,0]) #accel
ang=np.array([0,0,0]) #angoli
angd=np.array([0,0,0])
angdd=np.array([0,0,0])

F_tot=9.8 #modulo della forza applicata
tau=np.array([0,0,0]) #torques

syst.set_state(r,rd,rdd,ang,angd,angdd,F_tot,tau)

#inizializzo class per memorizzare i dati
data=physic_system_data()
data.store(r,rd,rdd,ang,angd,angdd,F_tot,tau)

iterate=200 #ricorda a che velocità va il sistema

number_test=7; #per salvare e riutilizzare i vari tests

if number_test==1:
    #caduta libera
    for i in range(iterate): 
        F_tot=0
        syst.update(F_tot,tau)
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
        
elif number_test==2:
    #stazionamento in un punto: hovering
    for i in range(iterate):
        F_tot=9.8
        tau=[0,0,0]
        syst.update(F_tot,tau)
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)

elif number_test==3:
    #moto lungo x, impulso sull'asse di beccheggio (tau_theta)
    tau=[.01,.01,0]
    syst.set_state(r,rd,rdd,ang,angd,angdd,F_tot,tau)
    syst.update(F_tot,tau)
    data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
    for i in range(iterate):
        F_tot=9.8
        tau=[0,0,0]
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)

elif number_test==4:
    #moto perpetuo attorno all'asse z (accelerazione di psi)
    tau=[0,0,.01]
    syst.set_state(r,rd,rdd,ang,angd,angdd,F_tot,tau)
    syst.update(F_tot,tau)
    data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
    for i in range(iterate):
        F_tot=9.8
        tau=[0,0,0.01]
        syst.update(F_tot,tau)
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)

elif number_test==5:
    #moto perpetuo attorno all'asse x (accelerazione di phi)
    tau=[.01,0,0]
    F_tot=9.8
    syst.set_state(r,rd,rdd,ang,angd,angdd,F_tot,tau)
    syst.update(F_tot,tau)
    data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
    for i in range(iterate):
        syst.update(F_tot,tau)
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)

elif number_test==6:
    tau=[.0,.0,.0]
    F_tot=9.8
    ang=np.array([np.pi/4,0,0])
    syst.set_state(r,rd,rdd,ang,angd,angdd,F_tot,tau)
    syst.update(F_tot,tau)
    data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
    for i in range(iterate):
        syst.update(F_tot,tau)
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)

elif number_test==7:
    tau=[.0,1,.0]
    F_tot=9.8
    ang=np.array([0,0,0])
    syst.set_state(r,rd,rdd,ang,angd,angdd,F_tot,tau)
    syst.update(F_tot,tau)
    data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
    for i in range(iterate):
        syst.update(F_tot,tau)
        data.store(syst.r,syst.rd,syst.rdd,syst.ang,syst.angd,syst.angdd,syst.F_tot,syst.torques)
#sembra vada tutto bene
    
    
###esempi utilizzo plot_methods        
#test plot 2d singolo
#plot.plot_single_variable(data.az,var_name='z acc')

#test plot 1 fig x 6 
plot.single_states_plot([data.x,data.y,data.z],[data.phi,data.theta,data.psi])   

#test plot static 3d
plot.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],opt='static')

#test plot dynamic 3d
#plot.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],opt='dynamic')

#debug followed vs desired 
#plot.followed_vs_desired([data.x,data.y,data.z],[data.x,data.y,data.z])

