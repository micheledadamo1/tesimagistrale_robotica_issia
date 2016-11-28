# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 00:14:44 2016
NOdo euler system
stabilizzazione attitude
@author: uso
"""

import plot_methods as pltm
import matplotlib.pyplot as plt
from euler_system import euler_system_class  as e_system
import numpy as np
from numpy import array
from data_stored import physic_system_data
from attitude_controller import attitude_controller_class as a_controller

system=e_system()
control=a_controller()

data=physic_system_data()
data_ref=physic_system_data()

#set stato iniziale
#system.ang=array([1,.3,0])
system.ang=array([0,0,0])
system.angd=array([0,0,0])
system.angdd=array([0,0,0])
system.set_torques(array([.0,.0,.0]))

control.set_state(system.ang,system.angd,system.angdd)

control.tau=array([.0,.0,.0])

data.store([0,0,0],[0,0,0],[0,0,0],system.ang,system.angd,system.angdd,0,control.tau)
data_ref.store([0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],0,[0,0,0])

t=0
#rates
#dt=system.t1=1.0/100 #funziona con control.lambdaa=10; control.Kd=.6; di sicuro, forse si possono alzare(Kd da problemi)
dt=system.t1=1.0/30
#params=[13,.5] sicuro a 100 hz
params=[15,.3]

control.lambdaa=params[0]
control.Kd=params[1]

ephi=[]; etheta=[]; epsi=[]
ephid=[]; ethetad=[]; epsid=[]

tt=[0];

for i in range(300):
    print(i)
    
    t=t+dt
    tt.append(t)
    #system.integration(control.tau)
    system.integration(array(control.tau))
    control.t=0 #quando ricomincio serve t=0 per fare la derivata
    #control.update(t,system.ang,system.angd,system.angdd)
    control.update(t,system.ang,system.angd,system.angdd)
    
    
    data.store([0,0,0],[0,0,0],[0,0,0],system.ang,system.angd,system.angdd,0,control.tau)
    data_ref.store([0,0,0],[0,0,0],[0,0,0],control.ang_ref,control.angd_ref,control.angdd_ref,0,control.tau)    
    
    ephi.append(control.error[0]); etheta.append(control.error[1]); epsi.append(control.error[2])
    ephid.append(control.errord[0]); ethetad.append(control.errord[1]); epsid.append(control.errord[2])
    
phi,theta,psi=np.fmod([data.phi,data.theta,data.psi],2*np.pi)
phi_ref,theta_ref,psi_ref=np.fmod([data_ref.phi,data_ref.theta,data_ref.psi],2*np.pi)

pltm.single_states_plot([data.x,data.y,data.z],[phi,theta,psi],tt)
pltm.single_states_plot([data_ref.x,data_ref.y,data_ref.z],[phi_ref,theta_ref,psi_ref],tt)
#pltm.single_states_plot([ephi,etheta,epsi],[ephi,etheta,epsi])
#
#plt.plot(epsi,epsid)
#plt.plot([0,1,-1],[0,-control.lambdaa,control.lambdaa]) 

#pltm.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],'dynamic')
    
