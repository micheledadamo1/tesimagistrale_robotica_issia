# -*- coding: utf-8 -*-
"""
Created on Sun Nov 13 15:13:15 2016
test roll subspace

@author: uso
"""

from controller_MLF import MLF_class
from physic_system2 import physic_system_class
from data_stored import physic_system_data
import plot_methods as plot

import matplotlib.pyplot as plt

import numpy as np

plant=physic_system_class()
controller=MLF_class()
controller.id1=2

#controller.delta_t=plant.delta_t=1.0/100
#controller.reference_traj.delta_t=1.0/100
#stabilization problem
#set starting (act) state
r=[0,0,5]
rd=[0,0,0]
rdd=[0,0,0]

ang=[0,0,1]
angd=[0,0,0]
angdd=[0,0,0]

#be secure to have a desired traj of type [0,Y,0] constant in time (see controller_MLF , desired_trajecotry)
#############################
#parametri che determinano le prestazioni del sistema (compresa velocità del sistema)
#da notare che per avere prestazioni sostenibili, bisogna regolare la convergenza dell'angolo sull'errore di y
#se l'angolo è già arrivato a convergenza, non c'è variazione che porti allo spostamento
#RICORDA DI CAMBIARE L'INDICE IN BYPASS DEL CONTROLLER

controller.lambda_psi=10
controller.Kvpsi=10
controller.lambda_z=5
controller.Kvz=20

#controller.delta_t=controller.reference_traj.delta_t=plant.delta_t=1.0/100 #con questo rate si può richiedere maggiore velocità di conv.
#############################

#plant.g=0 #annullo forza di gravità

plant.set_state(r,rd,rdd,ang,angd,angdd,plant.g,[0,0,0]) #per iniziare
controller.update_act(r,rd,rdd,ang,angd,angdd) #ho F_tot e torqes da qui
torques=[controller.tau_phi,controller.tau_theta,controller.tau_psi]



data=physic_system_data() #per memorizzo i dati
data_desired=physic_system_data() #perla i dati di riferimento

time=[]
ez=[]; ezd=[]
epsi=[]; epsid=[]

data.store(r,rd,rdd,ang,angd,angdd,controller.F_tot,torques)
time.append(controller.reference_traj.t) #per un plot più preciso
ez.append(controller.ez); ezd.append(controller.ezd)
epsi.append(controller.epsi); epsid.append(controller.epsid)

r,rd,rdd,ang,angd,angdd=controller.reference_traj.data_export()
data_desired.store(r,rd,rdd,ang,angd,angdd,0,[0,0,0])

F_tot=9.811111111111111
for i in range(300):
    #reazione sistema fisico
    plant.update(F_tot,torques)   
    
    
    #reazione controllore
    controller.update_act(plant.r,plant.rd,plant.rdd,plant.ang,plant.angd,plant.angdd)
    torques=[controller.tau_phi,controller.tau_theta,controller.tau_psi]
    print(torques)
    F_tot=controller.F_tot
    
    #salvataggio
    data.store(plant.r,plant.rd,plant.rdd,plant.ang,plant.angd,plant.angdd,plant.F_tot,plant.torques) 
    time.append(controller.reference_traj.t)
    
    ez.append(controller.ez); ezd.append(controller.ezd)
    epsi.append(controller.epsi); epsid.append(controller.epsid)

    r,rd,rdd,ang,angd,angdd=controller.reference_traj.data_export()    
    data_desired.store(r,rd,rdd,ang,angd,angdd,0,[0,0,0])    
    
#plot.single_states_plot([data.x,data.y,data.z],np.fmod([data.phi,data.theta,data.psi],2*np.pi),time=time)
#plot.single_states_plot([data_desired.x,data_desired.y,data_desired.z],[data_desired.phi,data_desired.theta,data_desired.psi],time=time)
#plot.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],opt='static')
#plot.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],opt='dynamic')

#plt.figure()
plt.plot(ez,ezd)
plt.plot([0,-1,1],[0,controller.lambda_z,-controller.lambda_z])


##### sliding surface test #copiato da yaw, vedere ora
#b=np.array(ez)
#a=np.array(ezd)
#c=a+controller.lambda_y*b
##sliding surface
#plt.plot([0,1,-1],[0,-controller.lambda_x,controller.lambda_x],label='sliding surf')
#plt.plot(ez,ezd,label='sx')
#plt.plot([0],[0],'o')
#plt.legend()

#plt.figure()
#plt.plot([0,1,-1],[0,-controller.lambda_theta,controller.lambda_theta],label='sliding surf')
#plt.plot(epsi,epsid,label='sx')
#plt.plot([0],[0],'o')
#plt.legend()
#
#rapp=a[-10:-2]/b[-10:-2] #il coeff. angolare non è lambda_z... perchè?, le oscillazioni sono attorno a questa retta