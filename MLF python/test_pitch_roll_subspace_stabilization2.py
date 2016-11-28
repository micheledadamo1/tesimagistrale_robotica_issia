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
iterate=[]

#controller.id1=1

#controller.delta_t=plant.delta_t=1.0/60
#controller.reference_traj.delta_t=1.0/60
#stabilization problem
#set starting (act) state
#r=[0,0,2]
#rd=[0,0,0]
#rdd=[0,0,0]
#
#ang=[1,1,0]
#angd=[0,0,0]
#angdd=[0,0,0]

#be secure to have a desired traj of type [0,Y,0] constant in time (see controller_MLF , desired_trajecotry)
#############################
#parametri che determinano le prestazioni del sistema (compresa velocità del sistema)
#da notare che per avere prestazioni sostenibili, bisogna regolare la convergenza dell'angolo sull'errore di y
#se l'angolo è già arrivato a convergenza, non c'è variazione che porti allo spostamento
#RICORDA DI CAMBIARE L'INDICE IN BYPASS DEL CONTROLLER

#set tracking X_Y !!


controller.noYS=1

#controller.delta_t=controller.reference_traj.delta_t=plant.t1=1.0/30

#interessanti a 100 Hz
#params=[3,3,10,10]  #a 100hz
#params=[9,9,15,15] #a 100hz
#params=[5,5,15,10]
#controller.delta_t=controller.reference_traj.delta_t=plant.t1=1.0/100; iterate=1000

#controller.delta_t=controller.reference_traj.delta_t=plant.t1=1.0/500; iterate=3000
#
#controller.lambda_theta=controller.lambda_phi=controller.lambda_psi=params[0]
#controller.Kvtheta=controller.Kvphi=controller.Kv_psi=params[1]
#controller.lambda_x=controller.lambda_y=controller.lambda_z=params[2]
#controller.Kvx=controller.Kvy=controller.Kvz=params[3]


#################test_ completo
#controller.delta_t=controller.reference_traj.delta_t=plant.t1=1.0/100; iterate=3000

#controller.delta_t=controller.reference_traj.delta_t=plant.t1=1.0/300; iterate=3000
#controller.noYS=0
params=[5,5,10,10]
#params=[20,20,20,20]
#params=[10,15,10,15]
controller.lambda_theta=controller.lambda_phi=controller.lambda_psi=params[0]
controller.Kvtheta=controller.Kvphi=controller.Kv_psi=params[1]
controller.lambda_x=controller.lambda_y=controller.lambda_z=params[2]
controller.Kvx=controller.Kvy=controller.Kvz=params[3]

#############################

#plant.g=0 #annullo forza di gravità

r,rd,rdd,ang,angd,angdd=controller.reference_traj.data_export()

plant.g=9.8
#ang[0]=ang[0]+.1
#ang[1]=ang[1]-.2
plant.set_state(r,rd,rdd,ang,angd,angdd,plant.g,[0,0,0]) #per iniziare
controller.update_act(r,rd,rdd,ang,angd,angdd) #ho F_tot e torqes da qui
torques=[controller.tau_phi,controller.tau_theta,controller.tau_psi]


data=physic_system_data() #per memorizzo i dati
data_desired=physic_system_data() #perla i dati di riferimento

time=[]
ex=[]; exd=[]
etheta=[]; ethetad=[]
ey=[]; eyd=[];
ephi=[]; ephid=[]

data.store(r,rd,rdd,ang,angd,angdd,controller.F_tot,torques)
time.append(controller.reference_traj.t) #per un plot più preciso
ex.append(controller.ex); exd.append(controller.exd)
etheta.append(controller.etheta); ethetad.append(controller.ethetad)
ey.append(controller.ey); eyd.append(controller.eyd)
ephi.append(controller.ephi); ephid.append(controller.ephid)


r,rd,rdd,ang,angd,angdd=controller.reference_traj.data_export()
data_desired.store(r,rd,rdd,ang,angd,angdd,0,[0,0,0])

F_tot=9.811111111111111
if iterate==[]:
    iterate=1300;

for i in range(iterate):
    #reazione sistema fisico
    plant.update(F_tot,torques)   
    
    #reazione controllore
    controller.update_act(plant.r,plant.rd,plant.rdd,plant.ang,plant.angd,plant.angdd)
    torques=[controller.tau_phi,controller.tau_theta,controller.tau_psi]
    #print(torques)
    F_tot=controller.F_tot
    
    #salvataggio
    data.store(plant.r,plant.rd,plant.rdd,plant.ang,plant.angd,plant.angdd,plant.F_tot,plant.torques) 
    time.append(controller.reference_traj.t)
    
    ex.append(controller.ex); exd.append(controller.exd)
    etheta.append(controller.etheta); ethetad.append(controller.ethetad)
    ey.append(controller.ey); eyd.append(controller.eyd)
    ephi.append(controller.ephi); ephid.append(controller.ephid)

    r,rd,rdd,ang,angd,angdd=controller.reference_traj.data_export()    
    data_desired.store(r,rd,rdd,ang,angd,angdd,0,[0,0,0])    
    
plot.single_states_plot([data.x,data.y,data.z],np.fmod([data.phi,data.theta,data.psi],2*np.pi),time=time)
plot.single_states_plot([data_desired.x,data_desired.y,data_desired.z],[data_desired.phi,data_desired.theta,data_desired.psi],time=time)
plt.suptitle(['lambda_ang=Kv_ang=',controller.lambda_phi,'lambda_pos=Kv_pos=',controller.lambda_x])
#plot.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],opt='static')
#plot.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],opt='dynamic')

#plt.figure()
#plt.plot(epsi,epsid)


##### sliding surface test #copiato da yaw, vedere ora
#b=np.array(ex)
#a=np.array(exd)
#c=a+controller.lambda_y*b
##sliding surface
#plt.plot([0,1,-1],[0,-controller.lambda_x,controller.lambda_x],label='sliding surf')
#plt.plot(ex,exd,label='sx')
#plt.plot([0],[0],'o')
#plt.legend()

#plt.figure()
#plt.plot([0,1,-1],[0,-controller.lambda_theta,controller.lambda_theta],label='sliding surf')
#plt.plot(etheta,ethetad,label='sx')
#plt.plot([0],[0],'o')
#plt.legend()
#
#rapp=a[-10:-2]/b[-10:-2] #il coeff. angolare non è lambda_z... perchè?, le oscillazioni sono attorno a questa retta