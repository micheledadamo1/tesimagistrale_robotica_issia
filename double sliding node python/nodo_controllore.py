# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 18:01:11 2016

@author: uso
"""

import plot_methods as pltm
from numpy import array
from data_stored import physic_system_data
import numpy as np
from sistema_fisico import physic_system_class as system
from controller_block import full_nl_controller_class

import matplotlib.pyplot as plt


ss=system()
controller=full_nl_controller_class()

#inizializzazione stato di partenza del sistema
###per inseguimento da posizione corretta
r,rd,rdd,psi,psid,psidd=controller.export_starting_ref()
ss.r,ss.rd,ss.rdd=r,rd,rdd
###
##stabilizzazione (scegliere in linear desired un probl. di stab.)
r=[0,0,2.5]; rd=[0,0,0]; rdd=[0,0,0]; 
ss.r,ss.rd,ss.rdd=r,rd,rdd

controller.start_set(r,rd,rdd) #act,ref
ss.ang,ss.angd,ss.angdd=controller.a_ref.data_export()

u=np.linalg.norm(controller.c_pos.F_tot)
torques=array([0,0,0])
ss.set_f_torques=(u,torques)

data=physic_system_data()
data.store(ss.r,ss.rd,ss.rdd,ss.ang,ss.angd,ss.angdd,ss.u,ss.tau)

controller.delta_t=ss.t1=delta_t=1.0/30

att_params=[10,.4]
pos_params=[1,1]
controller.set_params(pos_params,att_params)

ephi=[]; etheta=[]; epsi=[]
ephid=[]; ethetad=[]; epsid=[]
ex=[]; ey=[]; ez=[]
exd=[]; eyd=[]; ezd=[]

x_ref=[]; y_ref=[]; z_ref=[]; xd_ref=[]; yd_ref=[]; zd_ref=[]; xdd_ref=[]; ydd_ref=[]; zdd_ref=[];
phi_ref=[]; theta_ref=[]; psi_ref=[]; phid_ref=[]; thetad_ref=[]; psid_ref=[]; phidd_ref=[]; thetadd_ref=[]; psidd_ref=[];

t=0;
tt=[0]
iterate=700;

for i in range(iterate):
    print(i)
    tt.append(t+delta_t)
    ss.integration(u,torques)
    
    u,torques=controller.update(ss.r,ss.rd,ss.rdd,ss.ang,ss.angd,ss.angdd)
    
    er,erd,eang,eangd=controller.get_errors()    
    
    ephi.append(eang[0]); etheta.append(eang[1]); epsi.append(eang[2])
    ephid.append(eangd[0]); ethetad.append(eangd[1]); epsid.append(eangd[2])
    
    ex.append(er[0]);ey.append(er[1]);ez.append(er[2])
    exd.append(erd[0]);eyd.append(erd[1]);ezd.append(erd[2]);

    r,rd,rdd,ang,angd,angdd=controller.get_desired()
    x_ref.append(r[0]); y_ref.append(r[1]); z_ref.append(r[2]);
    xd_ref.append(r[0]); yd_ref.append(r[1]); zd_ref.append(r[2]);
    xdd_ref.append(r[0]); ydd_ref.append(r[1]); zdd_ref.append(r[2]);
    phi_ref.append(ang[0]); theta_ref.append(ang[1]); psi_ref.append(ang[2])
    phid_ref.append(ang[0]); thetad_ref.append(ang[1]); psid_ref.append(ang[2])
    phidd_ref.append(ang[0]); thetadd_ref.append(ang[1]); psidd_ref.append(ang[2])
   
pltm.single_states_plot([ss.data.x,ss.data.y,ss.data.z],[np.fmod(ss.data.phi,2*np.pi),np.fmod(ss.data.theta,2*np.pi),np.fmod(ss.data.psi,2*np.pi)])
pltm.single_states_plot([x_ref,y_ref,z_ref],[phi_ref,theta_ref,psi_ref])

#pltm.single_states_plot([ex,ey,ez],[ephi,etheta,epsi])

#pltm.single_states_plot([ex,ey,ez],[ephi,etheta,epsi])

#phi,theta,psi=list(np.fmod([data.phi,data.theta,data.psi],2*np.pi))
#
#pltm.single_states_plot([data.x,data.y,data.z],[phi,theta,psi])

#plt.plot(epsi,epsid)
#plt.plot([0,1,-1],[0,-control.lambdaa,control.lambdaa]) 

x=list(ss.data.x); y=list(ss.data.y); z=list(ss.data.z);
phi=list(ss.data.phi); theta=list(ss.data.theta); psi=list(ss.data.psi);
#pltm.plot_3d([sx,y,z],[phi,theta,psi],'static')
#pltm.plot_3d([x,y,z],[phi,theta,psi],'dynamic')
    










