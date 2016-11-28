# -*- coding: utf-8 -*-
"""
Created on Fri Nov 18 18:52:50 2016

@author: uso
"""

import numpy as np
from numpy import array, sin, cos, tan
from numpy.linalg import inv
from tf.transformations import euler_matrix

from data_stored import physic_system_data

from scipy.integrate import ode


class physic_system_class():
    def __init__(self):        
        self.r=None
        self.rd=None
        self.rdd=None
        self.ang=None
        self.angd=None
        self.angdd=None

        self.m=1.0 #kg      
        self.g=9.8 #accel. di gravità
        self.H=array([[8.1e-3,0,0],[0,8.1e-3,0],[0,0,14.2e-3]]) #nel nostro caso è costante
        
        self.tau=[None,None,None]
        self.u=None
        
        self.t1=1.0/30 #intervallo di tempo in cui calcolare l'evoluzione
        
        self.data=physic_system_data()
        
        self.time=[0];
    def calc_s_c_t(self):
        phi=self.ang[0]; theta=self.ang[1]; psi=self.ang[2];
        
        cphi=cos(phi); sphi=sin(phi);
        
        ctheta=cos(theta); stheta=sin(theta);
        
        cpsi=cos(psi); spsi=sin(psi);
        
        ttheta=tan(theta);
        
        return(cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta)
    
    def J_calc(self,cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta):
        self.J=array([[1,sphi*ttheta,cphi*ttheta],\
                        [0, cphi , -sphi],\
                        [0, sphi/ctheta , cphi/ctheta]])
                        
        self.J_inv=inv(self.J) #oppure J_inv=2*(I+ang'*ang)^-1*[I-x_skewsimm]
        
    def J_d_calc(self,cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta): #non va bene
        phid=self.angd[0]; 
        thetad=self.angd[1];
        
        self.J_d=array([[0,sphi/(ctheta**2)*thetad+cphi*ttheta*phid,cphi/(ctheta**2)*thetad-sphi*ttheta*phid],\
        [0,-sphi*phid,-cphi*phid],\
        [0,sphi*ttheta/ctheta*thetad+cphi/ctheta*phid,cphi/ctheta*thetad*ttheta-sphi/ctheta*phid]])
        
    def H_s_calc(self):
        self.H_s=np.transpose(self.J_inv).dot(self.H.dot(self.J_inv))
        
    def px_calc(self):
        p=self.H.dot(self.J_inv.dot(self.angd))
        self.px=array([[0,-p[2],p[1]],\
                    [p[2],0,-p[0]],\
                    [-p[1],p[0],0]])
        
    def C_s_calc(self):
        add1=self.J_inv.dot(self.J_d.dot(self.J_inv))
        add1=np.transpose(self.J_inv).dot(self.H.dot(add1))
        add2=np.transpose(self.J_inv).dot(self.px.dot(self.J_inv))
        
        self.C_s=-add1-add2
        
    def compute(self):
        cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta=self.calc_s_c_t()
        self.J_calc(cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta)
        self.J_d_calc(cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta)
        self.px_calc()
        
        self.H_s_calc()
        self.C_s_calc()
        
    
    def F_tot_calc(self):
        phi,theta,psi=self.ang #in realtà utilizzando solo la terza colonna e avendo già calcolato sin cos basterenne riscriverla
        R=euler_matrix(phi,theta,psi,'sxyz')
        R=R[:-1,:-1]
        self.F_tot=R.dot(array([0,0,self.u]))
        
        
    def differential_equation(self,t,z0):
        #print(t)
        #posizioni e velocità
        self.r=array(z0[0:3]);
        self.rd=array(z0[3:6]);
        self.ang=array(z0[6:9]);
        self.angd=array(z0[9:]);
        
        #aggiornamento per le accelerazioni date le condioni attuali (iniziali)
        self.F_tot_calc()
        self.compute()
        
        #sistema
        self.rdd=self.F_tot/self.m+array([0,0,-self.g])
        self.angdd=inv(self.H_s).dot(np.transpose(self.J_inv).dot(self.tau)-self.C_s.dot(self.angd))
        
        zd=np.append(self.rd,np.append(self.rdd,np.append(self.angd,self.angdd)))
        
        return zd
                
    def integration(self,u,tau):
        self.tau=tau
        self.u=u
        
        z0,t0= np.append(self.r,np.append(self.rd,np.append(self.ang,self.angd))),0
        
        t1=self.t1
        #dt=1.0/100
        
        self.t_old=0        
        
        #r= ode(self.differential_equation).set_integrator('lsoda',atol=1e-7 , rtol=1e-5)
        r= ode(self.differential_equation).set_integrator('dopri5',atol=1e-7 , rtol=1e-5)
        r.set_initial_value(z0,t0)
        self.update_data()
        #print(z0)
#        while r.successful() and r.t < t1:
#            z=r.integrate(r.t+dt)
#            print(r.t+dt, r.integrate(r.t+dt))
            
        z=r.integrate(t1) #i valori sono già 

        #self.r=z[0]; self.rd=z[1]; self.ang=z[2]; self.angd=z[3]
        #self.angdd e self.rdd già aggiornati
    
    def set_F_torques(self,F_tot,tau):
        self.u=F_tot
        self.tau=array(tau)
        
    def update_data(self):
        self.data.store(self.r,self.rd,self.rdd,self.ang,self.angd,self.angdd,self.u,self.tau)

        
    def prevision_ROS(self,ang,angd,tau):
        """
        metodo aggiunto per l'utilizzo sperimentale in ROS per il calcolo dell'orientazione
        desiderata. Serve un metodo per poter settare questa classe con le informazioni attuali 
        ricevute dai sensori. L'obbiettivo è utilizzare l'integrazione.
        input: ang,angd,angdd valori attuali misurati dai sensori
                tau ottenuto dal controllore
        r,rd,rdd,u,angdd non servono per integrare solo le equazioni di eulero
        """
        #per non avere errrìori
        self.r=array([0,0,0])
        self.rd=array([0,0,0])
        self.rdd=array([0,0,0])
        self.angdd=array([0,0,0])

        self.ang=ang
        self.angd=angd

        self.integration(0,array(tau))

        return(self.ang,self.angd)