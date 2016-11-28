# -*- coding: utf-8 -*-
"""
Created on Fri Nov 18 22:20:39 2016

@author: uso
"""

import numpy as np
from numpy import array, sin, cos, tan
from numpy.linalg import inv
from transformations import euler_matrix

from scipy.integrate import ode


class euler_system_class():
    def __init__(self):        

        self.ang=None
        self.angd=None
        self.angdd=None
        
        self.H=array([[8.1e-3,0,0],[0,8.1e-3,0],[0,0,14.2e-3]]) #nel nostro caso è costante
        
        self.tau=[None,None,None]
        
        self.t1=1.0/30 #intervallo di tempo in cui calcolare l'evoluzione
        
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
        

    def differential_equation(self,t,z0):
        #self.tt=t
        #print(t)
        
        self.ang=array(z0[0:3]);
        self.angd=array(z0[3:]);
        
        self.compute()
        self.angdd=inv(self.H_s).dot(np.transpose(self.J_inv).dot(self.tau)-self.C_s.dot(self.angd))
        
        zd=np.append(self.angd,self.angdd)

        return zd
        
    def integration(self,tau):
        self.tau=tau

        z0,t0= np.append(self.ang,self.angd),0

        t1=self.t1
                
        #dt=self.t1/30
        
        r= ode(self.differential_equation).set_integrator('lsoda',atol=1e-7 , rtol=1e-5)# , min_step=1.0/100 , max_step=1.0/30)
        #r= ode(self.differential_equation).set_integrator('dopri5')#,nsteps=20)
        r.set_initial_value(z0,t0)
        
        z=r.integrate(t1) #i valori sono già 
        
        self.ang=z[0:3]; self.angd=z[3:6] #li aggiorno nella function della ode
        #print('b',self.ang)        
        #self.angdd e self.rdd già aggiornati
        
        #print('fine')
    
    def set_torques(self,tau):
        self.tau=tau