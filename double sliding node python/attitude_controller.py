# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 17:18:36 2016

@author: uso
"""

from numpy import array, cos, sin, tan
from numpy.linalg import inv
import numpy as np

class attitude_controller_class():
    def __init__(self):
        self.H=array([[8.1e-3,0,0],[0,8.1e-3,0],[0,0,14.2e-3]])
        
        self.lambdaa=10;
        self.Kd=.6;
        
        self.tau=None;
    
        self.t=0
        self.delta_t=1.0/30
        self.error=array([0,0,0])

    def calc_s_c_t(self):
        phi=self.ang_act[0]; theta=self.ang_act[1]; psi=self.ang_act[2];
        
        cphi=cos(phi); sphi=sin(phi);
        
        ctheta=cos(theta); stheta=sin(theta);
        
        cpsi=cos(psi); spsi=sin(psi);
        
        ttheta=tan(theta);
        
        return(cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta)
    
    def  J_calc(self,cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta):
        self.J=array([[1,sphi*stheta,cphi*ttheta],\
                        [0, cphi , -sphi],\
                        [0, sphi/ctheta , cphi/ctheta]])
                        
        self.J_inv=inv(self.J) #oppure J_inv=2*(I+ang'*ang)^-1*[I-x_skewsimm]
        
    def J_d_calc(self,cphi,sphi,ctheta,stheta,cpsi,spsi,ttheta): #non va bene
        phid=self.angd_act[0]; thetad=self.angd_act[1];
        
        self.J_d=array([[0,sphi/(ctheta**2)*thetad+cphi*ttheta*phid,cphi/(ctheta**2)*thetad-sphi*ttheta*phid],\
                [0,-sphi*phid,-cphi*phid],\
                [0,sphi*ttheta/ctheta*thetad+cphi/ctheta*phid,cphi/ctheta*thetad*ttheta-sphi/ctheta*phid]])
        
    def H_s_calc(self):
        self.H_s=np.transpose(self.J_inv).dot(self.H.dot(self.J_inv))
        
    def px_calc(self):
        p=self.H.dot(self.J_inv.dot(self.angd_act))
        self.px=array([[0,-p[2],p[1]],\
                    [p[2],0,p[0]],\
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
    
        
    def update(self,ang,angd,angdd,ang_ref,angd_ref,angdd_ref):
        self.t=self.t+self.delta_t #non serve

        self.set_state(ang,angd,angdd)        
        self.set_desired(ang_ref,angd_ref,angdd_ref)

        self.error=self.ang_act-self.ang_ref
        self.errord=self.angd_act-self.angd_ref
        
        #self.s=self.errord+self.lambdaa*self.error
        
        self.angd_r=self.angd_ref-self.lambdaa*self.error
        
        self.s=self.angd_act-self.angd_r
        
        self.angdd_r=self.angdd_ref-self.lambdaa*self.errord
        
        self.compute()
        self.run_control()
    
    def run_control(self):
        
        self.tau=np.transpose(self.J).dot(self.H.dot(self.angdd_r)-self.C_s.dot(self.angd_r)-self.Kd*self.s)

    def set_state(self,ang,angd,angdd):
        
        self.ang_act=array(ang)
        self.angd_act=array(angd)
        self.angdd_act=array(angdd)
        
    def set_desired(self,ang,angd,angdd):
        
        self.ang_ref=array(ang)
        self.angd_ref=array(angd)
        self.angdd_ref=array(angdd)