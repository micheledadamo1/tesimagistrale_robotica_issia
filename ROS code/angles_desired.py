# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 17:03:31 2016

@author: uso
"""

import numpy as np
from numpy import cos, sin, arctan, arcsin
from numpy.linalg import norm, solve

class angles_desired_trajectory_class():
    def __init__(self):
        self.t=0.0
        self.delta_t=1.0/30
        
        #mandare in esecuzione start_set
        
        self.phid,self.thetad=[0,0]
        self.phidd,self.thetadd=[0,0]
        
    def euler_calc(self,z_vec,psi,psid,psidd):
        self.psi=psi; self.psid=psid; self.psidd=psidd
        
        Fn=z_vec/norm(z_vec)
        
        #inizializzazione matrice dei coefficienti
        s_psi=sin(self.psi)
        c_psi=cos(self.psi)
        A=np.array([[c_psi*Fn[2],s_psi],[s_psi*Fn[2],-c_psi]]) #det non nullo se Fn3 non nullo
        
        #soluzione sistema
        [t_theta,s_phi]=solve(A,Fn[:-1])

        #calcolo theta e phi con arctan e arcsin
        self.theta=arctan(t_theta)
        self.phi=arcsin(s_phi)
        
    def update(self,z_vec,psi,psid,psidd):
        
        for i in range(3):        
            self.t=self.t+self.delta_t
            
            ang_old=[self.phi,self.theta]
            self.euler_calc(z_vec,psi,psid,psidd)
            
            angd_old=[self.phid,self.thetad]
            self.phid,self.thetad=self.derivation([self.phi,self.theta],ang_old,self.delta_t)
            
            self.phidd,self.thetadd=self.derivation([self.phid,self.thetad],angd_old,self.delta_t)

    def derivation(self,v_act,v_old,delta_t):
        vn=np.array(v_act); vo=np.array(v_old)
        
        dv=(vn-vo)/delta_t
        
        return dv  

    def data_export(self):    
        ang=[self.phi,self.theta,self.psi]
        angd=[self.phid,self.thetad,self.psid]
        angdd=[self.phidd,self.thetadd,self.psidd]
        
        return(ang,angd,angdd)
        
    
        
    def start_set(self,z_vec,psi,psid,psidd):
        self.euler_calc(z_vec,psi,psid,psidd) #per calcolare gli angoli di eulero
        
        
        
        
        
        
        