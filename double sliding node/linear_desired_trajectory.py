# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 16:57:04 2016

@author: uso
"""

from numpy import array, sin, cos
import numpy as np

class linear_desired_trajectory_class():
    def __init__(self):
        self.t=0.0
        self.delta_t=1.0/30
        self.g=9.8 #modulo accelerazione di gravità
        self.m=1 #kg        
            
        #inizilizzazione stato iniziale (tanto la richiesta iniziale viene saltata con un hovering)
        self.x,self.y,self.z,self.vx,self.vy,self.vz,self.ax,self.ay,self.az,\
                    self.psi,self.psid,self.psidd=self.linear_equations(self.t)
                    
    def linear_equations(self,t):
            #test hovering ad altezza 2.5
#            x=0; vx=0; ax=0;
#            y=0; vy=0; ay=0;
#            z=3; vz=0; az=0; 
#            psi=1; psid=0; psidd=0;
        
            #test stabilizzazione y/phi : roll subspace #creare sbilanciamento con angolo phi
#            x=0; vx=0; ax=0;
#            y=2; vy=0; ay=0;
#            z=1; vz=0; az=0;
#            psi=0; psid=0; psidd=0;
            
            #test stabilizzazione x/theta : pitch subslace #creare sbilanciamento con angolo theta
    #        x=2; vx=0; ax=0;
    #        y=0; vy=0; az=0;
    #        z=1; vz=0; az=0;
    #        psi=0; psid=0; psidd=0;
            
            #test stabilizzazione XYZ
#            x=3; vx=0; ax=0
#            y=2.5; vy=0; ay=0
#            z=1; vz=0; az=0
#            psi=1; psid=0; psidd=0
            
            #test salita e rotazione su Z
#            x=0; vx=0; ax=0;
#            y=0; vy=0; ay=0;
#            z=.5+t; vz=1; az=0;
#            psi=np.fmod(t/10,np.pi*2); psid=1.0/10; psidd=0;
            
            #test 2 salita/discesa e rotazione su Z
    #        x=0; vx=0; ax=0;
    #        y=0; vy=0; ay=0;
    #        z=2+sin(t); vz=cos(t); az=sin(t)
    #        psi=np.fmod(t/10,np.pi*2); psid=1.0/10; psidd=0;
            
            #test movimento lungo y/phi
    #        x=0; vx=0; ax=0;
    #        y=t**2; vy=2*t; ay=2;
    #        z=2; vz=0; az=0;
    #        psi=0; psid=0; psidd=0;
            
            #test traking lungo x/theta
    #        x=t**2; vx=2*t; ax=2;
    #        y=0; vy=0; ay=0;
    #        z=2; vz=0; az=0;
    #        psi=0; psid=0; psidd=0;
       
            
            #movimento lungo x moto uniforme
    #        x=t
    #        y=0
    #        z=.5
    #        psi=0
            
            #circonferenza
            #print(t)
            x=cos(t); vx=-sin(t); ax=-cos(t)
            y=sin(t); vy=cos(t); ay=-sin(t)
            z=cos(t); vz=-sin(t); az=-cos(t);
            psi=t; psid=1; psidd=0;
            
    #        #circonferenza lenta
            #print(t)
#            x=cos(t/3); vx=-sin(t/3)/3; ax=-cos(t/3)/9
#            y=sin(t/3); vy=cos(t/3)/3; ay=-sin(t/3)/9
#            z=1.5; vz=0; az=0;
#            psi=0; psid=0; psidd=0;
            
            #circonferenza lenta
            #print(t)
    #        x=2*cos(t/3); vx=-2*sin(t/3)/3; ax=-2*cos(t/3)/9
    #        y=2*sin(t/3); vy=2*cos(t/3)/3; ay=-2*sin(t/3)/9
    #        z=1.5; vz=0; az=0;
    #        psi=0; psid=0; psidd=0;
            
#            test stabilizzazione XY phi,theta
#            x=2; vx=0; ax=0;
#            y=2; vy=0; ay=0;
#            z=2; vz=0; az=0;
#            psi=0; psid=0; psidd=0;
            
            return [x,y,z,vx,vy,vz,ax,ay,az,psi,psid,psidd]
        
    def update(self):
        self.t=self.t+self.delta_t
        self.x,self.y,self.z,self.vx,self.vy,self.vz,self.ax,self.ay,self.az,\
                self.psi,self.psid,self.psidd=self.linear_equations(self.t)
                
    
    def data_export(self):
        r=array([self.x,self.y,self.z])
        rd=array([self.vx,self.vy,self.vz])
        rdd=array([self.ax,self.ay,self.az])
        
        return(r,rd,rdd,self.psi,self.psid,self.psidd)
        