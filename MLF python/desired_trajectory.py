# -*- coding: utf-8 -*-
"""
Created on Sat Nov 12 17:20:02 2016
creazione traiettoria
come classe, in cui vengono salvati gli stati di riferimento
@author: uso
"""

import numpy as np
from numpy import cos, sin, arctan, arcsin
from numpy.linalg import norm, solve

class desired_trajectory_class():
    def __init__(self):
        self.t=0.0
        self.delta_t=1.0/30
        self.g=9.8 #modulo accelerazione di gravità
        self.m=1 #kg        
            
        #inizilizzazione stato iniziale (tanto la richiesta iniziale viene saltata con un hovering)
        self.x,self.y,self.z,self.vx,self.vy,self.vz,self.ax,self.ay,self.az,self.psi,self.psid,self.psidd=self.linear_equations(self.t)
        self.euler_calc() #per calcolare gli angoli di eulero
        self.phid,self.thetad,self.psid=[0,0,0]
        self.phidd,self.thetadd,self.psidd=[0,0,0]
        
        
    def linear_equations(self,t):
        #test hovering ad altezza 2.5
#        x=0; vx=0; ax=0;
#        y=0; vy=0; ay=0;
#        z=2.5; vz=0; az=0; 
#        psi=1; psid=0; psidd=0;
    
        #test stabilizzazione y/phi : roll subspace #creare sbilanciamento con angolo phi
#        x=0; vx=0; ax=0;
#        y=2; vy=0; ay=0;
#        z=1; vz=0; az=0;
#        psi=0; psid=0; psidd=0;
        
        #test stabilizzazione x/theta : pitch subslace #creare sbilanciamento con angolo theta
#        x=2; vx=0; ax=0;
#        y=0; vy=0; az=0;
#        z=1; vz=0; az=0;
#        psi=0; psid=0; psidd=0;
        
        #test salita e rotazione su Z
#        x=0; vx=0; ax=0;
#        y=0; vy=0; ay=0;
#        z=.5+t; vz=1; az=0;
#        psi=np.fmod(t/10,np.pi*2); psid=1.0/10; psidd=0;
        
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
        
        #circonferenza #veloce
        #print(t)
#        x=cos(t); vx=-sin(t); ax=-cos(t)
#        y=sin(t); vy=cos(t); ay=-sin(t)
#        z=.5; vz=0; az=0;
#        psi=0; psid=0; psidd=0;
        
        #circonferenza #lenta
        #print(t)
        den=1.0/10
        x=cos(t*den); vx=-sin(t*den)*den; ax=-cos(t*den)*(den**2)
        y=sin(t*den); vy=cos(t*den)*den; ay=-sin(t*den)*(den**2)
        z=.5; vz=0; az=0;
        psi=0; psid=0; psidd=0;
        
        #test stabilizzazione XY phi,theta
#        x=2; vx=0; ax=0;
#        y=2; vy=0; ay=0;
#        z=2; vz=0; az=0;
#        psi=0; psid=0; psidd=0;
        
        return [x,y,z,vx,vy,vz,ax,ay,az,psi,psid,psidd]

    def derivation(self,v_act,v_old,delta_t):
        vn=np.array(v_act); vo=np.array(v_old)
        
        dv=(vn-vo)/delta_t
        
        return dv
    
    def update(self):
        for i in range(3): #calcolo  a passo costante con passi più piccoli
            #print(self.t)
            self.t=self.t+self.delta_t/3
            
#            pos_old=[self.x,self.y,self.z]
            self.x,self.y,self.z,self.vx,self.vy,self.vz,self.ax,self.ay,self.az,self.psi,self.psid,self.psidd=self.linear_equations(self.t)
            
#            vel_old=[self.vx,self.vy,self.vz]
#            self.vx,self.vy,self.vz=self.derivation([self.x,self.y,self.z],pos_old,self.delta_t)
#            
#            self.ax,self.ay,self.az=self.derivation([self.vx,self.vy,self.vz],vel_old,self.delta_t)
    
            ang_old=[self.phi,self.theta,self.psi]
            self.euler_calc() #self.psi già c'è
            
            angd_old=[self.phid,self.thetad,self.psid]
            self.phid,self.thetad,self.psid=self.derivation([self.phi,self.theta,self.psi],ang_old,self.delta_t)
            
            self.phidd,self.thetadd,self.psidd=self.derivation([self.phid,self.thetad,self.psid],angd_old,self.delta_t)

    def euler_calc(self):
        """
        Si deve risolvere un problema lineare con incognite [tg(theta) , sin(phi)]
        """
        #calcolo forza da applicare in E-F  e normalizzazione
        F_tot=self.m*np.array([self.ax,self.ay,self.az+self.g])
        #thrust che dovrebbe essere generato        
        u=norm(F_tot)
        #i primi due saranno il termine noto del sistema, il terzo va con la matrice dei coeff
        Fn=F_tot/u
        
        #inizializzazione matrice dei coefficienti
        s_psi=sin(self.psi)
        c_psi=cos(self.psi)
        A=np.array([[c_psi*Fn[2],s_psi],[s_psi*Fn[2],-c_psi]]) #det non nullo se Fn3 non nullo
        
        #soluzione sistema
        [t_theta,s_phi]=solve(A,Fn[:-1])

        #calcolo theta e phi con arctan e arcsin
        self.theta=arctan(t_theta)
        self.phi=arcsin(s_phi)
        
    def data_export(self):
        r=[self.x,self.y,self.z]
        rd=[self.vx,self.vy,self.vz]
        rdd=[self.ax,self.ay,self.az]
        
        ang=[self.phi,self.theta,self.psi]
        angd=[self.phid,self.thetad,self.psid]
        angdd=[self.phidd,self.thetadd,self.psidd]
        
        return(r,rd,rdd,ang,angd,angdd)