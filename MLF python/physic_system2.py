# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 17:30:29 2016
classe sistema fisico
@author: uso
"""

import numpy as np

from scipy.integrate import ode

class physic_system_class():
    def __init__(self):
#        self.Ixx=8.1*10**(-3) #8.1*10**(-3) kg*m^2
#        self.Iyy=8.1*10**(-3) #8.1*10**(-3) kg*m^2
#        self.Izz=14.2*10**(-3) #14.2*10**(-3) kg*m^2
        self.I=np.array([[8.1e-3,0,0],[0,8.1e-3,0],[0,0,14.2e-3]])
        self.I_1=np.linalg.inv(self.I)
        self.m=1 #kg
        
        self.r=np.array([None,None,None])
        self.ang=np.array([None,None,None])
        
        self.rd=np.array([None,None,None])
        self.angd=np.array([None,None,None])
        
        self.rddd=np.array([None,None,None])

        self.angdd=np.array([None,None,None])
        
        self.F_tot=None
        self.torques=np.array([None,None,None])
        
        self.delta_t=1.0/60
        self.t0=0.0
        self.t=0.0; #per integrare        
        self.t1=1.0/30
        self.g=9.8 #potrebbe essere utile nei test, modulo della forza di gravità con segno
        
        self.ode_solver=ode(self.differential_equation).set_integrator('lsoda', atol=1e-7, rtol=1e-5) #, min_step=1.0/200, max_step=1.0/30)

    def update(self,F_tot,tau):
        self.F_tot=F_tot
        self.torques=np.array(tau)
        #aggiornamento attuali (sistema) 
        #integrazione
        
        state0,t0 = np.append(self.r,np.append(self.rd,np.append(self.ang,self.angd))),0
        
        self.ode_solver.set_initial_value(state0,t0)

        sol=self.ode_solver.integrate(self.t1)
    
    def R_3_calc(self):
        phi=self.ang[0]; theta=self.ang[1]; psi=self.ang[2]
        
        self.R_3=np.array([np.sin(psi)*np.sin(phi)+np.cos(psi)*np.sin(theta)*np.cos(phi),\
                    -np.cos(psi)*np.sin(phi)+np.sin(psi)*np.sin(theta)*np.cos(phi),\
                    np.cos(theta)*np.cos(phi)])
    
    def differential_equation(self,t,state):
#        print(t)

        self.r=state[0:3]
        
        self.ang=state[6:9]
        
        fatt=self.F_tot/self.m #valutare f tot in mlf dopo scelta
        
        #eq. Newton
        self.rd=state[3:6]
        self.R_3_calc() #self.ang aggiornato già
        self.rdd=self.R_3*fatt+np.array([0,0,-self.g])
        
        #eq.Eulero    
        self.angd=state[9:]
        self.angdd=self.I_1.dot(self.torques)-np.cross( self.angd , self.I.dot(self.angd) )

        
        return(np.append(self.rd,np.append(self.rdd,np.append(self.angd,self.angdd))))
        
        
        
    def set_state(self,r,rd,rdd,ang,angd,angdd,F_tot,tau):
        """
        metodo per settare lo stato iniziale (utilizzato per testare)
        """

        self.r=r; self.rd=rd; self.rdd=rdd
        
        self.ang=ang; self.angd=angd; self.angdd=angdd       
        
        self.F_tot=F_tot
        self.torques=np.array(tau)
    
#    def export_data(self): #non serve più
#        """
#        metodo per esportare i dati in liste invece che uno alla volta (leggibilità codice)
#        """
#        r=[self.x,self.y,self.z]
#        rd=[self.vx,self.vy,self.vz]
#        rdd=[self.ax,self.ay,self.az]
#        
#        ang=[self.phi,self.theta,self.psi]
#        angd=[self.phid,self.thetad,self.psid]
#        angdd=[self.phidd,self.thetadd,self.psidd]
#        
#        F_tot=self.F_tot
#        
#        tau=[self.tau_phi,self.tau_theta,self.tau_psi]
#        
#        return [r,rd,rdd,ang,angd,angdd,F_tot,tau]
        