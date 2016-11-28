# -*- coding: utf-8 -*-
"""
Created on Sat Nov 12 11:00:48 2016
classe per memorizzare stati del systema fisico
@author: uso
"""

class physic_system_data():
    def __init__(self):
        self.x,self.y,self.z=[],[],[]
        self.phi,self.theta,self.psi=[],[],[]
        
        self.vx,self.vy,self.vz=[],[],[]
        self.phid,self.thetad,self.psid=[],[],[]
        
        self.ax,self.ay,self.az=[],[],[]
        self.phidd,self.thetadd,self.psidd=[],[],[]
        
        self.F_tot=[]
        self.tau_phi=[]; self.tau_theta=[]; self.tau_psi=[]
    
    def store(self,r,rd,rdd,ang,angd,angdd,F_tot,tau):
        self.store_pos(r)
        self.store_vel(rd)
        self.store_acc(rdd)

        self.store_ang(ang)
        self.store_angd(angd)
        self.store_angdd(angdd)
        
        self.store_F_tot(F_tot)
        
        self.store_torques(tau)
        
    def store_pos(self,r):
        self.x.append(r[0])
        self.y.append(r[1])
        self.z.append(r[2])
        
    def store_vel(self,rd):
        self.vx.append(rd[0])
        self.vy.append(rd[1])
        self.vz.append(rd[2])    
    
    def store_acc(self,rdd):
        self.ax.append(rdd[0])
        self.ay.append(rdd[1])
        self.az.append(rdd[2])
          
    def store_ang(self,ang):
        self.phi.append(ang[0])
        self.theta.append(ang[1])
        self.psi.append(ang[2])
        
    def store_angd(self,angd):
        self.phid.append(angd[0])
        self.thetad.append(angd[1])
        self.psid.append(angd[2])
        
    def store_angdd(self,angdd):
        self.phidd.append(angdd[0])
        self.thetadd.append(angdd[1])
        self.psidd.append(angdd[2])
        
    def store_F_tot(self,F_tot):
        self.F_tot.append(F_tot)
    
    def store_torques(self,tau):
        self.tau_phi.append(tau[0])
        self.tau_theta.append(tau[1])
        self.tau_psi.append(tau[2])

        