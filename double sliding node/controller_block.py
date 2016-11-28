# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 17:23:16 2016

@author: uso
"""

from linear_desired_trajectory import linear_desired_trajectory_class
from newton_controller import newton_controller_class
from angles_desired import angles_desired_trajectory_class
from attitude_controller import attitude_controller_class

#from numpy import array
from numpy.linalg import norm

class full_nl_controller_class():
    def __init__(self):
        self.c_pos=newton_controller_class()
        self.c_att=attitude_controller_class()
        self.l_ref=linear_desired_trajectory_class()
        self.a_ref=angles_desired_trajectory_class()
        
        self.u=None
        self.torques=None
        
        self.delta_t=self.c_pos.delta_t=self.c_att.delta_t=\
            self.l_ref.delta_t=self.a_ref.delta_t=1.0/30 #uniformo intervalli di campionamento
    
    def start_set(self,r_act,rd_act,rdd_act):
        r_ref,rd_ref,rdd_ref,psi_ref,psid_ref,psidd_ref=self.l_ref.data_export()
        
        self.c_pos.update(r_act,rd_act,rdd_act,r_ref,rd_ref,rdd_ref)
        z_vec=self.c_pos.F_tot
        
        self.a_ref.start_set(z_vec,psi_ref,psid_ref,psidd_ref) #completata l'inizializzazione di c_att
            
    def update(self,r_act,rd_act,rdd_act,ang_act,angd_act,angdd_act):
        self.l_ref.update()
        r_ref,rd_ref,rdd_ref,psi_ref,psid_ref,psidd_ref=self.l_ref.data_export()
        
        self.c_pos.update(r_act,rd_act,rdd_act,r_ref,rd_ref,rdd_ref) #uscita .F_tot
        z_vec=self.c_pos.F_tot
        
        self.a_ref.update(z_vec,psi_ref,psid_ref,psidd_ref)
        ang_ref,angd_ref,angdd_ref=self.a_ref.data_export()
        
        self.c_att.update(ang_act,angd_act,angdd_act,ang_ref,angd_ref,angdd_ref) #uscita tau (torques)
    
        return(norm(self.c_pos.F_tot),self.c_att.tau) #restituisco il modulo della forza+torques (input plant/sistema)
        
    def export_starting_ref(self):
        r,rd,rdd,psi,psid,psidd=self.l_ref.data_export()
        return(r,rd,rdd,psi,psid,psidd)
        
    def set_params(self,pos,ang):
        """
        pos, ang liste di due elementi
        """
        self.c_pos.lambdan,self.c_pos.Kd=pos
        
        self.c_att.lambdaa,self.c_att.Kd=ang
    
    def get_errors(self):
        return(self.c_pos.e,self.c_pos.ed,self.c_att.error,self.c_att.errord)
        
    def get_desired(self):
        r,rd,rdd,psi,psid,psidd=self.l_ref.data_export()
        ang,angd,angdd=self.a_ref.data_export()
        return(r,rd,rdd,ang,angd,angdd)