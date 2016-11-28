# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 17:14:50 2016

@author: uso
"""

from numpy import array#, cos, sin, tan
#from numpy.linalg import inv
#import numpy as np
from numpy.linalg import norm

#from transformations import euler_matrix

class newton_controller_class():
    def __init__(self):
        self.g=9.8
        self.m=1
        self.lambdan=1
        self.Kd=1
        
        self.r=array([None,None,None])
        self.rd=array([None,None,None])
        self.rdd=array([None,None,None])
    
        self.e=array([0,0,0])
        
        self.delta_t=1.0/30
        
        self.t=0;
    
    def update(self,r,rd,rdd,r_ref,rd_ref,rdd_ref):
        self.t=self.t+self.delta_t
        
        self.set_state(r,rd,rdd)
        self.set_desired(r_ref,rd_ref,rdd_ref)
        
        self.e=self.r_act-self.r_ref
        self.ed=self.rd_act-self.rd_ref
        
        self.s=self.ed+self.lambdan*self.e
        
        self.run_control()
    
    def run_control(self):
        
        self.F_tot=self.m*(array([0,0,self.g])-self.Kd*self.s+self.rdd_ref-self.lambdan*self.ed)
    
    def set_state(self,r,rd,rdd):
        self.r_act=array(r)
        self.rd_act=array(rd)
        self.rdd_act=array(rdd)
        
        
    def set_desired(self,r,rd,rdd):
        self.r_ref=array(r)
        self.rd_ref=array(rd)
        self.rdd_ref=array(rdd)
    