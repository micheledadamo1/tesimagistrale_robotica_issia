# -*- coding: utf-8 -*-
"""
Created on Fri Nov 18 22:34:18 2016

@author: uso
"""

import plot_methods as pltm
from euler_system import euler_system_class  as system
from numpy import array
from data_stored import physic_system_data

ss=system()

#stato di partenza del sistema
ss.ang=array([0,0,0])
ss.angd=array([0,0,0])
ss.angdd=array([0,0,0])

r=[0,0,0]
rd=[0,0,0]
rdd=[0,0,0]

tau=array([1,1,0])
ss.set_torques(tau)

data=physic_system_data()
data.store(r,rd,rdd,ss.ang,ss.angd,ss.angdd,0,ss.tau)


#ss.t1=2
for i in range(100):
    print(i)
    ss.integration(tau)
    data.store(r,rd,rdd,ss.ang,ss.angd,ss.angdd,0,ss.tau)
    
pltm.plot_3d([data.x,data.y,data.z],[data.phi,data.theta,data.psi],'static')