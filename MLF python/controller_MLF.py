# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 22:04:46 2016
classe controllore MLF
@author: uso
"""

import numpy as np
from numpy import cos, sin
from desired_trajectory import desired_trajectory_class

#import warnings

class MLF_class():
    def __init__(self):
        self.Ixx=8.1*10**(-3) #8.1*10**(-3) kg*m^2
        self.Iyy=8.1*10**(-3) #8.1*10**(-3) kg*m^2
        self.Izz=14.2*10**(-3) #14.2*10**(-3) kg*m^2

        #calcolo già da ora i coefficienti dei tau_i
        self.coeff_tau_phi=(self.Iyy-self.Izz)/self.Ixx
        self.coeff_tau_theta=(self.Izz-self.Ixx)/self.Iyy
        self.coeff_tau_psi=(self.Ixx-self.Iyy)/self.Izz
        
        self.m=1.0 #kg
        self.g=9.8 #m/s**2
        
        self.delta_t=1.0/30
        
        #velocità di scivolamento
        self.lambda_x=.1; self.lambda_y=.1; self.lambda_z=15
        
        self.lambda_phi=.1; self.lambda_theta=.1; self.lambda_psi=1;
        
        #velocità di convergenza alla superf. di sliding
        self.Kvx=.1; self.Kvy=.1; self.Kvz=10; 
        
        self.Kvphi=.1; self.Kvtheta=.1; self.Kvpsi=1;
        
        self.epss=5*np.finfo(float).eps #per eliminare momentaneamente il den. nullo (test)
        

        #ho bisogno di due flag: start e go_control
        #start -> mi serve per far un ciclo in hovering in modo da ottenere un errore
        #vecchio e poter derivare
        #go_control -> mi serve per i casi in cui il denominatore di qualche
        #sottospazio si annulli, allora rimango in hovering (in realtà bisognerebbe vedere se anche l'altro è nullo, prob. no)
        #        
        #start==1 hovering
        #go_control==0 hovering
        self.start=1
        self.go_control=0
        
        self.reference_traj=desired_trajectory_class() #ho già uno stato iniziale
        self.set_ref()
        
        self.id1=None
        
        self.noYS=0 #per inibire YS porre =1
        
    def errors(self, ref , act , delta_t, e_prev ):
        """
        calcolo errore e derivate errori
        """
        e=act-ref
        ed=(e-e_prev)/delta_t
        
        return [e,ed]
        
    def PS(self):
        
        self.ex,self.exd=self.errors(self.x_ref,self.x_act,self.delta_t,self.ex)
        self.etheta,self.ethetad=self.errors(self.theta_ref,self.theta_act,self.delta_t,self.etheta)
        
        #sliding manifolds
        self.sx=self.exd+self.lambda_x*self.ex
        self.stheta=self.ethetad+self.lambda_theta*self.etheta
        self.s_p=np.array([self.sx,self.stheta]) #per la scelta del sottospazio
        
        self.up=self.m/(self.den_up+self.epss)*(self.xdd_ref-self.lambda_x*self.exd-self.Kvx*self.sx)
        self.tau_theta=self.Iyy*(self.thetadd_ref-self.coeff_tau_theta*self.phid_act*self.psid_act-self.lambda_theta*self.ethetad-self.Kvtheta*self.stheta)
#        print(self.thetadd_ref,self.ethetad,self.etheta)
#        print(self.theta_ref)
        
    def RS(self):
        self.ey,self.eyd=self.errors(self.y_ref,self.y_act,self.delta_t,self.ey)
        self.ephi,self.ephid=self.errors(self.phi_ref,self.phi_act,self.delta_t,self.ephi)        
        
        #sliding manifolds
        self.sy=self.eyd+self.lambda_y*self.ey
        self.sphi=self.ephid+self.lambda_phi*self.ephi
        
        self.s_r=np.array([self.sy,self.sphi])        
        
        self.ur=self.m/(self.den_ur+self.epss)*(self.ydd_ref-self.lambda_y*self.eyd-self.Kvy*self.sy)
        self.tau_phi=self.Ixx*(self.phidd_ref-self.lambda_phi*self.ephid-self.coeff_tau_phi*self.thetad_act*self.psid_act-self.Kvphi*self.sphi)
    
    def YS(self):
        self.ez,self.ezd=self.errors(self.z_ref,self.z_act,self.delta_t,self.ez)
        self.epsi,self.epsid=self.errors(self.psi_ref,self.psi_act,self.delta_t,self.epsi)
        
        #sliding manifolds
        self.sz=self.ezd+self.lambda_z*self.ez
#        print('ez',self.ez)
#        print('ezd',self.ezd)
#        print('sz',self.sz)
        self.spsi=self.epsid+self.lambda_psi*self.epsi        
        self.s_y=np.array([self.sz,self.spsi])
        
        self.uy=self.m/self.den_uy*(self.zdd_ref-self.lambda_z*self.ezd+self.g-self.Kvz*self.sz)
#        print('zdd_ref',self.zdd_ref)
#        print('uy',self.uy)
        """        
        osservazione andamento controllore, osservando i valori di uy e tau_psi si vede
        che questi sono alternati : uy è qualcosa, 9.8 , qualcosa, 9.8 ,
        mentre tau_psi: qualcosa ,0.0, qualcosa ,0.0
        credo abbia senso perchè questo sta ad indicare che l'accelerazione che ho richiesto 
        prima, che era calcolata sugli errori, è sufficiente anche ora, per cui mi assicuro di non decelerare con 9.8
        questo vuol dire che si da un comando a impulsi... buono?
        """
        self.tau_psi=self.Izz*(self.psidd_ref-self.lambda_psi*self.epsid-self.coeff_tau_psi*self.phid_act*self.thetad_act-self.Kvpsi*self.spsi)
        
    def update_denominator_subspaces(self):
        """
        metodo per rendere legibile le formule degli u_i,
        calcolando i vari denominatori qui
        c'è anche una agevolazione per cui i seni e coseni vengono calcolati
        una volta sola
        """
        c_phi=cos(self.phi_act); s_phi=sin(self.phi_act)
        c_theta=cos(self.theta_act); s_theta=sin(self.theta_act)
        c_psi=cos(self.psi_act); s_psi=sin(self.psi_act)
        
        self.den_up=c_psi*c_phi*s_theta+s_psi*s_phi
        self.den_ur=s_phi*s_theta*c_phi-c_psi*s_phi
        self.den_uy=c_theta*c_phi
        
        if (self.den_up==0)  or (self.den_ur==0):
            #warnings.warn('un denominatore è nullo')
            print('warning:un denominatore è nullo')            
            self.go_control=0
        else:
            self.go_control=1
            
        
    def run_MLF(self):
        
        self.update_denominator_subspaces() #se escono nulli (li metterei vicini a zero) allora vado in hovering
        
#        if self.go_control==0:
#            [self.F_tot,self.tau_phi,self.tau_theta,self.tau_psi]=[9.8,0,0,0]
#        else:
        self.reference_traj.update() #aggiorno
        self.set_ref()
        
        self.PS()
        self.RS()
        if self.noYS==0: #per lavorare solo con gli altri due
            self.YS()
            self.Vy=self.s_y.dot(self.s_y)
        elif self.noYS==1:
            self.Vy=-1
            self.uy=9.8
            self.tau_psi=0
            
        #valuto norma di s_i  , da paper dovrebbe usarsi Pp
                #MLF
        self.Vp=self.s_p.dot(self.s_p)
        self.Vr=self.s_r.dot(self.s_r)
        #self.Vy=self.s_y.dot(self.s_y) #è fatto sopra
        
        #print([self.Vp, self.Vr, self.Vy])        
        
        self.V_sigma=max([self.Vp, self.Vr, self.Vy]) 
        
        
        id1=[self.Vp, self.Vr, self.Vy]==self.V_sigma

        id1=[i for i in range(len(id1)) if id1[i]==True]
        ######da togliere bypass            
        #per bypassare la scelta automatica creo un metodo per l'immissione        
        #val=self.bypass_switch() #tolto per l'usi di self.id1, per bypassare e richiamare dall'esterno
        if self.id1==None:        
            id1=id1[0]
        else:
            id1=self.id1
        ######
        id=["pitch","roll","yaw"] #potrebbe mettersi in init (con self)
        
        self.id_MLF=id[id1]

        print(self.id_MLF)
        self.send_switch={"pitch":[self.up,0.0,self.tau_theta,0.0],
                          "roll":[self.ur,self.tau_phi,0.0,0.0],
                          "yaw": [self.uy,0.0,0.0,self.tau_psi]}
        
        [self.F_tot,self.tau_phi,self.tau_theta,self.tau_psi]=self.send_switch[self.id_MLF]
        #[self.F_tot,tau_phi,tau_theta,tau_psi]=self.send_switch[self.id_MLF] #per prendere tutti i torques invece di uno solo
        #print(self.send_switch[self.id_MLF])
        
    #i due metodi nella simulazione python vengono richiamati esternamente, in ROS l'update
    #attuale gira da solo alla velocità dei sensori
    #update ref gira quando gira il primo quindi è tutto interno ed automatico    
    def update_act(self,r,rd,rdd,ang,angd,angdd): #simulazione ricezione dati dai publisher
        """
        metodo per il set dei dati attuali, in python da physic_state, in ros è sostituito dalle callback
        """
        self.x_act,self.y_act,self.z_act=r
        self.vx_act,self.vy_act,self.vz_act=rd
        self.ax_act,self.ay_act,self.az_act=rdd

        self.phi_act,self.theta_act,self.psi_act=ang
        self.phid_act,self.thetad_act,self.psid_act=angd
        self.phidd_act,self.thetadd_act,self.psidd_act=angdd
        
        #quando ricevo il dato, si mette in funzione il controllore
        if self.start==1:
            # prima valutazione errori
            self.ex,self.ey,self.ez=self.x_act-self.x_ref , self.y_act-self.y_ref , self.z_act-self.z_ref
            self.ephi,self.etheta,self.epsi=self.phi_act-self.phi_ref , self.theta_act-self.theta_ref , self.psi_act-self.psi_ref
            self.start=0
            self.go_control=0
            self.run_MLF()
        else:
            self.reference_traj.update() #aggiorno     #qui in ros andrebbe inserito come input delta_t (quindi modificare anche des. traj)       
            self.set_ref()
            self.run_MLF()
        
    def set_ref(self):
        """
        metodo per il settaggio dei valori di riferimento (viene richiamato quando si modifica lo stato attuale)
        """
        [r,rd,rdd,ang,angd,angdd]=self.reference_traj.data_export() #esporto da desired traj.
        
        self.x_ref,self.y_ref,self.z_ref=r
        self.xd_ref,self.yd_ref,self.zd_ref=rd
        self.xdd_ref,self.ydd_ref,self.zdd_ref=rdd
        
        self.phi_ref,self.theta_ref,self.psi_ref=ang
        self.phid_ref,self.thetad_ref,self.psid_ref=angd
        self.phidd_ref,self.thetadd_ref,self.psidd_ref=angdd
              
        
        
        
    