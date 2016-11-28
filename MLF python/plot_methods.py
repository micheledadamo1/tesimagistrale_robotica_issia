# -*- coding: utf-8 -*-
"""
Created on Sat Nov 12 11:33:28 2016
serie di metodi utilizzati per plottare stato sistema fisico
(o anche per il plot di dati desiderati)

tutto è inteso con la convenzione roll-pitch-yaw angles (angoli di eulero)
(laddove si utilizzino rappresentazioni 3D)

@author: uso
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from transformations import euler_matrix


    
def plot_single_variable(var,time=[],var_name='unnamed'):
    """
    per plottare una variabile rispetto al tempo
    var e time liste unidimensionali
    se non c'è il tempo mette iterate
    la stringa var_name serve per farla uscire nel titolo
    
    questo metodo in realtà è generico, non usa i valori della classe
    """
    if time==[]:
        time=range(len(var))
        
    plot2d=plt.figure(1)
    plot2d.suptitle(var_name+' vs  time')
    plt.plot(time,var,'-k')
    plt.xlabel('time')
    plt.ylabel(var_name)

def single_states_plot(r1,r2,time=[]):
    """
    metodo per il plot in una figura di  sei  stati
    r1 prime tre variabili da plottare
    r2 seconte tre variabili da plottare
    
    (in entrambi i casi stringe del tipo [[],[],[]])

    assicurarsi che r1 ed re siano di variabili della stessa lunghezza
    cioè tutte posizioni, velocità o accelerazioni

    per non complicare ancora: evitato inserimento stringa nome variabili  
   
    """
    if time==[]:
        time=range(len(r1[0]))
    
    plotting=plt.figure(2)
    plotting.suptitle('states vs time')
    
    #plotto variabili posizione
    plt.subplot(231)
    plt.ylabel('var1/x')
    plt.xlabel('time')
    plt.plot(time,r1[0])
    
    plt.subplot(232)
    plt.ylabel('var2/y')
    plt.xlabel('time')
    plt.plot(time,r1[1])

    plt.subplot(233)
    plt.ylabel('var3/z')
    plt.xlabel('time')
    plt.plot(time,r1[2])
    
    #plotto angoli di eulero
    plt.subplot(234)
    plt.ylabel('var4/phi')
    plt.xlabel('time')
    plt.plot(time,r2[0])
    
    plt.subplot(235)
    plt.ylabel('var5/theta')
    plt.xlabel('time')
    plt.plot(time,r2[1])

    plt.subplot(236)
    plt.ylabel('var6/psi')
    plt.xlabel('time')
    plt.plot(time,r2[2])
    
def plot_3d(r1,r2,opt='static'):
    """
    metodo per il plot del sistema nel tempo
    l'orientazione è rappresentata dagli assi del sistema corpo
    opt='static' o 'dynamic'
    static-> plotto tutto in una volta (sovrapposizione)
    dynamic-> plotto con figure ad intervalli di tempo (motion)
    
    il metodo è più generale, l'utilizzo ne ha determinato la descrizione
    r1 ed r2 della stessa dimensione
    r1 inteso come pozisione
    r2 orientazione
    """
    x,y,z=r1
    phi,theta,psi=r2
    
    fig = plt.figure(3)
    ax = Axes3D(fig)
    ax = fig.add_subplot(111, projection='3d')

    if not(opt=='static') and not(opt=='dynamic'):
        opt=='static'
     
    for i in range(len(r1[0])):
        #earthframe (statico)
        ax.plot([0,1],[0,0],[0,0],'b');ax.plot([0,0],[0,1],[0,0],'r');ax.plot([0,0],[0,0],[0,1],'k')
        
        #body frame act
        [X,Y,Z]=rotation_axes(phi[i],theta[i],psi[i])
        
        #posizione origine
        origin=[x[i],y[i],z[i]]
        
        [plot_x,plot_y,plot_z]=to_plot_axes(origin,X,Y,Z)
        ax.plot(plot_x[0],plot_x[1],plot_x[2],'b',label='X')
        ax.plot(plot_y[0],plot_y[1],plot_y[2],'r',label='Y')
        ax.plot(plot_z[0],plot_z[1],plot_z[2],'k',label='Z')
        
        if opt=='dynamic':
            ax.set_aspect('equal','box')
            plt.xlabel('x')
            plt.ylabel('y')
            ax.legend()
            ax.set_xlim3d(-1, 5)
            ax.set_ylim3d(-1, 5)
            ax.set_zlim3d(-1, 5)
            plt.pause(.001)
            plt.cla()
    
    plt.xlabel('x')
    plt.ylabel('y')
    ax.set_xlim3d(-1, 5)
    ax.set_ylim3d(-1, 5)
    ax.set_zlim3d(-1, 5)
    ax.set_aspect('equal','box')
    #ax.legend()
    
        
    print('end plot')
    
def rotation_axes(phi,theta,psi):
    """
    per plottare gli assi dati gli angoli di eulero
    usa il pacchetto transformations
    ruota gli assi
    """
    
    X=np.array([1,0,0]); Y=np.array([0,1,0]); Z=np.array([0,0,1])
    
    R=euler_matrix(phi,theta,psi,'sxyz')
    R=R[:-1,:-1]
    
    X_r=R.dot(X); Y_r=R.dot(Y); Z_r=R.dot(Z)# in realtà basta prendere le colonne di R
    
    return([X_r,Y_r,Z_r])

def to_plot_axes(origin,X,Y,Z):
    """
    calcolo degli elementi per plottare gli assi
    X,Y,Z versori che individuano le direzioni degli assi
    origin coordinate dell'origine
    """
    x=origin[0];y=origin[1]; z=origin[2]
    #origin terna di posizione dell'origine, X,Y,Z rappresentazione degli assi (terne) in earth frame        
    plot_x=[ [x,x+X[0]], [y,y+X[1]] ,[z,z+X[2]] ]
    plot_y=[ [x,x+Y[0]], [y,y+Y[1]], [z,z+Y[2]] ]
    plot_z=[ [x,x+Z[0]], [y,y+Z[1]], [z,z+Z[2]] ]
    
    return([plot_x,plot_y,plot_z])

def followed_vs_desired(foll,des):
    """
    per confrontare effettivamente attuati vs desiderati 
    solo per posizione
    
    foll,des liste [[],[],[]]
    """
    x_act,y_act,z_act=foll
    x_ref,y_ref,z_ref=des

    fig = plt.figure(4)
    ax = Axes3D(fig)
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(x_act , y_act , z_act , label='actuated')
    ax.plot(x_ref , y_ref , z_ref , label='reference')
    ax.legend()

    