#!/usr/bin/env python
# -*_ coding: utf-8 -*-
"""
"""

import rospy
import message_filters
from geometry_msgs.msg import Accel , Quaternion , TwistStamped, PoseStamped
from sensor_msgs.msg import Imu

from std_msgs.msg import Int32 , Float32

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

from mavros_msgs.srv import SetMode

from numpy import array , cos , sin , tan
import numpy as np
from controller_block import full_nl_controller_class
from sistema_fisico import physic_system_class

from mavros_msgs.msg import AttitudeTarget

class control_node():
	def __init__(self):
		#inizializzazione nodo
		rospy.init_node('MLF_node',anonymous=None)

		#Publishers
		self.pub_control_out = rospy.Publisher('/mavros/setpoint_raw/attitude'\
                                                  , AttitudeTarget , queue_size=10)
		self.data_control=AttitudeTarget()
		#inserisco la pubblicazione su local position per poter portare il drone nella 
          #posizione iniziale della traiettoria
		self.pub_starting_position = rospy.Publisher('/mavros/setpoint_position/local'\
                                                  , PoseStamped , 	queue_size=10)
		self.data_setpoint=PoseStamped()

		self.pub_s = rospy.Publisher('switcher' , Int32 , queue_size=10)
		self.s=Int32(1)

		self.controller=full_nl_controller_class()
          #ci sono implementate le equazioni di eulero, serve per calcolare 
          #l'orientazione da pubblicare
		self.integrator=physic_system_class()

		self.now=rospy.get_rostime(); self.now=self.now.to_sec()

		#subscribers
          #in questo caso i subscribers sono insieriti all'interno di sinchronyzer
		self.sub_position=message_filters.Subscriber('/mavros/local_position/pose',\
                                                  PoseStamped) 
		self.sub_velocity=message_filters.Subscriber('mavros/local_position/velocity',\
                                                  TwistStamped)
		self.sub_imu=message_filters.Subscriber('/mavros/imu/data',Imu)

		self.flag=0 #primo giro di sinchro

		#possono essere sincronizzati sono messaggi con informazione di tempo 
          #(Stamped)
		ts = message_filters.ApproximateTimeSynchronizer([self.sub_position,\
                                                  self.sub_velocity,self.sub_imu],2,.1)
		ts.registerCallback(self.synchro_callback)

		rospy.wait_for_service('/mavros/set_mode')

		self.it=0 #contatore iterate
		self.I=array([[8.1e-3,0,0],[0,8.1e-3,0],[0,0,14.2e-3]]) #matrice di inerzia
		self.I_inv=np.linalg.inv(self.I)

		rospy.spin()

	def publish_setpoint_position(self,r):

		self.data_setpoint.pose.position.x,self.data_setpoint.pose.position.y,\
                                                  self.data_setpoint.pose.position.z=r

		self.pub_starting_position.publish(self.data_setpoint)

	def publish_control_out(self,u_normalized,quaternion_out,body_rates):
		"""
		metodo per il riempimento del dato da mandare al drone
		"""
		#self.data_control.header=rospy.get_rostime() #riferimento temporale


		[self.data_control.orientation.x,self.data_control.orientation.y,\
                      self.data_control.orientation.z,self.data_control.orientation.w]\
                                                  =quaternion_out
                                                  
		[self.data_control.body_rate.x,self.data_control.body_rate.y,\
                                              self.data_control.body_rate.z]=body_rates
		self.data_control.thrust=u_normalized #Float32

		self.pub_control_out.publish(self.data_control)


	#callbacks
     #input entranti nell'ordine in cui sono stati definiti prima
	def synchro_callback(self,position,velocity,imu):

		self.prev=self.now
		self.now=rospy.get_rostime(); self.now=self.now.to_sec()

		self.controller.delta_t=self.now-self.prev
		self.pref=self.now

		#creazione dati ATTUALI per controller_block
		r,rd,rdd,ang,angd,angdd=self.packing_data(position,velocity,imu)

		#settaggio informazioni attuali (entrate in callback nel controllore) ,
          #inizialmente si ha bisogno solo delle informazioni lineari
		#quelle angolari saranno necessarie per il funzionamento
		self.controller.start_set(r,rd,rdd)

		#non serve il set della traiettoria desiderata, essendo interna al 
          #controllore (linear desired trajectory per modifica)
		if self.flag==0:

			#invio risposta del drone neutra (hovering) o comando di andare 
               #nel punto iniziale della traiettoria e switchare
			r_ref,n1,n2,n3,n4,n5=self.controller.export_starting_ref()
			self.publish_setpoint_position(r_ref)

			self.it=self.it+1
			#calcolo distanza attuale dalla posizione desiderata prima di 
               #passare offboard
			dist=np.linalg.norm(array(r)-array(r_ref))
			print(self.it)

			if self.it>20:  #

				offboarding=rospy.ServiceProxy('/mavros/set_mode' , SetMode )
				resp=offboarding(0,'OFFBOARD')
				print('OFFBOARD')

			if dist<=5e-1: #controllo di prossimità nel caso di comando di 
                              #andare in una posizione
				self.flag=1 #passo al controllo
				self.controller.c_pos.F_tot=9.8
				print('controller on')

		elif self.flag==1:
			print('controlling')
			#aggiornamento controllore con dati attuali
			self.controller.update(r,rd,rdd,ang,angd,angdd)

			#presa uscita controllore
			u_normalized,tau=self.controller.output_controller()

			#pubblico
			#si necessita dell'orientazione desiderata, dato lo stato attuale
               #angolare e il torques desiderato si procede con integrazione
			#numerica sulle equazioni di eulero e condizioni iniziali quelle
			#attuali ad 1/rates
			#bisogna restituire un quaternione
			quaternion_out,body_rates=self.compute_att_out(ang,angd,tau)

			self.publish_control_out(u_normalized,quaternion_out,body_rates)

	def security_control(self,r_act):
		r_ref,n1,n2,n3,n4,n5=self.controller.export_starting_ref()
		dist=np.linalg.norm(array(r)-array(r_ref))
		if dist>=15e-1:
			self.flag=0



	def packing_data(self,position,velocity,imu):
		r=self.unpack(position.pose.position)
		rd=self.unpack(velocity.twist.linear)
		rdd=self.unpack(imu.linear_acceleration)
		
		#calcolo angoli di eulero in rpy convention da quaternione
		q1,q2,q3,q4=[position.pose.orientation.x,position.pose.orientation.y,\
                             position.pose.orientation.z,position.pose.orientation.w]
		
		ang=self.euler_calc(q1,q2,q3,q4)

    		#trasformazione velocità angolare in euler rates
		angd=self.compute_euler_rates(ang,velocity.twist.angular)
		
		angdd=array([0,0,0]) #non serve ai fini del controllo

		return(r,rd,rdd,ang,angd,angdd)

	def unpack(self,data): 
		"""
		tutti i dati in terne hanno nomi finali x,y,z quindi 
          creo questo metodo -> lista di 3 elementi
		"""
		out=array([data.x,data.y,data.z])
		return(out)

	def euler_calc(self,q1,q2,q3,q4):
		phi,theta,psi=tf.transformations.euler_from_quaternion([q1,q2,q3,q4],"sxyz")
		return( array([phi,theta,psi]) )

	def compute_euler_rates(self,ang,ang_vel):
		"""
		data l'orientazione attuale e la velocità angolare si può calcolare 
          il vettore degli euler rates (necessario al controllore)
		-l'alternativa sarebbe stimare la derivata con i precedenti 
          nell'intervallo di tempo
		"""
		phi=ang[0]; theta=ang[1]; psi=ang[2]
		cphi=cos(phi); sphi=sin(phi);
		ctheta=cos(theta); ttheta=tan(theta)

		M=array([[1 , sphi*ttheta , cphi*ttheta],\
                      [0 , cphi , -sphi],\
                      [0 , sphi/ctheta , cphi/ctheta]]) #non invertibile per theta=+-pi/2

		#calcolo euler_rates
		omega=array([ang_vel.x,ang_vel.y,ang_vel.z])
		euler_rates=M.dot(omega)

		return array(euler_rates)

	def compute_body_rate(self,ang,euler_rates):
		phi=ang[0]; theta=ang[1]; psi=ang[2]
		cphi=cos(phi); sphi=sin(phi);
		ctheta=cos(theta); ttheta=tan(theta)

		M=array([[1 , sphi*ttheta , cphi*ttheta],\
                      [0 , cphi , -sphi],\
                      [0 , sphi/ctheta , cphi/ctheta]]) #non invertibile per theta=+-pi/2

		M_inv=np.linalg.inv(M)

		angd=array(euler_rates)

		omega=M_inv.dot(angd)

		return(omega)

	def compute_att_out(self,ang_act,angd_act,tau):
		#integrazione
		ang,euler_rates=self.integrator.prevision_ROS(ang_act,angd_act,tau)

		phi,theta,psi=ang

		q1,q2,q3,q4=tf.transformations.quaternion_from_euler(phi,theta,psi,'sxyz')
		
		#l'integrazione restituisce anche il body rates come euler rates 
          #quindi si necessita  di trasformazione
		body_rates=self.compute_body_rate(ang,euler_rates)

		return([q1,q2,q3,q4],body_rates)


if __name__ == '__main__':
	try:
		c=control_node() #nel caso sia una classe bisogna istanziare un oggetto
                           #o non succederà nulla
		offboarding=rospy.ServiceProxy('/mavros/set_mode',SetMode)
		resp=offboarding(0,'OFFBOARD')
	except rospy.ROSInterruptException:
		pass