#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
data l'informazione sulla velocita' di rotazione delle eliche si vuole calcolare la forza generata (come approssimazione)
da cui, utilizzando, le equazioni differenziali per accelerazioni lineari ed angolari si calcoleranno tutti gli stati "attuali" del drone 
tenendo conto che il rate di aggiornamento e' di 10hz (messo nei nodi)

lo stato attuale del drone sara' dato dalla classe dynamic system
qui viene creato un nodo (dynamic system) che pubblichera' su canali con lo stesso nome di quelli di mavros in modo da poter interfacciare direttamente
il controllore.

Michele D'Adamo	29/09/2016
"""

import rospy
from dynamic_system_class_no_rotor import dynamic_system_no_rotor
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32
from std_msgs.msg import Float32

import numpy as np

def dynamic_system_by_MD():

	rospy.init_node('dynamic_system',anonymous=None)

# manca parte di sottoscrizione al drone, probabilmente bisogna aggiungerla nella classe
#dynamic_system
#bisogna aggiungere anche che pubblichi qualcosa autonomamente, prima di collegarsi al drone


#simulazione local_position , i publisher servono al controllore (oltre che alla classe dynamic_system, anche se su quest'ultima si potrebbe evitare 
#la sottoscrizione)


	pub_lp_pose=rospy.Publisher('/mavros/local_position/pose',PoseStamped, queue_size=10)
	lp_Pose_msg=PoseStamped()
	
	pub_lp_velocity=rospy.Publisher('/mavros/local_position/velocity',TwistStamped,queue_size=10)
	lp_velocity_msg=TwistStamped()

	pub_imu=rospy.Publisher('/mavros/imu/data',Imu,queue_size=10)
	imu_msg=Imu()


	pub_delta_t=rospy.Publisher('/delta_t',Float32,queue_size=10)
	msg_delta_t=Float32()

	#rospy.loginfo('prima')
	system=dynamic_system_no_rotor()
	#rospy.loginfo('dopo')
	r=30
	rate=rospy.Rate(r)
	system.delta_t=1.0/r

	rospy.loginfo('waiting 1.5 seconds for topics publication')
	rospy.sleep(1.5)
	rospy.loginfo('go')


	while (system.s==Int32(0) ) and (not rospy.is_shutdown()):

		rospy.loginfo('NO MLF')

		lp_Pose_msg.header.stamp=rospy.get_rostime()
		lp_Pose_msg.pose.position.x=0
		lp_Pose_msg.pose.position.y=0
		lp_Pose_msg.pose.position.z=2.5

#trasformazione da zxy a quat. SISTEMARE
		
		[q1,q2,q3,q4]=quaternion_from_euler(.0,.0,.0,'sxyz')

		lp_Pose_msg.pose.orientation.x=q1 #qua ci vuole la trasf. in quaternione
		lp_Pose_msg.pose.orientation.y=q2
		lp_Pose_msg.pose.orientation.z=q3
		lp_Pose_msg.pose.orientation.w=q4

		lp_velocity_msg.header.stamp=rospy.get_rostime()
		lp_velocity_msg.twist.linear.x=0
		lp_velocity_msg.twist.linear.y=0
		lp_velocity_msg.twist.linear.z=0
		
#trasformazione derivate eulero in velocita' angolare
		#R_interm=np.array([[1,0,0],
		#	[1,0,0],
		#	[0,0,1]]) #non è un pò strana?

		[omega_x,omega_y,omega_z]=[0,0,0]

		lp_velocity_msg.twist.angular.x=omega_x
		lp_velocity_msg.twist.angular.y=omega_y
		lp_velocity_msg.twist.angular.z=omega_z

		imu_msg.header.stamp=rospy.get_rostime()
		imu_msg.orientation.x=lp_Pose_msg.pose.orientation.x
		imu_msg.orientation.y=lp_Pose_msg.pose.orientation.y
		imu_msg.orientation.z=lp_Pose_msg.pose.orientation.z
		imu_msg.orientation.w=lp_Pose_msg.pose.orientation.w

		imu_msg.angular_velocity.x=lp_velocity_msg.twist.angular.x
		imu_msg.angular_velocity.y=lp_velocity_msg.twist.angular.y
		imu_msg.angular_velocity.z=lp_velocity_msg.twist.angular.z

		imu_msg.linear_acceleration.x=0
		imu_msg.linear_acceleration.y=0
		imu_msg.linear_acceleration.z=0

		rospy.loginfo(['posizione',lp_Pose_msg])
		[system.phi,system.theta,system.psi]=euler_from_quaternion([lp_Pose_msg.pose.orientation.x,lp_Pose_msg.pose.orientation.y,lp_Pose_msg.pose.orientation.z,lp_Pose_msg.pose.orientation.w])
		rospy.loginfo(['orientazione',system.phi,system.theta,system.psi])
		
		msg_delta_t=Float32(0.3)

		pub_lp_pose.publish(lp_Pose_msg)
		pub_lp_velocity.publish(lp_velocity_msg)
		pub_imu.publish(imu_msg)
		pub_delta_t.publish(msg_delta_t)

		rate.sleep()

	while not rospy.is_shutdown():

		system.update() #scrivere sub nella classe! per togliere gli inputs

		lp_Pose_msg.header.stamp=rospy.get_rostime()
		lp_Pose_msg.pose.position.x=system.x
		lp_Pose_msg.pose.position.y=system.y
		lp_Pose_msg.pose.position.z=system.z

#trasformazione da zxy a quat. SISTEMARE
		[q1,q2,q3,q4]=quaternion_from_euler(system.phi,system.theta,system.psi,'sxyz') 
		rospy.loginfo(['ang. eul.',system.phi,system.theta,system.psi])
		#rospy.loginfo(euler_from_quaternion([q1,q2,q3,q4],'sxyz'))

		lp_Pose_msg.pose.orientation.x=q1 #qua ci vuole la trasf. in quaternione
		lp_Pose_msg.pose.orientation.y=q2
		lp_Pose_msg.pose.orientation.z=q3
		lp_Pose_msg.pose.orientation.w=q4

		lp_velocity_msg.header.stamp=rospy.get_rostime()
		lp_velocity_msg.twist.linear.x=system.vx
		lp_velocity_msg.twist.linear.y=system.vy
		lp_velocity_msg.twist.linear.z=system.vz
		
#trasformazione derivate eulero in velocita' angolare

		#### poichè penso sia sbagliato perch'è viene calcolato con un procedimento secondo la terna mobile vedi, sistemo come segue
		#R_interm=np.array([[1,0,-np.sin(system.theta)],\
		#	[np.cos(system.phi),0,np.sin(system.phi)*np.cos(system.theta)],\
		#	[0,-np.sin(system.phi),np.cos(system.phi)*np.cos(system.theta)]])

		#[omega_x,omega_y,omega_z]=R_interm.dot(np.array([system.dphi,system.dtheta,system.dpsi]))
		#### con un nuovo metodo, faccio calcolare il vettore velocità di rotazione (nel sistema corpo) dalla classe
		#dopo aver mandato l'update
		#[omega_x,omega_y,omega_z]=system.body_frame_angular_velocity()

		#lp_velocity_msg.twist.angular.x=omega_x
		#lp_velocity_msg.twist.angular.y=omega_y
		#lp_velocity_msg.twist.angular.z=omega_z

		lp_velocity_msg.twist.angular.x=system.dphi #associo questo perchè ho fatto l'assunzione p=dphi ecc
		lp_velocity_msg.twist.angular.y=system.dtheta
		lp_velocity_msg.twist.angular.z=system.dpsi

		imu_msg.header.stamp=rospy.get_rostime()
		imu_msg.orientation.x=q1 #copia da sopra
		imu_msg.orientation.y=q2
		imu_msg.orientation.z=q3
		imu_msg.orientation.w=q4

		imu_msg.angular_velocity.x=lp_velocity_msg.twist.angular.x #copia da sopra
		imu_msg.angular_velocity.y=lp_velocity_msg.twist.angular.y
		imu_msg.angular_velocity.z=lp_velocity_msg.twist.angular.z

		imu_msg.linear_acceleration.x=system.ax
		imu_msg.linear_acceleration.y=system.ay
		imu_msg.linear_acceleration.z=system.az

		rospy.loginfo(['posizione',lp_Pose_msg])
		#rospy.loginfo(['orientazione',system.phi,system.theta,system.psi])

		msg_delta_t=Float32(system.delta_t)

		pub_lp_pose.publish(lp_Pose_msg)
		pub_lp_velocity.publish(lp_velocity_msg)
		pub_imu.publish(imu_msg)
		pub_delta_t.publish(msg_delta_t)



		rate.sleep()

		#aggiungere qui eventuali file da creare per vedere i plot


if __name__ == '__main__':
	try:
		dynamic_system_by_MD()
	except rospy.ROSInterruptException:
		pass