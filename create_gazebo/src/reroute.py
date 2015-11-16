#!/usr/bin/env python
import rospy
import tf
import numpy as np

#Support when Node is run
import os
import sys
import subprocess
import time
import struct
import math

from numpy.matlib import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose2D, Twist
from gazebo_msgs.msg import ModelStates

class Rerouting:

	def __init__(self):
		
		#Create rosnode
		rospy.init_node('Gazebo_To_Pose')
		self.rate = rospy.Rate(5)

		self.botID = rospy.get_param('~botID', 0)
		
		#PubSub
		self.pubPose = rospy.Publisher('/Robot_'+str(self.botID)+'/ground_pose', Pose2D, queue_size = 50)
		self.subTopic = rospy.Subscriber('/gazebo/model_states',ModelStates,self.reroute)	

		rospy.loginfo("Rerouting")
		self.pose = Pose2D()
		self.pose.x = float('infinity')
		self.pose.y = float('infinity')
		self.pose.theta = 0
		
		self.talk()

	def talk(self):

		while not rospy.is_shutdown():
			
			#Define the message
			print 'HEHE'
			rospy.loginfo("Pose: "+str(self.pose))

			#Publish
			self.pubPose.publish(self.pose)		
			self.rate.sleep()
	
	def reroute(self, ModelStates):
		self.name = 'create'+str(self.botID)
		self.pose = Pose2D()
		for i in range(len(ModelStates.name)):
			if ModelStates.name[i] == self.name:
				
				self.pose.x = ModelStates.pose[i].position.x
				self.pose.y = ModelStates.pose[i].position.y
				
				quaternion = (ModelStates.pose[i].orientation.x, ModelStates.pose[i].orientation.y, ModelStates.pose[i].orientation.z, ModelStates.pose[i].orientation.w)
				euler = tf.transformations.euler_from_quaternion(quaternion)
				
				self.pose.theta = (euler[2]+np.pi)%(2*np.pi)-np.pi
				#print ModelStates.pose[i].orientation.y 


if __name__ == '__main__':
    try:
        r = Rerouting()
    except rospy.ROSInterruptException, KeyboardInterrupt:
		print "Ending Program!!!!!!"
		sys.exit(1)		
