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
from geometry_msgs.msg import Pose2D
from dcsc_consensus.msg import bot_data_msg

class Flocking:

	def __init__(self):
		if '-h' in sys.argv or len(sys.argv) < 4:
			print "Usage:", sys.argv[0], "Num_of_Bots", "BotID", "Formation_No"
			print "Formation ID                Formation Description"
			print "------------                ---------------------"
			print "     1                               Circle      "
			print "     2                               Column      "
			print "     3                               Row      "
			print "     4                               Wedge      "
			sys.exit()
		#Properties
		self.start = False
		self.Rob_diam = 35.0/100.0;	#Robot Diameter in Metres
		self.botID = int(sys.argv[2])
		self.FormationID = int(sys.argv[3])
		self.Num_of_Bots = int(sys.argv[1])
		if(self.Num_of_Bots <= math.pi):
			self.Lmin = 3*self.Rob_diam/2;
		else:
			self.Lmin = 3*self.Num_of_Bots*self.Rob_diam/(2*math.pi)
		self.dx = 0
		self.dy = 0
		self.botCount = 0		#To keep track of bot poses counted
		self.pos_updated = [0]*self.Num_of_Bots
		self.bot_data = []
		for i in range(self.Num_of_Bots):
			self.bot_data.append([float("infinity")]*10)
			self.bot_data[i][0] = i+1
		
		self.bot_form_poses = []
		for i in range(self.Num_of_Bots):
			self.bot_form_poses.append([float("infinity")]*3)
		

		self.k = array([0.05,0.05]).T
		self.dt = 0.05

		#self.state = np.array([x,y,theta])
		#self.states = np.array([[x,y,theta]])

		self.xr = array([0,0]).T
		self.xl = array([0,0]).T

		#Create rosnode
		rospy.init_node('Flocking_Pose')
		self.rate = rospy.Rate(5)

		#Listening states
		self.state = array([0,0,0])
		self.states = array([[0,0,0]])

		self.pubFlock = rospy.Publisher('flocking_offset', Pose2D, queue_size = 50)
		self.subID = rospy.Subscriber('botID',Int32,self.setID)	

		self.subPoses = []
		for i in range(self.Num_of_Bots):		
			if i != self.botID-1:
				pose_topic_name = 'create'+str(i+1)+'/ground_pose'
			else:
				pose_topic_name = 'ground_pose'
			self.subPoses.append(rospy.Subscriber(pose_topic_name, Pose2D, self.listen, callback_args = (i+1))) 
		
		#Start control loop
		rospy.loginfo("Flocking initialized.")
		rospy.loginfo("Node initialized.")

		self.talk()

	def talk(self):

		while not rospy.is_shutdown():
			while not self.start:
				pass
			time.sleep(0.5*self.Num_of_Bots)
			#Update the state
			self.state = np.mean(self.states,axis=0)			
			self.states = np.array([self.state])
			
			self.flock_pose()
			#Define the message
			pose = Pose2D()
			pose.x = self.dx
			pose.y = self.dy
			pose.theta = 0

			rospy.loginfo(pose)

			#Publish
			self.pubFlock.publish(pose)		
			self.rate.sleep()

	def flock_pose(self):
		theta = 2*math.pi/self.Num_of_Bots
		f_dR = [0.0]*self.Num_of_Bots
		f_dTheta = [0.0]*self.Num_of_Bots
		f_Position = [[0.0]*2 for i in range(self.Num_of_Bots)]
		
		#Arrays to assign Positions to the Bots
		UnAssigned = [i for i in range(self.Num_of_Bots)]
		AssignToBot = [float("infinity")]*self.Num_of_Bots
		for i in range(self.Num_of_Bots):
			#For Circle
			if self.FormationID == 1:
				f_dR[i] = self.Lmin
				f_dTheta[i] = (math.pi/2.0) + (i)*theta
			#For Column			
			elif self.FormationID == 2:
				f_dR[i] = self.Lmin*(math.floor(self.Num_of_Bots/2)-i)
				f_dTheta[i] = math.pi/2.0
			#For Row
			elif self.FormationID == 3:
				f_dR[i] = self.Lmin*(math.floor(self.Num_of_Bots/2)-i)
				f_dTheta[i] = 0
			#For Wedge
			elif self.FormationID == 4:
				f_dR[i] = self.Lmin*math.floor((i+1)/2.0)
				f_dTheta[i] = math.fmod(i+1, 2)*(-math.pi/4.0) + (1-math.fmod(i+1,2))*(5.0*math.pi/4.0)
		
		for i in range(self.Num_of_Bots):
			f_Position[i][0] = self.state[0] + f_dR[i]*math.cos(f_dTheta[i])
			f_Position[i][1] = self.state[1] + f_dR[i]*math.sin(f_dTheta[i])
		
		for i in range(self.Num_of_Bots):
			DistMin = math.sqrt(2*((float("Infinity"))**2))	
			for j in range(len(UnAssigned)):
				distx = self.bot_data[i][1] - f_Position[UnAssigned[j]][0]
				disty = self.bot_data[i][2] - f_Position[UnAssigned[j]][1]
				dist = math.sqrt(distx**2 + disty**2)
				if dist <= DistMin:
					DistMin = dist
					AssignToBot[i] = UnAssigned[j]
			del UnAssigned[UnAssigned.index(AssignToBot[i])]
			self.bot_form_poses[i][0] = f_Position[AssignToBot[i]][0]
			self.bot_form_poses[i][1] = f_Position[AssignToBot[i]][1]
			if (i==self.botID-1):
				self.dx = f_Position[i][0] - self.state[0]
				self.dy = f_Position[i][1] - self.state[1]

	
	def listen(self,message, node):
		#rospy.loginfo('message received')
		if not self.start:
			self.states = array([[message.x,message.y,message.theta]])
			if node == self.botID:
				print "Yaaaaaay!!"
		else: 
			self.states = append(self.states,array([[message.x,message.y,message.theta]]),axis=0)		
		self.start = True
		self.botCount = self.botCount+1;
		rospy.loginfo("State is:"+str(self.states))
		nodeIndex = node-1
		if self.pos_updated[nodeIndex] == 0:
			self.bot_data[nodeIndex][1] = message.x
			self.bot_data[nodeIndex][2] = message.y
			self.bot_data[nodeIndex][3] = message.theta
			self.pos_updated[nodeIndex] = 1

	
	def setID(self, botID):
		self.botID = botID.data


if __name__ == '__main__':
    try:
        f = Flocking()
    except rospy.ROSInterruptException: pass
