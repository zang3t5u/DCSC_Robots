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
from dcsc_consensus.msg import bot_data_msg

class Flocking:

	def __init__(self):
		if '-h' in sys.argv or len(sys.argv) < 4:
			print "Usage:", sys.argv[0], "Max_ID_of_Bots", "BotID", "Formation_No", "Present Bots"
			print "Formation ID                Formation Description"
			print "------------                ---------------------"
			print "     1                               Circle      "
			print "     2                               Column      "
			print "     3                               Row      "
			print "     4                               Wedge      "
			sys.exit()
		#Properties
		self.start = False
		self.present_Bots = 3
		
		self.Rob_diam = 35.0/100.0;	#Robot Diameter in Metres
		self.botID = int(sys.argv[2])
		self.FormationID = int(sys.argv[3])
		self.Num_of_Bots = int(sys.argv[1])
		self.parent_topic = '/create'+str(self.botID)+'/'

		if(self.Num_of_Bots <= math.pi):
			self.Lmin = 3*self.Rob_diam/2;
		else:
			self.Lmin = 3*self.Num_of_Bots*self.Rob_diam/(2*math.pi)
		self.dx = 0
		self.dy = 0
		#Define the offset message
		self.flock_cent = Pose2D()
		self.offset_pose = Pose2D()
		self.botCount = 0		
		self.set_offset = False
		self.calculated_offset = False

		#To keep track of bot poses counted
		self.pos_updated = [0]*self.Num_of_Bots
		self.bot_data = []
		for i in range(self.Num_of_Bots):
			self.bot_data.append([float("infinity")]*10)
			self.bot_data[i][0] = i+1
		
		self.bot_form_poses = []
		self.x = array([float("infinity"), float("infinity"), 0]).T
		for i in range(self.Num_of_Bots):
			self.bot_form_poses.append([float("infinity"), float("infinity"), 0])
		
		#Virtual Structure Properties
		self.k = array([0.05,0.05]).T
		self.kv = array([0.4,0.1]).T
		self.dt = 0.05
		self.xgoal = array([0,0, 0]).T
		self.xr = array([0,0]).T
		self.xl = array([0,0]).T

		#Create rosnode
		rospy.init_node('Flocking_Control')
		self.rate = rospy.Rate(5)

		#Listening states
		self.state = array([0,0,0]).T
		self.states = array([[0,0,0]])
		
		#PubSub
		self.pubFlock = rospy.Publisher('/create'+str(self.botID)+'/flocking_offset', Pose2D, queue_size = 50)
		self.pubCent = rospy.Publisher('/flocking_centre', Pose2D, queue_size = 50)

		self.subPoses = []
		for i in range(self.Num_of_Bots):		
			pose_topic_name = '/create'+str(i+1)+'/ground_pose'
			if i != self.botID-1:			
				self.subPoses.append(rospy.Subscriber(pose_topic_name, Pose2D, self.listen, callback_args = (i+1))) 
			else:
				self.subPoses.append(rospy.Subscriber(pose_topic_name, Pose2D, self.ground)) 
			time.sleep(10/1000)
		self.subCent = rospy.Subscriber('/flocking_centre', Pose2D, self.goal)
		#Start control loop
		rospy.loginfo("Flocking initialized.")
		rospy.loginfo("Node initialized.")

		self.talk()

	def talk(self):

		while not rospy.is_shutdown():
			if not self.start:
				self.rate.sleep()
				continue
			#time.sleep(0.5*self.Num_of_Bots)
			#Update the state
			self.state = self.calc_centre()
			if self.botCount >= self.present_Bots:
				self.xr = array([self.state[0], self.state[1]]).T
				
			#Move the Virtual Leader state (not now)
			self.xl = self.xl + self.dt * self.k * (self.xr - self.xl)
			
			if self.set_offset == False and self.calculated_offset == False:
				if self.botCount < self.present_Bots:
					self.rate.sleep()
					continue
				self.flock_pose()
				
				self.offset_pose.x = self.dx
				self.offset_pose.y = self.dy
				self.offset_pose.theta = 0 
				
				rospy.set_param('~offset_x', self.dx)
				rospy.set_param('~offset_y', self.dy)		
				#Publish current center and desired orientation as a pose

				self.flock_cent.x = self.state[0]
				self.flock_cent.y = self.state[1]
				self.flock_cent.theta = self.state[2]

				rospy.loginfo("Offset: "+str(self.offset_pose))
				rospy.loginfo("Leader: "+str(self.flock_cent))

				#Publish current leader pose
				self.pubCent.publish(self.flock_cent)			
				#Calculate relative pose, once
				print "Published offset"
				self.set_offset = True

			#Publish calculated offset continuously
			self.pubFlock.publish(self.offset_pose)	
			self.xg = self.xgoal + array([self.bot_form_poses[self.botID-1][0], self.bot_form_poses[self.botID-1][1], 0]).T
			rospy.loginfo("Goal:    "+str(self.xg))
			rospy.loginfo("Current: "+str(self.x))
			self.rate.sleep()

	def calc_centre(self):
		self.botCount = sum(self.pos_updated)
		pos_sum = [0.0]*3
		for i in range(self.Num_of_Bots):
			if self.pos_updated[i] == 1:
				pos_sum[0] = pos_sum[0] + self.bot_data[i][1]  
				pos_sum[1] = pos_sum[1] + self.bot_data[i][2]
				pos_sum[2] = pos_sum[2] + self.bot_data[i][3]
		pos_sum = [x/self.botCount for x in pos_sum]
		return array([pos_sum[0], pos_sum[1], pos_sum[2]]).T

	def flock_pose(self):
		theta = 2*math.pi/self.botCount
		f_dR = [0.0]*self.Num_of_Bots
		f_dTheta = [0.0]*self.Num_of_Bots
		f_Position = [[0.0]*2 for i in range(self.Num_of_Bots)]
		offsets = [[0.0]*2 for i in range(self.Num_of_Bots)]

		#Arrays to assign Positions to the Bots
		UnAssigned = [i for i in range(self.botCount)]
		AssignToBot = [float("infinity")]*self.Num_of_Bots
		for i in range(self.botCount):
			#For Circle
			if self.FormationID == 1:
				f_dR[i] = self.Lmin
				f_dTheta[i] = (math.pi/2.0) + (i)*theta
			#For Column			
			elif self.FormationID == 2:
				f_dR[i] = self.Lmin*(math.floor(self.botCount/2)-i)
				f_dTheta[i] = math.pi/2.0
			#For Row
			elif self.FormationID == 3:
				f_dR[i] = self.Lmin*(math.floor(self.botCount/2)-i)
				f_dTheta[i] = 0
			#For Wedge
			elif self.FormationID == 4:
				f_dR[i] = self.Lmin*math.floor((i+1)/2.0)
				f_dTheta[i] = math.fmod(i+1, 2)*(-math.pi/4.0) + (1-math.fmod(i+1,2))*(5.0*math.pi/4.0)
		center = self.state		
		for i in range(self.botCount):
			offsets[i][0] = f_dR[i]*math.cos(f_dTheta[i])
			offsets[i][1] = f_dR[i]*math.sin(f_dTheta[i])
			f_Position[i][0] = center[0] + f_dR[i]*math.cos(f_dTheta[i])
			f_Position[i][1] = center[1] + f_dR[i]*math.sin(f_dTheta[i])
		rospy.loginfo("Offsets:"+str(offsets))
		for i in range(self.Num_of_Bots):
			if self.pos_updated[i] != 1:
				continue
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
				self.dx = offsets[AssignToBot[i]][0]
				self.dy = offsets[AssignToBot[i]][1]
				self.calculated_offset = True

	def goal(self, pose):
		self.xgoal = array([pose.x, pose.y, pose.theta]).T

	def ground(self, message):
		self.x[0] = message.x
		self.x[1] = message.y
		self.x[2] = message.theta
		
		self.start = True		
		node = self.botID
		rospy.loginfo("Node:"+str(node))
		nodeIndex = node-1
		
		self.bot_data[nodeIndex][1] = message.x
		self.bot_data[nodeIndex][2] = message.y
		self.bot_data[nodeIndex][3] = message.theta

		if self.pos_updated[nodeIndex] == 0:
			self.pos_updated[nodeIndex] = 1  

	def listen(self,message, node):
		
		rospy.loginfo("Node:"+str(node))
		nodeIndex = node-1
		
		self.bot_data[nodeIndex][1] = message.x
		self.bot_data[nodeIndex][2] = message.y
		self.bot_data[nodeIndex][3] = message.theta

		if self.pos_updated[nodeIndex] == 0:
			self.pos_updated[nodeIndex] = 1   
			

if __name__ == '__main__':
    try:
        f = Flocking()
    except rospy.ROSInterruptException, KeyboardInterrupt:
		print "Ending Program!!!!!!"
		sys.exit(1)		
