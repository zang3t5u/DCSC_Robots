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
		self.present_bots = 3
		
		self.Rob_diam = 35.0/100.0;	#Robot Diameter in Metres
		self.botID = int(sys.argv[2])
		self.FormationID = int(sys.argv[3])
		self.Num_of_Bots = int(sys.argv[1])
		self.parent_topic = '/create'+str(self.botID)+'/'

		if(self.Num_of_Bots <= math.pi):
			self.Lmin = 1.5*self.Rob_diam/2;
		else:
			self.Lmin = 1.5*self.Num_of_Bots*self.Rob_diam/(2*math.pi)
		self.dx = 0
		self.dy = 0
		#Define the offset message
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
		self.x = [float("infinity")]*2
		self.x.append(0)
		for i in range(self.Num_of_Bots):
			self.bot_form_poses.append([float("infinity"), float("infinity"), 0])
		
		#Virtual Structure Properties
		self.k = array([0.05,0.05]).T
		self.kv = array([0.4,0.1]).T
		self.dt = 0.05
		self.xr = array([0,0]).T
		self.xl = array([0,0]).T

		#Create rosnode
		rospy.init_node('Flocking_Control')
		self.rate = rospy.Rate(5)

		#Listening states
		self.state = array([0,0,0])
		self.states = array([[0,0,0]])
		
		#PubSub
		self.pubFlock = rospy.Publisher('/create'+str(self.botID)+'/flocking_offset', Pose2D, queue_size = 50)
		self.pubCent = rospy.Publisher('/create'+str(self.botID)+'/flocking_centre', Pose2D, queue_size = 50)
		self.pubVel = rospy.Publisher('/create'+str(self.botID)+'/cmd_vel', Twist, queue_size = 50)
		self.subID = rospy.Subscriber('/create'+str(self.botID)+'/botID',Int32,self.setID)	

		self.subPoses = []
		for i in range(self.Num_of_Bots):		
			if i != self.botID-1:
				pose_topic_name = '/create'+str(i+1)+'/ground_pose'
			else:
				pose_topic_name = '/create'+str(self.botID)+'/ground_pose'
			self.subPoses.append(rospy.Subscriber(pose_topic_name, Pose2D, self.listen, callback_args = (i+1))) 
		
		#Start control loop
		rospy.loginfo("Flocking initialized.")
		rospy.loginfo("Node initialized.")

		self.talk()

	def talk(self):

		while not rospy.is_shutdown():
			while not self.start:
				pass
			#time.sleep(0.5*self.Num_of_Bots)
			#Update the state
			self.state = self.calc_centre()
			if self.botCount > self.present_Bots:
				self.xr = array([self.state[0],self.state[1]]).T
			
			#self.state = np.mean(self.states,axis=0)			
			#self.states = np.array([self.state])
			
			#Move the Virtual Leader state (not now)
			self.xl = self.xl + self.dt * self.k * (self.xr - self.xl)
			
			
			if self.set_offset == False and self.calculated_offset == False:
				while self.botCount < self.present_Bots:
					pass
				self.flock_pose()
				
				self.offset_pose.x = self.dx
				self.offset_pose.y = self.dy
				self.offset_pose.theta = 0 
				
				rospy.set_param('~offset_x', self.dx)
				rospy.set_param('~offset_y', self.dy)				
				#Calculate relative pose, once
				print "Published offset"
				self.set_offset = True

			#Publish calculated offset continuously
			self.pubFlock.publish(self.offset_pose)	
			#Publish current center and desired orientation as a pose
			flock_cent = Pose2D()
			flock_cent.x = self.xr[0]
			flock_cent.y = self.xr[1]
			flock_cent.theta = 0

			rospy.loginfo("Offset: "+str(self.offset_pose))
			rospy.loginfo("Leader: "+str(flock_cent))

			#Publish current leader pose
			self.pubCent.publish(flock_cent)

			self.xg = self.bot_form_poses[self.botID-1]
			self.xg[2] = flock_cent.theta
			rospy.loginfo("Goal:    "+str(self.xg))
			rospy.loginfo("Current: "+str(self.x))
			'''
			res = self.move(flock_cent, pose)
			vel = Twist()
			vel.linear.x = self.satmin(res[0], 0.01)
			vel.angular.z = self.satmin(res[1], 0.05)
			rospy.loginfo("V: "+str(vel.linear.x)+" W: "+str(vel.angular.z))
			self.pubVel.publish(vel)
			'''
			self.rate.sleep()
	
	def satmin(self,val,valmin):
		if(val < valmin and val > -valmin):
			return 0
		else:
			return val
	
	def calc_centre(self):
		self.botCount = sum(self.pos_updated)
		pos_sum = [0.0]*3
		for i in range(self.Num_of_Bots):
			if self.pos_updated[i] == 1:
				pos_sum[0] = pos_sum[0] + self.bot_data[i][1]  
				pos_sum[1] = pos_sum[1] + self.bot_data[i][2]
				pos_sum[2] = pos_sum[2] + self.bot_data[i][3]
		pos_sum = [x/self.botCount for x in pos_sum]
		return pos_sum

	def flock_pose(self):
		theta = 2*math.pi/self.Num_of_Bots
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
	
	def move(self, flock_cent, offset_pose):	
		w = self.kv[1]*arctan2((self.xg[1] - self.x[1]), (self.xg[0] - self.x[0])) - self.x[2]
		v = self.kv[0]*(10*math.sqrt((self.xg[1] - self.x[1])**2+(self.xg[0] - self.x[0])**2))
		return [v,w]

	def listen(self,message, node):
		#rospy.loginfo('message received')
		if not self.start:
			self.states = array([[message.x,message.y,message.theta]])
			if node == self.botID:
				print "Yaaaaaay!!"
		else: 
			self.states = append(self.states,array([[message.x,message.y,message.theta]]),axis=0)		
		self.botCount = self.botCount+1;
		rospy.loginfo("Node:"+str(node))
		nodeIndex = node-1

		self.bot_data[nodeIndex][1] = message.x
		self.bot_data[nodeIndex][2] = message.y
		self.bot_data[nodeIndex][3] = message.theta

		if self.pos_updated[nodeIndex] == 0:
			self.pos_updated[nodeIndex] = 1   
		if self.botID == node:
			self.x[0] = message.x
			self.x[1] = message.y
			self.x[2] = message.theta
			self.start = True

	
	def setID(self, botID):
		self.botID = botID.data


if __name__ == '__main__':
    try:
        f = Flocking()
    except rospy.ROSInterruptException, KeyboardInterrupt:
		print "Ending Program!!!!!!"
		sys.exit(1)		
