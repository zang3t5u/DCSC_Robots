#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

#ROS Libraries
import rospy
import tf
import numpy as np

from std_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from dcsc_consensus.msg import bot_data_msg


#Support when Node is run
import os
import sys
import subprocess
import time
import struct
import math

#Enabling communication with WSN
import Bot_Net_ROS

def tinyROS():
	if '-h' in sys.argv or len(sys.argv) < 3:
		print "Usage:", sys.argv[0], "Num_of_Bots", "BotID" , "sf@localhost:9002"
		sys.exit()
	if(len(sys.argv) < 4):
		test = subprocess.Popen(['motelist| grep "/dev/ttyUSB"'], stdout=subprocess.PIPE, shell = True)
		output = test.communicate()[0].split("\n")
		print len(output)
		if (len(output) > 2 and int(sys.argv[2])!=0):
			base_index = output[1].find("/dev/ttyUSB")
		else:
			base_index = output[0].find("/dev/ttyUSB")
		base_index = output[0].find("/dev/ttyUSB")
		dev = output[0][base_index:base_index+12]
		
		arg = 'serial@'+dev+':115200'
	else:
		arg = sys.argv[3]
	
	print "------------------"
	print "Serial Started at:"+arg
	botID = int(sys.argv[2])
	print "botID      : ", botID
	Num_of_Bots = int(sys.argv[1]) 
	print "Total Bots : ", Num_of_Bots
	
	#Node to check	
	init_msg_recvd = 0
	node_recv_count = 0

	dl = Bot_Net_ROS.Bot_Net(arg, Num_of_Bots, botID)

	#Start ROS Node
	#Create Node
	node_name = 'Bot_Net_Node_'+str(int(botID))
	print "Starting ROSnode named: ", node_name
	rospy.init_node(node_name, anonymous=True)
	rate = rospy.Rate(60) # 60hz
	#----------------
	#ROS Publishing
	#----------------
	#Publish to all Robot Nodes
	pubWiFiCount = rospy.Publisher('Count_WiFi_Msgs', Float64, queue_size=10)
	pubETCount = rospy.Publisher('Count_Event_Msgs', Float64, queue_size=10)
	pubID = 	rospy.Publisher('botID', Int32, queue_size=10)
	pubNum = rospy.Publisher('Num_of_Bots', Int32, queue_size = 10)
	pubPoses = []
	pubVels = []
	for i in range(Num_of_Bots):		
		if i != botID-1:
			pose_topic_name = 'create'+str(i+1)+'/ground_pose'
			vel_topic_name = 'create'+str(i+1)+'/cmd_vel'
		else:
			pose_topic_name = 'ground_pose'
			vel_topic_name = 'cmd_vel'
		pubPoses.append(rospy.Publisher(pose_topic_name, Pose2D, queue_size=10)) 
		pubVels.append(rospy.Publisher(vel_topic_name, Twist, queue_size=10))
	publish_data = [0]*(Num_of_Bots)
	publish_dataType = [-1]*(Num_of_Bots)
	
	#----------------
	#ROS Subscribing
	#----------------
	#If dl is a Central Comp, subscribe to Optitrack data
	if botID == 0:
		#dl.send_msg(botID, 1, 1, [500.0, 1400.0, math.pi])
		#Get data of which nodes we are connected to
		sub_opti = []
		sub_vel = []
		for node in range(Num_of_Bots):
			print "Subscribing to Robot ", node+1
			sub_opti.append(rospy.Subscriber('/Robot_'+str(node+1)+'/ground_pose',Pose2D,dl.opti, callback_args=(node+1)))
			sub_vel.append(rospy.Subscriber('/create'+str(node+1)+'/cmd_vel', Twist, dl.vel, callback_args=(node+1)))
			time.sleep(Bot_Net_ROS.t_interval/1000)
	else:
	#If dl is a Robot then subscribe to only its own pose, vel and consensus		
		print "Subscribed"
		subPose = rospy.Subscriber('ground_pose', Pose2D, dl.broadcast_new, callback_args = (1))
		subCon = rospy.Subscriber('consensus', Pose2D, dl.broadcast_new, callback_args = (3))
		subVel = rospy.Subscriber('cmd_vel', Twist, dl.broadcast_new, callback_args = (2))

	sys.stdout.flush()

	while not rospy.is_shutdown():
		for i in range(Num_of_Bots):
			'''			
			if(botID==0):
				print "Subscribing to Robot ", i+1
				sub_opti = rospy.Subscriber('/Robot_'+str(i+1)+'/ground_pose',Pose2D,dl.opti, callback_args=(i+1))
				time.sleep(Bot_Net_ROS.t_interval/1000)
				sub_opti.unregister()
			'''
			#General data to be published
			IDdata = Int32()
			IDdata.data = botID
			
			count_WiFi = Float64()
			count_WiFi.data = dl.count_WiFi 
			
			count_ET = Float64()
			count_ET.data = dl.count_ET
			
			#Publish Bot Data if event-triggered
			if(dl.publish_data[i] == 1):		
				rospy.loginfo('Updating Create %d\'s Pose', i+1)
				pose = Pose2D()
				pose.x = dl.bot_data[i][1]
				pose.y = dl.bot_data[i][2]
				pose.theta = dl.bot_data[i][3]
				pubPoses[i].publish(pose)

				dl.publish_data[i] = 0
				
			elif(dl.publish_data[i]==2):
				rospy.loginfo('Updating Create %d\'s Vels', i+1)
				twist = Twist()
				twist.linear.x = dl.bot_data[i][4]
				twist.angular.z = dl.bot_data[i][5]
				pubVels[i].publish(twist)
				dl.publish_data[i] = 0

			pubID.publish(IDdata)	
			pubNum.publish(Num_of_Bots)		
			pubWiFiCount.publish(count_WiFi) 
			pubETCount.publish(count_ET)
			rate.sleep()

if __name__ == '__main__':
    try:
		tinyROS()
    except rospy.ROSInterruptException, KeyboardInterrupt:
		print "Ending Program!!!!!!"
		sys.exit(1)		
