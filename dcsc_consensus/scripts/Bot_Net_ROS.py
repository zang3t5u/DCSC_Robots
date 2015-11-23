#!/usr/bin/env python
# Software License Agreement (BSD License)
# 
# Copyright (c) 2015, TU Delft
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
# Author: Maitreya J Naik
# email: maitreyanaik@gmail.com
# WebSite: maitreyanaik.wordpress.com
#---------------------------------------------------------------------

import os
import sys
import time
import struct
import math

#tos stuff
import Bot_NetMsg
from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

#ROS Stuff
import rospy
import tf
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from dcsc_consensus.msg import bot_data_msg

#For Debugging, change to True
DEBUG = False

#Total TimeInterval in millisecs for 1 bot in TDMA
t_interval = 10
NBots = 1

#Check if Bot Stopped
bot_stopped_broadcast = False

#Check if UART is free
send_queue_size = 50
uartBusy = False
recvUART = False	#If message recvd on UART, UART is locked
	

class Bot_Net:
	def callback(self, data):
		rospy.loginfo(rospy.get_caller_id() + "I heard from %d", data.botID)

	def __init__(self, motestring, Num_of_Bots, botID):
		
		global NBots, send_queue_size
		#***************************************************************************************
		#***************************************************************************************
		#------------------------------------------------------
		#Add more event_trigger conditions for each topic below as:
		# self.event_trigger_<new_topic> = min_value_for_transmission;
		#------------------------------------------------------
		#Condition for broadcasting new values
		Num_Data = 5	#increase by 1 when adding new topics

		self.event_trigger_movement = 0.05;
		self.event_trigger_angle = 0.02;
		self.event_trigger_vel = 0.05;
		self.event_trigger_offset = 0.0005;
		self.event_trigger_centre = 0.0005;
		#------------------------------------------------------
		#Add more event_trigger conditions for each topic above
		#------------------------------------------------------

		#Count the Msgs both on WiFi and Zigbee(Event-Triggered)
		self.count_WiFi = 0.0;
		self.count_ET = 0.0;
		

		self.counter = 0
		self.total_msgs = 0.
		self.mif = MoteIF.MoteIF()
		self.tos_source = self.mif.addSource(motestring)
		self.mif.addListener(self, Bot_NetMsg.Bot_NetMsg)
		
		NBots = Num_of_Bots
		self.motestring = motestring
		self.botID = botID
		
		'''For Msg Communication
		self.send_queue = [Bot_NetMsg.Bot_NetMsg()]*send_queue_size
		self.next_msg_index = 0
		'''
		#Each bot Stores data of all bots
		self.recv_seqNo = [-1]*(NBots+1)	#Store last received seqNo. from Bots + centralComp to Discard duplicates
		self.discarded = [-1]*(NBots+1)		#Keep count of discarded messages from each source
		self.bot_data = []
		for i in range(NBots):
			self.bot_data.append([0]*(1+3*Num_Data))
			self.bot_data[i][0] = i+1

		#Tracking which Robot values were initialized
		self.bot_init_broadcast = [0]*NBots
		print self.bot_init_broadcast
		
		#Tracking changes in data to be published for each bot		
		self.publish_data = [0]*(NBots)
		'''
		#***************************************************************************************
		#***************************************************************************************
		#------------------------------------------------------
		# Add more event_trigger conditions for each topic below
		#-------------------------------------------------------
		Index								Data
		-----								----
		0									ID
		1									X
		2									Y
		3									Theta
		4									VelLinear
		5									VelAngular
		6									0
		7									Offset X
		8									Offset Y
		9									last_update_time
		10									Centre_X
		11									Centre_Y
		12									Centre_Theta
		13									New_Topic_X
		14									New_Topic_Y
		15									New_Topic_Theta
		#***************************************************************************************
		#***************************************************************************************
		#------------------------------------------------------
		# Add more event_trigger dataTypes for each topic below
		#-------------------------------------------------------
		Topic DataType						Topic
		--------------						----
		1									Pose
		2									Velocity
		3									Offset
		4									Centre
		5									New_Topic
		'''
		
		#For TimeSync
		self.start = False
		print "Start clock?: ", self.start
		self.start_time = time.time()
		self.timeOffset = self.botID*t_interval
		self.last_recv_time = 0   
		self.msg_count = 0
		print "TDMA Offset: ", self.timeOffset
		
	
	#---------------------------------------------------
	#	Function receive(self, src, msg)
	#	
	#	Use:
	#	Listens to data broadcasted
	#	Updates current topics and database
	#---------------------------------------------------
	def receive(self, src, msg):
		global recvUART
		recvUART = True  	
		#List of data to be returned
		abstracted_data = []
		if msg.get_amType() == Bot_NetMsg.AM_TYPE:
			self.last_recv_time = time.time() - self.start_time
			#Get Msg Source
			msg_source = msg.getAddr()
			m = Bot_NetMsg.Bot_NetMsg(msg.dataGet())
			#msg_source = m.get_send_ID()
			
			#If first message received from Central Comp, start timer
			if((not self.start) and msg_source==0):
				print "Starting Clock NOW!!!"
				self.start_time = self.last_recv_time + self.start_time - m.get_tx_timestamp()/1000.0
				print "at Time: ", self.start_time
				self.last_recv_time = 0
				self.recv_seqNo = [-1]*(NBots+1)
				self.discarded = [-1]*(NBots+1)
				self.start = True
			
			#Discard duplicate or old messages. 
			#If more than 2 messages discarded, accept message, To account for dropped messages during rollover from 255 to 0
			if((m.get_seqNo() < self.recv_seqNo[msg_source]) and (self.discarded[msg_source])<2):
				#print "Old ", m.get_seqNo()," from ", msg_source
				sys.stdout.flush()
				self.discarded[msg_source] += 1
				recvUART = False
				return
			else:
				self.recv_seqNo[msg_source] = (m.get_seqNo()+1)%256
				self.discarded[msg_source] = 0
			
			#Msg sent by Central Comp
			if(msg_source == 0):
				#Data in message describes the recipient
				msg_bot = m.get_recv_Rob_ID()
			#Msg sent by Bot
			else:
				#Data in message describes the sender
				msg_bot = msg_source
				
			#Extend list to include information
			abstracted_data.extend([float(m.get_tx_timestamp())])
			abstracted_data.extend([float(m.get_seqNo())])
			abstracted_data.extend([msg_bot])
			abstracted_data.extend([m.get_dataType()])
			abstracted_data.extend([m.get_data()])
			self.decode_data(msg_bot, m, msg_source)
			'''  			
			#Print information
			print "Received", msg
			sys.stdout.flush()
			'''
		#Return
		#Print information
		print "From ", msg_source, " Received ", abstracted_data, " at ", 1000*self.last_recv_time
		sys.stdout.flush()
		recvUART = False
		#self.bot_data[msg_bot].append(abstracted_data)

	#---------------------------------------------------
	#	Function send(self, sender_ID, recv_Rob_ID, dataType, data)
	#	
	#	Use:
	#	Creates TinyOS message from arguments
	#	Sends data to UART
	#---------------------------------------------------    
	def send_msg(self, send_ID, recv_Rob_ID, dataType, data):
    	
		global uartBusy, NBots, t_interval
		
		msg_was_sent = False
		
		if((self.start or (self.botID==0)) and (not uartBusy) and (not recvUART)):
			uartBusy = True			
			print "Sending packet ", self.counter
			smsg = Bot_NetMsg.Bot_NetMsg()
			smsg.set_seqNo(self.counter)
			smsg.set_send_ID(send_ID)
			smsg.set_recv_Rob_ID(recv_Rob_ID)
			smsg.set_dataType(dataType)
			if(len(data)<3):
				data.extend(0)
			smsg.set_data(data)

			#Wait for millisec timeslot 
			t = 1000*(time.time()-self.start_time) 
			cond = int((t%((NBots+1)*t_interval))/t_interval)
			if((not self.start) and (self.botID == 0)):
				print "Starting Clock NOW!!!"
				self.start_time = time.time()
				print "at Time: ", self.start_time
				self.recv_seqNo = [-1]*(NBots+1)
				self.discarded = [-1]*(NBots+1)
				self.start = True 
			
			timeslot = self.botID
			#if self.botID == 0:
				#timeslot = self.msg_count
			
			while(cond != timeslot):
				t = 1000*(time.time()-self.start_time)
				cond = int((t%((NBots+1)*t_interval))/t_interval)
				#print t    
			smsg.set_tx_timestamp(t)
			
			#Send Packet twice to ensure delivery and minimize number of dropped packets
			packetCount = 0
			#uartBusy = True
			#self.mif.sendMsg(self.tos_source, 0xFFFF, smsg.get_amType(), 0, smsg) 
			while(packetCount<1):
				#print "Sending packet ", self.counter," : Count ", packetCount
				while(recvUART):
					print "Receiving"
				self.mif.sendMsg(self.tos_source, 0xFFFF, smsg.get_amType(), 0, smsg) 
				packetCount+=1
				#time.sleep(t_interval/1000)

				#Increment Event triggered count
				self.count_ET = self.count_ET + 1.0
				print "Gaya"
			msg_was_sent = True
			uartBusy = False
			self.counter+=1
			self.counter%=256 
		print "Exit Sending"
		self.msg_count = (self.msg_count + 1)%(NBots+1)
		return msg_was_sent
   
	#---------------------------------------------------
	#	Function decode_data(self, msg_bot, m, msg_source)
	#	
	#	Use:
	#	Updates database
	#	Sets corresponding bot's falg to signal calling program to publish to the relevant topics
	#--------------------------------------------------- 	
	def decode_data(self, msg_bot, m, msg_source):
		botIndex = msg_bot-1
		dataType = m.get_dataType()
		t = m.get_tx_timestamp()/1000.0
		data = m.get_data()
		seq = m.get_seqNo()
		
		#Check if received update is indeed new one and not hopped msg
		if(self.bot_data[botIndex][9] > t):		#Old update
			print "Old Update"
			return
		self.bot_data[botIndex][0] = botIndex+1
		'''
		POSE Data => dataType = 1
		Vel Data => dataType = 2
		Leader Data => dataType = 3
		'''
		#Signal Publisher to update values
		self.publish_data[botIndex] = dataType;

		#print " Storing for Robot", msg_bot, "at index: ", botIndex
		self.bot_data[botIndex][1 + 3*(dataType-1)] = data[0]
		self.bot_data[botIndex][2 + 3*(dataType-1)] = data[1]
		self.bot_data[botIndex][3 + 3*(dataType-1)] = data[2]
		self.bot_data[botIndex][9] = t		#Store last update timestamp
		#self.display_database()
    
	def display_database(self):
		if(self.start):
			print "+++++++++++++ Bot Database"
			for i in range(len(self.bot_data)):
				print "Robot ", self.bot_data[i][0], ": "
				print "    x: ", self.bot_data[i][1]
				print "    y: ", self.bot_data[i][2]
				print "theta: ", self.bot_data[i][3]
				print "-------"
		sys.stdout.flush()
	
	#***************************************************************************************
	#***************************************************************************************	
	#------------------------------------------------------
	# Add more Central Computer functions for each subscribed event_trigger topic below here:
	# Use exisitng functions as reference
	# Function Definition should start as:
	#	def <new_topic_function>(self, ROS_Msg_Type, node):
	#	global uartBusy
	#	send_dataType = <new_topic_dataType>
	#	botIndex = node-1
	#-------------------------------------------------------
	#---------------------------------------------------
	#	Function opti(self, pose, robot_node_id)
	#	
	#	Use:
	#	Listens to connected robot's data from the optitrack system and broadcasts to robots if they move by a large distance
	#---------------------------------------------------
	def opti(self,pose, node):
		global uartBusy
		send_dataType = 1
		botIndex = node-1
		#If using /Robot_i/pose of msg type PoseStamped
		#quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
		#euler = tf.transformations.euler_from_quaternion(quaternion)
		#x_new = pose.pose.position.x
		#y_new = pose.pose.position.y
		#theta_new = (euler[2]+math.pi)%(2*math.pi)-math.pi
		
		#If using /Robot_i/ground_pose of msg type Pose2D		
		x_new = pose.x
		y_new = pose.y
		theta_new = pose.theta
		x_old = self.bot_data[botIndex][1]  
		y_old = self.bot_data[botIndex][2] 
		theta_old = self.bot_data[botIndex][3]
		dx = x_new - x_old
		dy = y_new - y_old
		dtheta = theta_new - theta_old

		movement = math.sqrt(dx**2 + dy**2)
		self.count_WiFi = self.count_WiFi + 1.0
		if movement > self.event_trigger_movement or abs(dtheta) > self.event_trigger_angle or not self.start:
			if not self.bot_init_broadcast[botIndex]:
				rospy.loginfo('Broadcasting initial Position of '+ str(node))
				self.bot_init_broadcast[botIndex] = 1
			while uartBusy:
				#print 'Busy'
				pass
			print "Sending New Pose for ", node
			if DEBUG == True:
				print "dTheta is ", abs(dtheta), " against ", self.event_trigger_angle
				print "movement is ", movement, " against ", self.event_trigger_movement
			msg_was_sent = self.send_msg(self.botID, node, send_dataType, [x_new, y_new, theta_new])
			self.publish_data[botIndex] = send_dataType
			if msg_was_sent:
				self.bot_data[botIndex][1] = x_new
				self.bot_data[botIndex][2] = y_new
				self.bot_data[botIndex][3] = theta_new

	#---------------------------------------------------
	#	Function vel(self, twist, robot_node_id)
	#	
	#	Use:
	#	Listens to connected robot's cmd_vel topic and transmits if significant change is given
	#---------------------------------------------------
	def vel(self, twist, node):
		global uartBusy
		send_dataType = 2
		botIndex = node-1
		x_new = twist.linear.x
		y_new = twist.angular.z
		theta_new = 0

		x_old = self.bot_data[botIndex][4]  
		y_old = self.bot_data[botIndex][5] 
		theta_old = self.bot_data[botIndex][6]

		dx = x_new - x_old
		dy = y_new - y_old
		dtheta = theta_new - theta_old
				
		change = math.sqrt(dx**2 + dy**2 + dtheta**2)
		condn = self.event_trigger_vel
		self.count_WiFi = self.count_WiFi + 1.0
		if change > condn:
			while uartBusy:
				#print "Busy for Vel"				
				pass
			print "Sending New Vel for ", node
			msg_was_sent = self.send_msg(self.botID, node, send_dataType, [x_new, y_new, theta_new])
			self.publish_data[botIndex] = send_dataType
			if msg_was_sent:
				self.bot_data[botIndex][4] = x_new
				self.bot_data[botIndex][5] = y_new
				self.bot_data[botIndex][6] = theta_new
	
	#---------------------------------------------------
	#	Function offset(self, pose, robot_node_id)
	#	
	#	Use:
	#	Listens to connected robot's formation_offset topic and transmits if significant change is given
	#---------------------------------------------------
	def offset(self, pose, node):
		global uartBusy
		send_dataType = 3
		botIndex = node-1
		x_new = pose.x
		y_new = pose.y
		theta_new = 0

		x_old = self.bot_data[botIndex][7]  
		y_old = self.bot_data[botIndex][8] 
		theta_old = 0

		dx = x_new - x_old
		dy = y_new - y_old
		dtheta = theta_new - theta_old
				
		change = math.sqrt(dx**2 + dy**2 + dtheta**2)
		condn = self.event_trigger_offset
		self.count_WiFi = self.count_WiFi + 1.0
		if change > condn:
			while uartBusy:
				#print "Busy for Offset"				
				pass
			print "Sending New Offset for ", node
			msg_was_sent = self.send_msg(self.botID, node, send_dataType, [x_new, y_new, theta_new])
			self.publish_data[botIndex] = send_dataType
			if msg_was_sent:
				self.bot_data[botIndex][7] = x_new
				self.bot_data[botIndex][8] = y_new
				#Preserve time of last update
				self.bot_data[botIndex][9] = self.bot_data[botIndex][9]

	#---------------------------------------------------
	#	Function centre(self, pose, robot_node_id)
	#	
	#	Use:
	#	Listens to connected robot's formation_centre topic and transmits if significant change is given
	#---------------------------------------------------
	def centre(self, pose, node):
		global uartBusy
		send_dataType = 4
		botIndex = node-1
		x_new = pose.x
		y_new = pose.y
		theta_new = pose.theta

		x_old = self.bot_data[botIndex][10]  
		y_old = self.bot_data[botIndex][11] 
		theta_old = self.bot_data[botIndex][12] 

		dx = x_new - x_old
		dy = y_new - y_old
		dtheta = theta_new - theta_old
				
		change = math.sqrt(dx**2 + dy**2 + dtheta**2)
		condn = self.event_trigger_centre
		self.count_WiFi = self.count_WiFi + 1.0
		if change > condn:
			while uartBusy:
				#print "Busy for Centre"				
				pass
			print "Sending New Centre for ", node
			msg_was_sent = self.send_msg(self.botID, node, send_dataType, [x_new, y_new, theta_new])
			self.publish_data[botIndex] = send_dataType
			if msg_was_sent:
				self.bot_data[botIndex][10] = x_new
				self.bot_data[botIndex][11] = y_new
				#Preserve time of last update
				self.bot_data[botIndex][12] = theta_new
		
	#***************************************************************************************
	#***************************************************************************************	
	#------------------------------------------------------
	# Add more Robot conditions for each subscribed event_trigger topic below here:
	#-------------------------------------------------------	
	#---------------------------------------------------
	#	Function broadcast_new(self, data, dataType)
	#	
	#	Use:
	#	Callback function run on Robots to broadcast new values if event triggering conditions are satisfied
	#---------------------------------------------------
	def broadcast_new(self, data, dataType):
		global uartBusy, bot_stopped_broadcast
		botIndex = self.botID-1
		x_old = self.bot_data[botIndex][1 + 3*(dataType-1)] 
		y_old = self.bot_data[botIndex][2 + 3*(dataType-1)]
		theta_old = self.bot_data[botIndex][3 + 3*(dataType-1)]
		strType = ''

		#***************************************************************************************
		#***************************************************************************************	
		#------------------------------------------------------
		# Add more Robot conditions for each subscribed event_trigger topic below here as:
		#elif dataType == <new_topic_dataType>:
		#	strType = 'New_Topic'
		#	x_new = ROS_Msg_Type.data1
		#	y_new = ROS_Msg_Type.data2
		#	theta_new = ROS_Msg_Type.data3
		#	condn = self.event_trigger_<new_topic>
		#-------------------------------------------------------	
		#If Pose data is to be updated
		if dataType == 1:
			strType = 'Pose'
			x_new = data.x
			y_new = data.y
			theta_new = data.theta
			condn = self.event_trigger_movement 
			self.count_WiFi = self.count_WiFi + 1.0
		#If Twist is to be transmitted
		elif dataType == 2:
			strType = 'Velocity'
			x_new = data.linear.x
			y_new = data.angular.z
			theta_new = 0
			if x_new != 0:
				bot_stopped_broadcast = False;
			condn = self.event_trigger_vel
		elif dataType == 3:
			strType = 'Offset'
			x_new = data.x
			y_new = data.y
			theta_new = 0
			condn = self.event_trigger_offset
		elif dataType == 4:
			strType = 'Centre'
			x_new = data.x
			y_new = data.y
			theta_new = data.theta
			condn = self.event_trigger_centre

		dx = x_new - x_old
		dy = y_new - y_old
		dtheta = theta_new - theta_old
		
		print "Bot Update"
		change = math.sqrt(dx**2 + dy**2 + dtheta**2)
		#***************************************************************************************
		#***************************************************************************************
		# Edit Transmission condition as needed
		#---------------------------------------------------------------------------------------
		if change > condn or abs(dtheta) > self.event_trigger_angle or (dataType==2 and x_new == 0 and not bot_stopped_broadcast):
			rospy.loginfo('Broadcasting new Values of ' + strType)
			while uartBusy:
				print "hehe"
			self.send_msg(self.botID, 0, dataType, [x_new, y_new, theta_new])
			t = self.bot_data[botIndex][9]		#Store last update timestamp
			self.bot_data[botIndex][1 + 3*(dataType-1)]  = x_new
			self.bot_data[botIndex][2 + 3*(dataType-1)]  = y_new
			self.bot_data[botIndex][3 + 3*(dataType-1)]  = theta_new
			self.bot_data[botIndex][9] = t		#Store last update timestamp
			if dataType==2 and x_new == 0 and not bot_stopped_broadcast:
				bot_stopped_broadcast = True;
			
'''
    def main_loop(self):
    	while 1:
    		time.sleep(1)
    		# send a message at 1 Hz
    		self.send_msg()

def main():
	if '-h' in sys.argv or len(sys.argv) < 2:
		print "Usage:", sys.argv[0], "sf@localhost:9002", "adc_rev_volt <Volt>" 
		sys.exit()
		
	dl = Bot_Net(sys.argv[1], float(sys.argv[2]))
	dl.main_loop()  # don't expect this to return...

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass
'''
