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

import rospy
from std_msgs.msg import String
from dcsc_consensus.msg import bot_data_msg

import os
import sys
import subprocess
import time
import struct
import math

#TOS Stuff
import Bot_Net_ROS

def talker():
	if '-h' in sys.argv or len(sys.argv) < 3:
		print "Usage:", sys.argv[0], "Num_of_Bots", "BotID" , "sf@localhost:9002"
		sys.exit()
	if(len(sys.argv) < 4):
		test = subprocess.Popen(['motelist| grep "/dev/ttyUSB"'], stdout=subprocess.PIPE, shell = True)
		output = test.communicate()[0].split("\n")
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

	node_name = 'talker'+str(int(botID))
	print "Starting ROSnode named: ", node_name
	rospy.init_node(node_name, anonymous=True)
	rate = rospy.Rate(10) # 10hz

	dl = Bot_Net_ROS.Bot_Net(arg, Num_of_Bots, botID)
	count = 0;
	sys.stdout.flush()

	while not rospy.is_shutdown():
		for i in range(Num_of_Bots):
			if(dl.publish_data[i] == 1):		
				data = bot_data_msg(botID = dl.bot_data[i][0], x = dl.bot_data[i][1], y = dl.bot_data[i][2], theta = dl.bot_data[i][3])
				rospy.loginfo(data)
				dl.pub.publish(data)
				dl.publish_data[i] = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        pass
