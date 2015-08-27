#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D

class filter:

	def __init__(self):

		#Initialize ROS
		rospy.init_node('create_pose', anonymous=True)

		#Robot state
		self.x = np.array([0,0,0]).T
		self.dx = np.array([0,0,0]).T	

		#Listen to camera data and input
    		rospy.Subscriber("camera", String, self.mz)
		rospy.Subscriber("vel", String, self.mu)

    		rospy.spin()

	def mz(self,data):
		pass

	def mu(self,data):
		pass
	   
    
if __name__ == '__main__':
	
	f = filter()
