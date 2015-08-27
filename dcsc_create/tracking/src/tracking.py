#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import tf
import numpy as np

class Controller:

	def __init__(self):

		self.d = 0.2

		self.xr = np.array([0,0,0]).T
		self.x = np.array([0,0,0]).T

		self.pub = rospy.Publisher('/create5/cmd_vel', Twist)
		self.sub_odom = rospy.Subscriber('/create5/odom',Odometry,self.odom)
    		self.sub_goal = rospy.Subscriber('/create5/goal',Pose2D,self.goal)
		rospy.init_node('robot_linear_control')
		self.rate = rospy.Rate(10)
		
		rospy.loginfo("Controller initialized.")

		self.control()

	def control(self):

		while not rospy.is_shutdown():
			
			#Define the Twist
			twist = Twist()

			#Get input
			e = np.array([self.xr[0]-self.x[0],self.xr[1]-self.x[1]]).T
			if np.linalg.norm(e) < 0.10:
				#Dont move when in position
				u = np.array([0,0]).T
				twist.linear.x = 0
				twist.angular.z = 0
			else:
				#Get control input
				u = e / 2
				rospy.loginfo("Moving the robot.")			
				#Transform to unicycle
				v = np.array([[np.cos(self.x[2]),np.sin(self.x[2])],[-np.sin(self.x[2])/self.d,np.cos(self.x[2])/self.d]]).dot(u)
				#Prevent inputs that are too small
				if(v[0]>0):
					twist.linear.x = max(0.05,v[0])
				else:
					twist.linear.x = min(-0.05,v[0])
				if(v[1]>0):
					twist.angular.z = max(0.05,v[1])
				else:
					twist.angular.z = min(-0.05,v[1])

			#Publish
			self.pub.publish(twist)		
			self.rate.sleep()
	
	def goal(self,pose):
		self.xr = np.array([pose.x,pose.y,pose.theta]).T

	def odom(self,odom):
		quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.x = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,(euler[2]+np.pi)%(2*np.pi)-np.pi]).T

if __name__ == '__main__':
    try:
        c = Controller()
    except rospy.ROSInterruptException: pass
