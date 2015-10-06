#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import tf
import numpy as np
import math

class Controller:

	def __init__(self):

		self.d = 0.2

		self.xr = np.array([0,0,0]).T
		self.x = np.array([0,0,0]).T

		rospy.init_node('robot_linear_control')
		self.rate = rospy.Rate(10)

		self.use_camera = rospy.get_param('~use_camera',0)
		self.camera_topic = rospy.get_param('~camera_topic','/Robot_1/ground_pose')

		self.pub = rospy.Publisher('/create5/cmd_vel', Twist)
		if self.use_camera:
			self.sub_opti = rospy.Subscriber(self.camera_topic,Pose2D,self.opti)
		else:	
			self.sub_odom = rospy.Subscriber('/create5/odom',Odometry,self.odom)				
		self.sub_goal = rospy.Subscriber('/goal',Pose2D,self.goal)		
		
		self.offset_x = rospy.get_param('~offset_x',0)
        	self.offset_y = rospy.get_param('~offset_y',0)

		rospy.loginfo("Controller initialized.")

		self.control()

	def minmax(self,val):
		if(val > 0):
			return max(1,val)
		else:
			return min(1,val)

	def control(self):

		while not rospy.is_shutdown():
			
			#Define the Twist
			twist = Twist()

			#Get input
			e = np.array([self.xr[0]-self.x[0],self.xr[1]-self.x[1]]).T

			if  np.linalg.norm(e)  < 0.10:
				#Dont move when in correct position
				u = np.array([0,0]).T
				twist.linear.x = 0
				twist.angular.z = 0
			else:

				k1 = 0.1
				k2 = 0.1
				k3 = 0.1

				vr = 0.5
				wr = -0.5

				C = np.array([[np.cos(self.xr[2]-self.x[2]),0],[0,1]])
				u1 = -k1 * e[0]
				u2 = k2 * vr * np.sinc(self.xr[2]-self.x[2])*(self.xr[1]-self.x[1]) - k3*(self.xr[2]-self.x[2])

				v = C.dot(np.array([vr,wr]).T)-np.array([u1,u2])
				twist.linear.x = v[0]
				twist.angular.z = v[1]

			#Publish
			self.pub.publish(twist)		
			self.rate.sleep()
	
	def normalize(self,angle):
		return (angle+np.pi)%(2*np.pi)-np.pi

	def goal(self,pose):
		self.xr = np.array([pose.x+self.offset_x,pose.y+self.offset_y,pose.theta]).T

	def opti(self,pose):
		self.x = np.array([pose.x,pose.y,pose.theta]).T
		self.x[2] = (self.x[2]+np.pi)%(2*np.pi)-np.pi

	def odom(self,odom):
		quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.x = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,(euler[2]+np.pi)%(2*np.pi)-np.pi]).T

if __name__ == '__main__':
    try:
        c = Controller()
    except rospy.ROSInterruptException: pass
