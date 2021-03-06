#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import tf
import numpy as np

class Controller:

	def __init__(self):

		self.d = 0.1
		self.pos_e = 0.10
		self.v_min = 0.05
		self.w_min = 0.05
		self.w_max = 1.0

		self.xr = np.array([0,0,0]).T
		self.x = np.array([0,0,0]).T

		rospy.init_node('robot_linear_control')
		self.rate = rospy.Rate(10)

		self.use_camera = rospy.get_param('~use_camera',0)
		self.camera_topic = rospy.get_param('~camera_topic','/Robot_5/ground_pose')

		self.pub = rospy.Publisher('/create5/cmd_vel', Twist)
		#if self.use_camera:
		self.sub_opti = rospy.Subscriber(self.camera_topic,Pose2D,self.opti)
		#else:	
		#	self.sub_odom = rospy.Subscriber('/create5/odom',Odometry,self.odom)				
		self.sub_goal = rospy.Subscriber('/Robot_4/ground_pose',Pose2D,self.goal)		
		
		self.offset_x = rospy.get_param('~offset_x',0.1)
        	self.offset_y = rospy.get_param('~offset_y',0.1)

		rospy.loginfo("Controller initialized.")

		#self.wp = 0

		self.control()

	def control(self):

		while not rospy.is_shutdown():
			
			#Define the Twist
			twist = Twist()

			#Get input
			e = np.array([self.xr[0]-self.x[0],self.xr[1]-self.x[1]]).T
			if np.linalg.norm(e) < self.pos_e:
				#Dont move when in position
				u = np.array([0,0]).T
				twist.linear.x = 0
				twist.angular.z = 0
			else:
				#Get control input
				u = - e / 2
				rospy.loginfo(np.linalg.norm(e))			
				#Transform to unicycle
				v = np.array([[np.cos(self.x[2]),np.sin(self.x[2])],[-np.sin(self.x[2])/self.d,np.cos(self.x[2])/self.d]]).dot(u)

				#Smooth out the turning				
				#self.wp = self.wp +  * 0.1

				#Prevent inputs that are too small
				if(v[0]>0):
					twist.linear.x = max(self.v_min,v[0])
				else:
					twist.linear.x = min(-self.v_min,v[0])
				if(v[1]>0):
					twist.angular.z = min(self.w_max,max(self.w_min,v[1]))
				else:
					twist.angular.z = max(-self.w_max,min(-self.w_min,v[1]))

			#Publish
			self.pub.publish(twist)		
			self.rate.sleep()
	
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
