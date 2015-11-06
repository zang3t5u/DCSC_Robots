#!/usr/bin/env python
import rospy
import tf

from numpy.matlib import *
from scipy.optimize import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import sys

class Controller:

	def __init__(self):
		if '-h' in sys.argv or len(sys.argv) < 3:
			print "Usage:", sys.argv[0], "Num_of_Bots", "BotID"
			sys.exit()
		self.botID = int(sys.argv[2])
		self.Num_of_Bots = int(sys.argv[1])

		#Initialize controller properties
		self.start=False
		self.Nc = 10
		self.u0 = linspace(0,0,2*self.Nc).T
		self.k = array([0.05,0.05]).T
		self.dt = 0.1
		self.bnds = list()
		for i in range(0,self.Nc):
		 	self.bnds.append((-0.1,5))
		for i in range(0,self.Nc):
			self.bnds.append((-1,1))
		self.bdns = tuple(self.bnds)
		self.x = array([0,0,0]).T		
		self.xr = array([0,0]).T
		self.xl = array([0,0]).T
		self.dl = array([0,0]).T
		self.al = 0

		#Create rosnode
		rospy.init_node('robot_mpc_control')
		self.rate = rospy.Rate(10)

		#Parameters
		#use_camera = rospy.get_param('~use_camera',0)
		camera_topic = rospy.get_param('~camera_topic','')
		#if(camera_topic == ''): 
		#	use_camera = 0

		dx = rospy.get_param('offset_x',0)
		dy = rospy.get_param('offset_y',0)

		#Pubsub
		self.pub = rospy.Publisher('cmd_vel', Twist)

		#if use_camera:
		#self.sub_opti = rospy.Subscriber(camera_topic,PoseStamped,self.opti)
		#	rospy.loginfo("Using camera feedback.")
		#else:	
		#	self.sub_odom = rospy.Subscriber('odom',Odometry,self.odom)	
		#	rospy.loginfo("Using odometry feedback.")
		#else:
		self.sub_pos = rospy.Subscriber('ground_pose',Pose2D,self.ground)

		self.sub_goal = rospy.Subscriber('flocking_centre',Pose2D,self.goal)		
		self.sub_leader = rospy.Subscriber('flocking_centre',Pose2D,self.leader)
		self.sub_relative_pose = rospy.Subscriber('flocking_offset',Pose2D,self.rpose)

		#Listen to other robot positions
		self.Num_of_Robots = rospy.get_param('Num_of_Bots', 5)
		self.botID = rospy.get_param('botID', int(sys.argv[2]))
		self.connected_to = rospy.get_param('connected_to',[])
		self.subs = []
		self.xobst = {}
		for node in self.connected_to:
			self.subs.append(rospy.Subscriber('/create'+str(node)+'/ground_pose',Pose2D,self.listen,callback_args=(node)))
			self.xobst[node] = (0,0)			
			rospy.loginfo("Subscribed to /create"+str(node)+'/ground_pose')
		
		#Get formation position
		self.dl = array([dx,dy]).T

		#Log
		rospy.loginfo("Controller initialized.")

		#Start control loop
		self.control()
	
	def satmin(self,val,valmin):
		if(val < valmin and val > -valmin):
			return 0
		else:
			return val

	def control(self):

		while not rospy.is_shutdown():
			
			#Define the Twist
			twist = Twist()
			while not self.start:
				pass
			#Run minimization (temporarily follow the target directly)
			res = minimize(self.cost,self.u0,args=(self.x,self.xl,self.xr),method='SLSQP',bounds=self.bnds,tol=1E-3,options={'maxiter': 20,'disp': False})
			v = res.x[0:self.Nc] 
			w = res.x[self.Nc:2*self.Nc]
			
			v, w = self.avoid(v,w)

			#Define error to VL
			e = (self.xl[0]+self.dl[0]-self.x[0])**2+(self.xl[1]+self.dl[1]-self.x[1])**2
			if e > 0.01:			
				twist.linear.x = self.satmin(v,0.01)
				twist.angular.z = self.satmin(w,0.05)
			else:
				twist.linear.x = 0
				twist.angular.z = 0

			#Update the Virtual Leader state (to be added later with consensus)
			#self.xl = self.xl + self.dt * self.k * (self.xr - self.xl)		
			#rospy.loginfo(self.xl)	

			#Publish and sleep
			self.pub.publish(twist)		
			self.rate.sleep()

	def bump(self,v,w):
		return v[0], w[0]	

	def avoidorigin(self,v,w):

		x1 = self.x
		xobst1 = array([0.5,2,0]).T
		if sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2) < 1:
			return 0, 0
		else:
			return v[0], w[0]

	def avoid(self,v,w):

		x1 = self.x
		x2 = x1 + self.dt * array([cos(x1[2])*v[0],sin(x1[2])*v[0],w[0]]).T
		
		xdot = array([x2[0]-x1[0],x2[1]-x1[1]]).T / self.dt
		d = 0.1

		for node in self.connected_to:			

			xobst1 = array([self.xobst[node][0],self.xobst[node][1],0]).T
			xobst2 = xobst1 + self.dt * array([self.k[0],self.k[1],0]) * (array([self.xr[0],self.xr[1],0]).T - xobst1)

			if sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2) < 0.75:
				
				return v[0] / 2, w[0] + pi

				u = array([[0,-1],[1,0]]).dot(xdot)
				u = array([[cos(self.x[2]),sin(self.x[2])],[-sin(self.x[2])/d,cos(self.x[2])/d]]).dot(u)

				#eR2R1 = x2 - x1
				#eO1R1 = xobst1 - x1
				#eO2O1 = xobst2 - xobst1

				#PERV = array([ [ eO1R1[1],  -eO1R1[1] , 0 ] , [ -eO1R1[0] , eO1R1[0] , 0 ] ])
				#c = PERV.dot(eO2O1)		
				#i = argmin(c)
				#pervmin = PERV[:,i]
				
				#rR2R1 = (linalg.norm(eO2O1) / linalg.norm(eO1R1)) * pervmin + array([eR2R1[0],eR2R1[1]]).T
				
				#u = array([[cos(self.x[2]),sin(self.x[2])],[-sin(self.x[2])/d,cos(self.x[2])/d]]).dot(rR2R1)
							
				return u[0], u[1]

		return v[0], w[0]
				

	def cost(self,u,x,xl,xr):	
		#Split input vector	
		v = u[0:self.Nc] 
		w = u[self.Nc:2*self.Nc]		
		#Cost 
		cost = 0		
		#Simulate
		for i in range(0,self.Nc):		
			#Simulate virtual leader
			xl = xl + self.dt * self.k * (xr - xl)
			#Simulate Create
			x = x + self.dt * array([cos(x[2])*v[i],sin(x[2])*v[i],w[i]]).T
			x[2] = (x[2] + pi) % (2 * pi) - pi		
			#Compute cost function at current time step
			cost = cost + 2E1 * (xl[0]+self.dl[0]-x[0])**2 + 2E1 * (xl[1]+self.dl[1]-x[1])**2 + (self.al-x[2])**2 + v[i]**2/1E2 + w[i]**2/1E2
		return cost
	
	def leader(self,pose):
		self.xl = array([pose.x,pose.y]).T
		self.al = pose.theta

	def goal(self,pose):
		self.xr = array([pose.x,pose.y]).T

	def rpose(self,pose):
		self.dl = array([pose.x,pose.y]).T

	def opti(self,pose):
		quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.x = array([pose.pose.position.x,pose.pose.position.y,(euler[2]+pi)%(2*pi)-pi]).T

	def odom(self,odom):
		quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.x = array([odom.pose.pose.position.x,odom.pose.pose.position.y,(euler[2]+pi)%(2*pi)-pi]).T
	
	def ground(self, pose):
		self.x = array([pose.x, pose.y, pose.theta]).T
		self.start = True

	def listen(self,pose,node):
		self.xobst[node] = (pose.x,pose.y)

if __name__ == '__main__':
    try:
        c = Controller()
    except rospy.ROSInterruptException: pass
