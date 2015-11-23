#!/usr/bin/env python
import rospy
import tf

from numpy.matlib import *
from scipy.optimize import *
from numpy.linalg import *
import numpy as np

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
		self.Nc = 5
		self.u0 = linspace(0,0,2*self.Nc).T
		#Position PD constants		
		self.k = array([0.5,0.5]).T
		self.kd = array([0.25, 0.25]).T
		#Velocity PD constants
		self.kvp = array([0.5,0.2]).T
		self.kvd = array([0,0]).T
		self.dt = 0.1
		#Movement Variables
		self.e0 = 0
		self.theta0 = 0
		self.alpha0 = 0
		self.mul = 0.5
		self.k_w = 1*self.mul
		self.gamma_v = 2*self.mul
		self.h_theta = 1*self.mul

		self.bnds = list()
		for i in range(0,self.Nc):
		 	self.bnds.append((0,1))
		for i in range(0,self.Nc):
			self.bnds.append((-1,1))
		self.bdns = tuple(self.bnds)

		#Initialize robot
		self.x = array([0,0,0]).T		
		self.xr = array([0,0]).T
		self.xr_theta = 0
		self.xl = array([0,0]).T
		self.dl = array([0,0]).T
		self.al = 0

		#Create rosnode
		rospy.init_node('create'+str(self.botID)+'_robot_mpc_control')
		self.rate = rospy.Rate(10)

		#Parameters
		camera_topic = rospy.get_param('~camera_topic','')

		#Pubsub
		self.pub = rospy.Publisher('/create'+str(self.botID)+'/cmd_vel', Twist)
		self.sub_pos = rospy.Subscriber('/create'+str(self.botID)+'/ground_pose',Pose2D,self.ground)
		self.sub_goal = rospy.Subscriber('/flocking_centre',Pose2D,self.goal)		
		self.sub_relative_pose = rospy.Subscriber('/create'+str(self.botID)+'/flocking_offset',Pose2D,self.relative_pose)
		
		#To keep track of bots counted
		self.pos_updated = [0]*self.Num_of_Bots	
		self.vel_updated = [0]*self.Num_of_Bots	
		self.offset_updated = 0
		self.subs = []
		self.xobst = {}
		self.subs_vel = []
		self.xobst_vel = {}
		for i in range(self.Num_of_Bots):
			if i == self.botID-1:
				continue
			self.subs.append(rospy.Subscriber('/create'+str(i+1)+'/ground_pose',Pose2D,self.listen_pose,callback_args=(i+1)))
			self.subs_vel.append(rospy.Subscriber('/create'+str(i+1)+'/cmd_vel',Twist,self.listen_vel,callback_args=(i+1)))
			self.xobst[i] = (float('infinity'),float('infinity'), 0)			
			self.xobst_vel[i] = (float('infinity'),float('infinity'))
			rospy.loginfo("Subscribed to /create"+str(i+1)+"/ground_pose and /create"+str(i+1)+"/cmd_vel")
	
		#Log
		rospy.loginfo("Controller initialized.")

		#Start control loop
		self.control()

	def control(self):
		self.dist_old = 0
		self.ang_old = 0
		while not rospy.is_shutdown():			
			#while not self.start:
			#	pass

			#Define the Twist
			twist = Twist()

			goal = self.xl+self.dl
			if(norm(array([self.x[0], self.x[1]]).T - (self.xr+self.dl)) < 0.1):
				goal = np.append(goal, self.xr_theta)
				rospy.loginfo("Near Goal----------------------------------------------")
			else:
				goal = np.append(goal, self.al)
			
			#Run unicycle model
			v, w = self.calc_vel(goal)

			#Run minimization
			#res = minimize(self.cost,self.u0,args=(self.x,self.xl,self.xr),method='SLSQP',bounds=self.bnds,tol=1E-3,options={'maxiter': 10,'disp': False})
			#v = res.x[0:self.Nc] 
			#w = res.x[self.Nc:2*self.Nc]
			#v = v[0]
			#w = w[0]
			v = min(max(0, v), 1)
			rospy.loginfo("YE DEKHO!!!!!!!!!!!!!!!!!!!"+str(array([v, w]).T))
			v, w = self.avoid(v, w)
			v = min(max(0, v), 0.5)
			v = self.satmin(v, 0.005)
			if(v == 0):
				w = self.satmin(w, 0.01)
			#Define error to VL
			e = sqrt((self.xl[0]+self.dl[0]-self.x[0])**2+(self.xl[1]+self.dl[1]-self.x[1])**2)
			self.phi = self.x[2] - goal[2]
			if e > 0.10 or abs(self.phi) > 0.3:			
				twist.linear.x = v
				twist.angular.z = w
			else:
				twist.linear.x = 0
				twist.angular.z = 0

			#Update the Virtual Leader state (to be added later with consensus)
			self.xl = self.xl + self.dt * self.k * (self.xr - self.xl)	
			self.al = arctan2(self.xr[1] - self.xl[1], self.xr[0] - self.xl[0])	

			#Update the robot pose
			#self.x = self.x + self.dt * array([ cos(self.x[2])*twist.linear.x , sin(self.x[2])*twist.linear.x , twist.angular.z ]).T
			#self.x[2] = (self.x[2] + pi) % (2 * pi) - pi	
			
			#Publish and sleep
			rospy.loginfo(self.x)
			rospy.loginfo('Leader'+str(self.xl))
			rospy.loginfo('Goal'+str(self.xr))
			self.pub.publish(twist)		
			self.rate.sleep()

	#Unicylce model for velocity calculation
	def calc_vel(self, goal):
		self.state = self.x
		self.phi = self.state[2] - goal[2]
		self.phi = (self.phi + pi) % (2 * pi) - pi	
		
		self.e0 = sqrt((goal[0] - self.state[0])**2 + (goal[1] - self.state[1])**2)
		if self.e0 < 0.10 and abs(self.phi)<0.3:
			rospy.loginfo("Angle: "+str(self.phi))
			return 0, 0
		if self.e0< 0.10:
			self.theta0 = 0
		else:
			self.theta0 = arctan2(goal[1] - self.state[1], goal[0] - self.state[0]) - goal[2]
		self.theta0 = (self.theta0 + pi) % (2 * pi) - pi
		#if(self.theta0>)
		print "Theta: "+str(self.theta0)
		self.alpha0 = self.theta0 - (self.phi)
		self.alpha0 = (self.alpha0 + pi) % (2 * pi) - pi
		print "Alpha: "+str(self.alpha0)

		if self.e0 > 0.10:
			v = self.gamma_v*cos(self.alpha0)*self.e0
		else:
			v = 0
		if abs(self.alpha0) > 0.02:
			w = (self.k_w*self.alpha0 + self.gamma_v*sin(2*self.alpha0)*(self.alpha0 + self.h_theta*self.theta0)/(2*self.alpha0))
		else:
			w = self.gamma_v*self.h_theta*self.theta0
		w = (w + pi) % (2 * pi) - pi
		return v, w

	def cost(self,u,x,xl,xr):	
		#Split input vector	
		v = u[0:self.Nc] 
		w = u[self.Nc:2*self.Nc]		
		dxl = array([0,0]).T
		xl_old = array([0,0]).T
		#Cost 
		cost = 0		
		#Simulate
		for i in range(0,self.Nc):		
			#Simulate virtual leader
			xl_old = xl
			xl = xl + self.dt * array([min(0.2,max(-0.2,self.k[0]*(xr[0]-xl[0])+self.kd[0]*dxl[0])),min(0.2,max(-0.2,self.k[1]*(xr[1]-xl[1])+self.kd[1]*dxl[1]))]).T
			dxl = (xl-xl_old) / self.dt
			#Simulate Create
			x = x + self.dt * array([cos(x[2])*v[i],sin(x[2])*v[i],w[i]]).T
			ar = arctan2(xl[1]+self.dl[1]-x[1],xl[0]+self.dl[0]-x[0])
			x[2] = (x[2] + pi) % (2 * pi) - pi		
			#Compute cost function at current time step
			cost = cost + (xl[0]+self.dl[0]-x[0])**2 + (xl[1]+self.dl[1]-x[1])**2 + (ar-x[2])**2 + v[i]**2/100 + w[i]**2/100
		return cost
	
	def avoid(self,v,w):

		x1 = self.x
		x2 = x1 + self.dt * array([cos(x1[2])*v,sin(x1[2])*v,w]).T
		
		xdot = array([x2[0]-x1[0],x2[1]-x1[1], x2[2] - x1[2]]).T / self.dt
		
		#Obstacle angle constraint for collision avoidance
		obst_ang_min = 0.5
		#Gains for collision avoidance and reaching goal
		k_avoid = 1
		k_goal = 0.25
		#Robot Diameter
		rob_dia = 0.35
		dists = []
		obst_index = []

		#Find displacement of closest obstacle in next time step 
		for node in range(self.Num_of_Bots):
			if self.pos_updated[node] == 0 or node == self.botID-1:
				continue
			xobst1 = array([self.xobst[node][0],self.xobst[node][1], self.xobst[node][2]]).T
			if self.vel_updated[node] == 1:
				vobst = self.xobst_vel[node][0]
				wobst = self.xobst_vel[node][1]
			else:
				vobst = 0
				wobst = 0
			xobst2 = xobst1 + self.dt * array([cos(xobst1[2])*vobst,sin(xobst1[2])*vobst,wobst]).T
			dists.append(sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2) + sqrt((xobst2[0]-x2[0])**2+(xobst2[1]-x2[1])**2))	
			obst_index.append(node)
		
		#If no obstacle, preserve current velocities
		rospy.loginfo("No. of obstacles: "+str(len(dists)))				
		if len(dists) == 0:
			return v, w

		#Check collsion with closest obstacle
		node = obst_index[dists.index(min(dists))]
		dist_node = dists[dists.index(min(dists))]
		
		if self.pos_updated[node] == 0:
				return v, w
		xobst1 = array([self.xobst[node][0],self.xobst[node][1], self.xobst[node][2]]).T
		if self.vel_updated[node] == 1:
			vobst = self.xobst_vel[node][0]
			wobst = self.xobst_vel[node][1]
		else:
			vobst = 0
			wobst = 0
		xobst2 = xobst1 + self.dt * array([cos(xobst1[2])*vobst,sin(xobst1[2])*vobst,wobst]).T
		rospy.loginfo("Closest robot: "+str(node+1)+" at: "+str(sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2)-rob_dia))

		#Check if robot is in collision zone of obstacle
		if (sqrt((xobst2[0]-x2[0])**2+(xobst2[1]-x2[1])**2)-rob_dia < 0.2) or (sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2)-rob_dia < 0.4):
			#If goal is nearer to bot than obstacle in the next time step, then obstacle won't hinder robot's path
			if (sqrt((self.xl[0]+self.dl[0]-x1[0])**2+(self.xl[1]+self.dl[1]-x1[1])**2)) < (sqrt((xobst2[0]-x2[0])**2+(xobst2[1]-x2[1])**2) - rob_dia/2):
				#If robot is moving closer to the goal
				if sqrt((self.xl[0]+self.dl[0]-x1[0])**2+(self.xl[1]+self.dl[1]-x1[1])**2) < sqrt((self.xl[0]+self.dl[0]-x2[0])**2+(self.xl[1]+self.dl[1]-x2[1])**2):
					return v, w

		#If obstacle is in collision radius, keep the obstacle to the right (Turn left)
		#if (sqrt((xobst2[0]-x2[0])**2+(xobst2[1]-x2[1])**2)-rob_dia < 0.4) and (sqrt((xobst2[0]-x2[0])**2+(xobst2[1]-x2[1])**2)<sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2)):
			rospy.loginfo('Too close to bot '+str(node+1))
			rospy.loginfo('Seperation: '+str(sqrt((xobst1[0]-x1[0])**2+(xobst1[1]-x1[1])**2)-rob_dia))
			#return max(-1, min(v/10, 1)), max(-1, min(w+pi, 1))
			
			#Check obstacle orientation with respect to robot and make according rotation
			eO1R1 = xobst1 - x1
			obst_ang = arctan2(eO1R1[1], eO1R1[0]) - arctan2(xdot[1], xdot[0])
			obst_ang = (obst_ang + pi) % (2 * pi) - pi
			rospy.loginfo("Obstacle Path Angle: "+str(obst_ang))

			#if -1*obst_ang_min < obst_ang and obst_ang <= 0:
			if abs(obst_ang) <= pi/2 and obst_ang <= 0:
				rospy.loginfo("Obstacle on Front Right, rotate anti-clockwise")
				#u = array([[0,-1, 0],[1,0, 0],[0, 0, 0]]).dot(eO1R1)
				u = array([[-1,0, 0],[0,-1, 0],[0, 0, 0]]).dot(eO1R1)
				u[2] = pi/2
				#u[2] = arctan2(u[1], u[0])
			elif abs(obst_ang) > pi/2 and obst_ang <= 0:
				rospy.loginfo("Obstacle on Rear Right, rotate clockwise")
				#u = array([[0,-1, 0],[1,0, 0],[0, 0, 0]]).dot(eO1R1)
				u = array([[-1,0, 0],[0,-1, 0],[0, 0, 0]]).dot(eO1R1)
				u[2] = -pi/2
				#u[2] = arctan2(u[1], u[0])
			elif abs(obst_ang) <= pi/2 and obst_ang > 0:
				rospy.loginfo("Obstacle on Front Left, rotate clockwise")
				#u = array([[0,1, 0],[-1,0, 0],[0, 0, 0]]).dot(eO1R1)
				u = array([[-1,0, 0],[0,-1, 0],[0, 0, 0]]).dot(eO1R1)
				u[2] = -pi/2
				#u[2] = arctan2(u[1], u[0])
			elif abs(obst_ang) > pi/2 and obst_ang > 0:
				rospy.loginfo("Obstacle on Rear Left, rotate anti-clockwise")
				#u = array([[0,1, 0],[-1,0, 0],[0, 0, 0]]).dot(eO1R1)
				u = array([[-1,0, 0],[0,-1, 0],[0, 0, 0]]).dot(eO1R1)
				u[2] = pi/2
				#u[2] = arctan2(u[1], u[0])
			else:
				u = array([[-1,0, 0],[0,-1, 0], [0, 0, 1]]).dot(eO1R1)
				u[2] = w
			rospy.loginfo("Old: "+str(eO1R1))
			rospy.loginfo("New: "+str(u))
			rospy.loginfo("Dot: "+str(u.dot(xdot)))
			ang = u[2]
			u = u/(norm(array([eO1R1[0], eO1R1[1]]).T)**2)
			u[2] = ang
			u = k_avoid*u + k_goal*array([xdot[0], xdot[1], 0]).T
			#Unicycle inverse kinematics to find required velocities
			#u[2] = (obst_ang + pi) % (2 * pi) - pi
			
			#Make magnitude and angle of resultant vector as velocity
			#v = sqrt(u[0]**2+u[1]**2)
			#w = arctan2(u[1], u[0])
			#u = array([v, w]).T

			u_new = array([[cos(self.x[2]), sin(self.x[2]), 0],[ 0, 0, 1]]).dot(u)
			u_new[1] = arctan2(u[1], u[0])
			u_new[1] = (u_new[1] + pi) % (2 * pi) - pi
			u = u_new
			rospy.loginfo("Old_Vel: "+str(array([v,w]).T))				
			rospy.loginfo("New_Vel: "+str(u))
					
			#eR2R1 = x2 - x1
			#eO1R1 = xobst1 - x1
			#eO2O1 = xobst2 - xobst1

			#PERV = array([ [ eO1R1[1],  -eO1R1[1] , 0 ] , [ -eO1R1[0] , eO1R1[0] , 0 ] ])
			#c = PERV.dot(eO2O1)		
			#i = argmin(c)
			#pervmin = PERV[:,i]
			
			#rR2R1 = (linalg.norm(eO2O1) / linalg.norm(eO1R1)) * pervmin + array([eR2R1[0],eR2R1[1]]).T
			
			#u = array([[cos(self.x[2]),sin(self.x[2])],[-sin(self.x[2])/d,cos(self.x[2])/d]]).dot(rR2R1)
			#return max(-0.5, min(sign(v)*v/2, 0.5)), max(-1.3, min(u[1], 1.3))	
		
			return max(-0.5, min(u[0], 0.5)), max(-1.3, min(u[1], 1.3))

		return v, w

	def satmin(self,val,valmin):
		if(val < valmin and val > -valmin):
			return 0
		else:
			return val
	
	def goal(self,pose):
		self.xr = array([pose.x,pose.y]).T
		self.xr_theta = pose.theta

	def relative_pose(self,pose):
		self.dl = array([pose.x,pose.y]).T
		if self.offset_updated == 0:
			self.offset_updated = 1
			self.xl = array([self.x[0] - self.dl[0], self.x[1] - self.dl[1]]).T
			self.al = 0
	
	def ground(self, pose):
		self.x = array([pose.x, pose.y, pose.theta]).T
		if self.start == False:
			self.xl = array([pose.x, pose.y]).T
			self.al = pose.theta
			self.xr = self.xl
			self.xr_theta = self.al
		self.start = True
	
	def listen_pose(self,pose,node):
		nodeIndex = node-1
		self.xobst[nodeIndex] = (pose.x,pose.y, pose.theta)
		if self.pos_updated[nodeIndex] == 0:
			self.pos_updated[nodeIndex] = 1
	
	def listen_vel(self,twist,node):
		nodeIndex = node-1
		self.xobst_vel[nodeIndex] = (twist.linear.x, twist.angular.z)
		if self.vel_updated[nodeIndex] == 0:
			self.vel_updated[nodeIndex] = 1

if __name__ == '__main__':
    try:
        c = Controller()
    except rospy.ROSInterruptException: 
		pass
