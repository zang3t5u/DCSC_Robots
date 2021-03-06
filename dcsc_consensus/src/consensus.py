#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import numpy as np

class Consensus:

	def __init__(self):

		rospy.init_node('consensus_node')

		x = rospy.get_param('~x',0)
		y = rospy.get_param('~y',0)
		theta = rospy.get_param('~theta',0)

		self.state = np.array([x,y,theta])
		self.states = np.array([[x,y,theta]])

		self.rate = rospy.Rate(1)

		self.pub = rospy.Publisher('consensus',Pose2D, queue_size=1)
		self.connected_to = rospy.get_param('~connected_to',[])
		self.sub = []
		for node in self.connected_to:
			self.sub.append(rospy.Subscriber('/node'+str(node)+'/consensus',Pose2D,self.listen))
			rospy.loginfo("Subscribed to node " + str(node))
		
		rospy.loginfo("Node initialized.")

		self.talk()

	def talk(self):

		while not rospy.is_shutdown():
			
			#Update the state
			self.state = np.mean(self.states,axis=0)			
			self.states = np.array([self.state])

			#Define the message
			pose = Pose2D()
			pose.x = self.state[0]
			pose.y = self.state[1]
			pose.theta = self.state[2]

			rospy.loginfo(pose)

			#Publish
			self.pub.publish(pose)		
			self.rate.sleep()

	def listen(self,message):
		self.states = np.append(self.states,np.array([[message.x,message.y,message.theta]]),axis=0)		

if __name__ == '__main__':
    try:
        c = Consensus()
    except rospy.ROSInterruptException: pass
