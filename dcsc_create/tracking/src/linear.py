#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

class Controller:

	def __init__(self):

		self.xr = 1
		self.x = 0

		self.pub = rospy.Publisher('/create5/cmd_vel', Twist)
		self.sub = rospy.Subscriber('/create5/odom',Odometry,self.odom)
    		self.subg = rospy.Subscriber('/create5/goal',Pose2D,self.goal)
		rospy.init_node('robot_linear_control')
		self.rate = rospy.Rate(10)
		
		self.control()

	def control(self):

		while not rospy.is_shutdown():
			
			if abs(self.xr - self.x) > 0.1:
				u = (self.xr - self.x) * 0.1			
				if u > 0:
					u = max(0.1,min(u,0.2))
				else:
					u = min(-0.1,max(u,-0.2))
			else:
				u = 0

			twist = Twist()
			twist.linear.x = u; 

			rospy.loginfo("Moving the robot forward.")
			self.pub.publish(twist)
		
			self.rate.sleep()
	
	def goal(self,pose):
		self.xr = pose.x

	def odom(self,odom):
		self.x = odom.pose.pose.position.x

if __name__ == '__main__':
    try:
        c = Controller()
    except rospy.ROSInterruptException: pass
