from numpy.matlib import *
from numpy.linalg import *

import numpy as np

import time

class UniCycle:

	def __init__(self, k_w, gamma_v, h_theta, dt, state = array([[0], [0], [0]])):
		self.k_w = k_w
		self.gamma_v = gamma_v
		self.h_theta = h_theta
		self.dt = dt
		self.e = 0 
		self.e_old = 0 
		self.alpha = 0 
		self.alpha_old = 0 
		self.theta = 0 
		self.theta_old = 0
		self.state = state
		self.delta = pi/6
		print self.state

	def step(self,goal):
		print goal
		self.phi = self.state[2] - goal[2]
		self.phi = (self.phi + pi) % (2 * pi) - pi	
		
		self.e0 = sqrt((goal[0] - self.state[0])**2 + (goal[1] - self.state[1])**2)
		if self.e0 < 0.1 and self.phi<0.18:
			return
		self.theta0 = arctan2(goal[1] - self.state[1], goal[0] - self.state[0]) - goal[2]
		self.theta0 = (self.theta0 + pi) % (2 * pi) - pi
		#if(self.theta0>)
		print "Theta: "+str(self.theta0)
		self.alpha0 = self.theta0 - (self.phi)
		self.alpha0 = (self.alpha0 + pi) % (2 * pi) - pi
		print "Alpha: "+str(self.alpha0)

		if self.e0 != 0:
			v = self.gamma_v*cos(self.alpha0)*self.e0
		else:
			v = 0
		if self.alpha0 != 0:
			w = (self.k_w*self.alpha0 + self.gamma_v*sin(2*self.alpha0)*(self.alpha0 + self.h_theta*self.theta0)/(2*self.alpha0))
		else:
			w = self.gamma_v*self.h_theta*self.theta0
		v = min(max(-1, v), 1)
		w = min(max(-1, w), 1)
		self.e = self.e_old + self.dt * (-v*(cos(self.alpha0)))
		if self.e0 != 0:
			self.alpha = self.alpha_old + self.dt * (-w + v*(sin(self.alpha0))/self.e0)
			self.theta = self.theta_old + self.dt * (v*sin(self.alpha0)/self.e0)
		self.state = self.state + self.dt * array([v*cos(self.phi), v*sin(self.phi), w]).T
		self.state[2] = (self.state[2] + pi) % (2 * pi) - pi
		print "V: "+str(v)+" w:"+str(w)

if __name__ == "__main__":
	mul = 0.1
	k_w = 2*mul
	g_v = 1*mul
	h_t = 2*mul
	dt = 0.1
	l = UniCycle(k_w, g_v, h_t, dt, array([-0.034, -0.4658, -0.100]).T)

	while True:
		l.step(array([-0.6, 1.3, 0]).T)
		print l.state.T
		time.sleep(l.dt)

