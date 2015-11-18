from numpy.matlib import *
from numpy.linalg import *

import numpy as np

import time

class Leader:

	def __init__(self, kp, kd, dt, state = array([[0], [0], [0], [0]])):
		self.A = array([[-kp+kd/dt, -kd/dt, 0, 0], [1, -1, 0, 0], [0, 0, -kp+kd/dt, -kd/dt], [0, 0, 1, -1]])
		self.B = array([[kp, 0], [0, 0], [0, kp], [0, 0]])
		self.state = state
		self.dt = dt
		self.kp = kp
		self.kd = kd
		self.sat = array([[1,float('infinity'),1,float('infinity')]]).T

	def step(self,u):
		delta = self.A.dot(self.state) + self.B.dot(u)
		self.state = self.state + self.dt * minimum ( self.sat , maximum(  delta , -self.sat ) )

	def stability(self):
		ev = eig(self.A)
		aminim = array([])
		amaxim = [-float('infinity')]
		for i in range(len(ev)):
			aminim = np.append(aminim,amin(ev[i]))
			amaxim = np.append(amaxim,amax(ev[i]))
		print min(aminim)
		print max(amaxim)

if __name__ == "__main__":

	l = Leader(1,0.05,0.1, array([[0, 0, 0, 0]]).T)
	l.stability()
	
	while True:

		l.step(array([[1,1]]).T)
		print l.state.T
		time.sleep(l.dt)

