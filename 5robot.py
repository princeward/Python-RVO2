#!/usr/bin/env python

import rvo2
import matplotlib.pyplot as plt
import numpy as np

def get_des_vel(cur_pos, des_pos):
	'''
	get the desired velocity (normalized)
	'''
	vel = [0.0, 0.0]
	vel[0] = des_pos[0] - cur_pos[0]
	vel[1] = des_pos[1] - cur_pos[1]
	norm = np.sqrt(vel[0]**2 + vel[1]**2)
	if norm > 1:
		vel[0] /= norm
		vel[1] /= norm
	return (vel[0], vel[1])

sim = rvo2.PyRVOSimulator(timeStep = 0.5, neighborDist = 20, maxNeighbors = 5, timeHorizon = 1.5, timeHorizonObst = 2, radius = 5, maxSpeed = 20)

N_Agent = 5
init_pos = np.array([[50.0, 80.0, 50.0, 20.0, 10.0],
					 [10.0, 50.0, 90.0, 75.0, 30.0]])
goal_pos = np.array([[50.0, 20.0, 10.0, 50.0, 80.0],
					 [90.0, 70.0, 30.0, 10.0, 50.0]])
assert(N_Agent == init_pos.shape[1])

agents = [sim.addAgent((init_pos[0][i], init_pos[1][i])) for i in range(N_Agent)]



print('Simulation has %i agents and %i obstacle vertices in it.' %
	  (sim.getNumAgents(), sim.getNumObstacleVertices()))

print('Running simulation')

for step in range(500):
	print step
	# set preferred velocity
	for i, a in enumerate(agents):
		pos = sim.getAgentPosition(a)
		des_vel = get_des_vel(pos, goal_pos[:,i])
		sim.setAgentPrefVelocity(a, des_vel)

	sim.doStep()

	plt.clf()
	for a in agents:
		pos = sim.getAgentPosition(a)
		plt.plot(pos[0], pos[1],'o', markersize = 10)

	plt.axis([0,100,0,100])
	plt.draw()
	plt.pause(0.01)

