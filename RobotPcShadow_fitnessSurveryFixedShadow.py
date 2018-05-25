import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as la
import random
import robotPC_fitnessSurvery_fixedShadow as rob
from pprint import pprint
from datetime import date as dt
import os.path as op
import os
from math import *
import sys

def create_genotype():

	genotype = []

	for  i in range(numNodes):
		if(i==0):
			genotype.append(random.uniform(0,0.1))
		elif(i==1):
			genotype.append(random.uniform(-0.1,0))
		else:
			genotype.append(random.uniform(0,0.1) * random.choice([-1,1]))

	for i in range(numEdges):
		genotype.append(random.uniform(1.4,1.6))

	for i in range(numEdges):
		genotype.append(random.uniform(0,1))

	return genotype

def fn_gauss(x, mu, sigma):
	return exp(-(x-mu)**2/(2*sigma**2))

def fn_s(x,agent_index):

	val = 0
	lamda = 2.0/13
	x_agent = rob_pos[agent_index]
	x_o_agent = rob_pos[abs(agent_index-1)]
	#------------------Fixed Peak Start---------------------------#

	valfixed=0
	mu = fixed_sensor_pos[abs(agent_index-1)]
	sig = 1.0/26
	valfixed = max(fn_gauss(x,mu,sig),fn_gauss(x,mu+1,sig),fn_gauss(x,mu-1,sig))

	#------------------Fixed Peak End-----------------------------#

	#------------------Agent Peak Start---------------------------#
	
	valagent=0
	mu = x_o_agent
	sig = 1.0/26
	valagent = max(fn_gauss(x,mu,sig),fn_gauss(x,mu+1,sig),fn_gauss(x,mu-1,sig))

	#------------------Agent Peak End-----------------------------#

	#-----------------Agent Shadow Peak Start---------------------#
	
	valshadow=0
	mu = (x_o_agent+agent_shadow_rel_pos[abs(agent_index-1)])%1
	sig = 1.0/26
	valshadow = max(fn_gauss(x,mu,sig),fn_gauss(x,mu+1,sig),fn_gauss(x,mu-1,sig))

	#-----------------Agent Shadow Peak End-----------------------#

	val = max(valagent,valfixed,valshadow)

	return val

def fitness(genotype):
	return rob.move_rob(adjMat,numNodes,numEdges,genotype[:numNodes],genotype[numNodes:numNodes+numEdges],genotype[numEdges+numNodes:],"")

numNodes = 3#int(raw_input("Input number of nodes:"))
numEdges = 0
adjMat = [[0]*numNodes for x in range(numNodes) ]

for i in range(numNodes):
	for j in range(numNodes):
		adjMat[i][j] = 0 if i==j else 1
		if(adjMat[i][j]==1):
			numEdges+=1

genotype = [0.07219304228491538, -0.071975366794119852, -0.013444000667569955, 1.5619422517732224, 1.5287782127679348, 1.6, 1.4070114051553197, 1.5816729116891357, 1.4, 0.8204346131400578, 1, 1, 1, 0.35573438030526583, 0.3557523518379714]
#value Folder 1

(mean_score,x_rob,nodes1,nodes2,t_rob,sensorvals,fixed_sensor_pos,agent_shadow_rel_pos,rob_distance) = fitness(genotype)

plt.figure("Test_"+sys.argv[1],figsize=(14.0, 7.0))
ax1=plt.subplot(511)
for i in range(numNodes):
	plt.plot(t_rob,[node[i] for node in nodes1],label="p"+str(i+1))
plt.legend(loc="upper right")

plt.subplot(512,sharex=ax1)
for i in range(numNodes):
	plt.plot(t_rob,[node[i] for node in nodes2],label="p"+str(i+1))
plt.legend(loc="upper right")

plt.subplot(513,sharex=ax1)
for i in range(numEdges):
	plt.plot(t_rob,[node[numNodes+i] for node in nodes1],label="t"+str(i+1))
plt.legend(loc="upper right")

plt.subplot(514,sharex=ax1)
for i in range(numEdges):
	plt.plot(t_rob,[node[numNodes+i] for node in nodes2],label="t"+str(i+1))
plt.legend(loc="upper right")

plt.subplot(515,sharex=ax1)
plt.scatter(t_rob,[x[0]%1 for x in x_rob],s=0.1,label="Robot 1 position")
plt.scatter(t_rob,[x[1]%1 for x in x_rob],s=0.1,label="Robot 2 position")
#plt.scatter(t_rob,[(x[0]+agent_shadow_rel_pos[0])%1 for x in x_rob],s=0.1,label="Robot 1 Shadow position")
#plt.scatter(t_rob,[(x[1]+agent_shadow_rel_pos[1])%1 for x in x_rob],s=0.1,label="Robot 2 Shadow position")
plt.axhline(y=fixed_sensor_pos[0], color='r', label="Distractor for r2")
plt.axhline(y=fixed_sensor_pos[1], color='g', label="Distractor for r1")
plt.legend(loc="upper right")


plt.figure("Test_RobMotion_"+sys.argv[1],figsize=(14.0, 7.0))
ax1=plt.subplot(311)
plt.scatter(t_rob,[x[0]%1 for x in x_rob],s=0.1,label="Robot 1 position")
plt.scatter(t_rob,[x[1]%1 for x in x_rob],s=0.1,label="Robot 2 position")
plt.scatter(t_rob,[(x[0]+agent_shadow_rel_pos[0])%1 for x in x_rob],s=0.1,label="Robot 1 Shadow position",color ='0.25')
plt.scatter(t_rob,[(x[1]+agent_shadow_rel_pos[1])%1 for x in x_rob],s=0.1,label="Robot 2 Shadow position",color = '0.50')
plt.axhline(y=fixed_sensor_pos[0], color='r', label="Distractor for r2")
plt.axhline(y=fixed_sensor_pos[1], color='g', label="Distractor for r1")
plt.legend(loc="upper right")

plt.subplot(312,sharex=ax1)
plt.scatter(t_rob,rob_distance,s=0.1,label="Robot Distance")

plt.subplot(313,sharex=ax1)
sensor1 = [vals[0] for vals in sensorvals]
sensor2 = [vals[1] for vals in sensorvals]

plt.scatter(t_rob,sensor1,s=0.1,label="sensor 1 vals")
plt.scatter(t_rob,sensor2,s=0.1,label="sensor 2 vals")

plt.show()
