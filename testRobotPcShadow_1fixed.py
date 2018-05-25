import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as la
import random
import robotPC_testShadow as rob
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
	mu = fixed_sensor_pos#[abs(agent_index-1)]
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

genotype = [0.013020932576043099, -0.045751410856785031, -0.001819715086840601, 1.4039646587649626, 1.4238199083297565, 1.5846514528104811, 1.5223922667326646, 1.4256967384388175, 1.48792933363907, 0.023246504939397727, 1, 1, 0.5736439899230569, 0.8286798002871504, 0.7301669112305669]
#[0.009622115824624789, -0.003513943711591333, -0.020010904274028508, 1.5965235427050117, 1.4787255486904849, 1.5888661762876015, 1.6, 1.5011997910005932, 1.5926924483335578, 0.8853305218563109, 0.2081689000070065, 0.8987042120980618, 0.8762087930730561, 1, 1]
#[0.0007094701604890785, -0.040653639820650025, 0.011027884121635618, 1.4621439859863454, 1.5011812195755612, 1.4210082490721514, 1.454987645223289, 1.5423834576443198, 1.4892671641060862, 0.3147931137671486, 0.36978068154416055, 0.7805087173340634, 1, 1, 0.7007966251370045]
#[0.04365323360174636, -0.006794664937386202, 0.017495481509219514, 1.4552699460898433, 1.5011812195755612, 1.4210082490721514, 1.4704385083084004, 1.4572391230141353, 1.57472329047496, 0.804800746523354, 0.4190573041308351, 0.11405524219961938, 0.4666133794780165, 0.3734122054556168, 0.9906162379709129]
#[0.0818170802451521, -0.00430991195695693, 0.08250625598571937,1.5225419826491993, 1.5799351735638107, 1.4714417186161908, 1.5377672875373072, 1.5576733411980659, 1.4608027213985197,0.12909335380343168, 0.7195274489466994, 0.804279822071339, 0.48384209108152143, 0.2939075323409486, 0.24534113406978375]


#Folder[0.0029789056462782059, -0.058198930649166113, 0.015193460317492594, 1.4621131498795943, 1.46579547868898, 1.5115519629570791, 1.4876903742974312, 1.5736921757428306, 1.4448122308178506, 0.5924515047516463, 0.2247123734843115, 1, 0.13215381771550772, 0.2250648495940878, 0.7327187510009169]
#Folder 8[0.05956485249602117, -0.020105125838286622, 0.041470127578087615,1.5799637665593009, 1.5353792237630686, 1.55273810001585, 1.532692964673815, 1.5496393419039651, 1.4665950226616908,0.017878585709140693, 0.8286210960888115, 0.5944626559114959, 1, 1, 0.4667086495730072]


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
plt.axhline(y=fixed_sensor_pos, color='r', label="Distractor")
#plt.axhline(y=fixed_sensor_pos[1], color='g', label="Distractor for r1")
plt.legend(loc="upper right")


plt.figure("Test_RobMotion_"+sys.argv[1],figsize=(14.0, 7.0))
ax1=plt.subplot(311)
plt.scatter(t_rob,[x[0]%1 for x in x_rob],s=0.1,label="Robot 1 position")
plt.scatter(t_rob,[x[1]%1 for x in x_rob],s=0.1,label="Robot 2 position")
plt.scatter(t_rob,[(x[0]+agent_shadow_rel_pos[0])%1 for x in x_rob],s=0.1,label="Robot 1 Shadow position",color ='0.25')
plt.scatter(t_rob,[(x[1]+agent_shadow_rel_pos[1])%1 for x in x_rob],s=0.1,label="Robot 2 Shadow position",color = '0.50')
plt.axhline(y=fixed_sensor_pos, color='r', label="Distractor")
plt.legend(loc="upper right")

plt.subplot(312,sharex=ax1)
plt.scatter(t_rob,rob_distance,s=0.1,label="Robot Distance")

plt.subplot(313,sharex=ax1)
sensor1 = [vals[0] for vals in sensorvals]
sensor2 = [vals[1] for vals in sensorvals]

plt.scatter(t_rob,sensor1,s=0.1,label="sensor 1 vals")
plt.scatter(t_rob,sensor2,s=0.1,label="sensor 2 vals")

plt.show()
