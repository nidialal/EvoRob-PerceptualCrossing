import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import ode
from scipy.integrate import odeint
from numpy import linalg as la
from math import log
import random
from multiprocessing import Pool
import traceback,sys

numNodes = 0
numEdges = 0
adjMat = []
edgeList=[]

t_rob = []
nodes = []

t1 = 200
init = []

A = 0.5
C = 2
D = 10
E = 4
F = 2

alpha = []
beta = []
gamma = []

score = []

xdiff = 0
np_pos = 0

def sum_square(list):
	return sum(map(lambda x: x*x, list))

def sum_Exp4(list):
	return sum(map(lambda x: x*x*x*x, list))

def g(y_k,l):
	return -y_k * (np.square(np.square(y_k)-1) + l)


def fn_s(x):
	val = 0

	xmin = 0
	xmax = 2.0/9
	if(x>xmin and x<xmax):
		val = np.sin((x+1)*np.pi*9/2)+1
		val/=2

	xmin = 5.5/13
	xmax = 7.5/13
	if(x>xmin and x<xmax):
		val = np.sin(x*np.pi*13)+1
		val/=2

	xmin = 7.0/9
	xmax = 1
	if(x>xmin and x<xmax):
		val = np.sin(x*np.pi*9/2)+1
		val/=2
		
	return val

def fn_s(x,xdiff):

	val = 0
	xmin = 0
	xmax = (1.0/9) + xdiff
	if(x>xmin and x<xmax):
		val = np.sin((x+1-xdiff)*np.pi*9/2)
		val = 0 if val < 0 else val
		return val

	xmin = 5.5/13
	xmax = 7.5/13
	if(x>xmin and x<xmax):
		val = np.sin(x*np.pi*13)+1
		return val/2

	xmin = (8.0/9) +xdiff
	xmax = 1
	if(x>xmin and x<xmax):
		val = np.sin((x-xdiff)*np.pi*9/2)
		val = 0 if val < 0 else val
		return val

	return val

def model(t,init):
	
	p_vals = init[0:numNodes]
	y_vals = init[numNodes:-1]
	x_val = init[-1]
	p_squared = sum_square(p_vals)
	p_Exp4 = sum_Exp4(p_vals)
	y_squared = sum_square(y_vals)
	p_y_x_dot = []
	for i in range(numNodes):
		pdot = (float)(p_vals[i] * ( F * (1 - p_squared) + D * (( np.square(p_vals[i]) * p_squared ) - p_Exp4)))
		row = adjMat[i]
		col = [r[i] for r in adjMat]
		Eterm = 0
		for j,val in enumerate(row):
			if(val>0):
				Eterm -= p_vals[i] * p_vals[j] * np.square(y_vals[edgeList.index((i,j))])
		for j,val in enumerate(col):
			if(val>0):
				Eterm += np.square(p_vals[j]) * np.square(y_vals[edgeList.index((j,i))])
		pdot += E * Eterm
		p_y_x_dot.append(pdot)
	
	for i,edge in enumerate(edgeList):
		ydot = g(y_vals[i], 
			A - (beta[i] * np.square(p_vals[edge[0]])) + (C * (y_squared - np.square(y_vals[i])))) + (gamma[i]*fn_s(x_val%1,xdiff))
		p_y_x_dot.append(ydot)

	x_dot = 0
	for i in range(numNodes):
		x_dot += alpha[i] * p_vals[i]

	p_y_x_dot.append(x_dot)

	return p_y_x_dot

def getScore(tName):

	global score
	global edgeList
	global xdiff
	global t1

	init = []
	edgeList = []
	near_axis = random.choice(range(numNodes))
	
	for i in range(numNodes):
		if(i==near_axis):
			init.append(random.uniform(0.89,0.99))
		else:
			init.append(random.uniform(0.01,0.11))

	for i in range(numNodes):
		for j in range(numNodes):
			if(adjMat[i][j]==1):
				edgeList.append((i,j))
				init.append(random.uniform(0.01,0.11))
	init.append(random.random())

	xdiff = random.uniform(-0.2,0.2)
	t1=int(random.random()*200 + 200)
	x_target = 0.5

	x_rob = []
	x_cur = 0
	r = ode(model).set_integrator('vode',rtol=10e-4,atol=10e-6,nsteps=5000)
	r.set_initial_value(init)
	dt = 0.05
	try:
		while r.successful() and r.t<t1:
			x_cur = r.y[-1]
			x_rob.append(x_cur%1)
			r.integrate(t1,step=True)
	except:
		traceback.print_exc()
		print sys.exc_info()[0]

	rob_dist = [abs(pos-x_target) for pos in x_rob[int(0.8*len(x_rob)):]]
	score_val = np.max(rob_dist)

	return score_val

def move_rob(adj_Mat,num_Nodes,num_Edges,a,b,c,fileName,folderName,test=0):
	
	global adjMat
	global numNodes
	global numEdges
	global alpha
	global beta
	global gamma

	adjMat = adj_Mat
	numNodes = num_Nodes
	numEdges = num_Edges
	alpha = a
	beta = b
	gamma = c

	count = 0
	score = []

	p = Pool(10)
	score = p.map(getScore,[str(i) for i in range(1,11)])
	p.close()
	mean_score = np.mean(score)
	if(mean_score<0.4999):
		file = open(folderName+"value"+fileName,'a+')
		file.write("Score=" + str(mean_score)+'\n')
		file.write("alpha=" + str(alpha)+'\n')
		file.write("beta=" + str(beta)+'\n')
		file.write("gamma=" + str(gamma)+'\n')
		file.write('\n')
		file.close()
	return mean_score
