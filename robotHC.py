import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import ode
from scipy.integrate import odeint
from numpy import linalg as la
from math import log
import random


numNodes = 0
numEdges = 0
adjMat = []
edgeList=[]

t_rob = []
nodes = []

t1 = 200
inits = []

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

rob_pos = [0,0] #[top,bottom]
fixed_sensor_pos = [0,0] #[top,bottom]

def sum_square(list):
	return sum(map(lambda x: x*x, list))

def sum_Exp4(list):
	return sum(map(lambda x: x*x*x*x, list))

def g(y_k,l):
	return -y_k * (np.square(np.square(y_k)-1) + l)


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

	val = max(valagent,valfixed)

	return val

def fn_s_sine(x,agent_index):

	val = 0
	lamda = 2.0/13
	x_agent = rob_pos[agent_index]
	x_o_agent = rob_pos[abs(agent_index-1)]
	#------------------Fixed Peak Start---------------------------#

	xfixed = fixed_sensor_pos[abs(agent_index-1)]
	xmin = x_o_agent - lamda/2
	xmax = x_o_agent + lamda/2
	if(x>xmin and x<xmax):
		valfixed = np.sin((min(x-xfixed,xfixed-x)+0.5/13)*np.pi*13)+1
		valfixed= valfixed/2

	#------------------Fixed Peak End-----------------------------#

	#------------------Agent Peak Start---------------------------#

	valagent=0
	xmin = x_o_agent - lamda/2
	xmax = x_o_agent + lamda/2
	if(xmin>=0 and xmax<=1 and x>xmin and x<xmax):
		valagent = np.sin((min(x-x_o_agent,x_o_agent-x)+0.5/13)*np.pi*13)+1
		valagent /= 2
	elif(xmin<0 and not(x>xmax and x<1+xmin)):
		valagent = np.sin((min(abs(x-x_o_agent),1-abs(x-x_o_agent))+0.5/13)*np.pi*13)+1
		valagent /= 2
	elif(xmax>1 and not(x>xmax-1 and x<xmin)):
		valagent = np.sin((min(abs(x-x_o_agent),1-abs(x-x_o_agent))+0.5/13)*np.pi*13)+1
		valagent /= 2	

	#------------------Agent Peak End-----------------------------#

	val = max(valagent,valfixed)

	return val

def model(inits,t):

	ret_vals =[]

	for agent_index in range(2):
		init = inits[agent_index]
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
				A - (beta[i] * np.square(p_vals[edge[0]])) + (C * (y_squared - np.square(y_vals[i])))) + (gamma[i]*fn_s(x_val%1,agent_index))
			p_y_x_dot.append(ydot)

		x_dot = 0
		for i in range(numNodes):
			x_dot += alpha[i] * p_vals[i]
		rob_pos[agent_index] = x_val + x_dot

		p_y_x_dot.append(x_dot)
		ret_vals.append(p_y_x_dot)

	return ret_vals

def move_rob(adj_Mat,num_Nodes,num_Edges,a,b,c,fileName,test=0):
	
	global adjMat
	global numNodes
	global numEdges
	global alpha
	global beta
	global gamma
	global inits
	global score
	global edgeList
	global t1

	adjMat = adj_Mat
	numNodes = num_Nodes
	numEdges = num_Edges
	alpha = a
	beta = b
	gamma = c

	count = 0
	score = []

	while count < 10:

		inits = []
		edgeList = []
		t_rob = []
		nodes = []
		near_axis = random.choice(range(numNodes))
		for agent in range(2):
			init = []
			for i in range(numNodes):
				if(i==near_axis):
					init.append(random.uniform(0.89,0.99))
				else:
					init.append(random.uniform(0.01,0.11))

			for i in range(numNodes):
				for j in range(numNodes):
					if(adjMat[i][j]==1):
						if(agent==0):
							edgeList.append((i,j))
						init.append(random.uniform(0.01,0.11))
			init.append(random.random())
			inits.append(init)
		
		t1=int(random.random()*200 + 200)

		x_rob = []
		x_cur = []
		'''
		r = ode(model).set_integrator('vode',rtol=10e-3,atol=10e-6)
		r.set_initial_value(inits)
		dt = 0.05
		while r.successful() and r.t<t1:
			x_cur = [r.y[0][-1]%1,r.y[1][-1]%1]
			x_rob.append(x_cur)
			nodes1.append(r.y[0][0:-1])
			nodes2.append(r.y[1][0:-1])
			t_rob.append(r.t)
			r.integrate(t1)
			print x_cur
		'''
		t_rob = np.linspace(0,t1,t1*1000)
		sols = odeint(model,init,t_rob)

		x_rob = []

		for sol in sols:
			x_rob.append([sol[0][-1]%1,sol[1][-1]%1])
			nodes1.append(sol[0][:-1])
			nodes2.append(sol[1][:-1])
 
		raw_input()

		rob_dist = [min(abs(rob_pos[0]-rob_pos[1]),1-abs(rob_pos[0]-rob_pos[1])) for rob_pos in x_rob[int(0.8*len(x_rob)):]]

		score_val = np.max(rob_dist)

		score.append(score_val)
		count +=1
	
	mean_score = np.mean(score)
	print "Mean of max distance:" + str(mean_score)
	if(mean_score<0.1):
		file = open("value"+fileName+".txt",'a+')
		file.write("Score=" + str(mean_score)+'\n')
		file.write("alpha=" + str(alpha)+'\n')
		file.write("beta=" + str(beta)+'\n')
		file.write("gamma=" + str(gamma)+'\n')
		file.write('\n')
		file.close()
	return mean_score
