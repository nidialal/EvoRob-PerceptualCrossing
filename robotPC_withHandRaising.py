import numpy as np
from scipy.integrate import odeint
from math import log
import random
from math import *


numNodes = 0
numEdges = 0
adjMat = []
edgeList=[]

t_rob = []


t1 = 200

A = 0.5
C = 2
D = 10
E = 4
F = 2
Z = 2

alpha = []
beta = []
gamma = []
delta = []

fixed_sensor_pos = [random.random(),random.random()] #[top,bottom]

shadow_direction = random.choice([1,-1])
agent_shadow_rel_pos = [shadow_direction*random.random()*0.5,shadow_direction*random.random()*0.5]

def sum_square(list):
	return sum(map(lambda x: x*x, list))

def sum_Exp4(list):
	return sum(map(lambda x: x*x*x*x, list))

def g(y_k,l):
	return -y_k * (np.square(np.square(y_k)-1) + l)

def fn_gauss(x, mu, sigma):
	return exp(-(x-mu)**2/(2*sigma**2))

def fn_s(x,agent_index,rob_pos):

	val = 0
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

def model(inits,t):

	p_y_x_dot = []

	rob_pos = [inits[len(inits)/2-1],inits[len(inits)-1]]

	for agent_index in range(2):

		init = inits[agent_index*(numNodes+numEdges+1):(agent_index+1)*(numNodes+numEdges+1)]

		p_vals = init[0:numNodes]
		y_vals = init[numNodes:-1]
		x_val = init[-1]

		p_squared = sum_square(p_vals)
		p_Exp4 = sum_Exp4(p_vals)
		y_squared = sum_square(y_vals)
		

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
				A - (beta[i] * np.square(p_vals[edge[0]])) + (C * (y_squared - np.square(y_vals[i])))) + (gamma[i]*fn_s(x_val%1,agent_index,rob_pos))
			p_y_x_dot.append(ydot)

		x_dot = 0
		for i in range(numNodes):
			x_dot += alpha[i] * p_vals[i]

		p_y_x_dot.append(x_dot)


	return p_y_x_dot

def move_rob(adj_Mat,num_Nodes,num_Edges,a,b,c,d,fileName,folder):
	
	global adjMat
	global numNodes
	global numEdges
	global alpha
	global beta
	global gamma
	global delta
	global edgeList
	global t1

	adjMat = adj_Mat
	numNodes = num_Nodes
	numEdges = num_Edges
	alpha = a
	beta = b
	gamma = c
	delta = d

	count = 0
	score = []

	while count < 20:

		init = []
		edgeList = []
		t_rob = []


		for agent_index in range(2):
			near_axis = random.choice(range(numNodes))
			for i in range(numNodes):
				if(i==near_axis):
					init.append(random.uniform(0.89,0.99))
				else:
					init.append(random.uniform(0.01,0.11))

			for i in range(numNodes):
				for j in range(numNodes):
					if(adjMat[i][j]==1):
						if(agent_index==0):
							edgeList.append((i,j))
						init.append(random.uniform(0.01,0.11))
			init.append(random.random())

		t1=int(random.random()*200 + 200)

		x_rob = []
		x_cur = []

		t_rob = np.linspace(0,t1,t1*1000)
		sols = odeint(model,init,t_rob)

		x_rob = []

		for sol in sols:

			hand1 = 0
			hand2 = 0

			rob1_pos = sol[9]%1
			rob2_pos = sol[len(sol)-1]%1
			rob_dist = 1 if min(abs(rob1_pos-rob2_pos),1-abs(rob1_pos-rob2_pos)) < 0.1 else 0

			for node_index in range(numNodes):
				hand1 += delta[node_index]*sol[node_index]
				hand2 += delta[node_index]*sol[(len(sol)/2)+node_index]

			score1 = Z * (0.5 - abs(hand1-rob_dist)) + (1-Z) * abs(hand1-rob_dist)
			score2 = Z * (0.5 - abs(hand2-rob_dist)) + (1-Z) * abs(hand2-rob_dist)
			score_val = np.mean([score1,score2])
 
		sol = None


		score.append(score_val)
		count +=1

	mean_score = np.mean(score)


	if(mean_score>0.8):
		file = open(folder+"value"+fileName,'a+')
		file.write("Score=" + str(mean_score)+'\n')
		file.write("alpha=" + str(alpha)+'\n')
		file.write("beta=" + str(beta)+'\n')
		file.write("gamma=" + str(gamma)+'\n')
		file.write("delta=" + str(delta)+'\n')
		file.write('\n')
		file.close()

	print mean_score
	return mean_score
