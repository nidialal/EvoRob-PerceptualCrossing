import numpy as np
import random
import robotPC as rob
from datetime import date as dt
import os.path as op
import os
import sys


REC_RATE = 0.5
MUT_RATE = 0.02
POP_SIZE = 30
DEME_SIZ = 5

def mutate(genotype):

	for  i in range(numNodes):
		if(random.random() < MUT_RATE):
			gene_new = np.random.normal(genotype[i],0.001,1)[0]
			if(gene_new<-0.1):
				gene_new = -0.1
			elif(gene_new>0.1):
				gene_new = 0.1
			genotype[i]=gene_new
			if(random.random()<0.05):
				genotype[i] = genotype[i] * -1
	for i in range(numEdges):
		if(random.random() < MUT_RATE):
			gene_new = np.random.normal(genotype[numNodes+i],0.01,1)[0]
			if(gene_new<1.4):
				gene_new = 1.4
			elif(gene_new>1.6):
				gene_new = 1.6
			genotype[numNodes+i]=gene_new
	for i in range(numEdges):
		if(random.random() < MUT_RATE):
			gene_new = np.random.normal(numEdges+numNodes+i,0.1,1)[0]
			if(gene_new<0):
				gene_new = 0
			elif(gene_new>1):
				gene_new = 1
			genotype[numEdges+numNodes+i]=gene_new
	return genotype


def recombine(genotypes): 
	'''
	**** Microbial Recombination
	**** First one is the winner and second is the loser
	'''
	for i in xrange(numNodes+(2*numEdges)):
		if(random.random() < REC_RATE):
			genotypes[1][i] = genotypes[0][i]
	return genotypes


def fitness(genotype):#adj_Mat,num_Nodes,num_Edges,a,b,c
	'''
	**** Robot Move
	'''
	return rob.move_rob(adjMat,numNodes,numEdges,genotype[:numNodes],genotype[numNodes:numNodes+numEdges],genotype[numEdges+numNodes:],fileName+".txt",folderName)

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

def tournament():
	index1 = int(random.random()*POP_SIZE)
	index2 = int(index1+(random.randint(1,DEME_SIZ) * random.choice([-1,1])))%POP_SIZE
	A = population[index1]
	B = population[index2]
	loser_index = 0
	winner_index = 0
	if (pop_fitness[index1]<pop_fitness[index2]):
		W = A
		L = B
		loser_index = index2
		winner_index = index1
	else:
		W = B
		L = A
		loser_index = index1
		winner_index = index2
	[W,L]=recombine([W,L])
	L=mutate(L)
	population[loser_index]=L
	return (loser_index,winner_index)

numNodes = 3#int(raw_input("Input number of nodes:"))
numEdges = 0
adjMat = [[0]*numNodes for x in range(numNodes) ]

for i in range(numNodes):
	for j in range(numNodes):
		adjMat[i][j] = 0 if i==j else 1
		if(adjMat[i][j]==1):
			numEdges+=1

population = [create_genotype() for i in range(POP_SIZE)]

counter=0
mean_fitness = []
peak_fitness = []
num = sys.argv[1]

folderName = "PC_"+str(num) +"_"+ str(dt.today())
fileName = num

if not op.exists(folderName):
	os.makedirs(folderName)

folderName += "/"

pop_fitness = map(fitness,population)

file_popfit = open(folderName+"PopFitness"+fileName+".txt",'a+')
for fit_score in pop_fitness:
	file_popfit.write(str(fit_score)+",")
file_popfit.write("\n\n")
file_popfit.close()

while (counter<6000):

	(loser_index,winner_index) = tournament()
	pop_fitness[loser_index] = fitness(population[loser_index])

	if(random.random()<0.1):
		pop_fitness[winner_index] = fitness(population[winner_index])

	file_popfit = open(folderName+"PopFitness"+fileName+".txt",'a+')
	for fit_score in pop_fitness:
		file_popfit.write(str(fit_score)+",")
	file_popfit.write("\n\n")
	file_popfit.close()

	file_m = open(folderName+"MeanFitness_"+fileName+".txt",'a+')
	mean_score = np.mean(pop_fitness)
	file_m.write(str(mean_score)+",")
	file_m.close()

	file_p = open(folderName+"PeakFitness_"+fileName+".txt",'a+')
	peak_score=np.min(pop_fitness)
	file_p.write(str(peak_score)+",")
	file_p.close()

	counter+=1


file = open(folderName+"FinalPop"+fileName+".txt",'a+')
for i,individual in enumerate(population):
	file.write("Score=" + str(pop_fitness[i])+'\n')
	file.write(str(individual)+'\n')
	file.write('\n')

file.write('\n\n\nFinal Population\n')
for individual in population:
	file.write(str(individual)+'\n')

file.close()