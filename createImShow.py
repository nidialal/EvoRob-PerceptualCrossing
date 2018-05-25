import numpy as np
import matplotlib.pyplot as plt
import sys

fName = sys.argv[1]
if(len(sys.argv)==1):
	print "Please provide input filename"
	sys.exit()
fitFile = open(fName,"r")

lines = fitFile.readlines()

fitFile.close()

fitArr = []

for line in lines:
	if(line=="\n"):
		continue
	fitArr.append([float(num) for num in line.split(",")[:-1]])

plt.figure()
imgplot=plt.imshow(np.array(fitArr),aspect='auto')
imgplot.set_cmap('viridis')
plt.colorbar()
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)
plt.savefig("PopFitness_PCShadow.pdf", bbox_inches='tight', pad_inches=0)

