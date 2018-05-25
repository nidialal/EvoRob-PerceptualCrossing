import numpy as np
import matplotlib.pyplot as plt

x1 = np.linspace(0,1,1000)

for xother in [0,0.001,0.003,0.005,0.007,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,0.95,0.98,0.99,0.995,0.997]:	
	y = []

	for x in x1:

		val = 0
		lamda = 2.0/13

		#------------------Fixed Peak Start---------------------------#
		valfixed=0
		xfixed = 0.25
		xmin = xfixed - lamda/2
		xmax = xfixed + lamda/2
		if(x>xmin and x<xmax):
			valfixed = np.sin((min(x-xfixed,xfixed-x)+0.5/13)*np.pi*13)+1
			valfixed= valfixed/2
	
		#------------------Fixed Peak End-----------------------------#

		#------------------Agent Peak Start---------------------------#
		valagent=0
		xmin = xother - lamda/2
		xmax = xother + lamda/2
		if(xmin>=0 and xmax<=1 and x>xmin and x<xmax):
			valagent = np.sin((min(x-xother,xother-x)+0.5/13)*np.pi*13)+1
			valagent /= 2
		elif(xmin<0 and not(x>xmax and x<1+xmin)):
			valagent = np.sin((min(abs(x-xother),1-abs(x-xother))+0.5/13)*np.pi*13)+1
			valagent /= 2
		elif(xmax>1 and not(x>xmax-1 and x<xmin)):
			valagent = np.sin((min(abs(x-xother),1-abs(x-xother))+0.5/13)*np.pi*13)+1
			valagent /= 2	
		#------------------Agent Peak End-----------------------------#
		val = max(valagent,valfixed)
		y.append(val)

	plt.plot(x1,y)
	plt.show()