import matplotlib.pyplot as plt
import numpy as np


plt.axis([0,60,0,4])

i = 0

while True:

	i = i+1
	y=np.random.random()
	plt.scatter(i,y)
	plt.pause(.05)
	if i > 60:
	plt.axis([0,i,0,4])


plt.show()

