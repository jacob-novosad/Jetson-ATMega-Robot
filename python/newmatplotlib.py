import matplotlib.pyplot as plt





graph_data = open('newSampleText.txt','r').read()
lines = graph_data.split('\n')
xs = []
ys = []
for line in lines:
	if len(line) > 1:
		x,y=line.split(',')
		xs.append(x)
		ys.append(y)

plt.plot(xs,ys, label='Loaded from file!')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Interesting Graph\nCheck it out')
plt.legend()
plt.show()
