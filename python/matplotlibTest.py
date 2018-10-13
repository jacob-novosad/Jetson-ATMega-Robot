import matplotlib.pyplot as plt
import matplotlib.animation as animation



from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
axis1= fig.add_subplot(1,1,1)



def animate(i):
	graph_data = open('sampleText.txt','r').read()
	lines = graph_data.split('\n')
	xs = []
	ys = []
	for line in lines:
		if len(line) > 1:
			x,y=line.split(',')
			xs.append(x)
			ys.append(y)
	axis1.clear()	
	axis1.plot(ys,xs)

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.xticks(rotation=45, ha='right')
plt.subplots_adjust(bottom=0.30)
plt.ylabel('RPM')
plt.show()
