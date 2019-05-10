import matplotlib.pyplot as plt
import csv

x = [0,1]
y = [0,0]

with open('../talonPID_data.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))

#plt.plot(x,y, label='Loaded from file!')
plt.scatter(x, y)
plt.ylabel('y')
plt.title('Interesting Graph\nCheck it out')
plt.legend()
#plt.show()
while(True):
	with open('../talonPID_data.txt','r') as csvfile:
	    plots = csv.reader(csvfile, delimiter=' ')
	    for row in plots:
		x.append(float(row[0]))
		y.append(float(row[1]))
        plt.cla()
	plt.scatter(x, y)
	axes = plt.gca()
        axes.set_xlim(x[-1]-10, x[-1]+10)
	plt.xlabel('x')
	plt.ylabel('y')
	plt.title('Interesting Graph\nCheck it out')
    	plt.pause(0.05)


