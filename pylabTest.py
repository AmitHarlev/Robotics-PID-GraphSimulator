import pylab as plt
import numpy as np
import keyboard
import random

Y = np.zeros(1000)

Y[998] = 1.0
Y[999] = 0.5


plt.ion()
graph = plt.plot(Y)[0]

while not keyboard.is_pressed('enter'):

    for i in range(1000):
        if(i != 999):
            Y[i] = Y[i+1]
        else:
            Y[999] = Y[999]+random.uniform(-0.01,0.01)
        

    graph.set_ydata(Y)
    plt.draw()
    plt.pause(0.01)