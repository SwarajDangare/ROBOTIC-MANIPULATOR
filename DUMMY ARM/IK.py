from cmath import acos, atan
import math
import itertools
from matplotlib import pyplot as plt
from matplotlib import animation as animation
import numpy as np
from numpy import pi, sin, cos, sqrt, radians

X=[2,2.001,4,4.001,2]
Y=[2,4,4.001,2,2.001]
# iniX = 2
# iniY = 2
# finX = 2
# finY = 2.001
corrX=[]
corrY=[]
x = 0
y = 0

l1 = 4
l2 = 2

for i in range(len(X)-1):
    corr_x = np.arange(X[i], X[i+1] + (X[i+1] - X[i]) / 100, (X[i+1] - X[i]) / 100)
    corr_y = np.arange(Y[i], Y[i+1] + (Y[i+1] - Y[i]) / 100, (Y[i+1] - Y[i]) / 100)
    
    for j in range(len(corr_x)-1):
        corrX.append(corr_x[j])
        corrY.append(corr_y[j])

print (corrX , corrY)


# corrX2 = np.arange(X2, X3 + (X3 - X2) / 100, (X3 - X2) / 100)
# corrY2 = np.arange(Y2, Y3 + (Y3 - Y2) / 100, (Y3 - Y2) / 100)

# corrX3 = np.arange(X3, X4 + (X4 - X3) / 100, (X4 - X3) / 100)
# corrY3 = np.arange(Y3, Y4 + (Y4 - Y3) / 100, (Y4 - Y3) / 100)

# corrX4 = np.arange(X4, finX + (finX - X4) / 100, (finX - X4) / 100)
# corrY4 = np.arange(Y4, finY + (finY - Y4) / 100, (finY - Y4) / 100)

# corrX = [list(corrX1),list(corrX2),list(corrX3),list(corrX4)]
# corrY = [list(corrY1),list(corrY2),list(corrY3),list(corrY4)]

# print(corrX)
# print(corrY)

T1 = []
T2 = []
iniX = []
iniY = []

t2 = []

i=0

for x, y in zip(corrX, corrY):

    t2 = acos((x**2 + y**2 - (l1**2 + l2**2)) / (2 * l1 * l2))
    T2.append(t2)


for x, y, t2 in zip(corrX, corrY, T2):
    b = atan((l2 * sin(t2)) / (l2 * cos(t2) + l1))
    a = atan(y / x)
    t1 = a + b
    T1.append(t1)
        


print(T1, T2)

for t1 in T1:

    x1 = l1 * cos(t1)
    y1 = l1 * sin(t1)

    iniX.append(x1)
    iniY.append(y1)

# set up the figure and subplot
fig = plt.figure()
ax = fig.add_subplot(
    111, aspect="equal", autoscale_on=False, xlim=(-7, 7), ylim=(-2, 7)
)
ax.grid()
ax.set_title("2R planer Robotic Manipulater")
ax.axes.xaxis.set_ticklabels([])
ax.axes.yaxis.set_ticklabels([])
(line,) = ax.plot([], [], "o-", lw=2, color="#de2d26")

# initialization function
def init():
    line.set_data([], [])
    return (line,)


# animation function
def animate(i):
    x_points = [0, iniX[i], corrX[i]]
    y_points = [0, iniY[i], corrY[i]]

    line.set_data(x_points, y_points)
    return (line,)


# call the animation
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(T1), interval=20, blit=True, repeat=False
)

plt.show()
print("Done")
