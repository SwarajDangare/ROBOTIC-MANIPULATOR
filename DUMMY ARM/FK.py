import math
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import numpy as np
from numpy import pi, sin, cos, sqrt ,radians, degrees

T1 = radians(45)
T2 = radians(45)

x = 0
y = 0

l1 = 4
l2 = 2

angles1 = np.arange(0, T1 + T1/90, T1/90)
angles2 = np.arange(0, T2 + T2/90, T2/90)
angles1 =list(angles1)
angles2 = list(angles2)

for i in angles1:
    print(degrees(i))
for i in angles2:
    print(degrees(i))

X1 = []
Y1 = []
X2 = []
Y2 = []

t1 = []

for theta in angles1:

    x1 = l1 * cos(theta)
    y1 = l1 * sin(theta)

    X1.append(x1)
    Y1.append(y1)
    t1.append(theta)

i=0
for theta in angles2:
    x2 = l1 * cos(t1[i]) + l2 * cos(t1[i] - theta)
    y2 = l1 * sin(t1[i]) + l2 * sin(t1[i] - theta)

    X2.append(x2)
    Y2.append(y2)
    i+=1

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

print('done')
# initialization function
def init():
    line.set_data([], [])
    return (line,)


# animation function
def animate(i):
    x_points = [0, X1[i], X2[i]]
    y_points = [0, Y1[i], Y2[i]]
    

    line.set_data(x_points, y_points)
    return (line,)


# call the animation
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(X1), interval=60, blit=True, repeat=False
)
print('done')
plt.show()
