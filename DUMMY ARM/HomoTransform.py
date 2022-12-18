import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def HTR(theta, axis, origin):
    if axis == 'x':
        tf = np.array([ [1, 0, 0, 0],
                        [0, np.cos(theta), -np.sin(theta), 0],
                        [0, np.sin(theta), np.cos(theta), 0],
                        [0, 0, 0, 1]])
        return np.dot(origin,tf)
    elif axis == 'y':
        tf = np.array([ [np.cos(theta), 0, np.sin(theta), 0],
                        [0, 1, 0, 0],
                        [-np.sin(theta), 0, np.cos(theta), 0],
                        [0, 0, 0, 1]])
        return np.dot(origin,tf)
    elif axis == 'z':
        tf = np.array([ [np.cos(theta), -np.sin(theta), 0, 0],
                        [np.sin(theta), np.cos(theta), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        return np.dot(origin,tf)

def HTL(d, axis, origin):
    if axis == 'x':
        tf = np.array([ [1, 0, 0, d],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        return np.dot(origin,tf)
    elif axis == 'y':
        tf = np.array([ [1, 0, 0, 0],
                        [0, 1, 0, d],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        return np.dot(origin,tf)
    elif axis == 'z':
        tf = np.array([ [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, d],
                        [0, 0, 0, 1]])
        return np.dot(origin,tf)
#Robot1
origin = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
theta1 = 0
theta2 = 0
theta3 = 0
l1 = 1
l2 = 0.5
l3 = 1
l4 = 0.5
l5 = 1
l6 = 1
l7 = 1
l8 = 1

origin = HTR(theta = theta1, axis='z', origin=origin)
tf1 = HTL(d = l1, axis='z', origin=origin)
tf2 = HTL(d = l2, axis='z', origin=tf1)
tf2 = HTL(d = l3, axis='y', origin=tf2)
tf2 = HTL(d = l4, axis='z', origin=tf2)
tf3 = HTR(theta = theta2, axis='z', origin=tf2)
tf3 = HTL(d = l5, axis='z', origin=tf3)
tf3 = HTL(d = l6, axis='y', origin=tf3)
tf4 = HTL(d = l7, axis='y', origin=tf3)
tf4 = HTR(theta = theta3, axis='z', origin=tf4)
tf5 = HTL(d = l8, axis='y', origin=tf4)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(origin[0,3],  origin[1,3],  origin[2,3],  color='r')
ax.scatter(tf1[0,3],  tf1[1,3],  tf1[2,3],  color='b')
ax.scatter(tf2[0,3],  tf2[1,3],  tf2[2,3],  color='g')
ax.scatter(tf3[0,3],  tf3[1,3],  tf3[2,3],  color='y')
ax.scatter(tf4[0,3],  tf4[1,3],  tf4[2,3],  color='k')
ax.scatter(tf5[0,3],  tf5[1,3],  tf5[2,3],  color='c')

#Robot2
origin = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
theta1 = 0
theta2 = 0
theta3 = 0
l1 = 1
l2 = 1
l3 = 1
l4 = 1
l5 = 1
l6 = 1

tf1 = HTL(d = l1, axis='z', origin=origin)
tf2 = HTR(theta = theta1, axis='x', origin=tf1)
tf2 = HTL(d = l2, axis='y', origin=tf2)
tf3 = HTR(theta = theta2, axis='z', origin=tf2)
tf3 = HTL(d = l3, axis='y', origin=tf3)
tf4 = HTR(theta = theta3, axis='z', origin=tf3)
tf4 = HTL(d = l4, axis='z', origin=tf4)
tf4 = HTL(d = l5, axis='y', origin=tf4)
tf5 = HTL(d = l6, axis='y', origin=tf4)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(origin[0,3],  origin[1,3],  origin[2,3],  color='r')
ax.scatter(tf1[0,3],  tf1[1,3],  tf1[2,3],  color='b')
ax.scatter(tf2[0,3],  tf2[1,3],  tf2[2,3],  color='g')
ax.scatter(tf3[0,3],  tf3[1,3],  tf3[2,3],  color='y')
ax.scatter(tf4[0,3],  tf4[1,3],  tf4[2,3],  color='k')
ax.scatter(tf5[0,3],  tf5[1,3],  tf5[2,3],  color='c')