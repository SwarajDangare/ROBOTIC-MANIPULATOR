from matplotlib import pyplot as plt

force =20
setVelocity = 80            # target
error = setVelocity
V = 0                  # initieal velocity
oldError = 80
dt = 0.1                    # for each instant

a = 60                      # loop for a minute
velocity = 0                # current velocity

# for ploting graph
t = 0
ov = 0
I = 0
time = []
currentVelocity = []
currentError = []

# PID constant
kp = 0.05
ki = 0.3
kd = 0.01


while a > 0:
    P = kp * error
    I = I + ki * error * dt            # integration i.e addition of all the error
    D = kd * (error - oldError) / dt   # derivative i.e  

    velocity = V + P + I + D

    oldError = error
    error = setVelocity - velocity

    currentVelocity.append(ov)
    currentError.append(error)
    time.append(t)
    ov = velocity
    t += 1
    a -= dt

figure, graph = plt.subplots(2)

graph[0].plot(time, currentVelocity)
# graph[0].set_xlabel("Time")
graph[0].set_ylabel("Velocity")
graph[0].set_title("Velocity")

graph[1].plot(time, currentError)
graph[1].set_xlabel("Time (1 min)")
graph[1].set_ylabel("Current Error")
graph[1].set_title("Error")
plt.show()
print(currentError)
print(currentVelocity)
