"""

Python script helps to understand particle filter

"""

import random
import time

from robot import Robot
from utilities import rmse, resample


landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
space = [100, 100]

# This is the reference particle without any noise (ground truth)
my_robot = Robot(space, landmarks)

# No. of particles
n = 2000  # Play with n to see the effect of particle number
particles = []
for i in range(n):
    particle = Robot(space, landmarks)
    particle.set_noise(0.05, 0.05, 5.0)
    particles.append(particle)

# Print initial RMSE
print(rmse(my_robot, particles, space))

step = 0
n_steps = 20
t_sum = 0.0  # time spent on sampling
while step < n_steps:
    step += 1

    # The robot moves randomly
    turn = random.random()*0.2
    forward = random.random()*5.0

    my_robot.move(turn, forward)
    measurements = my_robot.sense()

    # Move all the robots
    for particle in particles:
        particle.move(turn, forward)

    # Calculate the measurement probability for each robot
    for particle in particles:
        particle.measurement_update(measurements)

    # Resample the robots
    t0 = time.time()
    particles = resample(particles)
    t_sum += time.time() - t0

    print(rmse(my_robot, particles, space))

print("Total sampling time: {:.4f}".format(t_sum))
