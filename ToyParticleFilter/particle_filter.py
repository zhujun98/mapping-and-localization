"""

Python script helps to understand particle filter

"""

import random
import copy

from robot import Robot
from utilities import rmse


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
    weights = []
    for particle in particles:
        particle.measurement_update(measurements)
        weights.append(particle.prob)

    # Resample the robots
    particles_resample = []
    index = int(random.random() * n)
    beta = 0.0
    max_weight = max(weights)
    for _ in range(n):
        beta += random.random() * 2.0 * max_weight
        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % n

        # It is important to copy the Instance of the Robot() class
        # here!!! Otherwise, it will end up with multiple groups of
        # exactly same objects in the new particle swarm.
        particles_resample.append(copy.copy(particles[index]))

    particles = particles_resample

    print(rmse(my_robot, particles, space))


