"""

Python script helps to understand particle filter

"""

import random
import numpy as np
import matplotlib as plt

from robot import Robot


landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
space = [100, 100]

def eval(ref, particles):
    """Calculate the RMSE"""
    sum = 0.0
    for p in particles:
        dx = np.abs(p.x - ref.x)
        # Cyclic space
        if dx > space[0]/2.0:
            dx = space[0] - dx

        # Cyclic space
        dy = np.abs(p.y - ref.y)
        if dy > space[1]/2.0:
            dy = space[1] - dy

        sum += dx**2 + dy**2

    return np.sqrt(sum / float(len(particles)))


my_robot = Robot(space, landmarks)

# Initialize 1000 particles
n = 1000
particles = []
for i in range(n):
    particle = Robot(space, landmarks)
    particle.set_noise(0.05, 0.05, 5.0)
    particles.append(particle)

# Print initial RMSE
print(eval(my_robot, particles))

step = 0
n_steps = 10
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
    for i in range(n):
        beta += random.random() * 2.0 * max_weight
        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % n
        particles_resample.append(particles[index])

    particles = particles_resample

    print(eval(my_robot, particles))


