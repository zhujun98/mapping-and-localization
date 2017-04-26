"""

Help functions.

"""
import numpy as np
import random
import copy


def rmse(robot, particles, space):
    """Calculate the RMSE

    :param robot: Robot object
        Robot.
    :param particles: a list of Robot object
        Particles.
    :param space: list
        (x, y) limit of the moving space.
    """
    sum = 0.0
    for p in particles:
        dx = np.abs(p.x - robot.x)
        dy = np.abs(p.y - robot.y)

        # Cyclic space condition should not apply here since it is not
        # considered when calculating the distances to the landmarks.
        # if dy > space[1]/2.0:
        #     dy = space[1] - dy
        # if dx > space[0]/2.0:
        #     dx = space[0] - dx

        sum += dx**2 + dy**2

    return np.sqrt(sum / float(len(particles)))


def resample(particles, method='low_variance'):
    """Resample a particle set according to the weights

    :param particles: a list of Robot instances
        Particle set.

    :param method: string
        Method used for importance sampling. Possible choices:
        "low_variance", "wheel".

        References:
        [1] S. Thrun et al., Probabilistic Robotics, p.87
        [2] Douc, R., et al., Comparison of resampling schemes for
        particle filtering. In 4th International Symposium on Image
        and Signal Processing and Analysis (2005).
    """
    n = len(particles)

    weights = []
    for p in particles:
        weights.append(p.prob)
    # Normalize weights
    weights /= sum(weights)

    new_sample = []

    j = 0
    s = weights[0]
    if method == 'low_variance':
        r = random.random() * 1.0 / n
        for i in range(n):

            while r > s:
                j += 1
                s += weights[j]

            # It is important to copy the Instance of the Robot() class
            # here!!! Otherwise, it will end up with multiple groups of
            # exactly same objects in the new particle swarm.
            new_sample.append(copy.copy(particles[j]))
            r += 1.0 / n

    elif method == 'wheel':
        i = int(random.random() * n)
        beta = 0.0
        max_weight = max(weights)
        for _ in range(n):
            beta += random.random() * 2.0 * max_weight
            while beta > weights[i]:
                beta -= weights[i]
                i = (i + 1) % n

            new_sample.append(copy.copy(particles[i]))
    else:
        raise ValueError('Unknown method')

    return new_sample