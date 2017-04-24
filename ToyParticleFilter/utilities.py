"""

Help functions.

"""
import numpy as np


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