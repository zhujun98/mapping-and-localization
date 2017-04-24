"""

A moving Robot class.

"""
import random
import numpy as np


class Robot(object):
    """Robot class

    A robot who has three coordinates: x, y and orientation.
    """
    def __init__(self, space, landmarks):
        """Initialization

        :param space: array-like
            (x, y) limit of the space.

        :param landmarks: array like
            Coordinates of landmarks in the map.

        The coordinates of the robot will be initialized randomly.
        """
        self.space = space
        self.landmarks = landmarks
        self.prob = 1.0  # probability of this robot

        self._x = None
        self.x = random.random() * self.space[0]

        self._y = None
        self.y = random.random() * self.space[1]

        self._orientation = None
        self.orientation = random.random() * 2.0 * np.pi

        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    @property
    def x(self):
        """Property x"""
        return self._x

    @x.setter
    def x(self, value):
        """Property x setter

        Assuming cyclic space.
        """
        self._x = value % self.space[0]

    @property
    def y(self):
        """Property y"""
        return self._y

    @y.setter
    def y(self, value):
        """Property y setter

        Assuming cyclic space
        """
        self._y = value % self.space[1]

    @property
    def orientation(self):
        """Property orientation"""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        """Property orientation setter"""
        self._orientation = value % (2*np.pi)

    def set(self, x, y, orientation):
        """Set new coordinates"""
        self.x = x
        self.y = y
        self.orientation = orientation

    def set_noise(self, f_noise, t_noise, s_noise):
        """Set noises"""
        self.forward_noise = float(f_noise)
        self.turn_noise = float(t_noise)
        self.sense_noise = float(s_noise)

    def move(self, turn, forward):
        """Robot executes the moving command

        :param turn: float
            Rotation of the robot in rad (positive for counter clockwise)
        :param forward: float
            Translation of the robot.
        """
        if forward < 0:
            raise ValueError('Robot cannot move backwards')

        # turn, and add randomness to the rotation
        self.orientation += turn + random.gauss(0.0, self.turn_noise)

        # move, and add randomness to the translation
        dist = forward + random.gauss(0.0, self.forward_noise)
        self.x += np.cos(self.orientation)*dist
        self.y += np.sin(self.orientation)*dist

    def sense(self):
        """Measure distances from landmarks

        :return array-like
            Measurements.
        """
        measurements = []
        for landmark in self.landmarks:
            dist = np.sqrt((self.x - landmark[0])**2
                           + (self.y - landmark[1])**2)
            # Add noise
            dist += random.gauss(0.0, self.sense_noise)

            measurements.append(dist)

        return measurements

    def measurement_update(self, measurements):
        """Given landmarks, calculate the measurement probability"""
        assert len(measurements) == len(self.landmarks)

        prob = 1.0
        for landmark, measurement in zip(self.landmarks, measurements):
            dist = np.sqrt((self.x - landmark[0])**2
                           + (self.y - landmark[1])**2)

            prob *= self.gaussian(measurement, dist, self.sense_noise)

        self.prob = prob

    def gaussian(self, x, mu, sigma):
        """calculates the PDF of a 1-D distribution

        :param x: float
            Coordinate
        :param mu: float
            Mean
        :param sigma: float
            Standard deviation

        :return: float
            Probability
        """
        return np.exp(-1.0 * ((mu - x)**2) / sigma**2 / 2.0) \
               / np.sqrt(2.0*np.pi*sigma**2)

    def __repr__(self):
        return 'x=%.6s y=%.6s orient=%.6s' % \
               (str(self.x), str(self.y), str(self.orientation))