import numpy as np

from sparse_rrt import _sst_module


cdef double normalize_angle(double z):
    cdef double PI= np.pi
    return (z+PI) % (2 * PI) - PI


class TriStateDistance(_sst_module.IDistance):
    """
    Custom distance function for Tricycle robot.
    """
    def distance(self,double[:] point1,double[:] point2):
        """
        A cython function that calculates the euclidean distance between two
        states for the differential robot  by ignoring the kinematic state
        information. The index reference is as follows:
        point[0] - x axis coordinate
        point[1] - y axis coordinate
        point[2] - orientation of the robot
        point[5] - the wheel angle
        :param point1: First point of Tricycle robot.
        :param point2: Second point of Tricycle robot.
        :return: The euclidean distance between the points, ignoring
        kinematic states.
        """
        return (
            (point1[0]-point2[0])**2 +
            (point1[1]-point2[1])**2 +
            normalize_angle(point1[2]-point2[2])**2 +
            normalize_angle(point1[5]-point2[5])**2
        )**0.5


class DiffStateDistance(_sst_module.IDistance):
    """
    Custom distance function for Diffdrive robot.
    """
    def distance(self,double[:] point1,double[:] point2):
        """
        A cython function that calculates the euclidean distance between two
        states by ignoring the kinematic state information. The index reference
        is as follows:
        point[0] - x axis coordinate
        point[1] - y axis coordinate
        point[2] - orientation of the robot
        :param point1: First point of Diffdrive robot.
        :param point2: Second point of Diffdrive robot.
        :return: The euclidean distance between the points, ignoring
        kinematic states.
        """
        return (
            (point1[0]-point2[0])**2 +
            (point1[1]-point2[1])**2 +
            normalize_angle(point1[2]-point2[2])**2
        )**0.5
