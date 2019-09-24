import numpy as np

from sparse_rrt import _sst_module
from bc_gym_planning_env.utilities.coordinate_transformations import normalize_angle

class TriStateDistance(_sst_module.IDistance):
    """
    Custom distance function for Tricycle robot.
    """
    def distance(self,point1,point2):
        x = (point1[0]-point2[0])**2
        y = (point1[1]-point2[1])**2
        theta = normalize_angle(point1[2]-point2[2])**2
        gamma = normalize_angle(point1[5]-point2[5])**2
        return np.sqrt(x+y+theta+gamma)
