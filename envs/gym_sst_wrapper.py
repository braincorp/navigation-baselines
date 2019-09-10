import numpy as np

from sparse_rrt.systems.system_interface import BaseSystem


class gym_sst_wrapper(BaseSystem):
    """
    A class to wrap a gym environment to facilitate planning using sparse RRT
    """
    def __init__(self):
        raise NotImplementedError

    def propagate(self, start_state, control, num_steps, integration_steps):
        raise NotImplementedError

    def visualize_point(self, state):
        raise NotImplementedError

    def get_state_bounds(self):
        raise NotImplementedError

    def get_control_bounds(self):
        raise NotImplementedError

    def is_circular_topology(self):
        """
        Indicate whether state system has planar or circular topology
        :return boolean flag for each coordinate
        """
        return [False, True]
