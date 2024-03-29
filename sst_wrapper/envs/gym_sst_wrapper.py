"""A class to create a function that sets up a system for sst planning"""
from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from sparse_rrt.systems.system_interface import BaseSystem


class BcSstWrapper(BaseSystem):
    """
    A class to wrap a gym environment to facilitate planning using sparse RRT
    """
    def __init__(self, env):
        """
        Initialize the configuration set
        :param env : An environment of class sst_wrapper.envs.bc_gym_wrapper
        """
        self.env = env
        super().__init__()

    def propagate(self, start_state, control, num_steps, integration_step):
        """
        returns the state of the robot by propagating it for the
        required number of integration steps and control signals
        :param start_state: The start state of the robot
        :param control: The input control to robot
        :param num_steps: Number of steps take by the robot
        :param integration_step: The length of each time step
        :return: A numpy array with the current state of the robot.
        """
        self.env.set_state(start_state)

        # Move the environment with the required number of steps
        for _ in range(num_steps):
            obs, r, done, _ = self.env.step(control)
            if done:
                if r < -10:
                    return None
                self.env.reset()
                break

        state = obs.robot_state.to_numpy_array()
        return state

    def visualize_point(self, state):
        """
        Normalize the state and return points
        :param state : the state of the robot
        :return: x,y(tuple) of scaled x,y position of the robot.
        """
        visualize_bounds = self.env.get_visualize_bounds()
        min_x, max_x = visualize_bounds[0]
        min_y, max_y = visualize_bounds[1]
        x = (state[0] - min_x) / (max_x - min_x)
        y = (state[1] - min_y) / (max_y - min_y)
        return x, y

    def get_state_bounds(self):
        """
        Returns the observation state bounds
        :return: the state bounds of the environment
        """
        return self.env.observation_space_bounds

    def get_control_bounds(self):
        """
        Returns the actions bound
        :return: A list consisting of tuple indicating the min and max of each control action.
        """
        return self.env.action_space_bound

    def is_circular_topology(self):
        """
        Indicate whether state system has planar or circular topology
        :return: boolean flag for each coordinate
        """
        return self.env.circular_topology
