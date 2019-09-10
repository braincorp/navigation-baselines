import numpy as np

from sparse_rrt.systems.system_interface import BaseSystem
from sst_wrapper.utils.robot_state_factory import create_robot_state


class bc_sst_wrapper(BaseSystem):
    """
    A class to wrap a gym environment to facilitate planning using sparse RRT
    """
    def __init__(self, env, env_state, circular_topology=None):
        """
        Initialize the configuration set
        :param env : An object of typle bc_gym_planning_env.envs.PlanEnv
        :param env_state: A list of strings representing the state of the robot
        :param ciruclar_topology : A list of boolean values, identifying whether each state has a circular or planar topology
        """
        self.env = env._env
        self.initial_state = env._env.get_state()
        self.robot_type = env._env._state.robot_state.get_robot_type_name()
        self.env_state = env_state
        if circular_topology is None:
            self.circular_topology = [False] * len(env_state)
        else:
            self.circular_topology = circular_topology
        self.action_space_bound = list(
            zip(env.action_space.low, env.action_space.high))

        # TODO : hard coded values for observation bounds, need to specify it for each robot - change formats
        self.min_max_x = (-2.75, 2.75)
        self.min_max_y = (-2.75, 2.75)
        self.min_max_angle = (-np.pi, np.pi)
        self.min_max_v = (0, 1.047)
        self.min_max_w = (-1.086, 1.086)
        self.min_max_steer_angle = (-np.pi / 2, np.pi / 2)
        super(bc_sst_wrapper).__init__(BaseSystem)

    def propagate(self, start_state, control, num_steps, integration_steps):
        """
        returns the state of the robot by propogating it for the required numner of integration steps and control signals
        :param start_state:
        :param control:
        :param num_steps:
        :param integration_steps:
        :return: A numpy array with the current state of the robot.
        """
        start_state_dict = dict(self.env_states, start_state)
        env_state = create_robot_state(self.robot_type, start_state)

        # Set the state of the environment to the current start state
        self.env.set_state(env_state)
        # Move the environment with the required number of steps
        for _ in range(num_steps):
            obs, r, done, _ = self.env.step(control)
            # TODO: NEED A CLEAN WAY TO RESTART SEARCH
            if done:
                env.reset()
                break

        # Get the new state
        state = obs.robot_state.to_numpy_array().astype(np.float32)
        state = np.ascontiguous(state)
        return state

    def visualize_point(self, state):
        #  Normalize the state and return points
        return [0, 0]

    def get_state_bounds(self):
        '''
        :return :the state bounds of the environment
        '''
        return [
            self.min_max_x, self.min_max_y, self.min_max_angle, self.min_max_v,
            self.min_max_w, self.min_max_steer_angle
        ]

    def get_control_bounds(self):
        """
        :return : A list consisting of tuple indicating the min and max of each control action.
        """
        return self.action_space_bound

    def is_circular_topology(self):
        """
        Indicate whether state system has planar or circular topology
        :return boolean flag for each coordinate
        """
        return self.circular_topology
