import numpy as np
from bc_gym_planning_env.envs.base.env import PlanEnv
from bc_gym_planning_env.envs.base.action import Action
from sst_wrapper.utils.robot_state_factory import create_robot_state, get_robot_state_info


class bc_gym_wrapper():
    def __init__(self, env):
        assert issubclass(
            type(env),
            PlanEnv), "env should be an object of class or subclass of PlanEnv"
        self.env = env
        self.robot_type = env._state.robot_state.get_robot_type_name()
        start = env.reset()
        self.start = start.robot_state.to_numpy_array()
        self.set_goal_position()
        self.env_initial_state = env.get_state()

        self.env_state_keys, self.observation_space_bounds, self.circular_topology = get_robot_state_info(
            self.robot_type,
            env._state.costmap.world_size(),
            env._state.costmap.get_origin(),
        )

        self.action_space_bound = list(
            zip(env.action_space.low, env.action_space.high))

    def set_goal_position(self):
        goal_pose = self.env._reward_provider._state.current_goal_pose()
        goal_vel = np.array([1e-1] * (len(self.start) - len(goal_pose)))
        self.goal = np.concatenate((goal_pose, goal_vel))

    def get_visualize_bounds(self):
        return [
            self.observation_space_bounds[0], self.observation_space_bounds[1]
        ]

    def set_state(self, state):
        state_dict = dict(zip(self.env_state_keys, state))
        robot_state = create_robot_state(self.robot_type, **state_dict)
        self.env_initial_state.robot_state = robot_state
        self.env.set_state(self.env_initial_state)

    def step(self, action):
        obs, r, done, info = self.env.step(Action(np.array(action)))
        return obs, r, done, info

    def reset(self):
        obs = self.env.reset()
        return obs
