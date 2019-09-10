import numpy as np
from sparse_rrt.experiments.experiment_utils import run_config
from bc_gym_planning_env.envs.mini_env import RandomMiniEnv
from sst_wrapper.envs.gym_sst_wrapper import bc_sst_wrapper
from sparse_rrt.planners import SST


def run_planner1():
    env = RandomMiniEnv(turn_off_obstacles=True,
                        iteration_timeout=1200,
                        goal_spat_dist=0.5)
    start = env.reset()
    start = start.robot_state.to_numpy_array()
    goal_pose = env._env._reward_provider._state.current_goal_pose()
    goal = np.concatenate((goal_pose, np.zeros(3)))
    bc_tricycle = bc_sst_wrapper(
        env, ['x', 'y', 'angle', 'v', 'w', 'wheel_angle'],
        circular_topology=[False, False, True, False, False, True])

    point_config = dict(system=bc_tricycle,
                        planner='rrt',
                        start_state=start,
                        goal_state=goal,
                        goal_radius=0.5,
                        random_seed=100,
                        integration_step=0.05,
                        min_time_steps=1,
                        max_time_steps=20,
                        number_of_iterations=10000,
                        display_type='None')
    run_config(point_config)


def run_planner2():
    env = RandomMiniEnv(turn_off_obstacles=True,
                        iteration_timeout=1200,
                        goal_spat_dist=0.5)
    start = env.reset()
    start = start.robot_state.to_numpy_array()
    goal_pose = env._env._reward_provider._state.current_goal_pose()
    goal = np.concatenate((goal_pose, np.zeros(3)))

    system = bc_sst_wrapper(
        env, ['x', 'y', 'angle', 'v', 'w', 'wheel_angle'],
        circular_topology=[False, False, True, False, False, True])
    planner = SST(state_bounds=system.get_state_bounds(),
                  control_bounds=system.get_control_bounds(),
                  distance=system.distance_computer(),
                  start_state=start,
                  goal_state=goal,
                  goal_radius=0.5,
                  random_seed=0,
                  sst_delta_near=0.4,
                  sst_delta_drain=0.2)

    # Run planning and print out solution is some statistics every few iterations.
    for iteration in range(1000):
        planner.step(system, 2, 20, 0.1)

        if iteration % 100 == 0:
            solution = planner.get_solution()
            print("Solution: %s, Number of nodes: %s" %
                  (planner.get_solution(), planner.get_number_of_nodes()))


if __name__ == "__main__":
    run_planner2()
