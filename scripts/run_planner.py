from bc_gym_planning_env.envs.base.action import Action
from bc_gym_planning_env.envs.base.env import PlanEnv
from bc_gym_planning_env.envs.base.params import EnvParams
from bc_gym_planning_env.envs.mini_env import RandomMiniEnv
from bc_gym_planning_env.envs.rw_corridors.tdwa_test_environments import get_random_maps_squeeze_between_obstacle_in_corridor_on_path
from bc_gym_planning_env.envs.synth_turn_env import RandomAisleTurnEnv
from bc_gym_planning_env.robot_models.standard_robot_names_examples import StandardRobotExamples
from sparse_rrt.experiments.experiment_utils import run_config
from sst_wrapper.envs.bc_gym_wrapper import bc_gym_wrapper
from sst_wrapper.envs.gym_sst_wrapper import bc_sst_wrapper
import argparse
import numpy as np
import time


def run_planner(env, args):
    # write a planner for a given environment and iterations
    start, goal = env.start, env.goal
    bc_robot = bc_sst_wrapper(env)
    if args.graph:
        display_type = 'tree'
    else:
        display_type = None
    point_config = dict(system=bc_robot,
                        planner='sst',
                        sst_delta_near=0.65,
                        sst_delta_drain=0.05,
                        start_state=start,
                        goal_state=goal,
                        goal_radius=0.5,
                        random_seed=0,
                        integration_step=0.05,
                        min_time_steps=1,
                        max_time_steps=10,
                        number_of_iterations=args.iterations,
                        display_type=display_type)
    planner = run_config(point_config)
    solution = planner.get_solution()

    return solution


def parse_arguments():

    parser = argparse.ArgumentParser()

    parser.add_argument('robot',
                        choices=['tri', 'diff'],
                        help="The robot in the environment")
    parser.add_argument('env',
                        choices=['mini', 'turn', 'map'],
                        help="The environment to run the experiment on")
    parser.add_argument('--iterations',
                        type=int,
                        default=10000,
                        help="Number of iterations to run the SST Planner")
    parser.add_argument('--graph',
                        dest='graph',
                        default=False,
                        action="store_true",
                        help="visualize the planning ggraph build")
    parser.add_argument(
        '--render',
        dest='render',
        default=False,
        action='store_true',
        help=
        "visualize the final path returned from the planner if there is one")
    parser.add_argument('--seed',
                        type=int,
                        default=None,
                        help="set the seed of the environment")
    args = parser.parse_args()
    return args


def create_env(robot, env, seed=None):
    """
    Creates the environment based on the arguments passed
    """
    if robot in 'tri':
        robot_name = StandardRobotExamples.INDUSTRIAL_TRICYCLE_V1
    elif robot in 'diff':
        robot_name = StandardRobotExamples.INDUSTRIAL_DIFFDRIVE_V1

    env_param = EnvParams(iteration_timeout=1200,
                          goal_ang_dist=0.2,
                          goal_spat_dist=0.2,
                          robot_name=robot_name)
    if seed is None:
        seed = np.random.randint(0, 1000)
        print("Chosen seed {}".format(seed))
    if env in 'mini':
        temp = RandomMiniEnv(params=env_param,
                             draw_new_turn_on_reset=False,
                             seed=seed)
        return temp._env
    if env in 'turn':
        temp = RandomAisleTurnEnv(params=env_param,
                                  draw_new_turn_on_reset=False,
                                  seed=seed)
        return temp._env
    if env in 'map':
        _, path, test_maps = get_random_maps_squeeze_between_obstacle_in_corridor_on_path(
        )
        return PlanEnv(costmap=test_maps[seed], path=path, params=env_param)


if __name__ == "__main__":
    args = parse_arguments()
    env = create_env(args.robot, args.env, args.seed)
    wrapped_env = bc_gym_wrapper(env)
    solution = run_planner(wrapped_env, args)

    # Evaluate the solution
    if solution and args.render:
        _, actions, time_duration = solution
        time_duration = time_duration / 0.05
        obs = env.reset()
        env.render()
        for i, a in zip(time_duration, actions):
            for _ in range(int(i)):
                env.step(Action(a))
                env.render()
                time.sleep(0.02)
