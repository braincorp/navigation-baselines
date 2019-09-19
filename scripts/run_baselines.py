from scripts.run_planner import run_planner, parse_arguments, create_env
from sparse_rrt.planners import SST
from bc_gym_planning_env.envs.base.env import EnvParams
from sst_wrapper.envs.bc_gym_wrapper import bc_gym_wrapper
from sst_wrapper.envs.gym_sst_wrapper import bc_sst_wrapper
import multiprocessing as mp
import numpy as np


class parameter_class():
    def __init__(self, robot, env):
        self.robot = robot
        self.env = env

    def run_planner(self, s):
        """
        Run the planner for seed s
        :param s : the seed to run the planner on.
        """
        env = create_env(self.robot, self.env, s)
        wrapped_env = bc_gym_wrapper(env)
        start, goal = wrapped_env.start, wrapped_env.goal
        bc_robot = bc_sst_wrapper(wrapped_env)
        planner = SST(
            state_bounds=bc_robot.get_state_bounds(),
            control_bounds=bc_robot.get_control_bounds(),
            distance=bc_robot.distance_computer(),
            start_state=start,
            goal_state=goal,
            goal_radius=0.5,
            random_seed=0,
            sst_delta_near=self.sst_delta_near,
            sst_delta_drain=self.sst_delta_drain,
        )
        for iteration in range(100000):
            planner.step(bc_robot, 1, self.sst_step_size, .05)
            if iteration % 10000 == 0:
                solution = planner.get_solution()
                if solution:
                    return 1
        solution = planner.get_solution()
        return 0


def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument('robot',
                        choices=['tri', 'diff'],
                        help="The robot in the environment")
    parser.add_argument('env',
                        choices=['mini', 'turn', 'map'],
                        help="The environment to run the experiment on")
    parser.add_argument('--num_env',
                        type=int,
                        default=100,
                        help="Number of environments to run baselines on")
    parser.add_argument('--num_workers',
                        type=int,
                        default=2,
                        help="Number of workers to run baselines")
    args = parser.parse_args()
    return args


def run_baselines():
    args = parse_arguments()
    pool = mp.Pool(processes=args.num_workers)
    sst_planner = parameter_class(args.robot, args.env)
    results = pool.map(sst_planner.run_planner, range(args.num_env))
    pool.close()
    print("Accuracy : {}".format(100 * np.sum(results) / args.num_env))


if __name__ == "__main__":
    run_baselines()
