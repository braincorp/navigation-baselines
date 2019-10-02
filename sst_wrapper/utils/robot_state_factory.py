"""Various helper functions to create a robot object and give corresponding
 bounds given the robot name."""
from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from bc_gym_planning_env.robot_models.standard_robot_names_examples \
    import StandardRobotExamples
from bc_gym_planning_env.robot_models.tricycle_model import TricycleRobotState
from bc_gym_planning_env.robot_models.differential_drive \
    import DiffdriveRobotState
import numpy as np


def create_robot_state(robot_name, **state):
    """
    Given a robot name and state information, create a robot_state object.
    :param robot_name: A string robot name
    :param state: The state information of the current robot
    :returns : the Modelstate object
    """
    if robot_name in StandardRobotExamples.INDUSTRIAL_TRICYCLE_V1:
        return TricycleRobotState(**state)

    elif robot_name in StandardRobotExamples.INDUSTRIAL_DIFFDRIVE_V1:
        return DiffdriveRobotState(**state)

    else:
        raise Exception('No robot name "{}" exists'.format(robot_name))


def get_robot_state_info(robot_name, world_size, origin):
    """
    Given a robot name, return the state info , bounds and circular topology
    flag.
    :param robot_name: A string robot name
    :param world_size: The dimensions of the map
    :param origin: The origin of the static map
    :returns : A tuple with the environment state, bounds and circular
    topology flags
    """
    max_x_y = world_size + origin
    min_x_y = origin
    if robot_name in StandardRobotExamples.INDUSTRIAL_TRICYCLE_V1:
        return (
            ['x', 'y', 'angle', 'v', 'w', 'wheel_angle'],
            [
                (min_x_y[0], max_x_y[0]),
                (min_x_y[1], max_x_y[1]),
                (-np.pi, np.pi),
                (0, 1.047),
                (-1.086, 1.086),
                (-np.pi / 2, np.pi / 2),
            ],
            [False, False, True, False, False, True],
        )
    if robot_name in StandardRobotExamples.INDUSTRIAL_DIFFDRIVE_V1:
        return (
            ['x', 'y', 'angle', 'v', 'w'],
            [
                (min_x_y[0], max_x_y[0]),
                (min_x_y[1], max_x_y[1]),
                (-np.pi, np.pi),
                (0, 1.047),
                (-1.086, 1.086),
            ],
            [False, False, True, False, False],
        )

    raise ValueError("Unknown robot type")
