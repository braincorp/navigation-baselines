from bc_gym_planning_env.robot_models.standard_robot_names_examples import StandardRobotExamples
from bc_gym_planning_env.robot_models.tricycle_model import TricycleRobotState
from bc_gym_planning_env.robot_models.differential_drive import DiffdriveRobotState
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


def get_robot_state_info(robot_name):
    """
    Given a robot name, return the state info , bounds and circular topology flag.
    :param robot_name: A string robot name
    :returns : A tuple with the environment state, bounds and circular topology flags
    """
    if robot_name in StandardRobotExamples.INDUSTRIAL_TRICYCLE_V1:
        return (
            ['x', 'y', 'angle', 'v', 'w', 'wheel_angle'],
            [
                (-2.75, 2.75),
                (-2.75, 2.75),
                (-np.pi, np.pi),
                (0, 1.047),
                (-1.086, 1.086),
                (-np.pi / 2, np.pi / 2),
            ],
            [False, False, True, False, False, True],
        )
    if robot_name in StandardRobotExamples.INDUSTRIAL_DIFFDRIVE_V1:
        return ([], [], [])
