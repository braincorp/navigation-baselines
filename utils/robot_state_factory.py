from bc_gym_planning_env.robot_models.standard_robot_names_examples import StandardRobotExamples
from bc_gym_planning_env.robot_models.tricycle_model import TricycleRobotState
from bc_gym_planning_env.robot_models.differential_drive import DiffdriveRobotState


def create_robot_state(robot_name,**state):
    """
    Given a robot name and state information, create a robot_state object.
    :param robot_name: A string robot name
    :param state: The state information of the current robot
    :return the Modelstate object
    """
    if robot_name in StandardRobotExamples.INDUSTRIAL_TRICYCLE_V1:
        return TricycleRobotState(**state)

    elif robot_name in StandardRobotExamples.INDUSTRIAL_DIFFDRIVE_V1:
        return DiffdriveRobotState(**state)

    else:
        raise Exception('No robot name "{}" exists'.format(robot_name))
