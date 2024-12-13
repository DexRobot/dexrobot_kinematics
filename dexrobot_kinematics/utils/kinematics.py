import pinocchio as pin
import copy


def dict_to_q(robot, q_dict):
    """Convert a dict mapping joint names to angles to a q vector.

    Parameters:
    robot (RobotWrapper): The robot model.
    q_dict (dict): A dictionary mapping joint names to angles.

    Returns:
    np.array: The configuration vector.
    """
    q = pin.neutral(robot.model)
    for joint_name, angle in q_dict.items():
        joint_id = robot.model.getJointId(joint_name)
        if joint_id != -1:
            # The zeroth element in robot.model.names is "universe", so we need to subtract 1
            q[joint_id - 1] = angle
    return q


def get_frame_pose(robot, q, frame_id):
    """
    Get the pose of a frame in the robot model given the configuration.

    Parameters:
    robot (RobotWrapper): The robot model.
    q (np.array): The configuration of the robot.
    frame_id (int): The ID of the frame.

    Returns:
    SE3: The pose of the frame.
    """
    new_robot = copy.deepcopy(robot)
    pin.forwardKinematics(new_robot.model, new_robot.data, q)
    pin.updateFramePlacements(new_robot.model, new_robot.data)
    return new_robot.data.oMf[frame_id]
