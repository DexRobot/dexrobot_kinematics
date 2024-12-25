import numpy as np
import pinocchio as pin
import pytest
from dexrobot_kinematics.hand.base import HandKinematicsBase
from dexrobot_kinematics.utils.types import JointAngles, Position, Pose

@pytest.fixture
def right_hand():
    """Fixture providing initialized right hand kinematics"""
    urdf_path = Path("dexrobot_urdf/urdf/dexhand021_right.urdf")
    return HandKinematicsBase(urdf_path)

def test_forward_kinematics_zero(right_hand):
    """Test FK with zero configuration"""
    # All joints at zero
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}

    # Get fingertip poses
    poses = right_hand.forward_kinematics(joint_angles)

    # Check results
    assert len(poses) == 5  # All fingers
    for pose in poses.values():
        assert isinstance(pose, pin.SE3)
        # Fingertips should be in front of hand base
        assert pose.translation[2] > 0

def test_forward_kinematics_base_pose(right_hand):
    """Test FK with hand base pose"""
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}

    # Set base pose (45 deg rotation around z)
    R = pin.utils.rpyToMatrix(0, 0, np.pi/4)
    t = np.array([1.0, 0.0, 0.0])
    base_pose = pin.SE3(R, t)

    poses = right_hand.forward_kinematics(
        joint_angles,
        base_pose=base_pose,
        frame='world'
    )

    # Verify transformation
    for pose in poses.values():
        # Points should be rotated and translated
        assert pose.translation[0] > 0.5  # x shifted
        assert abs(pose.translation[1]) > 0.1  # y affected by rotation

def test_inverse_kinematics_finger(right_hand):
    """Test single finger IK"""
    # Target position for index fingertip
    target = np.array([0.0, 0.0, 0.15])  # Straight ahead

    joint_angles, success = right_hand.inverse_kinematics_finger(
        finger='index',
        target_pos=target
    )

    assert success

    # Verify result
    poses = right_hand.forward_kinematics(joint_angles)
    actual_pos = poses['index'].translation
    assert np.allclose(actual_pos, target, atol=0.01)

def test_inverse_kinematics_finger_unreachable(right_hand):
    """Test IK with unreachable target"""
    # Target too far away
    target = np.array([0.0, 0.0, 1.0])

    joint_angles, success = right_hand.inverse_kinematics_finger(
        finger='index',
        target_pos=target
    )

    assert not success

def test_inverse_kinematics_grasp(right_hand):
    """Test multi-finger grasp IK"""
    # Define pinch grasp targets
    targets = {
        'thumb': np.array([0.0, -0.02, 0.1]),
        'index': np.array([0.0, 0.02, 0.1])
    }

    joint_angles, success = right_hand.inverse_kinematics_grasp(targets)

    assert success

    # Verify results
    poses = right_hand.forward_kinematics(joint_angles)
    for finger, target in targets.items():
        actual_pos = poses[finger].translation
        assert np.allclose(actual_pos, target, atol=0.01)
