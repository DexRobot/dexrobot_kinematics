import numpy as np
import pytest
from dexrobot_kinematics.utils.transforms import transform_points, compute_optimal_hand_pose

def test_point_transformation():
    """Test point transformation utility"""
    # Create test transform
    R = pin.utils.rpyToMatrix(0, 0, np.pi/2)
    t = np.array([1.0, 0.0, 0.0])
    transform = pin.SE3(R, t)

    # Test single point
    point = np.array([0.0, 1.0, 0.0])
    transformed = transform_points(point, transform)
    assert np.allclose(transformed, np.array([1.0, -1.0, 0.0]))

    # Test dictionary of points
    points = {
        'p1': np.array([0.0, 1.0, 0.0]),
        'p2': np.array([0.0, 2.0, 0.0])
    }
    transformed = transform_points(points, transform)
    assert np.allclose(transformed['p1'], np.array([1.0, -1.0, 0.0]))
    assert np.allclose(transformed['p2'], np.array([1.0, -2.0, 0.0]))

def test_optimal_hand_pose():
    """Test optimal hand pose computation"""
    # Define pinch grasp targets
    targets = {
        'thumb': np.array([0.0, -0.02, 0.1]),
        'index': np.array([0.0, 0.02, 0.1])
    }

    pose = compute_optimal_hand_pose(
        targets,
        approach_axis=np.array([0, 0, 1])
    )

    # Verify reasonable pose
    assert isinstance(pose, pin.SE3)
    # Hand should be slightly behind targets
    assert pose.translation[2] < 0.1
    # Z axis should align with approach
    assert np.allclose(pose.rotation[:, 2], [0, 0, 1], atol=0.01)
