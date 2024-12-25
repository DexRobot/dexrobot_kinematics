from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np
import pinocchio as pin

from .base import HandKinematicsBase
from ..utils.types import Position, Pose, JointAngles, FingerTargets

class RightHand(HandKinematicsBase):
    """Right hand kinematics implementation"""
    
    def __init__(self, urdf_path: Optional[Path] = None, config_path: Optional[Path] = None):
        """Initialize right hand kinematics"""
        if urdf_path is None:
            urdf_path = Path("dexrobot_urdf/urdf/dexhand021_right.urdf")
        if config_path is None:
            config_path = Path("../config/right_hand_default.yaml")
            
        super().__init__(urdf_path, config_path)
        
    def compute_single_finger_fk(
        self,
        finger: str,
        joint_angles: JointAngles,
        frame: str = 'hand',
        end_effector: str = 'finger'
    ) -> Pose:
        """
        Forward kinematics for a single finger
        
        Args:
            finger: Target finger name ('thumb', 'index', etc.)
            joint_angles: Current joint angles
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('finger' or 'fingertip')
        Returns:
            Finger end effector pose
        """
        poses = self.forward_kinematics(joint_angles, frame=frame, end_effector=end_effector)
        return poses[finger]
        
    def compute_multi_finger_fk(
        self,
        fingers: List[str],
        joint_angles: JointAngles,
        frame: str = 'hand',
        end_effector: str = 'finger'
    ) -> Dict[str, Pose]:
        """
        Forward kinematics for multiple fingers
        
        Args:
            fingers: List of target finger names
            joint_angles: Current joint angles 
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('finger' or 'fingertip')
        Returns:
            Dictionary of finger poses
        """
        all_poses = self.forward_kinematics(joint_angles, frame=frame, end_effector=end_effector)
        return {finger: all_poses[finger] for finger in fingers}
        
    def solve_single_finger_ik(
        self,
        finger: str,
        target_pos: Position,
        frame: str = 'hand',
        end_effector: str = 'finger',
        initial_guess: Optional[JointAngles] = None
    ) -> Tuple[JointAngles, bool]:
        """
        Inverse kinematics for single finger
        
        Args:
            finger: Target finger name
            target_pos: Target position
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('finger' or 'fingertip')
            initial_guess: Initial joint angles
        Returns:
            Tuple of (joint angles, success flag)
        """
        return self.inverse_kinematics_finger(
            finger,
            target_pos,
            frame=frame,
            end_effector=end_effector,
            initial_guess=initial_guess
        )
        
    def solve_multi_finger_ik(
        self,
        finger_targets: Dict[str, Position],
        frame: str = 'hand',
        end_effector: str = 'finger',
        initial_guess: Optional[JointAngles] = None
    ) -> Tuple[JointAngles, bool]:
        """
        Inverse kinematics for multiple fingers
        
        Args:
            finger_targets: Dictionary of finger targets
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('finger' or 'fingertip')
            initial_guess: Initial joint angles
        Returns:
            Tuple of (joint angles, success flag)
        """
        targets = FingerTargets(positions=finger_targets)
        return self.inverse_kinematics_grasp(
            targets,
            frame=frame,
            end_effector=end_effector,
            initial_guess=initial_guess
        )
