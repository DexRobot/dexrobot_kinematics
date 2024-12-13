from pathlib import Path
import numpy as np
import pinocchio as pin
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
import yaml
from .ik_solver import HandIKSolver, Position

@dataclass
class Pose:
    """Represents position and optional orientation"""
    position: Position
    orientation: Optional[np.ndarray] = None
    
@dataclass 
class JointAngles:
    """Container for joint angles"""
    angles: Dict[str, float]
    
@dataclass
class FingerTargets:
    """Container for finger target positions"""
    positions: Dict[str, Position]

class HandKinematicsBase:
    """Base class for hand kinematics calculations"""
    
    def __init__(self, urdf_path: Path, config_path: Path):
        """
        Initialize hand kinematics with URDF and config files
        
        Args:
            urdf_path: Path to robot URDF file
            config_path: Path to configuration YAML file
        """
        self.robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path))
        self.config_path = config_path
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self._init_frames()
        self.ik_solvers = {}

    def _init_frames(self):
        """Initialize frame IDs for each finger tip and reference frames"""
        # Load finger frames from config
        self.finger_frames = self.config['finger_frames']
        self.frame_ids = {
            finger: self.robot.model.getFrameId(frame)
            for finger, frame in self.finger_frames.items()
        }
        
        # Get reference frame IDs
        self.hand_frame_id = self.robot.model.getFrameId(self.config['frames']['hand'])
        self.world_frame_id = self.robot.model.getFrameId(self.config['frames']['world'])

    def _transform_position(self, pos: Position, from_frame: str, to_frame: str) -> Position:
        """
        Transform position between reference frames
        
        Args:
            pos: Position to transform
            from_frame: Source reference frame ('hand' or 'world')
            to_frame: Target reference frame ('hand' or 'world')
        Returns:
            Transformed position
        """
        if from_frame == to_frame:
            return pos
            
        # Convert position to numpy array
        pos_array = np.array([pos.x, pos.y, pos.z])
        
        if from_frame == 'world' and to_frame == 'hand':
            # World to hand transformation
            oMh = self.robot.data.oMf[self.hand_frame_id]
            transformed_pos = oMh.inverse().act(pos_array)
        elif from_frame == 'hand' and to_frame == 'world':
            # Hand to world transformation
            oMh = self.robot.data.oMf[self.hand_frame_id]
            transformed_pos = oMh.act(pos_array)
        else:
            raise ValueError(f"Unsupported frame transformation: {from_frame} to {to_frame}")
            
        return Position(x=transformed_pos[0], y=transformed_pos[1], z=transformed_pos[2])

    def forward_kinematics(
        self,
        joint_angles: JointAngles,
        base_pose: Optional[Pose] = None,
        frame: str = 'hand'
    ) -> Dict[str, Pose]:
        """
        Compute forward kinematics for all fingers
        
        Args:
            joint_angles: Current joint angles
            base_pose: Optional base pose of the hand
            frame: Reference frame ('hand' or 'world')
        Returns:
            Dictionary of finger poses in specified frame
        """
        # First compute FK in hand frame
        q = np.zeros(self.robot.model.nq)
        for joint, angle in joint_angles.angles.items():
            idx = self.robot.model.getJointId(joint)
            q[idx] = angle
            
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        
        poses = {}
        for finger, frame_id in self.frame_ids.items():
            transform = self.robot.data.oMf[frame_id]
            pos = Position(
                x=transform.translation[0],
                y=transform.translation[1],
                z=transform.translation[2]
            )
            
            # Transform to requested frame if needed
            if frame == 'world':
                pos = self._transform_position(pos, 'hand', 'world')
                
            poses[finger] = Pose(
                position=pos,
                orientation=transform.rotation
            )
            
        return poses

    def inverse_kinematics_finger(
        self,
        finger: str,
        target_pos: Position,
        frame: str = 'hand',
        initial_guess: Optional[JointAngles] = None
    ) -> Tuple[JointAngles, bool]:
        """
        Solve IK for a single finger
        
        Args:
            finger: Target finger name
            target_pos: Target position
            frame: Reference frame ('hand' or 'world')
            initial_guess: Initial joint angles
        Returns:
            Tuple of (joint angles, success flag)
        """
        # Transform target position to hand frame if needed
        if frame == 'world':
            target_pos = self._transform_position(target_pos, 'world', 'hand')

        if finger not in self.ik_solvers:
            self.ik_solvers[finger] = HandIKSolver(
                self.robot,
                self.finger_frames[finger],
                config_path=self.config_path
            )

        if initial_guess is not None:
            q0 = np.zeros(self.robot.model.nq)
            for joint, angle in initial_guess.angles.items():
                idx = self.robot.model.getJointId(joint)
                q0[idx] = angle
        else:
            q0 = None
            
        q, success = self.ik_solvers[finger].solve(target_pos, q0)
        
        result = {}
        for joint in self.robot.model.names:
            if joint.startswith(f'r_f_joint{finger[0]}'):
                idx = self.robot.model.getJointId(joint)
                result[joint] = q[idx]
                
        return JointAngles(angles=result), success

    def inverse_kinematics_grasp(
        self,
        finger_targets: FingerTargets,
        frame: str = 'hand',
        initial_guess: Optional[JointAngles] = None
    ) -> Tuple[JointAngles, bool]:
        """
        Solve IK for multiple fingers simultaneously
        
        Args:
            finger_targets: Target positions for each finger
            frame: Reference frame ('hand' or 'world')
            initial_guess: Initial joint angles
        Returns:
            Tuple of (joint angles for all fingers, success flag)
        """
        all_results = {}
        all_success = True
        
        for finger, target in finger_targets.positions.items():
            result, success = self.inverse_kinematics_finger(
                finger, target, frame, initial_guess
            )
            all_results.update(result.angles)
            all_success &= success
            
        return JointAngles(angles=all_results), all_success
