from pathlib import Path
import re
import numpy as np
import pinocchio as pin
from typing import Dict, Tuple, Optional
import yaml

from .ik_solver import HandIKSolver
from dexrobot_kinematics.utils.types import Position, Pose


class HandKinematicsBase:
    """Base class for hand kinematics calculations"""

    def __init__(self, handedness: str, urdf_path: Path, config_path: Path):
        """
        Initialize hand kinematics with URDF and config files

        Args:
            handedness: Handedness of the robot ("right" or "left")
            urdf_path: Path to robot URDF file
            config_path: Path to configuration YAML file
        """
        self.handedness = handedness
        mesh_path = urdf_path.parent / "../meshes"
        self.robot = pin.RobotWrapper.BuildFromURDF(
            str(urdf_path), package_dirs=[mesh_path]
        )
        self.config_path = config_path

        # Load configuration
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self._init_frames()
        self.ik_solvers = {}

    def _finger_name_to_id(self, finger: str) -> int:
        """Convert finger name to integer ID"""
        return {
            "thumb": 1,
            "index": 2,
            "middle": 3,
            "ring": 4,
            "little": 5,
        }.get(finger, -1)

    def _init_frames(self):
        """Initialize frame IDs for both finger pads and fingertips"""
        # Load finger frames from config
        self.fingerpad_frames = self.config["fingerpad_frames"]
        self.fingertip_frames = self.config["fingertip_frames"]

        # Initialize frame IDs for both types
        self.fingerpad_ids = {
            finger: self.robot.model.getFrameId(frame)
            for finger, frame in self.fingerpad_frames.items()
        }
        self.fingertip_ids = {
            finger: self.robot.model.getFrameId(frame)
            for finger, frame in self.fingertip_frames.items()
        }

    def forward_kinematics(
        self,
        joint_angles: Dict[str, float],
        base_pose: Optional[Pose] = None,
        frame: str = "hand",
        end_effector: str = "fingerpad",  # 'fingerpad' or 'fingertip'
    ) -> Dict[str, Pose]:
        """
        Compute forward kinematics for all fingers

        Args:
            joint_angles: Current joint angles
            base_pose: Optional base pose of the hand, used to transform output pose when frame is 'world'
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('fingerpad' or 'fingertip')
        Returns:
            Dictionary of finger poses in specified frame
        """
        # Convert joint angles to configuration vector
        q = np.zeros(self.robot.model.nq)
        for joint, angle in joint_angles.items():
            idx = self.robot.model.getJointId(joint)
            q[idx - 1] = angle  # Skip the "universe" joint

        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        # Select appropriate frame IDs based on end effector type
        target_ids = (
            self.fingertip_ids if end_effector == "fingertip" else self.fingerpad_ids
        )

        poses = {}
        for finger, frame_id in target_ids.items():
            pose = Pose.from_se3(self.robot.data.oMf[frame_id])

            # Transform to world frame if needed
            if frame == "world" and base_pose is not None:
                pose = base_pose.to_se3() * pose.to_se3()
                pose = Pose.from_se3(pose)

            poses[finger] = pose

        return poses

    def inverse_kinematics_finger(
        self,
        finger: str,
        target_pos: Position,
        base_pose: Optional[Pose] = None,
        frame: str = "hand",
        end_effector: str = "fingerpad",
        initial_guess: Optional[Dict[str, float]] = None,
    ) -> Tuple[Dict[str, float], bool]:
        """
        Solve IK for a single finger

        Args:
            finger: Target finger name
            target_pos: Target position
            base_pose: Optional base pose of the hand, used to transform target when frame is 'world'
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('fingerpad' or 'fingertip')
            initial_guess: Initial joint angles
        Returns:
            Tuple of (joint angles, success flag)
        """
        # Transform target position to hand frame if needed
        if frame == "world" and base_pose is not None:
            target_pos = (
                base_pose.to_se3().inverse() * Pose(position=target_pos).to_se3()
            )
            target_pos = Position.from_array(target_pos.translation)

        # Select target frame based on end effector type
        target_frame = (
            self.fingertip_frames[finger]
            if end_effector == "fingertip"
            else self.fingerpad_frames[finger]
        )

        if finger not in self.ik_solvers:
            self.ik_solvers[finger] = HandIKSolver(
                self.robot, self.handedness, target_frame, config_path=self.config_path
            )

        if initial_guess is not None:
            q0 = np.zeros(self.robot.model.nq)
            for joint, angle in initial_guess.items():
                idx = self.robot.model.getJointId(joint)
                q0[idx] = angle
        else:
            q0 = None

        q, success = self.ik_solvers[finger].solve(target_pos, q0)

        result = {}
        for joint in self.robot.model.names:
            if re.match(f"[rl]_f_joint{self._finger_name_to_id(finger)}", joint):
                idx = self.robot.model.getJointId(joint)
                result[joint] = q[idx - 1]

        return result, success

    def inverse_kinematics_grasp(
        self,
        finger_targets: Dict[str, Position],
        base_pose: Optional[Pose] = None,
        frame: str = "hand",
        end_effector: str = "fingerpad",
        initial_guess: Optional[Dict[str, float]] = None,
    ) -> Tuple[Dict[str, float], bool]:
        """Solve IK for multiple fingers simultaneously.

        Args:
            finger_targets: Target positions for each finger
            base_pose: Optional base pose of the hand, used to transform target when frame is 'world
            frame: Reference frame ('hand' or 'world')
            end_effector: Target frame type ('fingerpad' or 'fingertip')
            initial_guess: Initial joint angles
        Returns:
            Tuple of (joint angles for all fingers, success flag)
        """
        all_results = {}
        all_success = True

        for finger, target in finger_targets.items():
            result, success = self.inverse_kinematics_finger(
                finger, target, base_pose, frame, end_effector, initial_guess
            )
            all_results.update(result)
            all_success &= success

        return all_results, all_success
