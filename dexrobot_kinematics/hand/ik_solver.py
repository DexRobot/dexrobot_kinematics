import numpy as np 
import pinocchio as pin
from typing import Tuple, Optional
import yaml
from pathlib import Path

from dexrobot_kinematics.utils.types import Position

class HandIKSolver:
    """A Pinocchio-based IK solver for robotic hand that handles joint limits and finger joint synchronization"""

    def __init__(
        self,
        robot: pin.RobotWrapper,
        target_frame: str,
        config_path: Path,
        eps: float = 0.001,
        max_iter: int = 100,
        damping: float = 0.1
    ):
        """
        Args:
            robot: Pinocchio robot wrapper instance
            target_frame: Target frame name for IK
            config_path: Path to joint limits configuration file
            eps: Convergence threshold
            max_iter: Maximum iterations
            damping: Damping factor for least squares
        """
        self.robot = robot
        self.frame_id = robot.model.getFrameId(target_frame)
        self.eps = eps
        self.max_iter = max_iter
        self.damping = damping
        self.nq = robot.model.nq
        self.nv = robot.model.nv
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            self.joint_limits = config['joint_limits']

    def _enforce_joint_limits(self, q: np.ndarray) -> np.ndarray:
        """
        Args:
            q: Joint configuration vector
        Returns:
            Configuration vector with enforced joint limits
        """
        q_limited = q.copy()
        for joint_name, limits in self.joint_limits.items():
            joint_id = self.robot.model.getJointId(joint_name)
            if joint_id != 0:
                q_limited[joint_id] = np.clip(q[joint_id], limits[0], limits[1])
        return q_limited

    def _synchronize_finger_joints(self, q: np.ndarray) -> np.ndarray:
        """
        Args:
            q: Joint configuration vector
        Returns:
            Configuration vector with synchronized joints 3 and 4 for each finger
        """
        q_sync = q.copy()
        fingers = ['r_f_joint1', 'r_f_joint2', 'r_f_joint3', 'r_f_joint4', 'r_f_joint5']
        
        for finger in fingers:
            j3_id = self.robot.model.getJointId(f'{finger}_3')
            j4_id = self.robot.model.getJointId(f'{finger}_4')
            
            if j3_id != 0 and j4_id != 0:
                avg_angle = (q_sync[j3_id] + q_sync[j4_id]) / 2.0
                j3_limits = self.joint_limits[f'{finger}_3']
                j4_limits = self.joint_limits[f'{finger}_4']
                
                min_limit = max(j3_limits[0], j4_limits[0])
                max_limit = min(j3_limits[1], j4_limits[1])
                
                q_sync[j3_id] = q_sync[j4_id] = np.clip(avg_angle, min_limit, max_limit)
        
        return q_sync

    def solve(
        self,
        target_pos: Position,
        initial_q: Optional[np.ndarray] = None,
        orientation_weight: float = 0.0
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve inverse kinematics
        
        Args:
            target_pos: Target end-effector position
            initial_q: Initial joint configuration
            orientation_weight: Weight for orientation constraint
        Returns:
            Tuple of (final joint configuration, success flag)
        """
        try:
            if initial_q is None:
                q = pin.neutral(self.robot.model)
            else:
                q = initial_q.copy()
                
            target = target_pos.to_array()
            
            for i in range(self.max_iter):
                pin.forwardKinematics(self.robot.model, self.robot.data, q)
                pin.updateFramePlacements(self.robot.model, self.robot.data)
                
                current_pos = self.robot.data.oMf[self.frame_id].translation
                err = target - current_pos
                
                if np.linalg.norm(err) < self.eps:
                    q = self._enforce_joint_limits(q)
                    q = self._synchronize_finger_joints(q)
                    return q, True
                    
                J = pin.computeFrameJacobian(
                    self.robot.model, 
                    self.robot.data,
                    q,
                    self.frame_id
                )[:3]
                
                JJt = J @ J.T
                lambda2 = self.damping ** 2
                v = J.T @ np.linalg.solve(JJt + lambda2 * np.eye(3), err)
                
                q = pin.integrate(self.robot.model, q, v)
                q = self._enforce_joint_limits(q)
                q = self._synchronize_finger_joints(q)
                
            return q, False

        except Exception as e:
            print(f"IK solver error: {str(e)}")
            return pin.neutral(self.robot.model), False
