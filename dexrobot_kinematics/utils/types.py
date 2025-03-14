from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple, Union
import numpy as np
import pinocchio as pin

@dataclass
class Position:
    """3D position representation with x, y, z coordinates"""
    x: float
    y: float
    z: float

    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([self.x, self.y, self.z])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'Position':
        """Create from numpy array"""
        return cls(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

@dataclass
class Pose:
    """Represents position and orientation"""
    position: Position
    orientation: Optional[np.ndarray] = None  # 3x3 rotation matrix

    def to_se3(self) -> pin.SE3:
        """Convert to pinocchio SE3"""
        R = self.orientation if self.orientation is not None else np.eye(3)
        return pin.SE3(R, self.position.to_array())

    @classmethod
    def from_se3(cls, se3: pin.SE3) -> 'Pose':
        """Create from pinocchio SE3"""
        return cls(
            position=Position.from_array(se3.translation),
            orientation=se3.rotation
        )

    def to_quaternion(self) -> np.ndarray:
        """Convert orientation to quaternion"""
        return pin.rpy.matrixToQuaternion(self.orientation)
