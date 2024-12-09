from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple, Union
import numpy as np
import pinocchio as pin

# Type aliases
Pose = pin.SE3  # Using pinocchio's SE3 type
Position = np.ndarray  # 3D position vector
Rotation = np.ndarray  # 3x3 rotation matrix
JointAngles = Dict[str, float]
FingerTargets = Dict[str, Position]
