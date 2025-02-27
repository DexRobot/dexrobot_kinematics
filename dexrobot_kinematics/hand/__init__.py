from .base import HandKinematicsBase
from typing import Optional
from pathlib import Path


class RightHandKinematics(HandKinematicsBase):
    """Class for right hand kinematics calculations"""

    def __init__(
        self, urdf_path: Optional[Path] = None, config_path: Optional[Path] = None
    ):
        """Initialize right hand kinematics"""
        current_dir = Path(__file__).parent
        if urdf_path is None:
            urdf_path = current_dir / "../../../dexrobot_urdf/urdf/dexhand021_right.urdf"
        if config_path is None:
            config_path = (
                current_dir / "../config/right_hand_default.yaml"
            )
        super().__init__("right", urdf_path, config_path)


class LeftHandKinematics(HandKinematicsBase):
    """Class for left hand kinematics calculations"""

    def __init__(
        self, urdf_path: Optional[Path] = None, config_path: Optional[Path] = None
    ):
        """Initialize left hand kinematics"""
        current_dir = Path(__file__).parent
        if urdf_path is None:
            urdf_path = current_dir / "../../../dexrobot_urdf/urdf/dexhand021_left.urdf"
        if config_path is None:
            config_path = (
                current_dir / "../config/left_hand_default.yaml"
            )
        super().__init__("left", urdf_path, config_path)
