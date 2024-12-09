@dataclass
class KinematicsConfig:
    left_hand_urdf: Path
    right_hand_urdf: Path
    arm_urdf: Optional[Path] = None
    arm_mount_frame: Optional[str] = None

    @classmethod
    def from_yaml(cls, config_path: str) -> 'KinematicsConfig':
        """Load config from YAML file"""
        pass
