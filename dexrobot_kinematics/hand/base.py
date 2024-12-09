
class HandKinematicsBase:
    """Base class for hand kinematics"""

    def __init__(self, urdf_path: Path):
        self.robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path))
        self._init_frames()

    def _init_frames(self):
        """Initialize frame IDs and transforms"""
        pass

    def forward_kinematics(
        self,
        joint_angles: JointAngles,
        base_pose: Optional[Pose] = None,
        frame: str = 'hand'
    ) -> Dict[str, Pose]:
        """Compute fingertip poses from joint angles"""
        pass

    def inverse_kinematics_finger(
        self,
        finger: str,
        target_pos: Position,
        frame: str = 'hand',
        initial_guess: Optional[JointAngles] = None
    ) -> Tuple[JointAngles, bool]:
        """Solve IK for single finger"""
        pass

    def inverse_kinematics_grasp(
        self,
        finger_targets: FingerTargets,
        frame: str = 'hand',
        initial_guess: Optional[JointAngles] = None
    ) -> Tuple[JointAngles, bool]:
        """Solve IK for multiple fingers"""
        pass
