class HandIKSolver:
    """Pinocchio-based IK solver adapted for hand"""

    def __init__(
        self,
        robot: pin.RobotWrapper,
        target_frame: str,
        eps: float = 0.02,
        max_iter: int = 100
    ):
        self.robot = robot
        self.frame_id = robot.model.getFrameId(target_frame)
        self.eps = eps
        self.max_iter = max_iter

    def solve(
        self,
        target_pos: Position,
        initial_q: Optional[np.ndarray] = None,
        orientation_weight: float = 0.0
    ) -> Tuple[np.ndarray, bool]:
        """Solve IK to target position with optional orientation constraint"""
        pass
