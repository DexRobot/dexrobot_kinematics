def transform_points(
    points: Union[Position, Dict[str, Position]],
    transform: Pose
) -> Union[Position, Dict[str, Position]]:
    """Transform points by SE3 pose"""
    pass

def compute_optimal_hand_pose(
    finger_targets: FingerTargets,
    approach_axis: Optional[np.ndarray] = None
) -> Pose:
    """Compute optimal hand pose for given targets"""
    pass
