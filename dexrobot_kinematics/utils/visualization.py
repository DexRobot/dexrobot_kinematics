import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Dict, List, Optional
from ..utils.types import Position, Pose, JointAngles

class HandVisualizer:
    """Visualizer for hand kinematics"""
    
    def __init__(self, robot: pin.RobotWrapper):
        self.robot = robot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
    def plot_frame(self, pose: Pose, scale: float = 0.05):
        """Plot coordinate frame"""
        origin = pose.position.to_array()
        if pose.orientation is not None:
            for i in range(3):
                direction = pose.orientation[:, i]
                self.ax.quiver(
                    origin[0], origin[1], origin[2],
                    direction[0], direction[1], direction[2],
                    length=scale,
                    color=['r', 'g', 'b'][i]
                )
    
    def plot_hand(
        self,
        joint_angles: JointAngles,
        finger_poses: Optional[Dict[str, Pose]] = None,
        target_positions: Optional[Dict[str, Position]] = None
    ):
        """Plot hand configuration with optional targets"""
        # Clear previous plot
        self.ax.clear()
        
        # Update robot configuration
        q = np.zeros(self.robot.model.nq)
        for joint, angle in joint_angles.angles.items():
            idx = self.robot.model.getJointId(joint)
            q[idx] = angle
            
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        
        # Plot joint frames
        for frame_id in range(self.robot.model.nframes):
            pose = Pose.from_se3(self.robot.data.oMf[frame_id])
            self.plot_frame(pose, scale=0.02)
            
        # Plot finger poses if provided
        if finger_poses:
            for finger, pose in finger_poses.items():
                self.plot_frame(pose, scale=0.03)
                
        # Plot target positions if provided
        if target_positions:
            for finger, pos in target_positions.items():
                self.ax.scatter(
                    pos.x, pos.y, pos.z,
                    color='r', marker='*', s=100
                )
                
        # Set plot properties
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y') 
        self.ax.set_zlabel('Z')
        self.ax.set_box_aspect([1,1,1])
        plt.draw()
        
    def show(self):
        """Display the plot"""
        plt.show()
        
    def save(self, filename: str):
        """Save plot to file"""
        plt.savefig(filename)
