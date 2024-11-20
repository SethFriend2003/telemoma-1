import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import threading
from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopObservation, TeleopAction
import socket
import struct
from human_interface.runner import Runner
from telemoma.utils.general_utils import run_threaded_command
import time
import queue

class MocopiVisualizer:
    """Visualizer for Mocopi motion capture data in 3D."""

    # Define the skeleton connections for Mocopi
    SKELETON_LINKS = [
        # Torso

        (0, 6),   # pelvis to collarbone
        (6, 10),   # collarbone to head

        # Left arm
        (11, 12),  # left shoulder to upper arm
        (12, 13),  #upper arm to elbow
        (13, 14), # elbow to hand


        # Right arm
        (15, 16),  # right shoulder to upper arm
        (16, 17), # upper arm to elbow
        (17, 18), # elbow to hand
    

        # Left leg
        (0, 19),   # pelvis to left_hip
        (19, 20),   # left_hip to left_knee
        (20, 21),   # left_knee to left_ankle
        (21, 22),  # left_ankle to left_toe

        # Right leg
        (0, 23),   # pelvis to right_hip
        (23, 24),   # right_hip to right_knee
        (24, 25),   # right_knee to right_ankle
        (25, 26),  # right_ankle to right_toe
    ]

    def __init__(self):
        """Initialize the visualizer."""
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Enable interactive mode

    def start(self):
        self.ax.clear()
        plt.show()

    def extract_bone_positions(self, pose_data):
        """Extract bone positions from Mocopi pose data."""
        bone_positions = {}

        if "fram" in pose_data:
            for bone in pose_data["fram"]["btrs"]:
                bone_id = bone["bnid"]
                # Get position from translation data (first 3 values)
                position = np.array(bone["tran"][:3])
                bone_positions[bone_id] = position

        return bone_positions

    def visualize_frame(self, pose_data):
        """Visualize a single frame of Mocopi data."""
        self.ax.clear()

        # Set axis limits and labels
        self.ax.set_xlim3d(-1.0, 1.0)
        self.ax.set_ylim3d(-1.0, 1.0)
        self.ax.set_zlim3d(-0.5, 1.5)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Extract bone positions
        bone_positions = self.extract_bone_positions(pose_data)

        # Draw skeleton links
        for link in self.SKELETON_LINKS:
            if link[0] in bone_positions and link[1] in bone_positions:
                start_pos = bone_positions[link[0]]
                end_pos = bone_positions[link[1]]

                self.ax.plot3D(
                    [start_pos[0], end_pos[0]],
                    [start_pos[1], end_pos[1]],
                    [start_pos[2], end_pos[2]],
                    'b-'  # Blue lines for skeleton
                )

        # Plot joint positions
        for bone_id, position in bone_positions.items():
            self.ax.scatter(
                position[0],
                position[1],
                position[2],
                c='r',  # Red dots for joints
                marker='o',
                s=50
            )

        # Add frame number and timestamp if available
        if "fram" in pose_data:
            self.ax.set_title(f"Frame: {pose_data['fram']['fnum']} Time: {pose_data['fram']['time']}")

        plt.draw()
        plt.pause(0.01)

    def close(self):
        """Close the visualization window."""
        plt.close(self.fig)
