import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class MocopiVisualizer:
    """Visualizer for Mocopi motion capture data in 3D."""

    # Define the skeleton connections for Mocopi
    SKELETON_LINKS = [
        # Torso
        (0, 3),   # pelvis to spine
        (3, 6),   # spine to chest
        (6, 9),   # chest to upper_chest
        (9, 12),  # upper_chest to neck
        (12, 15), # neck to head

        # Left arm
        (9, 13),  # upper_chest to left_shoulder
        (13, 16), # left_shoulder to left_elbow
        (16, 18), # left_elbow to left_wrist
        (18, 20), # left_wrist to left_hand

        # Right arm
        (9, 14),  # upper_chest to right_shoulder
        (14, 17), # right_shoulder to right_elbow
        (17, 19), # right_elbow to right_wrist
        (19, 21), # right_wrist to right_hand

        # Left leg
        (0, 1),   # pelvis to left_hip
        (1, 4),   # left_hip to left_knee
        (4, 7),   # left_knee to left_ankle
        (7, 10),  # left_ankle to left_toe

        # Right leg
        (0, 2),   # pelvis to right_hip
        (2, 5),   # right_hip to right_knee
        (5, 8),   # right_knee to right_ankle
        (8, 11),  # right_ankle to right_toe
    ]

    def __init__(self):
        """Initialize the visualizer."""
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Enable interactive mode

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

# Example usage:
def run_visualization(receiver):
    """Run visualization with a Mocopi receiver."""
    visualizer = MocopiVisualizer()

    try:
        while True:
            if not receiver.queue.empty():
                pose_data = receiver.queue.get()
                visualizer.visualize_frame(pose_data)
    except KeyboardInterrupt:
        print("Visualization stopped by user")
    finally:
        visualizer.close()

# To use:
if __name__ == "__main__":
    receiver = Receiver(addr="192.168.0.110", port=12351)
    receiver.start()
    run_visualization(receiver)