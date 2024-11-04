import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go

# Load the robot states log and rope states log from CSV files
robot_states_file = "robot_states_log.csv"
rope_states_file = "rope_states_log.csv"

# Load CSV files into pandas DataFrames
robot_df = pd.read_csv(robot_states_file)
rope_df = pd.read_csv(rope_states_file)

# Visualization for Robot States
def visualize_robot_states():
    # Plot the joint angles over time
    joint_columns = [col for col in robot_df.columns if "joint" in col]
    time = robot_df['time']
    
    plt.figure(figsize=(10, 6))
    for joint in joint_columns:
        plt.plot(time, robot_df[joint], label=joint)
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angles (radians)")
    plt.title("Robot Joint Angles Over Time")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot the end-effector trajectory in 3D space
    fig = go.Figure(data=[go.Scatter3d(
        x=robot_df['end_effector_x'],
        y=robot_df['end_effector_y'],
        z=robot_df['end_effector_z'],
        mode='lines',
        line=dict(color='blue', width=2)
    )])
    fig.update_layout(
        title='End-Effector Trajectory',
        scene=dict(
            xaxis_title='X Position (m)',
            yaxis_title='Y Position (m)',
            zaxis_title='Z Position (m)'
        )
    )
    fig.show()

# Visualization for Rope States
def visualize_rope_states():
    time = rope_df['time']
    
    # Plot positions of rope segments over time (in 2D plots)
    segment_positions_x = [col for col in rope_df.columns if "_x" in col and "segment" in col]
    segment_positions_y = [col for col in rope_df.columns if "_y" in col and "segment" in col]
    segment_positions_z = [col for col in rope_df.columns if "_z" in col and "segment" in col]

    plt.figure(figsize=(10, 6))
    for segment in segment_positions_z:
        plt.plot(time, rope_df[segment], label=segment)
    plt.xlabel("Time (s)")
    plt.ylabel("Z Position (m)")
    plt.title("Rope Segment Z Positions Over Time")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plot 3D positions of all segments over time to visualize how the rope moves
    fig = go.Figure()
    for i in range(len(segment_positions_x)):
        fig.add_trace(go.Scatter3d(
            x=rope_df[segment_positions_x[i]],
            y=rope_df[segment_positions_y[i]],
            z=rope_df[segment_positions_z[i]],
            mode='lines',
            name=f'Segment {i}'
        ))
    
    fig.update_layout(
        title='Rope Segments Trajectory',
        scene=dict(
            xaxis_title='X Position (m)',
            yaxis_title='Y Position (m)',
            zaxis_title='Z Position (m)'
        )
    )
    fig.show()

    # Heatmap of rope segment velocities to visualize movement intensity over time
    segment_velocities = [col for col in rope_df.columns if "_v" in col and "segment" in col]
    velocities_data = rope_df[segment_velocities].to_numpy()
    
    plt.figure(figsize=(10, 6))
    plt.imshow(velocities_data.T, aspect='auto', cmap='hot', extent=[time.min(), time.max(), 0, len(segment_velocities)])
    plt.colorbar(label='Velocity Magnitude (m/s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Rope Segments')
    plt.title('Rope Segment Velocities Heatmap')
    plt.show()

# Main function to visualize both robot and rope states
if __name__ == "__main__":
    visualize_robot_states()
    visualize_rope_states()
