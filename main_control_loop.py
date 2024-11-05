import os
import pybullet as p
import pybullet_data
import time
import yaml
import numpy as np
from src.robot_control import RobotControl
from src.rope_setup import RopeSetup
from src.logger import Logger
from src.controllers.mpc_controller import mpc_control
from trajectories.circular_trajectory import CircularTrajectory
from src.analysis.state_matrix_loader import load_dmdc_matrices
from datetime import datetime

# Load configuration file
with open('config/experiment_1.yaml', 'r') as file:
    config = yaml.safe_load(file)

# Generate a unique directory for this data collection run
data_log_dir = os.path.join('logs/data_collection', datetime.now().strftime('%Y%m%d_%H%M%S'))
os.makedirs(data_log_dir, exist_ok=True)

# Initialize PyBullet simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(*config['simulation']['gravity'])
p.setRealTimeSimulation(0)

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Initialize RobotControl, Logger, and RopeSetup
robot_control = RobotControl(config['robot'])
logger = Logger(config)
rope_setup = RopeSetup(config['rope'], robot_control.robot_id, robot_control.end_effector_index)

# Define and log initial end-effector position
initial_end_effector_position = [0.4, 0.0, 0.6]
robot_control.move_end_effector_position(initial_end_effector_position)
print("Initial End Effector Position:", initial_end_effector_position)

# Stabilize initial position
for _ in range(100):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

# Set up rope and stabilize
rope_setup.create_rope(initial_end_effector_position)
rope_setup.add_segment_constraints()
rope_setup.add_ghost_anchor(initial_end_effector_position)
stabilization_time = 2.0
for _ in range(int(stabilization_time / config['simulation']['time_step'])):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)
rope_setup.remove_ghost_anchor()

# Load matrices A and B and log their details
A, B = load_dmdc_matrices(config['output']['directory'])
print("Loaded A matrix shape:", A.shape)
print("Loaded A matrix:\n", A)
print("Loaded B matrix shape:", B.shape)
print("Loaded B matrix:\n", B)

# Define C matrix to select x and y states of the rope's last segment
C = np.zeros((2, A.shape[0]))
C[0, -3] = 1  # Select x of last segment
C[1, -2] = 1  # Select y of last segment
print("Defined C matrix shape:", C.shape)
print("Defined C matrix:\n", C)

# Define the circular trajectory parameters
center = [initial_end_effector_position[0], initial_end_effector_position[1]]
frequency = 0.5  # 0.5 Hz frequency for the circular motion

# Initialize the CircularTrajectory with center and frequency
trajectory = CircularTrajectory(config['trajectory'], frequency=frequency, center=center)


# Main control loop
t = 0
dt = config['simulation']['time_step']
simulation_duration = config['simulation']['duration']

while t < simulation_duration:
    # Get the target position in a circular trajectory at time t
    target_position = trajectory.get_target_position(t)[:2]  # Only x and y for control
    print(f"Target position at time {t:.2f}s:", target_position)

    # Get the initial state of the rope
    x0 = np.array(rope_setup.get_rope_state())
    print("Initial rope state (x0):", x0)

    # Compute control input using MPC
    control_input = mpc_control(A, B, C, x0, target_position)
    print("Computed control input:", control_input)

    # Move the robot's end effector using the computed control input
    end_effector_target = [control_input[0], control_input[1], initial_end_effector_position[2]]
    robot_control.move_end_effector_position(end_effector_target)

    # Step the simulation
    p.stepSimulation()

    # Log data
    logger.log(t, robot_control, rope_setup)

    # Increment time
    time.sleep(dt)
    t += dt

# Disconnect PyBullet
p.disconnect()
