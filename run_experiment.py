import argparse
import yaml
import time
import pybullet as p
import pybullet_data
from src.robot_control import RobotControl
from src.rope_setup import RopeSetup
from trajectories.sinusoidal_trajectory import SinusoidalTrajectory
from src.logger import Logger

# Parse command-line arguments to get config file and trajectory
parser = argparse.ArgumentParser(description="Run rope control experiment")
parser.add_argument('--config', required=True, help="Path to experiment config file")
parser.add_argument('--trajectory', required=True, choices=['sinusoidal', 'complex'], help="Type of trajectory")
args = parser.parse_args()

# Load configuration
with open(args.config, 'r') as file:
    config = yaml.safe_load(file)

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(*config['simulation']['gravity'])
p.loadURDF("plane.urdf")

# Setup robot and rope
robot = RobotControl(config['robot'])
rope = RopeSetup(config['rope'], robot.robot_id, robot.end_effector_index)
initial_end_effector_position = [0.5, 0.0, 0.6]
robot.move_end_effector(initial_end_effector_position)

# Stabilize the robot and rope
for _ in range(100):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

# Create rope and ghost anchor to stabilize
rope.create_rope(initial_end_effector_position)
rope.add_segment_constraints()
rope.add_ghost_anchor(initial_end_effector_position)

# Allow time for stabilization with the ghost anchor
for _ in range(int(2.0 / config['simulation']['time_step'])):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

# Remove the ghost anchor to allow free hanging
rope.remove_ghost_anchor()

# Set trajectory type
if args.trajectory == 'sinusoidal':
    trajectory = SinusoidalTrajectory(config['trajectory'], initial_position=initial_end_effector_position)

# Create a logger instance
logger = Logger(config['data_logging'])

# Start data logging and experiment
t = 0
dt = config['simulation']['time_step']
simulation_duration = config['simulation']['duration']

while t < simulation_duration:
    # Get the target position based on the trajectory
    target_position = trajectory.get_target_position(t)
    
    # Move the end-effector to the target position
    robot.move_end_effector(target_position)
    
    # Step the simulation
    p.stepSimulation()
    
    # Log data for both robot and rope
    logger.log_robot_state(t, robot)
    logger.log_rope_state(t, rope)

    # Increment time
    time.sleep(dt)
    t += dt

# Stop data logging
logger.close_logs()

# Disconnect from PyBullet
p.disconnect()
