import pybullet as p
import pybullet_data
import time
import yaml
from src.robot_control import RobotControl
from src.rope_setup import RopeSetup
from trajectories.sinusoidal_trajectory import SinusoidalTrajectory
from src.logger import Logger

# Load config file
with open('config/experiment_1.yaml', 'r') as file:
    config = yaml.safe_load(file)

# Initialize PyBullet simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load URDF files
p.resetSimulation()
p.setGravity(*config['simulation']['gravity'])

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Initialize RobotControl instance
robot_control = RobotControl(config['robot'])

# Set up Logger
logger = Logger(config)

# Move the end-effector to the initial position
initial_end_effector_position = [0.4, 0.0, 0.6]  # Initial position
robot_control.move_end_effector(initial_end_effector_position)

# Let the robot stabilize at the initial position
for _ in range(100):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

# Set up the rope, attach to the robot, and stabilize
rope_setup = RopeSetup(config['rope'], robot_control.robot_id, robot_control.end_effector_index)
rope_setup.create_rope(initial_end_effector_position)
rope_setup.add_segment_constraints()
rope_setup.add_ghost_anchor(initial_end_effector_position)

# Allow time for stabilization with the ghost anchor in place
stabilization_time = 2.0
for _ in range(int(stabilization_time / config['simulation']['time_step'])):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

# Remove the ghost anchor
rope_setup.remove_ghost_anchor()

# Define the trajectory to be followed by the robot
trajectory = SinusoidalTrajectory(config['trajectory'])

# Run the simulation for the trajectory while logging data
t = 0
dt = config['simulation']['time_step']
simulation_duration = config['simulation']['duration']
target_position= initial_end_effector_position

while t < simulation_duration:
    # Get incremental target position from the trajectory
    increment = trajectory.get_target_position(t)
    #print(f"Time: {t:.2f}s, Incremental Target Position: x={increment[0]:.4f}, y={increment[1]:.4f}, z={increment[2]:.4f}")

    # Move the end effector incrementally
    current_position = robot_control.get_end_effector_position()
    target_position = [initial_end_effector_position[i] + increment[i] for i in range(3)]

    robot_control.move_end_effector(target_position)

    # Step the simulation
    p.stepSimulation()

    # Log the data
    logger.log(t, robot_control, rope_setup)

    # Increment time step
    time.sleep(dt)
    t += dt

# Cleanup and disconnect
p.disconnect()
