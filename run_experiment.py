import pybullet as p
import pybullet_data
import time
import yaml
from src.robot_control import RobotControl
from src.rope_setup import RopeSetup
from trajectories.sinusoidal_trajectory import SinusoidalTrajectory
from src.logger import Logger

# Load configuration file
with open('config/experiment_1.yaml', 'r') as file:
    config = yaml.safe_load(file)

# Initialize PyBullet simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(*config['simulation']['gravity'])
p.setRealTimeSimulation(0)  # 0 for off (manual stepping)

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Initialize RobotControl and Logger
robot_control = RobotControl(config['robot'])
logger = Logger(config)

# Read the initial end-effector position
initial_end_effector_position = [0.4, 0.0, 0.6]
robot_control.move_end_effector_position(initial_end_effector_position)

# Stabilize initial position
for _ in range(100):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

# Set up rope and stabilize
rope_setup = RopeSetup(config['rope'], robot_control.robot_id, robot_control.end_effector_index)
rope_setup.create_rope(initial_end_effector_position)
rope_setup.add_segment_constraints()
rope_setup.add_ghost_anchor(initial_end_effector_position)

# Stabilization with ghost anchor
stabilization_time = 2.0
for _ in range(int(stabilization_time / config['simulation']['time_step'])):
    p.stepSimulation()
    time.sleep(config['simulation']['time_step'] / 10)

rope_setup.remove_ghost_anchor()

# Define the sinusoidal trajectory
trajectory = SinusoidalTrajectory(config['trajectory'])

# Main simulation loop
t = 0
dt = config['simulation']['time_step']
simulation_duration = config['simulation']['duration']

while t < simulation_duration:
    # Get the sinusoidal target position at time t
    sinusoidal_offset = trajectory.get_target_position(t)
    
    # Calculate the absolute target position by adding offset to initial position
    target_position = [
        initial_end_effector_position[i] + sinusoidal_offset[i] for i in range(3)
    ]
    
    # Log target position details
    print(f"Time: {t:.2f}s, Sinusoidal Offset: {sinusoidal_offset}, Target Position: {target_position}")

    # Move the end effector to the computed target position
    robot_control.move_end_effector_position(target_position)

    # Step the simulation
    p.stepSimulation()

    # Log the actual end-effector position after movement
    actual_position = robot_control.get_end_effector_position()
    print(f"Time: {t:.2f}s, Actual End Effector Position: {actual_position}")

    # Log data
    logger.log(t, robot_control, rope_setup)

    # Increment time
    time.sleep(dt)
    t += dt


# Disconnect PyBullet
p.disconnect()
