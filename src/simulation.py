import pybullet as p
import pybullet_data
import time
import yaml
from src.robot_control import RobotControl
from src.rope_setup import RopeSetup
from trajectories.trajectory_base import TrajectoryBase

class Simulation:
    def __init__(self, config_file):
        # Load configuration
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        
        # Connect to the simulation
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load URDF files

        # Set gravity
        p.setGravity(*self.config['simulation']['gravity'])

        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Initialize RobotControl instance
        self.robot_control = RobotControl(self.config['robot'])

    def get_robot_control(self):
        """
        Return the RobotControl instance.
        """
        return self.robot_control

    def run(self, trajectory: TrajectoryBase):
        # Move the end-effector to the desired initial position
        initial_end_effector_position = [0.5, 0.0, 0.6]
        self.robot_control.move_end_effector(initial_end_effector_position)

        # Let the robot stabilize at the initial position
        for _ in range(100):
            p.stepSimulation()
            time.sleep(self.config['simulation']['time_step'] / 10)

        # Set up the rope, attach to the robot, and stabilize
        rope_setup = RopeSetup(self.config['rope'], self.robot_control.robot_id, self.robot_control.end_effector_index)
        rope_setup.create_rope(initial_end_effector_position)
        rope_setup.add_segment_constraints()
        rope_setup.add_ghost_anchor(initial_end_effector_position)

        # Allow time for stabilization with the ghost anchor in place
        stabilization_time = 2.0
        for _ in range(int(stabilization_time / self.config['simulation']['time_step'])):
            p.stepSimulation()
            time.sleep(self.config['simulation']['time_step'] / 10)

        # Remove the ghost anchor
        rope_setup.remove_ghost_anchor()

        # Run the trajectory to stimulate the rope and collect data
        t = 0
        dt = self.config['simulation']['time_step']

        while t < self.config['simulation']['duration']:
            # Get relative offset from the trajectory
            offset = trajectory.get_offset(t)

            # Get the current end-effector position
            current_position, _ = p.getLinkState(self.robot_control.robot_id, self.robot_control.end_effector_index)[:2]

            # Calculate the absolute target position based on the current position and the offset
            target_position = [
                current_position[0] + offset[0],
                current_position[1] + offset[1],
                current_position[2] + offset[2]
            ]

            # Calculate the difference to determine the direction and distance to move
            diff_position = [
                target_position[0] - current_position[0],
                target_position[1] - current_position[1],
                target_position[2] - current_position[2]
            ]

            # Apply a scaling factor based on dt to move incrementally towards the target
            move_step = [
                current_position[0] + diff_position[0] * dt,
                current_position[1] + diff_position[1] * dt,
                current_position[2] + diff_position[2] * dt
            ]

            # Move the end-effector incrementally to the new target position
            self.robot_control.move_end_effector(move_step)

            # Step the simulation and update the time
            p.stepSimulation()
            time.sleep(dt)
            t += dt

    def cleanup(self):
        p.disconnect()

# Example usage (replace this with appropriate script or command-line interface):
if __name__ == "__main__":
    config_file = "config/experiment_1.yaml"
    from trajectories.sinusoidal_trajectory import SinusoidalTrajectory

    # Load configuration and create trajectory
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    # Create the trajectory
    trajectory = SinusoidalTrajectory(config['trajectory'])

    # Initialize and run simulation
    sim = Simulation(config_file)
    sim.run(trajectory)
    sim.cleanup()
