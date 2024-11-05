import yaml
from src.simulation import Simulation
from trajectories.sinusoidal_trajectory import SinusoidalTrajectory

if __name__ == "__main__":
    # Specify the path to the configuration file
    config_file = "config/experiment_1.yaml"

    # Load the configuration from the YAML file
    try:
        with open(config_file, "r") as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: Config file not found at {config_file}")
        exit(1)

    # Create the SinusoidalTrajectory object with the parameters from the configuration
    trajectory = SinusoidalTrajectory(config["trajectory"])

    # Initialize the Simulation with the configuration file path
    sim = Simulation(config_file)

    try:
        # Run the simulation with the specified trajectory
        sim.run(trajectory)
    finally:
        # Cleanup after the simulation
        sim.cleanup()
