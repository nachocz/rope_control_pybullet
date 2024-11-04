# Cable Control Project

This repository contains a simulation-based project aimed at controlling a rope suspended from a robotic arm (Franka Panda), using a PyBullet simulation environment. The goal is to explore control strategies such as Dynamic Mode Decomposition with control (DMDc) and Reinforcement Learning (RL) to control the movement of the rope's loose end.

## Overview

In this project, we aim to develop and compare control strategies for a robot arm manipulating a rope. The robot holds the rope from one end, while the other end is free. As the robot moves, the rope moves naturally. The objectives are:

- To generate and evaluate control strategies for the dynamics and trajectory of the rope.
- To develop modular scripts for different control strategies including reinforcement learning and model-based approaches.
- To ensure a robust environment for simulation using PyBullet and Python.

## Project Structure

This repository is organized into the following directories and files:

- **`config/`**: Contains YAML configuration files used to set various simulation parameters like robot initial configuration, rope properties, and control strategies.

- **`logs/`**: Directory for saving the simulation logs including the robot states and rope states in CSV format.

- **`src/`**: Contains the core Python scripts for running the simulation, controlling the robot, and setting up the rope:
  - `robot_control.py`: Manages the robot's movements and controls the gripper.
  - `rope_setup.py`: Manages the creation and stabilization of the hanging rope.
  - `simulation.py`: Orchestrates the entire simulation, including robot initialization, trajectory following, and data logging.

- **`trajectories/`**: Includes different trajectory generation classes that the robot can follow:
  - `trajectory_base.py`: The base class for defining robot trajectories.
  - `sinusoidal_trajectory.py`: Implements a sinusoidal trajectory for the end effector.

- **`visualization/`**: Contains scripts for visualizing the robot and rope data:
  - `visualize_logs.py`: Uses matplotlib and Plotly to visualize the logged data.

- **`.gitignore`**: Specifies files and folders to be ignored by Git, such as logs, temporary files, and CSV data.

- **`README.md`**: Provides an overview of the project, its objectives, and usage instructions.

## Getting Started

### Prerequisites

Ensure that you have the following installed on your machine:

- **Python 3.8+**
- **PyBullet**: Physics simulation for robotics research
- **Git**: For version control
- **Other Python packages**: You can install the required Python packages using the following command:
  
  ```sh
  pip install -r requirements.txt
  ```

### Setting Up the Repository

1. Clone the repository to your local machine:
   ```sh
   git clone https://github.com/YOUR_USERNAME/cable_control_project.git
   cd cable_control_project
   ```
2. Ensure that you have created a `config/experiment_1.yaml` file, or use the existing example file to configure the parameters for the experiment.

### Running the Simulation

To run an experiment with a predefined trajectory:

```sh
python run_experiment.py --config config/experiment_1.yaml --trajectory sinusoidal
```

This script will run the simulation using the parameters defined in `experiment_1.yaml` and save the results to the `logs/` directory.

### Visualization

After running the experiment, you can visualize the data using:

```sh
python visualization/visualize_logs.py
```

This will generate plots for the robot's trajectory, joint angles, and rope dynamics over time.

## Future Work

- **Implementation of Reinforcement Learning Control**: Develop and compare RL-based control strategies for dynamic manipulation.
- **Advanced Trajectories**: Implement more complex trajectories for testing the control robustness.
- **Optimization**: Tune control parameters to achieve better performance and energy efficiency.

## Contributing

Contributions are welcome! Please feel free to open issues or submit pull requests to improve the project.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- **PyBullet**: For providing a powerful physics simulation engine for robotics.
- **OpenAI and Community Contributors**: For support and inspiration in developing control strategies for robotic systems.

