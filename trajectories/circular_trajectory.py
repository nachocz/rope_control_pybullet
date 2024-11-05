import numpy as np

class CircularTrajectory:
    def __init__(self, trajectory_params, frequency=0.5, center=[0.0, 0.0]):
        self.amplitude = trajectory_params['x_amplitude']
        self.frequency = frequency
        self.center = center

    def get_target_position(self, time):
        """
        Calculate the target position in the circular trajectory at a given time.
        
        Args:
            time (float): The time point for which to calculate the position.
        
        Returns:
            list: Target (x, y, z) position for the rope's loose end in the circular trajectory.
        """
        x_target = self.center[0] + self.amplitude * np.cos(2 * np.pi * self.frequency * time)
        y_target = self.center[1] + self.amplitude * np.sin(2 * np.pi * self.frequency * time)
        return [x_target, y_target, 0.0]  # Only x and y are controlled; z can be set to 0 or any fixed value
