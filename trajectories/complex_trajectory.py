# Updated import using relative import
from .trajectory_base import TrajectoryBase
import numpy as np

class ComplexTrajectory(TrajectoryBase):
    def __init__(self, params):
        super().__init__(params)

    def get_target_position(self, t):
        x_amplitude = self.params['x_amplitude'] * (1 + 0.1 * np.sin(0.1 * t))  # Time-varying amplitude
        y_amplitude = self.params['y_amplitude']
        z_amplitude = self.params['z_amplitude']
        x_frequency = self.params['x_frequency'] + 0.05 * t  # Frequency increases over time

        return [
            x_amplitude * np.sin(2 * np.pi * x_frequency * t),
            y_amplitude * np.cos(2 * np.pi * self.params['y_frequency'] * t),
            z_amplitude
        ]
