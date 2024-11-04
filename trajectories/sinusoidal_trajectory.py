# Updated import using relative import
from .trajectory_base import TrajectoryBase
import numpy as np

class SinusoidalTrajectory(TrajectoryBase):
    def __init__(self, params):
        super().__init__(params)

    def get_offset(self, t):
        # Retrieve amplitude and frequency from parameters
        x_amplitude = self.params['x_amplitude']
        y_amplitude = self.params['y_amplitude']
        z_amplitude = self.params['z_amplitude']
        x_frequency = self.params['x_frequency']
        y_frequency = self.params['y_frequency']
        z_frequency = self.params['z_frequency']

        # Calculate the incremental sinusoidal offset
        offset = [
            x_amplitude * np.sin(2 * np.pi * x_frequency * t),
            y_amplitude * np.sin(2 * np.pi * y_frequency * t),
            z_amplitude * np.sin(2 * np.pi * z_frequency * t)
        ]

        return offset
