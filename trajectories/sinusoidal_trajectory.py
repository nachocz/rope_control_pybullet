from .trajectory_base import TrajectoryBase
import numpy as np

class SinusoidalTrajectory(TrajectoryBase):
    def __init__(self, params):
        super().__init__(params)
        self.params = params
        self.x_amplitude = params['x_amplitude']  # Total amplitude
        self.y_amplitude = params['y_amplitude']
        self.z_amplitude = params['z_amplitude']
        self.x_frequency = params['x_frequency']
        self.y_frequency = params['y_frequency']
        self.z_frequency = params['z_frequency']

    def get_target_position(self, t):
        # Calculate target offset for each axis at time t using full amplitude
        target_offset = [
            self.x_amplitude * np.sin(2 * np.pi * self.x_frequency * t),
            self.y_amplitude * np.sin(2 * np.pi * self.y_frequency * t),
            self.z_amplitude * np.sin(2 * np.pi * self.z_frequency * t)
        ]

        return target_offset
