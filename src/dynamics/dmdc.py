import numpy as np
import pandas as pd
import yaml
import pickle
from scipy.linalg import svd

class DMDc:
    def __init__(self, config_path, robot_log_path, rope_log_path):
        # Load configuration
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        
        # Load robot states and rope states
        self.robot_data = pd.read_csv(robot_log_path)
        self.rope_data = pd.read_csv(rope_log_path)

    def compute_dmdc_model(self):
        # Extract control inputs (U) and system states (X)
        U = self.extract_control_inputs()
        X = self.extract_system_states()

        # Create shifted matrices X1 (initial states) and X2 (states at next time)
        X1 = X[:, :-1]  # All but the last time step
        X2 = X[:, 1:]   # All but the first time step
        
        # Use Singular Value Decomposition to find the reduced-order model
        U_tilde = U[:, :-1]  # Only use until second to last timestep

        # Concatenate X1 and U
        XU = np.vstack((X1, U_tilde))

        # Compute SVD of XU
        U, S, Vh = svd(XU, full_matrices=False)

        # Define truncation limit based on config parameter
        r = self.config['dmdc']['rank_truncation']
        U_r, S_r, V_r = U[:, :r], np.diag(S[:r]), Vh[:r, :]
        
        # Compute the low-rank approximation of A and B
        A = X2 @ V_r.T @ np.linalg.inv(S_r) @ U_r.T
        B = X2 - A @ X1

        return A, B

    def extract_control_inputs(self):
        # Assuming the control inputs are the end-effector positions and velocities
        control_columns = ['end_effector_x', 'end_effector_y', 'end_effector_z',
                           'end_effector_vx', 'end_effector_vy', 'end_effector_vz']
        return self.robot_data[control_columns].values.T

    def extract_system_states(self):
        # Extracting rope segment positions and velocities
        state_columns = [col for col in self.rope_data.columns if "segment" in col]
        return self.rope_data[state_columns].values.T

    def save_model(self, A, B, model_path):
        model = {'A': A, 'B': B}
        with open(model_path, 'wb') as f:
            pickle.dump(model, f)
        print(f"Model saved at {model_path}")

if __name__ == "__main__":
    # Define paths
    config_path = "../config/experiment_dmdc.yaml"
    robot_log_path = "../data/logs/experiment1/robot_states_log.csv"
    rope_log_path = "../data/logs/experiment1/rope_states_log.csv"
    model_path = "../data/models/dmdc_model_exp1.pkl"
    
    # Create DMDc instance
    dmdc = DMDc(config_path, robot_log_path, rope_log_path)
    
    # Compute and save model
    A, B = dmdc.compute_dmdc_model()
    dmdc.save_model(A, B, model_path)
