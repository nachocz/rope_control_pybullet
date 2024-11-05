import numpy as np
import pandas as pd
import yaml
from pydmd import DMDc
import os

def load_config(config_path):
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def load_data(robot_log_path, rope_log_path):
    robot_data = pd.read_csv(robot_log_path)
    rope_data = pd.read_csv(rope_log_path)
    return robot_data, rope_data

def preprocess_data(robot_data, rope_data):
    data = pd.merge_asof(robot_data, rope_data, on='time', direction='nearest')
    state_vars = data.filter(regex='segment_.*_[xyz]').values.T

    # Compute control inputs as increments between successive positions
    end_effector_positions = data[['end_effector_x', 'end_effector_y', 'end_effector_z']].values.T
    control_inputs = np.diff(end_effector_positions, axis=1)  # Incremental differences

    return state_vars, control_inputs

def perform_dmdc(state_vars, control_inputs, B_full, svd_rank=-1):
    # Prepare snapshot matrices
    X = state_vars # N columns, pydmd will split it into X and Xnext properly
    U = control_inputs  # Already N-1 columns after the np.diff operation

    print("X shape:", X.shape)
    print("U shape:", U.shape)
    print("B shape:", B_full.shape)

    # Instantiate DMDc with specified svd_rank for truncation
    dmdc = DMDc(svd_rank=svd_rank)
    dmdc.fit(X, U, B=B_full)

    # Retrieve reduced-order A matrix and modes
    A_reduced = dmdc.operator._Atilde
    Phi = dmdc.modes  # Full-state projection matrix (eigenmodes of DMDc)

    print("A_reduced shape:", A_reduced.shape)
    print("Phi shape:", Phi.shape)

    # Project reduced-order A matrix back to full-state space
    A_full = Phi @ A_reduced @ np.linalg.pinv(Phi)
    print("A_full shape:", A_full.shape)

    return A_full

def create_control_matrix(robot_data, rope_data, config):
    num_segments = config['rope']['num_segments']
    control_dim = 3
    B = np.zeros((num_segments * 3, control_dim))
    B[:control_dim, :control_dim] = np.eye(control_dim)
    return B

def save_matrices(A, B, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    np.save(f'{output_dir}/A_matrix.npy', A)
    np.save(f'{output_dir}/B_matrix.npy', B)

if __name__ == "__main__":
    config_path = 'config/experiment_1.yaml'
    config = load_config(config_path)
    
    robot_log_path = config['logging']['robot_log_path']
    rope_log_path = config['logging']['rope_log_path']
    output_dir = config['output']['directory']
    
    svd_rank = config['dmdc'].get('svd_rank', -1)
    if isinstance(svd_rank, str):
        svd_rank = float(svd_rank) if '.' in svd_rank else int(svd_rank)
    
    robot_data, rope_data = load_data(robot_log_path, rope_log_path)
    state_vars, control_inputs = preprocess_data(robot_data, rope_data)
    
    B_matrix = create_control_matrix(robot_data, rope_data, config)
    
    A_full = perform_dmdc(state_vars, control_inputs, B_matrix, svd_rank)
    
    save_matrices(A_full, B_matrix, output_dir)
