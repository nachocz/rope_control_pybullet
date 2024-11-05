import numpy as np
import os

def load_dmdc_matrices(output_dir):
    """Loads A and B matrices from specified directory."""
    A = np.load(os.path.join(output_dir, 'A_matrix.npy'))
    B = np.load(os.path.join(output_dir, 'B_matrix.npy'))
    return A, B
