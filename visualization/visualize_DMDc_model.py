import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_matrix_surface(matrix, title="Matrix Surface Plot"):
    """
    Visualizes a matrix as a 3D surface plot.
    
    Args:
    - matrix (numpy.ndarray): The matrix to plot.
    - title (str): Title of the plot.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Create coordinate arrays
    x = np.arange(matrix.shape[1])
    y = np.arange(matrix.shape[0])
    X, Y = np.meshgrid(x, y)
    Z = matrix
    
    # Plot surface
    ax.plot_surface(X, Y, Z, cmap='viridis')
    
    # Labels and title
    ax.set_xlabel('Column Index')
    ax.set_ylabel('Row Index')
    ax.set_zlabel('Value')
    ax.set_title(title)
    plt.show()

def load_and_plot_matrices(A_path, B_path):
    """
    Load matrices A and B from .npy files and plot them.
    
    Args:
    - A_path (str): Path to the A matrix .npy file.
    - B_path (str): Path to the B matrix .npy file.
    """
    # Load matrices from files
    A = np.load(A_path)
    B = np.load(B_path)
    
    # Plot each matrix
    plot_matrix_surface(A, title="System Matrix A Surface Plot")
    plot_matrix_surface(B, title="Control Matrix B Surface Plot")

if __name__ == "__main__":
    # Update these paths based on your configuration or script location
    A_path = "logs/A_matrix.npy"
    B_path = "logs/B_matrix.npy"
    
    load_and_plot_matrices(A_path, B_path)
