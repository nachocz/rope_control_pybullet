import numpy as np
from scipy.optimize import minimize

def mpc_control(A, B, C, x0, target):
    """
    MPC controller to compute the control input to drive the rope's end effector
    along a circular trajectory.
    """
    # Horizon length
    N = 10
    control_dim = B.shape[1]

    # Initial control guess
    u_init = np.zeros((N, control_dim))

    def cost_function(u):
        u = u.reshape(N, control_dim)
        x = np.copy(x0)
        cost = 0.0

        # Cost function based on tracking error
        for i in range(N):
            x = A @ x + B @ u[i]
            y = C @ x
            error = y - target
            cost += np.dot(error, error)
            print(f"Iteration {i}: state x = {x}, output y = {y}, error = {error}")

        return cost

    # Solve optimization
    solution = minimize(cost_function, u_init.ravel(), options={'disp': True})
    u_optimal = solution.x.reshape(N, control_dim)

    # Return only the first control action
    return u_optimal[0]
