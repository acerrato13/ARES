import numpy as np
from scipy.optimize import minimize

class NMPCOpponent:
    def __init__(self, prediction_horizon=10, dt=0.1):
        """
        Initialize the NMPC opponent.

        Parameters:
        prediction_horizon : int
            Number of steps in the prediction horizon.
        dt : float
            Time step for the prediction.
        """
        self.prediction_horizon = prediction_horizon
        self.dt = dt

    def dynamics(self, state, control):
        """
        Define the dynamics of the opponent.

        Parameters:
        state : np.ndarray
            Current state of the opponent [x, y, z, vx, vy, vz].
        control : np.ndarray
            Control inputs [ax, ay, az].

        Returns:
        np.ndarray
            Next state of the opponent.
        """
        x, y, z, vx, vy, vz = state
        ax, ay, az = control

        # Update positions and velocities
        x_next = x + vx * self.dt
        y_next = y + vy * self.dt
        z_next = z + vz * self.dt
        vx_next = vx + ax * self.dt
        vy_next = vy + ay * self.dt
        vz_next = vz + az * self.dt

        return np.array([x_next, y_next, z_next, vx_next, vy_next, vz_next])

    def cost_function(self, controls, initial_state, target_state):
        """
        Define the cost function for NMPC.

        Parameters:
        controls : np.ndarray
            Control inputs over the prediction horizon.
        initial_state : np.ndarray
            Initial state of the opponent.
        target_state : np.ndarray
            Target state to track.

        Returns:
        float
            Total cost over the prediction horizon.
        """
        state = initial_state
        total_cost = 0

        for t in range(self.prediction_horizon):
            control = controls[t * 3:(t + 1) * 3]
            state = self.dynamics(state, control)

            # Compute tracking cost (L2 norm)
            tracking_cost = np.linalg.norm(state[:3] - target_state[:3])

            # Add control effort cost
            control_cost = np.linalg.norm(control)

            total_cost += tracking_cost + 0.1 * control_cost

        return total_cost

    def compute_control(self, initial_state, target_state):
        """
        Compute the optimal control inputs using NMPC.

        Parameters:
        initial_state : np.ndarray
            Initial state of the opponent.
        target_state : np.ndarray
            Target state to track.

        Returns:
        np.ndarray
            Optimal control inputs [ax, ay, az].
        """
        # Initial guess for controls
        controls_guess = np.zeros(self.prediction_horizon * 3)

        # Optimization bounds
        bounds = [(-1, 1)] * len(controls_guess)

        # Solve the optimization problem
        result = minimize(
            self.cost_function,
            controls_guess,
            args=(initial_state, target_state),
            bounds=bounds,
            method='SLSQP'
        )

        # Return the first control input
        return result.x[:3] if result.success else np.zeros(3)
