import numpy as np
from scipy.optimize import minimize

class NMPCOpponent:
    def __init__(self, prediction_horizon=10, dt=0.1):
        """
        Initialize the NMPC opponent (2D version).

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
        Define the 2D dynamics of the opponent.

        Parameters:
        state : np.ndarray
            Current state of the opponent [x, y, vx, vy].
        control : np.ndarray
            Control inputs [ax, ay].

        Returns:
        np.ndarray
            Next state of the opponent [x, y, vx, vy].
        """
        x, y, vx, vy = state
        ax, ay = control

        # Euler integration for simple kinematics
        x_next = x + vx * self.dt
        y_next = y + vy * self.dt
        vx_next = vx + ax * self.dt
        vy_next = vy + ay * self.dt

        return np.array([x_next, y_next, vx_next, vy_next])

    def cost_function(self, controls, initial_state, target_state):
        """
        Define the cost function for NMPC (2D version).

        Parameters:
        controls : np.ndarray
            Flattened control inputs over the prediction horizon.
        initial_state : np.ndarray
            Initial state of the opponent [x, y, vx, vy].
        target_state : np.ndarray
            Target state to track [x, y, vx, vy].

        Returns:
        float
            Total cost over the prediction horizon.
        """
        state = initial_state.copy()
        total_cost = 0.0

        for t in range(self.prediction_horizon):
            control = controls[t * 2:(t + 1) * 2]
            state = self.dynamics(state, control)

            # Position tracking cost
            tracking_cost = np.linalg.norm(state[:2] - target_state[:2])

            # Control effort penalty
            control_cost = np.linalg.norm(control)

            total_cost += tracking_cost + 0.1 * control_cost

        return total_cost

    def compute_control(self, initial_state, target_state):
        """
        Compute the optimal control inputs using NMPC (2D version).

        Parameters:
        initial_state : np.ndarray
            Initial state of the opponent [x, y, vx, vy].
        target_state : np.ndarray
            Target state [x, y, vx, vy].

        Returns:
        np.ndarray
            Optimal control inputs [ax, ay].
        """
        # Initial guess for all control inputs
        controls_guess = np.zeros(self.prediction_horizon * 2)

        # Define control bounds
        bounds = [(-1, 1)] * len(controls_guess)

        # Solve optimization
        result = minimize(
            self.cost_function,
            controls_guess,
            args=(initial_state, target_state),
            bounds=bounds,
            method='SLSQP'
        )

        # Return the first control input if successful
        return result.x[:2] if result.success else np.zeros(2)
