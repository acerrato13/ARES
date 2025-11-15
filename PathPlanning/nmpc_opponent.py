import numpy as np
from scipy.optimize import minimize

class NMPCOpponent3D:
    def __init__(self, prediction_horizon=10, dt=0.1):
        self.prediction_horizon = prediction_horizon
        self.dt = dt

    def dynamics(self, state, control):
        """
        3D dynamics: state = [x, y, z, vx, vy, vz]
                     control = [ax, ay, az]
        """
        x, y, z, vx, vy, vz = state
        ax, ay, az = control

        x_next = x + vx * self.dt
        y_next = y + vy * self.dt
        z_next = z + vz * self.dt

        vx_next = vx + ax * self.dt
        vy_next = vy + ay * self.dt
        vz_next = vz + az * self.dt

        return np.array([x_next, y_next, z_next, vx_next, vy_next, vz_next])

    def cost_function(self, controls, initial_state, target_state):
        """
        Cost over prediction horizon.
        Controls is flattened: [ax0, ay0, az0, ax1, ay1, az1, ...]
        """
        state = initial_state.copy()
        total_cost = 0.0

        for t in range(self.prediction_horizon):
            control = controls[t * 3:(t + 1) * 3]
            state = self.dynamics(state, control)

            # Position tracking (x,y,z)
            tracking_cost = np.linalg.norm(state[:3] - target_state[:3])

            # Control effort
            control_cost = np.linalg.norm(control)

            total_cost += tracking_cost + 0.1 * control_cost

        return total_cost

    def compute_control(self, initial_state, target_state):
        """
        Return optimal [ax, ay, az]
        """
        controls_guess = np.zeros(self.prediction_horizon * 3)

        bounds = [(-1, 1)] * len(controls_guess)

        result = minimize(
            self.cost_function,
            controls_guess,
            args=(initial_state, target_state),
            bounds=bounds,
            method='SLSQP'
        )

        return result.x[:3] if result.success else np.zeros(3)

