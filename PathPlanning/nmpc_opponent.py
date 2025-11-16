import numpy as np
from scipy.optimize import minimize


class NMPCOpponent:
    """
    Simple NMPC opponent controller in 3D.

    State vector: [x, y, z, vx, vy, vz]
    Control vector: [ax, ay, az]

    This is the 3D generalization of your 2D NMPC controller.
    """

    def __init__(self, prediction_horizon: int = 10, dt: float = 0.1, max_acc: float = 1.0):
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.max_acc = float(max_acc)

    def dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Discrete-time kinematic dynamics in 3D (Euler integration).
        state:  [x,y,z,vx,vy,vz]
        control: [ax,ay,az]
        """
        x, y, z, vx, vy, vz = state
        ax, ay, az = control

        # propagate position
        x_next = x + vx * self.dt
        y_next = y + vy * self.dt
        z_next = z + vz * self.dt

        # propagate velocity
        vx_next = vx + ax * self.dt
        vy_next = vy + ay * self.dt
        vz_next = vz + az * self.dt

        return np.array([x_next, y_next, z_next, vx_next, vy_next, vz_next])

    def cost_function(self, controls: np.ndarray, initial_state: np.ndarray, target_state: np.ndarray) -> float:
        """
        Evaluate NMPC cost over horizon.
        """
        state = initial_state.copy()
        total_cost = 0.0

        for t in range(self.prediction_horizon):
            control = controls[t * 3:(t + 1) * 3]
            state = self.dynamics(state, control)

            tracking_cost = np.linalg.norm(state[:3] - target_state[:3])
            control_cost = np.linalg.norm(control)

            total_cost += tracking_cost + 0.1 * control_cost

        return float(total_cost)

    def compute_control(self, initial_state: np.ndarray, target_state: np.ndarray) -> np.ndarray:
        """
        Solve NMPC and return first control input.
        """
        controls_guess = np.zeros(self.prediction_horizon * 3)
        bounds = [(-self.max_acc, self.max_acc)] * len(controls_guess)

        result = minimize(
            self.cost_function,
            controls_guess,
            args=(initial_state, target_state),
            bounds=bounds,
            method='SLSQP',
            options={'maxiter': 200}
        )

        if result.success:
            return result.x[:3]

        return np.zeros(3)


# ---------------------------------------------------------
# Optional smoke test
# ---------------------------------------------------------
if __name__ == "__main__":
    npc = NMPCOpponent(prediction_horizon=6, dt=0.1, max_acc=1.0)
    init_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_state = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])

    u0 = npc.compute_control(init_state, target_state)
    print("First control (ax, ay, az):", u0)
