import numpy as np
from core import DroneState
from config import NRHDGConfig
from dynamic_role_switching import DynamicRoleSwitching2D
from typing import Tuple
from scipy.optimize import minimize

class NRHDGController2D:
    """
    NRHDG Controller for 2D dynamic role switching.
    """

    def __init__(self, role_switcher: DynamicRoleSwitching2D, config: NRHDGConfig):
        self.config = config
        self.role_switcher = role_switcher

    def dynamics(self, state: DroneState, control: np.ndarray, t: float) -> np.ndarray:
        """
        Compute 2D point-mass dynamics: X'' = u/m - k*X'
        """
        pos = state.position()[:2]
        vel = state.velocity()[:2]
        k = self.config.drag_coeff

        acc = control / self.config.mass - k * vel

        r_theta = self.role_switcher.path_function(state.theta)[:2]
        r_theta_prime = self._path_derivative(state.theta)

        tangent = r_theta_prime / np.linalg.norm(r_theta_prime)
        theta_dot = np.dot(vel, tangent) / np.linalg.norm(r_theta_prime)

        return np.concatenate([vel, acc, [theta_dot], [0]])

    def path_derivative(self, theta: float) -> np.ndarray:
        # Derivative of 2D path
        return np.array([
            6 * np.cos(theta),
            3 * np.cos(2 * theta)
        ])

    def stage_cost(self, ego_state: DroneState, opp_state: DroneState,
                   u_ego: np.ndarray, u_opp: np.ndarray, t: float) -> float:
        """
        Stage cost in 2D (equation 18 equivalent)
        """
        progress_cost = -self.config.w_progress * ego_state.theta_dot

        r_ego = self.role_switcher.path_function(ego_state.theta)[:2]
        deviation = np.linalg.norm(ego_state.position()[:2] - r_ego)
        deviation_cost = self.config.w_deviation * deviation**2

        control_cost = self.config.w_control * np.linalg.norm(u_ego)**2

        G_O = self.role_switcher.compute_G_O(
            ego_state.position()[:2], ego_state.theta,
            opp_state.position()[:2], opp_state.theta
        )
        interaction_cost = -G_O

        return progress_cost + deviation_cost + control_cost + interaction_cost

    def terminal_cost(self, ego_state: DroneState, opp_state: DroneState) -> float:
        theta_delta = opp_state.theta - ego_state.theta
        r_ego = self.role_switcher.path_function(ego_state.theta)[:2]
        deviation = np.linalg.norm(ego_state.position()[:2] - r_ego)
        return -10.0 * theta_delta + 5.0 * deviation**2

    def compute_objective(self, ego_state: DroneState, opp_state: DroneState,
                          u_sequence: np.ndarray, v_sequence: np.ndarray) -> float:
        ego_traj = [ego_state]
        opp_traj = [opp_state]
        integral_cost = 0.0

        for k in range(self.config.n_steps):
            u_k = u_sequence[k*2:(k+1)*2]
            v_k = v_sequence[k*2:(k+1)*2]

            integral_cost += self.stage_cost(
                ego_traj[-1], opp_traj[-1], u_k, v_k, k * self.config.dt
            ) * self.config.dt

            ego_next = self._propagate_state(ego_traj[-1], u_k, self.config.dt)
            opp_next = self._propagate_state(opp_traj[-1], v_k, self.config.dt)

            ego_traj.append(ego_next)
            opp_traj.append(opp_next)

        terminal = self.terminal_cost(ego_traj[-1], opp_traj[-1])
        return terminal + integral_cost

    def propagate_state(self, state: DroneState, control: np.ndarray, dt: float) -> DroneState:
        pos = state.position()[:2]
        vel = state.velocity()[:2]

        acc = control / self.config.mass - self.config.drag_coeff * vel
        new_pos = pos + vel * dt
        new_vel = vel + acc * dt

        r_prime = self._path_derivative(state.theta)
        r_prime_norm = np.linalg.norm(r_prime)
        theta_dot = np.dot(new_vel, r_prime / r_prime_norm) / r_prime_norm
        new_theta = state.theta + state.theta_dot * dt

        return DroneState(new_pos[0], new_pos[1],
                          new_vel[0], new_vel[1],
                          acc[0], acc[1],
                          new_theta, theta_dot, state.timestamp + dt)

    def solve_saddle_point(self, ego_state: DroneState, opp_state: DroneState) -> Tuple[np.ndarray, np.ndarray]:
        u_seq = np.zeros(self.config.n_steps * 2)
        v_seq = np.zeros(self.config.n_steps * 2)

        max_iterations = 20
        tolerance = 1e-4

        for iteration in range(max_iterations):
            u_old, v_old = u_seq.copy(), v_seq.copy()

            def objective_u(u): return self.compute_objective(ego_state, opp_state, u, v_seq)
            bounds_u = [(-self.config.max_thrust, self.config.max_thrust)] * len(u_seq)
            result_u = minimize(objective_u, u_seq, method='SLSQP', bounds=bounds_u)
            u_seq = result_u.x

            def objective_v(v): return -self.compute_objective(ego_state, opp_state, u_seq, v)
            bounds_v = [(-self.config.max_thrust, self.config.max_thrust)] * len(v_seq)
            result_v = minimize(objective_v, v_seq, method='SLSQP', bounds=bounds_v)
            v_seq = result_v.x

            if np.linalg.norm(u_seq - u_old) < tolerance and np.linalg.norm(v_seq - v_old) < tolerance:
                print(f"Converged after {iteration + 1} iterations")
                break

        return u_seq, v_seq

    def compute_control(self, ego_state: DroneState, opp_state: DroneState) -> np.ndarray:
        u_sequence, v_sequence = self.solve_saddle_point(ego_state, opp_state)
        u_optimal = u_sequence[:2]
        thrust_mag = np.linalg.norm(u_optimal)

        if thrust_mag > self.config.max_thrust:
            u_optimal = u_optimal * (self.config.max_thrust / thrust_mag)

        return u_optimal


if __name__ == "__main__":

    role_switcher = DynamicRoleSwitching2D()
    config = NRHDGConfig(T=1.0, dt=0.1, n_steps=10)
    controller = NRHDGController2D(role_switcher, config)

    ego_state = DroneState(0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0, 0.0)

    opp_state = DroneState(0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0, 0.0)

    print("Computing optimal control (2D)...")
    u_optimal = controller.compute_control(ego_state, opp_state)
    print(f"Optimal control thrust: {u_optimal}")
    print(f"Thrust magnitude: {np.linalg.norm(u_optimal):.3f}")
