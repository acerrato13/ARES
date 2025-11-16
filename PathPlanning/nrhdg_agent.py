import numpy as np
from PathPlanning.core.State import DroneState
from PathPlanning.config import NRHDGConfig
from PathPlanning.dynamic_role_switching import DynamicRoleSwitching3D
from typing import Tuple
from scipy.optimize import minimize


class NRHDGController3D:
    """
    NRHDG Controller for path-following with dynamic role switching in 3D.

    Simplifications vs. your previous version:
    - We assume each player applies a *constant* 3D control over the horizon.
      So the optimization variables are:
          u in R^3 (ego), v in R^3 (opponent),
      instead of 30D sequences. This makes SLSQP much more stable.
    - SLSQP calls are given small maxiter so the solver can't hang.
    - _propagate_state now updates theta using the *new* theta_dot.
    """

    def __init__(self, role_switcher: DynamicRoleSwitching3D, config: NRHDGConfig):
        self.config = config
        self.role_switcher = role_switcher

    # ---------- Path and Dynamics ----------

    def dynamics(self, state: DroneState, control: np.ndarray, t: float) -> np.ndarray:
        """
        Reduced dynamics used by optimizers:
        returns [vx, vy, vz, ax, ay, az, theta_dot, 0]
        """
        pos = state.position[:3]
        vel = state.velocity[:3]
        k = self.config.drag_coeff

        acc = control / self.config.mass - k * vel

        r_theta_prime = self._path_derivative(state.theta)
        tangent = r_theta_prime / (np.linalg.norm(r_theta_prime) + 1e-9)
        theta_dot = np.dot(vel, tangent) / (np.linalg.norm(r_theta_prime) + 1e-9)

        return np.concatenate([vel, acc, [theta_dot], [0.0]])

    def _path_derivative(self, theta: float) -> np.ndarray:
        # Derivative of your 3D path; adjust if you change the path.
        return np.array([
            6 * np.cos(theta),
            3 * np.cos(2 * theta),
            0.0           # keep z derivative simple for now
        ])

    # ---------- Costs ----------

    def stage_cost(self, ego_state: DroneState, opp_state: DroneState,
                   u_ego: np.ndarray, u_opp: np.ndarray, t: float) -> float:
        """
        Stage cost in 3D (progress reward, deviation penalty, control cost,
        and interaction term from G_O).
        """
        # Progress (reward for moving forward along path)
        progress_cost = -self.config.w_progress * ego_state.theta_dot

        # Deviation from path
        r_ego = self.role_switcher.path_function(ego_state.theta)[:3]
        deviation = np.linalg.norm(ego_state.position[:3] - r_ego)
        deviation_cost = self.config.w_deviation * deviation ** 2

        # Control effort
        control_cost = self.config.w_control * np.linalg.norm(u_ego) ** 2

        # Game-theoretic interaction
        G_O = self.role_switcher.compute_G_O(
            ego_state.position[:3], ego_state.theta,
            opp_state.position[:3], opp_state.theta
        )
        interaction_cost = -G_O  # high G_O is good for ego

        return progress_cost + deviation_cost + control_cost + interaction_cost

    def terminal_cost(self, ego_state: DroneState, opp_state: DroneState) -> float:
        theta_delta = opp_state.theta - ego_state.theta
        r_ego = self.role_switcher.path_function(ego_state.theta)[:3]
        deviation = np.linalg.norm(ego_state.position[:3] - r_ego)
        return -10.0 * theta_delta + 5.0 * deviation ** 2

    # ---------- Propagation ----------

    def _propagate_state(self, state: DroneState, control: np.ndarray, dt: float) -> DroneState:
        """
        Simple Euler propagation for the reduced 3D point-mass model.
        Returns a new DroneState using array-style fields.
        """
        pos = state.position[:3]
        vel = state.velocity[:3]

        acc = control / self.config.mass - self.config.drag_coeff * vel

        new_pos = pos + vel * dt
        new_vel = vel + acc * dt

        r_prime = self._path_derivative(state.theta)
        r_prime_norm = np.linalg.norm(r_prime) + 1e-9
        # progress along path
        theta_dot = np.dot(new_vel, r_prime / r_prime_norm) / r_prime_norm
        new_theta = state.theta + theta_dot * dt

        return DroneState(
            position=np.array([new_pos[0], new_pos[1], new_pos[2]]),
            velocity=np.array([new_vel[0], new_vel[1], new_vel[2]]),
            acceleration=np.array([acc[0], acc[1], acc[2]]),
            alpha=state.alpha.copy(),
            theta=new_theta,
            theta_dot=theta_dot,
            timestamp=state.timestamp + dt
        )

    # ---------- Objective with constant controls over horizon ----------

    def compute_objective_constant(self, ego_state: DroneState, opp_state: DroneState,
                                   u_ego: np.ndarray, u_opp: np.ndarray) -> float:
        """
        Compute total cost over the horizon assuming constant controls u_ego, u_opp.
        """
        ego = ego_state
        opp = opp_state
        integral_cost = 0.0

        for k in range(self.config.n_steps):
            t_k = k * self.config.dt
            integral_cost += self.stage_cost(ego, opp, u_ego, u_opp, t_k) * self.config.dt
            ego = self._propagate_state(ego, u_ego, self.config.dt)
            opp = self._propagate_state(opp, u_opp, self.config.dt)

        terminal = self.terminal_cost(ego, opp)
        return terminal + integral_cost

    # ---------- Saddle-point Solver (reduced dimension) ----------

    def solve_saddle_point(self, ego_state: DroneState, opp_state: DroneState) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solve min_u max_v J(u, v) in a simplified form where u and v are 3D
        constant controls over the horizon.
        """
        u = np.zeros(3)
        v = np.zeros(3)

        max_outer_iters = 10
        tolerance = 1e-3

        thrust_bounds = [(-self.config.max_thrust, self.config.max_thrust)] * 3

        for it in range(max_outer_iters):
            u_old = u.copy()
            v_old = v.copy()

            # --- Minimize wrt u (ego) ---
            def objective_u(u_vec):
                return self.compute_objective_constant(ego_state, opp_state, u_vec, v)

            res_u = minimize(
                objective_u, u,
                method="SLSQP",
                bounds=thrust_bounds,
                options={"maxiter": 20, "ftol": 1e-3, "disp": False}
            )
            u = res_u.x

            # --- Maximize wrt v (opponent) -> minimize negative objective ---
            def objective_v(v_vec):
                return -self.compute_objective_constant(ego_state, opp_state, u, v_vec)

            res_v = minimize(
                objective_v, v,
                method="SLSQP",
                bounds=thrust_bounds,
                options={"maxiter": 20, "ftol": 1e-3, "disp": False}
            )
            v = res_v.x

            # Convergence check
            if np.linalg.norm(u - u_old) < tolerance and np.linalg.norm(v - v_old) < tolerance:
                # print(f"[NRHDG3D] Converged after {it+1} outer iterations")
                break

        return u, v

    # ---------- Public API ----------

    def compute_control(self, ego_state: DroneState, opp_state: DroneState) -> np.ndarray:
        u_opt, v_opt = self.solve_saddle_point(ego_state, opp_state)

        # Enforce thrust bound again in case optimizer overshot numerically
        thrust_mag = np.linalg.norm(u_opt)
        if thrust_mag > self.config.max_thrust:
            u_opt = u_opt * (self.config.max_thrust / thrust_mag)

        return u_opt
