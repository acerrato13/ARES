import numpy as np
from core.State import DroneState
from config import NRHDGConfig
from dynamic_role_switching import DynamicRoleSwitching3D
from typing import Tuple, List
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

class WaypointPathManager:
    """
    Manages waypoint-based path for NRHDG controller integration with Simulink.
    """

    def __init__(self, waypoints: np.ndarray):
        """
        Initialize with waypoints from Simulink.

        Args:
            waypoints: Nx3 array of [x, y, z] waypoint positions
        """
        self.waypoints = np.array(waypoints)
        self.n_waypoints = len(waypoints)

        # Compute cumulative arc length as path parameter
        self.arc_lengths = self._compute_arc_lengths()
        self.total_length = self.arc_lengths[-1]

        # Create cubic spline interpolation for smooth path
        self.spline_x = CubicSpline(self.arc_lengths, waypoints[:, 0])
        self.spline_y = CubicSpline(self.arc_lengths, waypoints[:, 1])
        self.spline_z = CubicSpline(self.arc_lengths, waypoints[:, 2])

    def _compute_arc_lengths(self) -> np.ndarray:
        """Compute cumulative arc length along waypoints."""
        arc_lengths = np.zeros(self.n_waypoints)
        for i in range(1, self.n_waypoints):
            segment_length = np.linalg.norm(self.waypoints[i] - self.waypoints[i-1])
            arc_lengths[i] = arc_lengths[i-1] + segment_length
        return arc_lengths

    def path_function(self, theta: float) -> np.ndarray:
        """
        Evaluate 3D position on path at parameter theta.

        Args:
            theta: Path parameter (arc length)

        Returns:
            3D position [x, y, z]
        """
        # Clamp theta to valid range
        theta = np.clip(theta, 0, self.total_length)

        return np.array([
            self.spline_x(theta),
            self.spline_y(theta),
            self.spline_z(theta)
        ])

    def path_derivative(self, theta: float) -> np.ndarray:
        """
        Compute tangent vector (derivative) at parameter theta.

        Args:
            theta: Path parameter (arc length)

        Returns:
            3D tangent vector [dx/dθ, dy/dθ, dz/dθ]
        """
        theta = np.clip(theta, 0, self.total_length)

        return np.array([
            self.spline_x.derivative()(theta),
            self.spline_y.derivative()(theta),
            self.spline_z.derivative()(theta)
        ])

    def get_closest_theta(self, position: np.ndarray) -> float:
        """
        Find path parameter theta closest to given position.
        Useful for initializing theta from current drone position.

        Args:
            position: 3D position [x, y, z]

        Returns:
            Path parameter theta
        """
        # Sample the path and find closest point
        theta_samples = np.linspace(0, self.total_length, 100)
        min_dist = float('inf')
        best_theta = 0.0

        for theta in theta_samples:
            path_pos = self.path_function(theta)
            dist = np.linalg.norm(position - path_pos)
            if dist < min_dist:
                min_dist = dist
                best_theta = theta

        return best_theta


class NRHDGController:
    """
    NRHDG Controller for 3D dynamic role switching with waypoint integration.
    """

    def __init__(self, path_manager: WaypointPathManager,
                 role_switcher: DynamicRoleSwitching3D,
                 config: NRHDGConfig):
        self.config = config
        self.role_switcher = role_switcher
        self.path_manager = path_manager

        # Override role_switcher's path function to use waypoints
        self.role_switcher.path_function = self.path_manager.path_function

    def dynamics(self, state: DroneState, control: np.ndarray, t: float) -> np.ndarray:
        """
        Compute 3D point-mass dynamics: X'' = u/m - k*X'
        """
        pos = state.position
        vel = state.velocity
        k = self.config.drag_coeff

        acc = control / self.config.mass - k * vel

        r_theta = self.path_manager.path_function(state.theta)
        r_theta_prime = self.path_manager.path_derivative(state.theta)

        tangent = r_theta_prime / np.linalg.norm(r_theta_prime)
        theta_dot = np.dot(vel, tangent) / np.linalg.norm(r_theta_prime)

        return np.concatenate([vel, acc, [theta_dot], [0]])

    def stage_cost(self, ego_state: DroneState, opp_state: DroneState,
                   u_ego: np.ndarray, u_opp: np.ndarray, t: float) -> float:
        """
        Stage cost in 3D (equation 18 equivalent)
        """
        progress_cost = -self.config.w_progress * ego_state.theta_dot

        r_ego = self.path_manager.path_function(ego_state.theta)
        deviation = np.linalg.norm(ego_state.position - r_ego)
        deviation_cost = self.config.w_deviation * deviation**2

        control_cost = self.config.w_control * np.linalg.norm(u_ego)**2

        G_O = self.role_switcher.compute_G_O(
            ego_state.position, ego_state.theta,
            opp_state.position, opp_state.theta
        )
        interaction_cost = -G_O

        return progress_cost + deviation_cost + control_cost + interaction_cost

    def terminal_cost(self, ego_state: DroneState, opp_state: DroneState) -> float:
        theta_delta = opp_state.theta - ego_state.theta
        r_ego = self.path_manager.path_function(ego_state.theta)
        deviation = np.linalg.norm(ego_state.position - r_ego)
        return -10.0 * theta_delta + 5.0 * deviation**2

    def compute_objective(self, ego_state: DroneState, opp_state: DroneState,
                          u_sequence: np.ndarray, v_sequence: np.ndarray) -> float:
        ego_traj = [ego_state]
        opp_traj = [opp_state]
        integral_cost = 0.0

        for k in range(self.config.n_steps):
            u_k = u_sequence[k*3:(k+1)*3]
            v_k = v_sequence[k*3:(k+1)*3]

            integral_cost += self.stage_cost(
                ego_traj[-1], opp_traj[-1], u_k, v_k, k * self.config.dt
            ) * self.config.dt

            ego_next = self._propagate_state(ego_traj[-1], u_k, self.config.dt)
            opp_next = self._propagate_state(opp_traj[-1], v_k, self.config.dt)

            ego_traj.append(ego_next)
            opp_traj.append(opp_next)

        terminal = self.terminal_cost(ego_traj[-1], opp_traj[-1])
        return terminal + integral_cost

    def _propagate_state(self, state: DroneState, control: np.ndarray, dt: float) -> DroneState:
        pos = state.position
        vel = state.velocity

        if self.config.mass == 0:
            raise ValueError("Mass cannot be zero")

        acc = control / self.config.mass - self.config.drag_coeff * vel
        new_pos = pos + vel * dt
        new_vel = vel + acc * dt

        r_prime = self.path_manager.path_derivative(state.theta)
        r_prime_norm = np.linalg.norm(r_prime)

        if r_prime_norm == 0:
            theta_dot = 0.0
        else:
            theta_dot = np.dot(new_vel, r_prime / r_prime_norm) / r_prime_norm

        new_theta = state.theta + state.theta_dot * dt

        # Clamp theta to valid path range
        new_theta = np.clip(new_theta, 0, self.path_manager.total_length)

        return DroneState(
            position=new_pos,
            velocity=new_vel,
            acceleration=acc,
            alpha=state.alpha,
            theta=new_theta,
            theta_dot=theta_dot,
            timestamp=state.timestamp + dt
        )

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

    def compute_control(self, ego_state: DroneState, opp_state: DroneState) -> np.ndarray:
        u_sequence, v_sequence = self.solve_saddle_point(ego_state, opp_state)
        u_optimal = u_sequence[:3]
        thrust_mag = np.linalg.norm(u_optimal)

        if thrust_mag > self.config.max_thrust:
            u_optimal = u_optimal * (self.config.max_thrust / thrust_mag)

        return u_optimal


# ============================================================================
# SIMULINK INTEGRATION INTERFACE
# ============================================================================

class SimulinkNRHDGInterface:
    """
    Interface class for Simulink integration using Python Function block.
    """

    def __init__(self, waypoints: np.ndarray, config: NRHDGConfig):
        """
        Initialize the controller with waypoints from Simulink.

        Args:
            waypoints: Nx3 array of waypoints from Simulink
            config: NRHDG configuration
        """
        self.path_manager = WaypointPathManager(waypoints)
        self.role_switcher = DynamicRoleSwitching3D()
        self.controller = NRHDGController(self.path_manager, self.role_switcher, config)

    def compute_control_from_simulink(self,
                                      ego_pos: np.ndarray,
                                      ego_vel: np.ndarray,
                                      opp_pos: np.ndarray,
                                      opp_vel: np.ndarray,
                                      timestamp: float) -> np.ndarray:
        """
        Compute control from Simulink inputs.

        Args:
            ego_pos: [x, y, z] ego position
            ego_vel: [vx, vy, vz] ego velocity
            opp_pos: [x, y, z] opponent position
            opp_vel: [vx, vy, vz] opponent velocity
            timestamp: Current simulation time

        Returns:
            control: [ux, uy, uz] control thrust vector
        """
        # Initialize theta from current position if needed
        ego_theta = self.path_manager.get_closest_theta(ego_pos)
        opp_theta = self.path_manager.get_closest_theta(opp_pos)

        # Create drone states
        ego_state = DroneState(
            position=np.array(ego_pos),
            velocity=np.array(ego_vel),
            acceleration=np.zeros(3),
            alpha=np.zeros(3),
            theta=ego_theta,
            theta_dot=0.0,
            timestamp=timestamp
        )

        opp_state = DroneState(
            position=np.array(opp_pos),
            velocity=np.array(opp_vel),
            acceleration=np.zeros(3),
            alpha=np.zeros(3),
            theta=opp_theta,
            theta_dot=0.0,
            timestamp=timestamp
        )

        # Compute optimal control
        control = self.controller.compute_control(ego_state, opp_state)

        return control


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":

    # Example: Define waypoints (from Simulink map)
    waypoints = np.array([
        [0.0, 0.0, 0.0],
        [10.0, 5.0, 2.0],
        [20.0, 8.0, 3.0],
        [30.0, 10.0, 2.5],
        [40.0, 12.0, 1.0],
        [50.0, 15.0, 0.0]
    ])

    # Create configuration
    config = NRHDGConfig(
        T=1.0,
        dt=0.1,
        n_steps=10,
        mass=1.0,
        drag_coeff=0.1,
        max_thrust=20.0
    )

    # Create Simulink interface
    simulink_interface = SimulinkNRHDGInterface(waypoints, config)

    # Simulate control computation (this would be called from Simulink)
    ego_pos = np.array([1.0, 0.5, 0.2])
    ego_vel = np.array([1.0, 0.5, 0.1])
    opp_pos = np.array([2.0, 1.0, 0.3])
    opp_vel = np.array([0.8, 0.4, 0.05])
    timestamp = 0.0

    print("Computing control with waypoint-based trajectory...")
    print(f"Waypoints: {len(waypoints)} points")
    print(f"Total path length: {simulink_interface.path_manager.total_length:.2f} m")

    control = simulink_interface.compute_control_from_simulink(
        ego_pos, ego_vel, opp_pos, opp_vel, timestamp
    )

    print(f"\nOptimal control thrust: {control}")
    print(f"Thrust magnitude: {np.linalg.norm(control):.3f} N")
