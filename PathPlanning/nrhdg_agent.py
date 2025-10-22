import numpy as np
from core import NRHDGConfig, DroneState
from functions_anu import DynamicRoleSwitching
from typing import Tuple, Dict, Callable
from scipy.optimize import minimize

class NRHDGController: 
    """
    todo: add docstring 
    todo: add dynamic role switching from anu 
    """
    def __init__(self, role_switcher: DynamicRoleSwitching, config: NRHDGConfig): 
        self.config = config
        self.role_switcher = role_switcher
    
    def dynamics(self, state: DroneState, control: np.ndarray, t: float) -> np.ndarray:
        """
        compute state derivatives for drone dynamics

        point mass model: X" = u/m - k*X'
        where u is thrust vecotr, m is mass, k is drag coef
        """

        pos = state.position()
        vel = state.velocity()
       
        k = self.config.drag_coeff
        
        acc = control / self.config.mass - k * vel
        
        r_theta = self.role_switcher.path_function(state.theta)
        r_theta_prime = self._path_derivative(state.theta)

        tangent = r_theta_prime / np.linalg.norm(r_theta_prime)
        theta_dot = np.dot(vel, tangent) / np.linalg.norm(r_theta_prime)

        return np.concatenate([vel, acc, [theta_dot], [0]])

    def _path_derivative(self, theta: float) -> np.ndarray:
        
        return np.array([
            6 * np.cos(theta), 
            6 * np.cos(2 * theta), 
            3 * np.cos(theta / 2)])

    def stage_cost(self, ego_state: DroneState, opp_state: DroneState, 
               u_ego: np.ndarray, u_opp: np.ndarray, t: float) -> float:
        """
        Stage cost L_D(x_D(τ), u(τ), v(τ), τ) for the differential game.
        
        This is the integrand in equation (18).
        """
        # Path progress term (ego wants to maximize theta_dot)
        progress_cost = -self.config.w_progress * ego_state.theta_dot
        
        # Path deviation penalty
        r_ego = self.role_switcher.path_function(ego_state.theta)
        deviation = np.linalg.norm(ego_state.position() - r_ego)
        deviation_cost = self.config.w_deviation * deviation**2
        
        # Control effort penalty
        control_cost = self.config.w_control * np.linalg.norm(u_ego)**2
        
        # Opponent interaction term (using G_O potential)
        G_O = self.role_switcher.compute_G_O(
            ego_state.position(), ego_state.theta,
            opp_state.position(), opp_state.theta
        )
        interaction_cost = -G_O  # Negative because we want to maximize G_O effect
        
        return progress_cost + deviation_cost + control_cost + interaction_cost
    
    def terminal_cost(self, ego_state: DroneState, opp_state: DroneState) -> float:
        """
        Terminal cost φ_D(x_D(t + T)) from equation (18).
        """
        # Penalize being behind opponent at end of horizon
        theta_delta = opp_state.theta - ego_state.theta
        
        # Penalize deviation from path
        r_ego = self.role_switcher.path_function(ego_state.theta)
        deviation = np.linalg.norm(ego_state.position() - r_ego)
        
        return -10.0 * theta_delta + 5.0 * deviation**2
    
    def compute_objective(self, ego_state: DroneState, opp_state: DroneState,
                         u_sequence: np.ndarray, v_sequence: np.ndarray) -> float:
        """
        Compute objective function J_D[u, v] from equation (18).
        
        J_D[u, v] = φ_D(x_D(t + T)) + ∫_t^(t+T) L_D(x_D(τ), u(τ), v(τ), τ) dτ
        """
        # Simulate forward in time
        ego_traj = [ego_state]
        opp_traj = [opp_state]
        
        integral_cost = 0.0
        
        for k in range(self.config.n_steps):
            # Get controls for this time step
            u_k = u_sequence[k*3:(k+1)*3]
            v_k = v_sequence[k*3:(k+1)*3]
            
            # Add stage cost
            integral_cost += self.stage_cost(
                ego_traj[-1], opp_traj[-1], u_k, v_k, 
                k * self.config.dt
            ) * self.config.dt
            
            # Propagate dynamics (simple Euler integration)
            ego_next = self._propagate_state(ego_traj[-1], u_k, self.config.dt)
            opp_next = self._propagate_state(opp_traj[-1], v_k, self.config.dt)
            
            ego_traj.append(ego_next)
            opp_traj.append(opp_next)
        
        # Add terminal cost
        terminal = self.terminal_cost(ego_traj[-1], opp_traj[-1])
        
        return terminal + integral_cost
    
    def _propagate_state(self, state: DroneState, control: np.ndarray, 
                        dt: float) -> DroneState:
        """Propagate state forward using simple Euler integration"""
        pos = state.position()
        vel = state.velocity()
        
        # Compute acceleration: a = u/m - k*v
        acc = control / self.config.mass - self.config.drag_coeff * vel
        
        # Integrate position and velocity
        new_pos = pos + vel * dt
        new_vel = vel + acc * dt
        
        # Update theta: project velocity onto path tangent
        r_prime = self._path_derivative(state.theta)
        r_prime_norm = np.linalg.norm(r_prime)
        
        if r_prime_norm > 1e-6:
            tangent = r_prime / r_prime_norm
            theta_dot = np.dot(new_vel, tangent) / r_prime_norm
        else:
            theta_dot = state.theta_dot
        
        new_theta = state.theta + state.theta_dot * dt
        return DroneState(state.mass,
                          new_pos[0], new_pos[1], new_pos[2],
                          new_vel[0], new_vel[1], new_vel[2],
                          acc[0], acc[1], acc[2],
                          new_theta, theta_dot, state.timestamp + dt)
    
    def solve_saddle_point(self, ego_state: DroneState, 
                          opp_state: DroneState) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solve for saddle-point solution (u⁰, v⁰) satisfying equation (19):
        J_D[u⁰, v] ≤ J_D[u⁰, v⁰] ≤ J_D[u, v⁰]
        
        This uses iterative optimization:
        1. Fix v, minimize over u (ego's best response)
        2. Fix u, maximize over v (opponent's best response)
        3. Repeat until convergence
        """
        # Initialize control sequences
        u_seq = np.zeros(self.config.n_steps * 3)
        v_seq = np.zeros(self.config.n_steps * 3)
        
        max_iterations = 20
        tolerance = 1e-4
        
        for iteration in range(max_iterations):
            u_old = u_seq.copy()
            v_old = v_seq.copy()
            
            # Step 1: Minimize over u (ego's move)
            def objective_u(u):
                return self.compute_objective(ego_state, opp_state, u, v_seq)
            
            bounds_u = [(-self.config.max_thrust, self.config.max_thrust)] * len(u_seq)
            result_u = minimize(objective_u, u_seq, method='SLSQP', bounds=bounds_u)
            u_seq = result_u.x
            
            # Step 2: Maximize over v (opponent's move) = minimize negative
            def objective_v(v):
                return -self.compute_objective(ego_state, opp_state, u_seq, v)
            
            bounds_v = [(-self.config.max_thrust, self.config.max_thrust)] * len(v_seq)
            result_v = minimize(objective_v, v_seq, method='SLSQP', bounds=bounds_v)
            v_seq = result_v.x
            
            # Check convergence
            u_change = np.linalg.norm(u_seq - u_old)
            v_change = np.linalg.norm(v_seq - v_old)
            
            if u_change < tolerance and v_change < tolerance:
                print(f"Converged after {iteration + 1} iterations")
                break
        
        return u_seq, v_seq
    
    def compute_control(self, ego_state: DroneState, 
                       opp_state: DroneState) -> np.ndarray:
        """
        Main control computation using NRHDG.
        
        Returns the first control action from the saddle-point solution.
        """
        # Solve for saddle-point solution over the horizon
        u_sequence, v_sequence = self.solve_saddle_point(ego_state, opp_state)
        
        # Apply first control action (receding horizon principle)
        u_optimal = u_sequence[:3]
        
        # Clip to maximum thrust
        thrust_magnitude = np.linalg.norm(u_optimal)
        if thrust_magnitude > self.config.max_thrust:
            u_optimal = u_optimal * (self.config.max_thrust / thrust_magnitude)
        
        return u_optimal


# Example usage and testing
if __name__ == "__main__":
    from functions_anu import DynamicRoleSwitching
    
    # Initialize components
    role_switcher = DynamicRoleSwitching()
    config = NRHDGConfig(T=1.0, dt=0.1, n_steps=10)
    controller = NRHDGController(role_switcher, config)
    
    # Create initial states
    ego_state = DroneState(
        5,             # mass
        0.0, 0.0, 0.0, # x y z
        1.0, 0.0, 0.5, # vx vy vz
        0.0, 0.0, 0.0, # ax ay az
        0.0, 0.5,      # theta, theta_dot
        0.0            # timestamp
    )
    
    opp_state = DroneState( # have opp start ahead
        5,             # mass
        2.0, 2.0, 1.0, # x y z
        1.2, 0.1, 0.6, # vx vy vz
        0.0, 0.0, 0.0, # ax ay az
        0.5, 0.6,      # theta, theta_dot
        0.0            # timestamp
    )
    
    # Compute optimal control
    print("Computing optimal control using NRHDG...")
    u_optimal = controller.compute_control(ego_state, opp_state)
    
    print(f"\nOptimal control thrust: {u_optimal}")
    print(f"Thrust magnitude: {np.linalg.norm(u_optimal):.3f}")
    
    # Show role-switching behavior
    details = role_switcher.compute_G_O_with_details(
        ego_state.position(), ego_state.theta,
        opp_state.position(), opp_state.theta
    )
    print(f"\nRole-switching mode: {details['mode']}")
    print(f"G_O potential value: {details['G_O']:.4f}")
