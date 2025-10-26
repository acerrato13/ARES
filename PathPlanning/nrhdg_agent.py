"""
NRHDG Controller for Competitive Drone Racing
Compatible with existing DroneState and NRHDGConfig classes
"""

import numpy as np
from scipy.optimize import minimize
from typing import Tuple, Dict


class NRHDGController:
    """
    Nonlinear Receding-Horizon Differential Game controller for drone racing.
    
    Implements a two-player zero-sum game where:
    - Player U (ego drone) minimizes the objective
    - Player V (opponent) maximizes the objective
    """
    
    def __init__(self, role_switcher, config):
        """
        Parameters:
        -----------
        role_switcher : DynamicRoleSwitching instance
        config : NRHDGConfig instance with attributes:
            - T, dt, n_steps
            - w_progress, w_deviation, w_control
            - max_thrust, mass, drag_coeff
        """
        self.role_switcher = role_switcher
        self.config = config
        
    def _path_derivative(self, theta: float) -> np.ndarray:
        """Compute dr/dtheta for the reference path"""
        return np.array([
            6 * np.cos(theta),
            6 * np.cos(2 * theta),
            3 * np.cos(theta / 2)
        ])
    
    def _propagate_state(self, state, control: np.ndarray, dt: float):
        """
        Propagate state forward in time using Euler integration.
        
        Dynamics: ma = u - k*v + mg (point-mass with drag and gravity)
        """
        # Current state
        pos = state.position()
        vel = state.velocity()
        
        # Gravity (pointing down in Z)
        g = 0.0  # m/s^2
        gravity = np.array([0.0, 0.0, -g])
        
        # Compute acceleration: a = u/m - k*v + g
        accel = control / self.config.mass - self.config.drag_coeff * vel + gravity
        
        # Integrate position and velocity
        new_pos = pos + vel * dt
        new_vel = vel + accel * dt
        
        # Update theta: project velocity onto path tangent
        r_prime = self._path_derivative(state.theta)
        r_prime_norm = np.linalg.norm(r_prime)
        
        if r_prime_norm > 1e-6:
            tangent = r_prime / r_prime_norm
            theta_dot = np.dot(new_vel, tangent) / r_prime_norm
        else:
            theta_dot = state.theta_dot
        
        new_theta = state.theta + state.theta_dot * dt
        
        # Import DroneState - adjust this import to match your file structure
        from core import DroneState
        
        # Create new state
        return DroneState(
            mass=state.mass,
            x=float(new_pos[0]),
            y=float(new_pos[1]),
            z=float(new_pos[2]),
            vx=float(new_vel[0]),
            vy=float(new_vel[1]),
            vz=float(new_vel[2]),
            ax=float(accel[0]),
            ay=float(accel[1]),
            az=float(accel[2]),
            theta=float(new_theta),
            theta_dot=float(theta_dot),
            timestamp=state.timestamp + dt
        )
    
    def stage_cost(self, ego_state, opp_state, u_ego: np.ndarray, 
                   u_opp: np.ndarray, t: float) -> float:
        """
        Stage cost L_D(x, u, v, t) - the integrand in the objective function.
        
        Components:
        1. Progress: Reward moving along path (minimize -theta_dot)
        2. Deviation: Penalize distance from reference path
        3. Control effort: Penalize large control inputs
        4. Interaction: Use G_O potential for strategic behavior
        """
        # 1. Progress term
        progress_cost = -self.config.w_progress * ego_state.theta_dot
        
        # 2. Deviation penalty
        r_ego = self.role_switcher.path_function(ego_state.theta)
        deviation = np.linalg.norm(ego_state.position() - r_ego)
        deviation_cost = self.config.w_deviation * deviation**2
        
        # 3. Control effort
        control_cost = self.config.w_control * np.linalg.norm(u_ego)**2
        
        # 4. Opponent interaction (G_O potential)
        G_O = self.role_switcher.compute_G_O(
            ego_state.position(), ego_state.theta,
            opp_state.position(), opp_state.theta
        )
        interaction_cost = -10.0 * G_O
        
        return progress_cost + deviation_cost + control_cost + interaction_cost
    
    def terminal_cost(self, ego_state, opp_state) -> float:
        """
        Terminal cost φ_D(x(T)) - penalizes poor final state.
        """
        # Penalize being behind opponent
        theta_delta = opp_state.theta - ego_state.theta
        position_cost = -10.0 * theta_delta
        
        # Penalize deviation from path
        r_ego = self.role_switcher.path_function(ego_state.theta)
        deviation = np.linalg.norm(ego_state.position() - r_ego)
        deviation_cost = 5.0 * deviation**2
        
        return position_cost + deviation_cost
    
    def compute_objective(self, ego_state, opp_state,
                         u_sequence: np.ndarray, v_sequence: np.ndarray) -> float:
        """
        Compute J_D[u, v] = φ_D(x(T)) + ∫ L_D(x, u, v, t) dt
        
        Parameters:
        -----------
        ego_state : Initial ego drone state
        opp_state : Initial opponent drone state  
        u_sequence : Ego controls, shape (n_steps*3,)
        v_sequence : Opponent controls, shape (n_steps*3,)
        
        Returns:
        --------
        Total objective value (scalar)
        """
        ego_current = ego_state
        opp_current = opp_state
        
        integral_cost = 0.0
        
        # Integrate over horizon
        for k in range(self.config.n_steps):
            # Extract controls at time step k
            u_k = u_sequence[k*3:(k+1)*3]
            v_k = v_sequence[k*3:(k+1)*3]
            
            # Accumulate stage cost
            L_k = self.stage_cost(ego_current, opp_current, u_k, v_k, k * self.config.dt)
            integral_cost += L_k * self.config.dt
            
            # Propagate states
            ego_current = self._propagate_state(ego_current, u_k, self.config.dt)
            opp_current = self._propagate_state(opp_current, v_k, self.config.dt)
        
        # Add terminal cost
        phi_T = self.terminal_cost(ego_current, opp_current)
        
        return phi_T + integral_cost
    
    def solve_saddle_point(self, ego_state, opp_state, 
                          max_iterations: int = 10,  # Reduced from 15
                          tolerance: float = 5e-3,    # Loosened from 1e-3
                          verbose: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solve saddle-point problem: J[u*, v] ≤ J[u*, v*] ≤ J[u, v*]
        
        Uses alternating minimization:
        - Fix v, minimize over u (ego's best response)
        - Fix u, maximize over v (opponent's best response)
        - Repeat until convergence
        
        Returns:
        --------
        (u_sequence, v_sequence) : Saddle-point control sequences
        """
        # Initialize with small random values
        np.random.seed(42)  # For reproducibility
        u_seq = np.random.randn(self.config.n_steps * 3) * 0.1
        v_seq = np.random.randn(self.config.n_steps * 3) * 0.1
        
        # Control bounds
        bounds = [(-self.config.max_thrust, self.config.max_thrust)] * (self.config.n_steps * 3)
        
        for iteration in range(max_iterations):
            u_old = u_seq.copy()
            v_old = v_seq.copy()
            
            # Ego's move: minimize J[u, v_fixed]
            def objective_u(u):
                try:
                    return self.compute_objective(ego_state, opp_state, u, v_seq)
                except Exception as e:
                    print(f"Error in objective_u: {e}")
                    return 1e10  # Return large penalty on error
            
            result_u = minimize(
                objective_u, 
                u_seq, 
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 50, 'ftol': 1e-6}
            )
            
            if result_u.success:
                u_seq = result_u.x
            else:
                if verbose:
                    print(f"Warning: u-optimization did not converge: {result_u.message}")
            
            # Opponent's move: maximize J[u_fixed, v] = minimize -J[u_fixed, v]
            def objective_v(v):
                try:
                    return -self.compute_objective(ego_state, opp_state, u_seq, v)
                except Exception as e:
                    print(f"Error in objective_v: {e}")
                    return 1e10
            
            result_v = minimize(
                objective_v, 
                v_seq, 
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 50, 'ftol': 1e-6}
            )
            
            if result_v.success:
                v_seq = result_v.x
            else:
                if verbose:
                    print(f"Warning: v-optimization did not converge: {result_v.message}")
            
            # Check convergence
            u_change = np.linalg.norm(u_seq - u_old)
            v_change = np.linalg.norm(v_seq - v_old)
            max_change = max(u_change, v_change)
            
            if verbose:
                J_current = self.compute_objective(ego_state, opp_state, u_seq, v_seq)
                print(f"Iter {iteration+1}: J={J_current:.4f}, ||Δu||={u_change:.4f}, ||Δv||={v_change:.4f}")
            
            if max_change < tolerance:
                if verbose:
                    print(f"✓ Converged after {iteration+1} iterations")
                break
        else:
            if verbose:
                print(f"⚠ Reached max iterations ({max_iterations})")
        
        return u_seq, v_seq
    
    def compute_control(self, ego_state, opp_state, verbose: bool = False) -> np.ndarray:
        """
        Compute optimal control using NRHDG.
        
        Main interface for getting control commands.
        
        Parameters:
        -----------
        ego_state : Current ego drone state
        opp_state : Current opponent state
        verbose : Print debug info
        
        Returns:
        --------
        u_optimal : 3D control thrust vector [N]
        """
        # Solve saddle-point problem
        u_sequence, v_sequence = self.solve_saddle_point(
            ego_state, opp_state, verbose=verbose
        )
        
        # Extract first control (receding horizon)
        u_optimal = u_sequence[:3]
        
        # Enforce thrust limit
        thrust_mag = np.linalg.norm(u_optimal)
        if thrust_mag > self.config.max_thrust:
            u_optimal = u_optimal * (self.config.max_thrust / thrust_mag)
        
        if verbose:
            print(f"\n{'='*50}")
            print(f"Optimal Control: [{u_optimal[0]:.3f}, {u_optimal[1]:.3f}, {u_optimal[2]:.3f}] N")
            print(f"Magnitude: {thrust_mag:.3f} N")
            
            # Show game state
            details = self.role_switcher.compute_G_O_with_details(
                ego_state.position(), ego_state.theta,
                opp_state.position(), opp_state.theta
            )
            print(f"\nBehavior Mode: {details['mode']}")
            print(f"G_O Potential: {details['G_O']:.4f}")
            print(f"Theta Delta: {details['theta_delta']:.4f} ({'ahead' if details['theta_delta'] > 0 else 'behind'})")
            print(f"{'='*50}")
        
        return u_optimal
    
    def compute_control_with_diagnostics(self, ego_state, opp_state) -> Dict:
        """
        Compute control and return full diagnostics.
        
        Returns:
        --------
        Dictionary with keys:
        - control: Optimal 3D thrust
        - u_sequence: Full ego control trajectory
        - v_sequence: Full opponent control trajectory  
        - objective_value: J_D at saddle point
        - mode: 'Overtaking' or 'Obstructing'
        - diagnostics: Additional metrics
        """
        u_seq, v_seq = self.solve_saddle_point(ego_state, opp_state)
        
        J_saddle = self.compute_objective(ego_state, opp_state, u_seq, v_seq)
        
        u_optimal = u_seq[:3]
        thrust_mag = np.linalg.norm(u_optimal)
        if thrust_mag > self.config.max_thrust:
            u_optimal = u_optimal * (self.config.max_thrust / thrust_mag)
        
        details = self.role_switcher.compute_G_O_with_details(
            ego_state.position(), ego_state.theta,
            opp_state.position(), opp_state.theta
        )
        
        return {
            'control': u_optimal,
            'u_sequence': u_seq,
            'v_sequence': v_seq,
            'objective_value': J_saddle,
            'mode': details['mode'],
            'diagnostics': {
                'theta_delta': details['theta_delta'],
                'deviation_difference': details['R'],
                'G_O': details['G_O'],
                'thrust_magnitude': thrust_mag,
                'ego_theta': ego_state.theta,
                'opp_theta': opp_state.theta
            }
        }


if __name__ == "__main__":
    print("NRHDG Controller - Ready for Integration")
    print("\nUsage:")
    print("""
    from nrhdg_controller import NRHDGController
    from core import DroneState, NRHDGConfig
    from dynamic_role_switching import DynamicRoleSwitching
    
    # Setup
    role_switcher = DynamicRoleSwitching()
    config = NRHDGConfig()
    controller = NRHDGController(role_switcher, config)
    
    # Create states (adjust values as needed)
    ego = DroneState(
        mass=5.0, x=0, y=0, z=0, 
        vx=1, vy=0, vz=0.5,
        ax=0, ay=0, az=0, 
        theta=0.0, theta_dot=0.5
    )
    
    opp = DroneState(
        mass=5.0, x=2, y=1, z=1, 
        vx=1.2, vy=0.1, vz=0.6,
        ax=0, ay=0, az=0, 
        theta=0.5, theta_dot=0.6
    )
    
    # Compute control
    u = controller.compute_control(ego, opp, verbose=True)
    """)
