import sys
import os
# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from PathPlanning.functions_anu import DynamicRoleSwitching
from PathPlanning.nrhdg_agent import NRHDGController
from core import DroneState, NRHDGConfig
from nmpc_opponent import NMPCOpponent
import matplotlib.pyplot as plt


def simulate_nrhdg_agent(controller, initial_state, opponent_state, steps, dt=0.1, use_game_theoretic=True):
    """
    Simulate the NRHDG agent.
    
    Parameters:
    -----------
    controller : NRHDGController
        The NRHDG controller instance
    initial_state : DroneState
        Initial state of the drone
    opponent_state : DroneState
        Opponent state (can be virtual/reference follower)
    steps : int
        Number of simulation steps
    dt : float
        Time step size
    use_game_theoretic : bool
        If True, uses NRHDG controller. If False, uses simple path following
    
    Returns:
    --------
    trajectory : list of DroneState
        Trajectory of the drone
    controls : list of np.ndarray
        Control inputs applied
    """
    trajectory = [initial_state]
    controls = []
    
    ego_state = initial_state
    opp_state = opponent_state
    
    for step in range(steps):
        if use_game_theoretic:
            # Use actual NRHDG game-theoretic controller
            u = controller.compute_control(ego_state, opp_state, verbose=(step % 20 == 0))
        else:
            # Use simple path-following controller
            u = compute_path_following_control(
                ego_state, 
                controller.role_switcher, 
                controller.config
            )
        
        controls.append(u)
        
        # Update ego state
        ego_state = update_drone_state(ego_state, u, dt, controller.config)
        
        # Update virtual opponent (follows path at constant speed)
        u_opp = compute_path_following_control(
            opp_state,
            controller.role_switcher,
            controller.config
        )
        opp_state = update_drone_state(opp_state, u_opp, dt, controller.config)
        
        trajectory.append(ego_state)
    
    return trajectory, controls


def compute_path_following_control(state, role_switcher, config, kp=5.0, kd=2.0, forward_gain=3.0):
    """
    Simple PD controller for path following with gravity compensation.
    
    Generates control to:
    1. Track the reference path r(theta)
    2. Maintain forward progress along the path
    3. Compensate for gravity
    
    Parameters:
    -----------
    state : DroneState
        Current drone state
    role_switcher : DynamicRoleSwitching
        Has the path_function method
    config : NRHDGConfig
        Configuration with mass parameter
    kp : float
        Position gain
    kd : float
        Velocity gain
    forward_gain : float
        Gain for forward progress along path
    
    Returns:
    --------
    control : np.ndarray
       [<73;20;1M[<73;20;1M Control thrust vector [ux, uy, uz]
    """
    # Get reference position on path
    r_ref = role_switcher.path_function(state.theta)
    
    # Get reference tangent (for forward motion)
    dr_dtheta = np.array([
        6 * np.cos(state.theta),
        6 * np.cos(2 * state.theta),
        3 * np.cos(state.theta / 2)
    ])
    tangent = dr_dtheta / (np.linalg.norm(dr_dtheta) + 1e-6)
    
    # Position error (current position vs reference path)
    pos_error = r_ref - state.position()
    
    # Desired velocity: towards path + along path
    vel_desired = kp * pos_error + forward_gain * tangent
    
    # Velocity error
    vel_error = vel_desired - state.velocity()
    
    # PD control: F = m * (kp * pos_error + kd * vel_error)
    control = config.mass * (kp * pos_error + kd * vel_error)
    
    # Add gravity compensation
    g = 0.0  # m/s^2
    gravity_compensation = config.mass * np.array([0.0, 0.0, g])
    control = control + gravity_compensation
    
    # Limit control magnitude
    control_mag = np.linalg.norm(control)
    if control_mag > config.max_thrust:
        control = control * (config.max_thrust / control_mag)
    
    return control


def simulate_nmpc_agent(agent, initial_state, role_switcher, steps, dt=0.1):
    """
    Simulate the NMPC agent following the reference path.
    
    Parameters:
    -----------
    agent : NMPCOpponent
        The NMPC agent instance
    initial_state : DroneState
        Initial state as DroneState
    role_switcher : DynamicRoleSwitching
        For accessing the reference path
    steps : int
        Number of simulation steps
    dt : float
        Time step
    
    Returns:
    --------
    trajectory : list of DroneState
        Trajectory of the agent
    """
    trajectory = [initial_state]
    state = initial_state
    
    for step in range(steps):
        # Get target as the next point on the reference path
        target_theta = state.theta + 0.5  # Look ahead on path
        target_pos = role_switcher.path_function(target_theta)
        
        # Convert to format NMPC expects [x, y, z, vx, vy, vz]
        state_vec = np.array([state.x, state.y, state.z, state.vx, state.vy, state.vz])
        target_vec = np.concatenate([target_pos, np.zeros(3)])
        
        # Compute NMPC control
        control = agent.compute_control(state_vec, target_vec)
        
        # Update state based on NMPC control
        state_vec_new = agent.dynamics(state_vec, control)
        
        # Update theta based on progress
        vel = state_vec_new[3:6]
        dr_dtheta = np.array([
            6 * np.cos(state.theta),
            6 * np.cos(2 * state.theta),
            3 * np.cos(state.theta / 2)
        ])
        tangent = dr_dtheta / (np.linalg.norm(dr_dtheta) + 1e-6)
        theta_dot = np.dot(vel, tangent) / (np.linalg.norm(dr_dtheta) + 1e-6)
        new_theta = state.theta + theta_dot * dt
        
        # Create new DroneState
        from core import DroneState
        state = DroneState(
            mass=0.5,
            x=float(state_vec_new[0]),
            y=float(state_vec_new[1]),
            z=float(state_vec_new[2]),
            vx=float(state_vec_new[3]),
            vy=float(state_vec_new[4]),
            vz=float(state_vec_new[5]),
            ax=0.0, ay=0.0, az=0.0,  # Could compute from control
            theta=float(new_theta),
            theta_dot=float(theta_dot),
            timestamp=state.timestamp + dt
        )
        
        trajectory.append(state)
    
    return trajectory


def update_drone_state(state, control, dt, config):
    """
    Update drone state using simple point-mass dynamics.
    
    Parameters:
    -----------
    state : DroneState
        Current drone state
    control : np.ndarray
        Control thrust vector [ux, uy, uz]
    dt : float
        Time step
    config : NRHDGConfig
        Configuration with mass and drag_coeff
    
    Returns:
    --------
    new_state : DroneState
        Updated drone state
    """
    # Get current state
    pos = state.position()
    vel = state.velocity()
    
    # Gravity vector (pointing down in Z)
    g = 0.0  # m/s^2
    gravity = np.array([0.0, 0.0, -g])
    
    # Compute acceleration: a = u/m - k*v + g
    accel = control / config.mass - config.drag_coeff * vel + gravity
    
    # Integrate using Euler method
    new_pos = pos + vel * dt
    new_vel = vel + accel * dt
    
    # Update theta (path parameter) based on velocity projection
    # Simplified: assume theta_dot is proportional to forward velocity
    theta_dot = np.linalg.norm(new_vel) * 0.5  # Simplified
    new_theta = state.theta + theta_dot * dt
    
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


def compare_agents():
    """
    Compare the NRHDG and NMPC agents in a racing scenario.
    """
    print("Initializing agents...")
    
    # Initialize NRHDG agent
    role_switcher = DynamicRoleSwitching()
    nrhdg_config = NRHDGConfig(
        T=1.0,
        dt=0.1,
        n_steps=10,
        w_progress=1.0,
        w_deviation=2.0,
        w_control=0.1,
        max_thrust=250.0,
        mass=5.0,
        drag_coeff=0.1
    )
    nrhdg_controller = NRHDGController(role_switcher, nrhdg_config)
    
    # Initialize NMPC agent
    nmpc_agent = NMPCOpponent(prediction_horizon=10, dt=0.1)
    
    # Define initial conditions
    print("Setting up initial conditions...")
    
    # NRHDG drone starts at origin
    nrhdg_initial = DroneState(
        mass=0.5, 
        x=0.0, y=0.0, z=0.0,
        vx=1.0, vy=0.0, vz=0.5,
        ax=0.0, ay=0.0, az=0.0,
        theta=0.0,
        theta_dot=0.5,
        timestamp=0.0
    )
    
    # Virtual opponent for NRHDG (starts slightly ahead to trigger competitive behavior)
    nrhdg_opponent = DroneState(
        mass=0.5,
        x=1.0, y=0.5, z=0.5,
        vx=1.0, vy=0.0, vz=0.5,
        ax=0.0, ay=0.0, az=0.0,
        theta=0.2,  # Slightly ahead
        theta_dot=0.5,
        timestamp=0.0
    )
    
    # NMPC drone starts at same position as NRHDG
    nmpc_initial = DroneState(
        mass=0.5, 
        x=0.0, y=0.0, z=0.0,
        vx=1.0, vy=0.0, vz=0.5,
        ax=0.0, ay=0.0, az=0.0,
        theta=0.0,
        theta_dot=0.5,
        timestamp=0.0
    )
    
    # Simulate agents
    print("Running simulations...")
    steps = 100
    dt = 0.1
    
    # NRHDG simulation (using game-theoretic controller)
    print("\nSimulating NRHDG agent with game-theoretic controller...")
    nrhdg_trajectory, nrhdg_controls = simulate_nrhdg_agent(
        nrhdg_controller, nrhdg_initial, nrhdg_opponent, steps, dt, use_game_theoretic=True
    )
    
    # NMPC simulation
    print("\nSimulating NMPC agent...")
    nmpc_trajectory = simulate_nmpc_agent(nmpc_agent, nmpc_initial, role_switcher, steps, dt)
    
    # Extract positions for plotting
    nrhdg_positions = np.array([[s.x, s.y, s.z] for s in nrhdg_trajectory])
    nmpc_positions = np.array([[s.x, s.y, s.z] for s in nmpc_trajectory])
    
    # Get reference path for visualization
    theta_range = np.linspace(0, max(nrhdg_trajectory[-1].theta, nmpc_trajectory[-1].theta), 200)
    path_points = role_switcher.path_function(theta_range)
    
    # Plot results
    print("\nGenerating plots...")
    
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(nrhdg_positions[:, 0], nrhdg_positions[:, 1], nrhdg_positions[:, 2], 
             'b-', linewidth=2, label="NRHDG")
    ax1.plot(nmpc_positions[:, 0], nmpc_positions[:, 1], nmpc_positions[:, 2], 
             'r-', linewidth=2, label="NMPC")
    ax1.plot(path_points[0, :], path_points[1, :], path_points[2, :], 
             'g:', linewidth=1, alpha=0.5, label="Reference Path")
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)
    
    # XY trajectory plot
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(nrhdg_positions[:, 0], nrhdg_positions[:, 1], 'b-', linewidth=2, label="NRHDG")
    ax2.plot(nmpc_positions[:, 0], nmpc_positions[:, 1], 'r-', linewidth=2, label="NMPC")
    ax2.plot(path_points[0, :], path_points[1, :], 'k:', linewidth=1, alpha=0.3, label="Reference")
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('XY Trajectory Comparison')
    ax2.legend()
    ax2.grid(True)
    ax2.axis('equal')
    
    # Theta (path parameter) evolution
    ax3 = fig.add_subplot(2, 2, 3)
    time = np.arange(len(nrhdg_trajectory)) * dt
    nrhdg_theta = [s.theta for s in nrhdg_trajectory]
    nmpc_theta = [s.theta for s in nmpc_trajectory]
    ax3.plot(time, nrhdg_theta, 'b-', linewidth=2, label="NRHDG")
    ax3.plot(time, nmpc_theta, 'r-', linewidth=2, label="NMPC")
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('θ (Path Parameter)')
    ax3.set_title('Progress Along Path')
    ax3.legend()
    ax3.grid(True)
    
    # Control effort comparison
    ax4 = fig.add_subplot(2, 2, 4)
    nrhdg_controls_array = np.array(nrhdg_controls)
    nrhdg_control_magnitude = np.linalg.norm(nrhdg_controls_array, axis=1)
    time_controls = np.arange(len(nrhdg_controls)) * dt
    ax4.plot(time_controls, nrhdg_control_magnitude, 'b-', linewidth=2, label='NRHDG')
    ax4.axhline(y=nrhdg_config.max_thrust, color='gray', linestyle='--', 
                linewidth=1, label='Max Thrust', alpha=0.5)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Control Magnitude [N]')
    ax4.set_title('Control Effort')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    plt.savefig('nrhdg_vs_nmpc_comparison.png', dpi=300, bbox_inches='tight')
    print("Plot saved as 'nrhdg_vs_nmpc_comparison.png'")
    plt.show()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("SIMULATION SUMMARY")
    print("="*60)
    print(f"Simulation time: {steps * dt:.1f} seconds ({steps} steps)")
    
    print(f"\nNRHDG Drone:")
    print(f"  Final position: [{nrhdg_trajectory[-1].x:.2f}, {nrhdg_trajectory[-1].y:.2f}, {nrhdg_trajectory[-1].z:.2f}]")
    print(f"  Final theta: {nrhdg_trajectory[-1].theta:.3f}")
    print(f"  Average speed: {np.mean([s.speed() for s in nrhdg_trajectory]):.2f} m/s")
    print(f"  Average control: {np.mean(nrhdg_control_magnitude):.2f} N")
    
    print(f"\nNMPC Drone:")
    print(f"  Final position: [{nmpc_trajectory[-1].x:.2f}, {nmpc_trajectory[-1].y:.2f}, {nmpc_trajectory[-1].z:.2f}]")
    print(f"  Final theta: {nmpc_trajectory[-1].theta:.3f}")
    print(f"  Average speed: {np.mean([s.speed() for s in nmpc_trajectory]):.2f} m/s")
    
    # Calculate path tracking errors
    nrhdg_errors = []
    nmpc_errors = []
    for s in nrhdg_trajectory:
        r_ref = role_switcher.path_function(s.theta)
        nrhdg_errors.append(np.linalg.norm(s.position() - r_ref))
    for s in nmpc_trajectory:
        r_ref = role_switcher.path_function(s.theta)
        nmpc_errors.append(np.linalg.norm(s.position() - r_ref))
    
    print(f"\nPath Tracking Performance:")
    print(f"  NRHDG mean error: {np.mean(nrhdg_errors):.3f} m")
    print(f"  NMPC mean error: {np.mean(nmpc_errors):.3f} m")
    
    theta_diff = nrhdg_trajectory[-1].theta - nmpc_trajectory[-1].theta
    if abs(theta_diff) < 0.01:
        print(f"\n≈ Both agents finished at similar positions")
    elif theta_diff > 0:
        print(f"\n✓ NRHDG finished AHEAD by {theta_diff:.3f} radians")
    else:
        print(f"\n✗ NRHDG finished BEHIND by {-theta_diff:.3f} radians")
    print("="*60)


if __name__ == "__main__":
    compare_agents()
