import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import your 2D agent classes
from PathPlanning.nrhdg_agent import NRHDGController2D
from PathPlanning.dynamic_role_switching import DynamicRoleSwitching2D
from PathPlanning.core import DroneState 
from config import NRHDGConfig
from nmpc_opponent import NMPCOpponent


def simulate_nrhdg_agent_2d(controller, initial_state, opponent_state, steps, dt=0.1):
    """
    Simulate the 2D NRHDG agent.
    
    Parameters:
    -----------
    controller : NRHDGController2D
        The NRHDG controller instance
    initial_state : DroneState
        Initial state of the drone
    opponent_state : DroneState
        Opponent state (can be virtual/reference follower)
    steps : int
        Number of simulation steps
    dt : float
        Time step size
    
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
        # Compute NRHDG control (2D)
        try:
            u = controller.compute_control(ego_state, opp_state)
            
            if step % 20 == 0:
                print(f"Step {step}: theta={ego_state.theta:.3f}, control={u}")
        except Exception as e:
            print(f"Warning: NRHDG control failed at step {step}: {e}")
            # Fallback to path following
            u = compute_path_following_control_2d(
                ego_state,
                controller.role_switcher,
                controller.config
            )
        
        controls.append(u)
        
        # Update ego state
        ego_state = update_drone_state_2d(ego_state, u, dt, controller.config)
        
        # Update virtual opponent (follows path)
        u_opp = compute_path_following_control_2d(
            opp_state,
            controller.role_switcher,
            controller.config
        )
        opp_state = update_drone_state_2d(opp_state, u_opp, dt, controller.config)
        
        trajectory.append(ego_state)
    
    return trajectory, controls


def compute_path_following_control_2d(state, role_switcher, config, kp=3.0, kd=2.0, forward_gain=1.5):
    """
    Simple PD controller for 2D path following.
    
    Parameters:
    -----------
    state : DroneState
        Current drone state
    role_switcher : DynamicRoleSwitching2D
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
        Control thrust vector [ux, uy]
    """
    # Get reference position on path (2D)
    r_ref = role_switcher.path_function(state.theta)[:2]
    
    # Get reference tangent (for forward motion)
    dr_dtheta = np.array([
        6 * np.cos(state.theta),
        6 * np.cos(2 * state.theta)
    ])
    tangent = dr_dtheta / (np.linalg.norm(dr_dtheta) + 1e-6)
    
    # Position error (current position vs reference path)
    pos_error = r_ref - state.position()[:2]
    
    # Desired velocity: towards path + along path
    vel_desired = kp * pos_error + forward_gain * tangent
    
    # Velocity error
    vel_error = vel_desired - state.velocity()[:2]
    
    # PD control: F = m * (kp * pos_error + kd * vel_error)
    control = config.mass * (kp * pos_error + kd * vel_error)
    
    # Limit control magnitude
    control_mag = np.linalg.norm(control)
    if control_mag > config.max_thrust:
        control = control * (config.max_thrust / control_mag)
    
    return control


def update_drone_state_2d(state, control, dt, config):
    """
    Update 2D drone state using simple point-mass dynamics.
    
    Parameters:
    -----------
    state : DroneState
        Current drone state
    control : np.ndarray
        Control thrust vector [ux, uy]
    dt : float
        Time step
    config : NRHDGConfig
        Configuration with mass and drag_coeff
    
    Returns:
    --------
    new_state : DroneState
        Updated drone state
    """
    # Get current state (2D only)
    pos = state.position()[:2]
    vel = state.velocity()[:2]
    
    # Compute acceleration: a = u/m - k*v (no gravity in 2D)
    accel = control / config.mass - config.drag_coeff * vel
    
    # Integrate using Euler method
    new_pos = pos + vel * dt
    new_vel = vel + accel * dt
    
    # Update theta (path parameter) based on velocity projection onto path tangent
    dr_dtheta = np.array([
        6 * np.cos(state.theta),
        6 * np.cos(2 * state.theta)
    ])
    
    # Compute path tangent and project velocity
    dr_norm = np.linalg.norm(dr_dtheta)
    if dr_norm > 1e-6:
        tangent = dr_dtheta / dr_norm
        # theta_dot = (velocity · tangent) / ||dr/dtheta||
        theta_dot = np.dot(new_vel, tangent) / dr_norm
    else:
        theta_dot = 0.0
    
    new_theta = state.theta + theta_dot * dt
    
    # Create new state (2D DroneState)
    return DroneState(
        x=float(new_pos[0]),
        y=float(new_pos[1]),
        vx=float(new_vel[0]),
        vy=float(new_vel[1]),
        ax=float(accel[0]),
        ay=float(accel[1]),
        theta=float(new_theta),
        theta_dot=float(theta_dot),
        timestamp=state.timestamp + dt
    )


def simulate_nmpc_agent_2d(agent, initial_state, role_switcher, steps, dt=0.1):
    """
    Simulate the NMPC agent following the 2D reference path.
    
    Parameters:
    -----------
    agent : NMPCOpponent
        The NMPC agent instance
    initial_state : DroneState
        Initial state as DroneState
    role_switcher : DynamicRoleSwitching2D
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
        target_pos = role_switcher.path_function(target_theta)[:2]
        
        # Convert to format NMPC expects [x, y, vx, vy]
        state_vec = np.array([state.x, state.y, state.vx, state.vy])
        target_vec = np.concatenate([target_pos, np.zeros(2)])
        
        # Compute NMPC control
        try:
            control = agent.compute_control(state_vec, target_vec)
            
            # Update state based on NMPC control
            state_vec_new = agent.dynamics(state_vec, control)
        except Exception as e:
            print(f"Warning: NMPC failed at step {step}: {e}")
            # Use simple control
            state_vec_new = state_vec + np.array([state.vx, state.vy, 0, 0]) * dt
        
        # Update theta based on progress
        vel = state_vec_new[2:4]
        dr_dtheta = np.array([
            6 * np.cos(state.theta),
            6 * np.cos(2 * state.theta)
        ])
        tangent = dr_dtheta / (np.linalg.norm(dr_dtheta) + 1e-6)
        theta_dot = np.dot(vel, tangent) / (np.linalg.norm(dr_dtheta) + 1e-6)
        new_theta = state.theta + theta_dot * dt
        
        # Create new DroneState
        state = DroneState(
            x=float(state_vec_new[0]),
            y=float(state_vec_new[1]),
            vx=float(state_vec_new[2]),
            vy=float(state_vec_new[3]),
            ax=0.0, ay=0.0,
            theta=float(new_theta),
            theta_dot=float(theta_dot),
            timestamp=state.timestamp + dt
        )
        
        trajectory.append(state)
    
    return trajectory


def compare_agents_2d():
    """
    Compare the 2D NRHDG and NMPC agents in a racing scenario.
    """
    print("Initializing 2D agents...")
    
    # Initialize NRHDG agent
    role_switcher = DynamicRoleSwitching2D()
    nrhdg_config = NRHDGConfig(
        T=0.5,
        dt=0.05,
        n_steps=10,
        w_progress=3.0,
        w_deviation=1.0,
        w_control=0.3,
        max_thrust=50.0,
        mass=0.5,
        drag_coeff=0.1
    )
    nrhdg_controller = NRHDGController2D(role_switcher, nrhdg_config)
    
    # Initialize NMPC agent
    nmpc_agent = NMPCOpponent(prediction_horizon=10, dt=0.1)
    
    # Define initial conditions
    print("Setting up initial conditions...")
    
    # NRHDG drone starts at origin
    nrhdg_initial = DroneState(
        x=0.0, y=0.0,
        vx=0.0, vy=0.0,
        ax=0.0, ay=0.0,
        theta=0.0,
        theta_dot=0.5,
        timestamp=0.0
    )
    
    # Virtual opponent for NRHDG (starts slightly ahead)
    nrhdg_opponent = DroneState(
        x=1.0, y=0.5,
        vx=0.0, vy=0.0,
        ax=0.0, ay=0.0,
        theta=0.2,
        theta_dot=0.5,
        timestamp=0.0
    )
    
    # NMPC drone starts at same position as NRHDG
    nmpc_initial = DroneState(
        x=0.0, y=0.0,
        vx=0.0, vy=0.0,
        ax=0.0, ay=0.0,
        theta=0.0,
        theta_dot=0.5,
        timestamp=0.0
    )
    
    # Simulate agents
    print("Running simulations...")
    steps = 100
    dt = 0.1
    
    # NRHDG simulation
    print("\nSimulating 2D NRHDG agent...")
    nrhdg_trajectory, nrhdg_controls = simulate_nrhdg_agent_2d(
        nrhdg_controller, nrhdg_initial, nrhdg_opponent, steps, dt
    )
    
    # NMPC simulation
    print("\nSimulating 2D NMPC agent...")
    nmpc_trajectory = simulate_nmpc_agent_2d(nmpc_agent, nmpc_initial, role_switcher, steps, dt)
    
    # Extract positions for plotting
    nrhdg_positions = np.array([[s.x, s.y] for s in nrhdg_trajectory])
    nmpc_positions = np.array([[s.x, s.y] for s in nmpc_trajectory])
    
    # Get reference path for visualization
    theta_range = np.linspace(0, max(nrhdg_trajectory[-1].theta, nmpc_trajectory[-1].theta), 200)
    path_points = role_switcher.path_function(theta_range)
    
    # Plot results
    print("\nGenerating plots...")
    
    fig = plt.figure(figsize=(15, 5))
    
    # XY trajectory plot
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.plot(nrhdg_positions[:, 0], nrhdg_positions[:, 1], 'b-', linewidth=2, label="NRHDG")
    ax1.plot(nmpc_positions[:, 0], nmpc_positions[:, 1], 'r-', linewidth=2, label="NMPC")
    ax1.plot(path_points[0, :], path_points[1, :], 'k:', linewidth=1, alpha=0.3, label="Reference")
    ax1.scatter(nrhdg_positions[0, 0], nrhdg_positions[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('2D Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Theta (path parameter) evolution
    ax2 = fig.add_subplot(1, 3, 2)
    time = np.arange(len(nrhdg_trajectory)) * dt
    nrhdg_theta = [s.theta for s in nrhdg_trajectory]
    nmpc_theta = [s.theta for s in nmpc_trajectory]
    ax2.plot(time, nrhdg_theta, 'b-', linewidth=2, label="NRHDG")
    ax2.plot(time, nmpc_theta, 'r-', linewidth=2, label="NMPC")
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('θ (Path Parameter)')
    ax2.set_title('Progress Along Path')
    ax2.legend()
    ax2.grid(True)
    
    # Control effort
    ax3 = fig.add_subplot(1, 3, 3)
    nrhdg_controls_array = np.array(nrhdg_controls)
    nrhdg_control_magnitude = np.linalg.norm(nrhdg_controls_array, axis=1)
    time_controls = np.arange(len(nrhdg_controls)) * dt
    ax3.plot(time_controls, nrhdg_control_magnitude, 'b-', linewidth=2, label='NRHDG')
    ax3.axhline(y=nrhdg_config.max_thrust, color='gray', linestyle='--',
                linewidth=1, label='Max Thrust', alpha=0.5)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Control Magnitude [N]')
    ax3.set_title('Control Effort (2D)')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('nrhdg_vs_nmpc_comparison_2d.png', dpi=300, bbox_inches='tight')
    print("Plot saved as 'nrhdg_vs_nmpc_comparison_2d.png'")
    plt.show()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("2D SIMULATION SUMMARY")
    print("="*60)
    print(f"Simulation time: {steps * dt:.1f} seconds ({steps} steps)")
    
    print(f"\nNRHDG Drone:")
    print(f"  Final position: [{nrhdg_trajectory[-1].x:.2f}, {nrhdg_trajectory[-1].y:.2f}]")
    print(f"  Final theta: {nrhdg_trajectory[-1].theta:.3f}")
    print(f"  Average speed: {np.mean([np.linalg.norm([s.vx, s.vy]) for s in nrhdg_trajectory]):.2f} m/s")
    print(f"  Average control: {np.mean(nrhdg_control_magnitude):.2f} N")
    
    print(f"\nNMPC Drone:")
    print(f"  Final position: [{nmpc_trajectory[-1].x:.2f}, {nmpc_trajectory[-1].y:.2f}]")
    print(f"  Final theta: {nmpc_trajectory[-1].theta:.3f}")
    print(f"  Average speed: {np.mean([np.linalg.norm([s.vx, s.vy]) for s in nmpc_trajectory]):.2f} m/s")
    
    # Calculate path tracking errors
    nrhdg_errors = []
    nmpc_errors = []
    for s in nrhdg_trajectory:
        r_ref = role_switcher.path_function(s.theta)[:2]
        nrhdg_errors.append(np.linalg.norm(np.array([s.x, s.y]) - r_ref))
    for s in nmpc_trajectory:
        r_ref = role_switcher.path_function(s.theta)[:2]
        nmpc_errors.append(np.linalg.norm(np.array([s.x, s.y]) - r_ref))
    
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

def testing():
    ego_state = DroneState(
        x=0.0, y=0.0,
        vx=0.0, vy=0.0,
        ax=0.0, ay=0.0,
        theta=0.0,
        theta_dot=0.5,
        timestamp=0.0
    )

    opp_state = DroneState(
        x=1.0, y=0.5,
        vx=0.0, vy=0.0,
        ax=0.0, ay=0.0,
        theta=0.2,
        theta_dot=0.5,
        timestamp=0.0
    )

    role_switcher = DynamicRoleSwitching2D()

    nrhdg_config = NRHDGConfig(
        T=0.5,
        dt=0.05,
        n_steps=10,
        w_progress=3.0,
        w_deviation=1.0,
        w_control=0.3,
        max_thrust=50.0,
        mass=0.5,
        drag_coeff=0.1
    )

    controller = NRHDGController2D(role_switcher, nrhdg_config)

    u = controller.compute_control(ego_state, opp_state)

    dt = 0.1

    new_state = update_drone_state_2d(ego_state, u, dt, controller.config)

    #position = [new_state.x, new_state.y]
    position = [1, 2]

    print(f"Position: {position}")

    return position

if __name__ == "__main__":
    compare_agents_2d()
