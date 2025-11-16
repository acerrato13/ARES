import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import your 3D agent classes
from PathPlanning.nrhdg_agent import NRHDGController3D
from PathPlanning.dynamic_role_switching import DynamicRoleSwitching3D
from PathPlanning.core.State import DroneState 
from PathPlanning.config import NRHDGConfig
from PathPlanning.nmpc_opponent import NMPCOpponent
from PathPlanning.control.DynamicProjections import DynamicProjections
from PathPlanning.core.Shape import DRONE_PRESETS


def compute_path_following_control_3d(state, role_switcher, config, kp_xy=3.0, kd_xy=2.0, kp_z=2.0, kd_z=1.0, forward_gain=1.5):
    """
    Simple 3D PD controller that follows the reference path produced by
    `role_switcher.path_function(theta)` which returns a 3D point.
    Returns an acceleration vector [ax, ay, az] (treat as acceleration/force
    in DynamicProjections which uses forces as accelerations).
    """
    # Reference point on the path (3D)
    r_ref = role_switcher.path_function(state.theta)

    # XY part: follow the path projection
    r_xy = r_ref[:2]
    pos_xy = state.position[:2]
    vel_xy = state.velocity[:2]

    # Approximate path tangent in XY (finite-difference style from analytic form)
    dr_dtheta_xy = np.array([6 * np.cos(state.theta), 3 * np.cos(2 * state.theta)])
    tangent_xy = dr_dtheta_xy / (np.linalg.norm(dr_dtheta_xy) + 1e-9)

    pos_error_xy = r_xy - pos_xy
    vel_desired_xy = kp_xy * pos_error_xy + forward_gain * tangent_xy
    vel_error_xy = vel_desired_xy - vel_xy
    accel_xy = kp_xy * pos_error_xy + kd_xy * vel_error_xy

    # Z part: simple PD to follow reference altitude
    z_ref = r_ref[2]
    pos_z = state.position[2]
    vel_z = state.velocity[2]
    z_error = z_ref - pos_z
    accel_z = kp_z * z_error - kd_z * vel_z

    # Clip accelerations to vehicle limits (approx)
    accel = np.array([accel_xy[0], accel_xy[1], accel_z])
    accel_mag = np.linalg.norm(accel)
    max_acc = getattr(config, 'max_acceleration', 10.0)
    if accel_mag > max_acc:
        accel = accel * (max_acc / accel_mag)

    return accel


def simulate_nrhdg_agent_3d(controller, initial_state, opponent_state, projections: DynamicProjections, steps, dt=0.1):
    """
    Simulate NRHDG controller in 3D by using the 2D controller outputs for XY
    and a simple PD altitude controller for Z. Uses `projections.predict_state`
    to propagate the DroneState.
    """
    trajectory = [initial_state]
    controls = []

    ego_state = initial_state
    opp_state = opponent_state

    for step in range(steps):
        # Compute NRHDG control (2D output)
        try:
            u_xy = controller.compute_control(ego_state, opp_state)  # 2D control output
        except Exception as e:
            print(f"Warning: NRHDG control failed at step {step}: {e}")
            # Fallback to path following accel
            applied_acc = compute_path_following_control_3d(ego_state, controller.role_switcher, controller.config)
        else:
            # Altitude controller using path z reference
            r_ref = controller.role_switcher.path_function(ego_state.theta)
            z_ref = r_ref[2]
            kp_z = 2.0
            kd_z = 1.0
            accel_z = kp_z * (z_ref - ego_state.position[2]) - kd_z * ego_state.velocity[2]

            # Combine 2D control with Z control
            applied_acc = np.array([u_xy[0], u_xy[1], accel_z])

        controls.append(applied_acc)

        # Propagate ego using DynamicProjections (applied_acc treated as force/acc)
        ego_state = projections.predict_state(ego_state, dt, applied_acc)

        # Update virtual opponent (follows path) using path-following 3D controller
        u_opp_acc = compute_path_following_control_3d(opp_state, controller.role_switcher, controller.config)
        opp_state = projections.predict_state(opp_state, dt, u_opp_acc)

        # Update theta based on projection of velocity onto path tangent
        dr_dtheta = np.array([6 * np.cos(ego_state.theta), 3 * np.cos(2 * ego_state.theta), 6 * np.cos(ego_state.theta/2)])
        dr_norm = np.linalg.norm(dr_dtheta[:2]) + 1e-9
        tangent = dr_dtheta[:2] / dr_norm
        theta_dot = np.dot(ego_state.velocity[:2], tangent) / dr_norm
        ego_state.theta = ego_state.theta + theta_dot * dt
        ego_state.theta_dot = theta_dot

        trajectory.append(ego_state)

    return trajectory, controls


def simulate_nmpc_agent_3d(agent: NMPCOpponent, initial_state: DroneState, role_switcher, projections: DynamicProjections, steps, dt=0.1):
    trajectory = [initial_state]
    state = initial_state

    for step in range(steps):
        # Lookahead on path
        target_theta = state.theta + 0.5
        target_pos = role_switcher.path_function(target_theta)

        # Build vectors for NMPC (state: [x,y,z,vx,vy,vz], target: [x,y,z,0,0,0])
        state_vec = np.concatenate([state.position, state.velocity])
        target_vec = np.concatenate([target_pos, np.zeros(3)])

        try:
            control_acc = agent.compute_control(state_vec, target_vec)
            # propagate
            state = projections.predict_state(state, dt, control_acc)
        except Exception as e:
            print(f"Warning: NMPC failed at step {step}: {e}")
            state = projections.predict_state(state, dt, None)

        # Update theta as for NRHDG
        dr_dtheta = np.array([6 * np.cos(state.theta), 3 * np.cos(2 * state.theta), 6 * np.cos(state.theta/2)])
        dr_norm = np.linalg.norm(dr_dtheta[:2]) + 1e-9
        tangent = dr_dtheta[:2] / dr_norm
        theta_dot = np.dot(state.velocity[:2], tangent) / dr_norm
        state.theta = state.theta + theta_dot * dt
        state.theta_dot = theta_dot

        trajectory.append(state)

    return trajectory


def compare_agents_3d():
    """Compare NRHDG vs NMPC in a 3D simulated environment using DynamicProjections."""
    print("Initializing 3D agents...")

    # Setup role switcher and NRHDG controller (uses 2D core but we lift to 3D)
    role_switcher = DynamicRoleSwitching3D()
    nrhdg_config = NRHDGConfig(
        T=0.5,
        dt=0.05,
        n_steps=10,
        w_progress=3.0,
        w_deviation=1.0,
        w_control=0.3,
        max_thrust=50.0,
        mass=2.0,
        drag_coeff=0.1
    )
    nrhdg_controller = NRHDGController3D(role_switcher, nrhdg_config)

    # NMPC agent
    nmpc_agent = NMPCOpponent(prediction_horizon=10, dt=0.1)

    # Projections and drone shape
    drone_shape = DRONE_PRESETS["Standard_Quadcopter"]
    projections = DynamicProjections(drone_shape=drone_shape, air_resistance_coeff=0.1, gravity=9.81)

    # Initial states (array-style DroneState)
    nrhdg_initial = DroneState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        acceleration=np.zeros(3),
        alpha=np.zeros(3),
        theta=0.0,
        theta_dot=0.0,
        timestamp=0.0
    )

    nrhdg_opponent = DroneState(
        position=np.array([1.0, 0.5, 0.5]),
        velocity=np.array([0.0, 0.0, 0.0]),
        acceleration=np.zeros(3),
        alpha=np.zeros(3),
        theta=0.2,
        theta_dot=0.0,
        timestamp=0.0
    )

    nmpc_initial = DroneState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        acceleration=np.zeros(3),
        alpha=np.zeros(3),
        theta=0.0,
        theta_dot=0.0,
        timestamp=0.0
    )

    steps = 200
    dt = 0.1

    print("Running 3D simulations...")
    nrhdg_traj, nrhdg_ctrls = simulate_nrhdg_agent_3d(nrhdg_controller, nrhdg_initial, nrhdg_opponent, projections, steps, dt)
    nmpc_traj = simulate_nmpc_agent_3d(nmpc_agent, nmpc_initial, role_switcher, projections, steps, dt)

    # Extract positions
    nrhdg_positions = np.array([s.position for s in nrhdg_traj])
    nmpc_positions = np.array([s.position for s in nmpc_traj])

    # Plot XY and Z
    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.plot(nrhdg_positions[:, 0], nrhdg_positions[:, 1], 'b-', label='NRHDG')
    ax1.plot(nmpc_positions[:, 0], nmpc_positions[:, 1], 'r-', label='NMPC')
    path_theta = np.linspace(0, max(nrhdg_traj[-1].theta, nmpc_traj[-1].theta), 300)
    path_pts = role_switcher.path_function(path_theta)
    ax1.plot(path_pts[0, :], path_pts[1, :], 'k--', alpha=0.3, label='Reference')
    ax1.set_title('XY Trajectory')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.axis('equal')

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(np.arange(len(nrhdg_traj)) * dt, nrhdg_positions[:, 2], 'b-', label='NRHDG Z')
    ax2.plot(np.arange(len(nmpc_traj)) * dt, nmpc_positions[:, 2], 'r-', label='NMPC Z')
    ax2.set_title('Altitude over time')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Z [m]')
    ax2.legend()

    plt.tight_layout()
    plt.savefig('nrhdg_vs_nmpc_comparison_3d.png', dpi=200)
    print("Saved 3D comparison plot: nrhdg_vs_nmpc_comparison_3d.png")
    plt.show()
    # (2D summary printing removed — use compare_agents_2d() to get 2D results)

def testing():
    # ego_state = DroneState(
    #     x=0.0, y=0.0,
    #     vx=0.0, vy=0.0,
    #     ax=0.0, ay=0.0,
    #     theta=0.0,
    #     theta_dot=0.5,
    #     timestamp=0.0
    # )

    # opp_state = DroneState(
    #     x=1.0, y=0.5,
    #     vx=0.0, vy=0.0,
    #     ax=0.0, ay=0.0,
    #     theta=0.2,
    #     theta_dot=0.5,
    #     timestamp=0.0
    # )

    # role_switcher = DynamicRoleSwitching2D()

    # nrhdg_config = NRHDGConfig(
    #     T=0.5,
    #     dt=0.05,
    #     n_steps=10,
    #     w_progress=3.0,
    #     w_deviation=1.0,
    #     w_control=0.3,
    #     max_thrust=50.0,
    #     mass=0.5,
    #     drag_coeff=0.1
    # )

    # controller = NRHDGController2D(role_switcher, nrhdg_config)

    # u = controller.compute_control(ego_state, opp_state)

    # dt = 0.1

    # new_state = update_drone_state_2d(ego_state, u, dt, controller.config)

    #position = [new_state.x, new_state.y]
    position = [1, 2]

    print(f"Position: {position}")

    return position

if __name__ == "__main__":
    # Run the 3D comparison by default (invokes NRHDG lifted to 3D, NMPC 3D,
    # dynamic role switching, and DynamicProjections).
    compare_agents_3d()
