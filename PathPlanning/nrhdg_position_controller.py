import numpy as np
from nrhdg_agent import SimulinkNRHDGInterface, NRHDGConfig

_controller_instance = None

def initialize_controller(waypoints_array):
    """Initialize once at simulation start"""
    global _controller_instance
    
    waypoints = np.array(waypoints_array)
    
    config = NRHDGConfig(
        T=0.5,          # Shorter horizon for position control
        dt=0.05,        # Smaller timestep
        n_steps=10,
        mass=1.0,
        drag_coeff=0.1,
        max_thrust=20.0,
        w_progress=2.0,     # Higher for aggressive racing
        w_deviation=1.0,
        w_control=0.05      # Lower for responsive control
    )
    
    _controller_instance = SimulinkNRHDGInterface(waypoints, config)
    print("NRHDG Position Controller initialized")


def compute_velocity_command(current_position, current_velocity, 
                             opponent_position, opponent_velocity,
                             desired_yaw, time):
    """
    Compute desired velocity command using NRHDG.
    
    Inputs:
        current_position: [x, y, z] (3x1)
        current_velocity: [vx, vy, vz] (3x1)
        opponent_position: [x, y, z] (3x1)
        opponent_velocity: [vx, vy, vz] (3x1)
        desired_yaw: scalar (rad)
        time: scalar (s)
    
    Outputs:
        velocity_cmd: [vx_cmd, vy_cmd, vz_cmd] (3x1)
    """
    global _controller_instance
    
    if _controller_instance is None:
        return np.zeros(3)
    
    try:
        # Get optimal acceleration from NRHDG
        accel_nrhdg = _controller_instance.compute_control_from_simulink(
            np.array(current_position),
            np.array(current_velocity),
            np.array(opponent_position),
            np.array(opponent_velocity),
            time
        )
        
        # Convert acceleration to velocity command
        # Using simple integration: v_cmd = v_current + a * dt
        dt = 0.1  # Command update rate
        velocity_cmd = current_velocity + accel_nrhdg * dt
        
        # Velocity limits (adjust to your UAV specs)
        max_velocity = 5.0  # m/s
        velocity_cmd = np.clip(velocity_cmd, -max_velocity, max_velocity)
        
        return velocity_cmd
        
    except Exception as e:
        print(f"NRHDG error: {e}")
        return current_velocity  # Fallback to current velocity
