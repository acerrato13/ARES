import numpy as np
from nrhdg_agent import SimulinkNRHDGInterface, NRHDGConfig

# Global controller instance
_controller = None
_initialized = False

def initialize(waypoints):
    """
    Initialize controller (called once)
    waypoints: Nx3 numpy array
    """
    global _controller, _initialized
    
    if not _initialized:
        config = NRHDGConfig(
            T=0.5,
            dt=0.05,
            n_steps=10,
            mass=1.0,
            drag_coeff=0.1,
            max_thrust=20.0,
            w_progress=2.0,
            w_deviation=1.0,
            w_control=0.05
        )
        
        _controller = SimulinkNRHDGInterface(waypoints, config)
        _initialized = True
        print("NRHDG Controller initialized")
    
    return True


def compute_velocity_cmd(current_pos, current_vel, opp_pos, opp_vel, time):
    """
    Main function called by Simulink at each timestep.
    
    Inputs:
        current_pos: (3,) array [x, y, z]
        current_vel: (3,) array [vx, vy, vz]
        opp_pos: (3,) array [x, y, z]
        opp_vel: (3,) array [vx, vy, vz]
        time: float
    
    Returns:
        velocity_cmd: (3,) array [vx_cmd, vy_cmd, vz_cmd]
    """
    global _controller
    
    if _controller is None:
        return np.zeros(3)
    
    try:
        # Get optimal acceleration from NRHDG
        accel = _controller.compute_control_from_simulink(
            current_pos, current_vel, opp_pos, opp_vel, time
        )
        
        # Convert to velocity command
        dt = 0.1
        vel_cmd = current_vel + accel * dt
        
        # Apply velocity limits
        max_vel = 5.0
        vel_cmd = np.clip(vel_cmd, -max_vel, max_vel)
        
        return vel_cmd
        
    except Exception as e:
        print(f"NRHDG error: {e}")
        return current_vel


def compute_blended_velocity_cmd(current_pos, current_vel, opp_pos, opp_vel, 
                                  obstacle_vel_cmd, time, blend_alpha=0.6):
    """
    Blend NRHDG with obstacle avoidance.
    
    Inputs:
        current_pos: (3,) array
        current_vel: (3,) array
        opp_pos: (3,) array
        opp_vel: (3,) array
        obstacle_vel_cmd: (3,) array from original position controller
        time: float
        blend_alpha: float 0-1 (weight for NRHDG)
    
    Returns:
        blended_velocity_cmd: (3,) array
    """
    global _controller
    
    if _controller is None:
        return obstacle_vel_cmd
    
    try:
        # Get NRHDG velocity command
        vel_nrhdg = compute_velocity_cmd(current_pos, current_vel, 
                                         opp_pos, opp_vel, time)
        
        # Blend
        vel_cmd = blend_alpha * vel_nrhdg + (1 - blend_alpha) * obstacle_vel_cmd
        
        return vel_cmd
        
    except Exception as e:
        print(f"NRHDG error: {e}")
        return obstacle_vel_cmd
