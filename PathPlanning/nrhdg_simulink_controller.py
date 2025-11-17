import numpy as np
from nrhdg_agent import SimulinkNRHDGInterface, NRHDGConfig

# Global controller instance (persistent across Simulink calls)
_controller_instance = None
_waypoints = None

def initialize_controller(waypoints_array):
    """Initialize controller with waypoints (call once at simulation start)"""
    global _controller_instance, _waypoints
    
    # Convert to numpy array if needed
    _waypoints = np.array(waypoints_array)
    
    # Configure NRHDG
    config = NRHDGConfig(
        T=1.0,
        dt=0.1,
        n_steps=10,
        mass=1.0,           # Adjust to your quadrotor mass
        drag_coeff=0.1,
        max_thrust=20.0,
        w_progress=1.0,
        w_deviation=0.5,
        w_control=0.1
    )
    
    _controller_instance = SimulinkNRHDGInterface(_waypoints, config)
    print(f"NRHDG Controller initialized with {len(_waypoints)} waypoints")
    

def compute_nrhdg_control(uav_state, opponent_state, desired_position, time):
    """
    Compute NRHDG control signal.
    
    Inputs from Simulink:
        uav_state: [px, py, pz, vx, vy, vz] (6x1)
        opponent_state: [px, py, pz, vx, vy, vz] (6x1)
        desired_position: [x, y, z] from waypoint follower (3x1)
        time: Current simulation time (scalar)
    
    Outputs to Simulink:
        control_signal: [ux, uy, uz] thrust vector (3x1)
    """
    global _controller_instance
    
    if _controller_instance is None:
        # If not initialized, return zero control
        return np.zeros(3)
    
    try:
        # Extract positions and velocities
        ego_pos = np.array(uav_state[0:3])
        ego_vel = np.array(uav_state[3:6])
        opp_pos = np.array(opponent_state[0:3])
        opp_vel = np.array(opponent_state[3:6])
        
        # Compute optimal control
        control = _controller_instance.compute_control_from_simulink(
            ego_pos, ego_vel, opp_pos, opp_vel, time
        )
        
        return control
        
    except Exception as e:
        print(f"NRHDG control error: {e}")
        return np.zeros(3)
