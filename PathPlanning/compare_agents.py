import sys
import os

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from PathPlanning.functions_anu import DynamicRoleSwitching
from nmpc_opponent import NMPCOpponent
import matplotlib.pyplot as plt

def simulate_nrhdg_agent(agent, initial_state, target_state, steps):
    """
    Simulate the NRHDG agent.

    Parameters:
    agent : DynamicRoleSwitching
        The NRHDG agent instance.
    initial_state : np.ndarray
        Initial state of the agent.
    target_state : np.ndarray
        Target state to track.
    steps : int
        Number of simulation steps.

    Returns:
    np.ndarray
        Trajectory of the agent.
    """
    trajectory = [initial_state]
    state = initial_state

    for _ in range(steps):
        # Example: Compute G_O and update state (implement dynamics)
        # Placeholder for actual dynamics update
        state = state + np.random.randn(*state.shape) * 0.1  # Random walk for now
        trajectory.append(state)

    return np.array(trajectory)

def simulate_nmpc_agent(agent, initial_state, target_state, steps):
    """
    Simulate the NMPC agent.

    Parameters:
    agent : NMPCOpponent
        The NMPC agent instance.
    initial_state : np.ndarray
        Initial state of the agent.
    target_state : np.ndarray
        Target state to track.
    steps : int
        Number of simulation steps.

    Returns:
    np.ndarray
        Trajectory of the agent.
    """
    trajectory = [initial_state]
    state = initial_state

    for _ in range(steps):
        control = agent.compute_control(state, target_state)
        # Update state based on NMPC control
        state = agent.dynamics(state, control)
        trajectory.append(state)

    return np.array(trajectory)

def compare_agents():
    """
    Compare the NRHDG and NMPC agents.
    """
    # Initialize agents
    nrhdg_agent = DynamicRoleSwitching()
    nmpc_agent = NMPCOpponent(prediction_horizon=10, dt=0.1)

    # Define initial conditions and target
    initial_state = np.array([0, 0, 0, 0, 0, 0])  # [x, y, z, vx, vy, vz]
    target_state = np.array([10, 10, 0, 0, 0, 0])

    # Simulate agents
    steps = 100
    nrhdg_trajectory = simulate_nrhdg_agent(nrhdg_agent, initial_state, target_state, steps)
    nmpc_trajectory = simulate_nmpc_agent(nmpc_agent, initial_state, target_state, steps)

    # Plot results
    plt.plot(nrhdg_trajectory[:, 0], nrhdg_trajectory[:, 1], label="NRHDG")
    plt.plot(nmpc_trajectory[:, 0], nmpc_trajectory[:, 1], label="NMPC")
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Trajectory Comparison")
    plt.show()

if __name__ == "__main__":
    compare_agents()