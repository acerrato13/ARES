import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the parent directory to the Python path (so imports work if in subfolder)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import your 2D agent classes
from PathPlanning.functions_anu import DynamicRoleSwitching
from nmpc_opponent import NMPCOpponent


def simulate_nrhdg_agent(agent, initial_state, target_state, steps):
    """
    Simulate the NRHDG (DynamicRoleSwitching-based) agent.

    Parameters:
    agent : DynamicRoleSwitching
        The NRHDG agent instance.
    initial_state : np.ndarray
        Initial state of the agent [x, y, vx, vy].
    target_state : np.ndarray
        Target state [x, y, vx, vy].
    steps : int
        Number of simulation steps.

    Returns:
    np.ndarray
        Trajectory of the agent with shape (steps, 4).
    """
    trajectory = [initial_state.copy()]
    state = initial_state.copy()

    for _ in range(steps):
        # Example usage of DynamicRoleSwitching (if relevant)
        # You can modify how G_O influences state dynamics here.
        # For now, just a placeholder motion model.
        state = state + np.random.randn(*state.shape) * 0.05  # Small random walk
        trajectory.append(state.copy())

    return np.array(trajectory)


def simulate_nmpc_agent(agent, initial_state, target_state, steps):
    """
    Simulate the NMPC agent.

    Parameters:
    agent : NMPCOpponent
        The NMPC agent instance.
    initial_state : np.ndarray
        Initial state [x, y, vx, vy].
    target_state : np.ndarray
        Target state [x, y, vx, vy].
    steps : int
        Number of simulation steps.

    Returns:
    np.ndarray
        Trajectory of the agent with shape (steps, 4).
    """
    trajectory = [initial_state.copy()]
    state = initial_state.copy()

    for _ in range(steps):
        # Compute control using your NMPC logic
        control = agent.compute_control(state, target_state)

        # Update state using your NMPC dynamics
        state = agent.dynamics(state, control)
        trajectory.append(state.copy())

    return np.array(trajectory)


def compare_agents():
    """
    Compare the 2D trajectories of NRHDG and NMPC agents.
    """
    # Initialize agents
    nrhdg_agent = DynamicRoleSwitching()
    nmpc_agent = NMPCOpponent(prediction_horizon=10, dt=0.1)

    # Define initial and target states [x, y, vx, vy]
    initial_state = np.array([0.0, 0.0, 0.0, 0.0])
    target_state = np.array([10.0, 10.0, 0.0, 0.0])

    # Simulate both agents
    steps = 200
    nrhdg_trajectory = simulate_nrhdg_agent(nrhdg_agent, initial_state, target_state, steps)
    nmpc_trajectory = simulate_nmpc_agent(nmpc_agent, initial_state, target_state, steps)

    # Plot position trajectories
    plt.figure(figsize=(8, 6))
    plt.plot(nrhdg_trajectory[:, 0], nrhdg_trajectory[:, 1], label="NRHDG (DynamicRoleSwitching)", linewidth=2)
    plt.plot(nmpc_trajectory[:, 0], nmpc_trajectory[:, 1], label="NMPC", linewidth=2)
    plt.scatter(target_state[0], target_state[1], c='red', marker='x', s=100, label='Target')

    plt.legend()
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("2D Trajectory Comparison: NRHDG vs NMPC")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    compare_agents()
