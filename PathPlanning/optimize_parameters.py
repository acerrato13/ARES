"""
Avantika Shah - use CMA-ES to optimize path planning parameters
Path: PathPlanning/optimize_parameters.py
"""
import cma
import numpy as np
from nrhdg_agent import NRHDGController2D, NRHDGConfig, DroneState, DynamicRoleSwitching2D


def objective_function(params):
    """
    Objective function for CMA-ES optimization.
    """
    try:
        # Unpack parameters safely
        w_progress, w_deviation, w_control, drag_coeff = params[:4]

        config = NRHDGConfig(
            T=1.0, dt=0.1, n_steps=10,
            w_progress=w_progress,
            w_deviation=w_deviation,
            w_control=w_control,
            drag_coeff=drag_coeff,
            mass=5.0,
            max_thrust=10.0
        )

        role_switcher = DynamicRoleSwitching2D()
        controller = NRHDGController2D(role_switcher, config)

        ego_state = DroneState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        opp_state = DroneState(2.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0)

        u_sequence, v_sequence = controller.solve_saddle_point(ego_state, opp_state)
        # Updated cost computation to evaluate path-following performance
        total_cost = controller.compute_path_following_cost(ego_state, opp_state, u_sequence, v_sequence)
        return total_cost

    except Exception as e:
        print(f"Error evaluating params {params}: {e}")
        return 1e6  # Penalize invalid configurations


def compute_path_cost(controller, initial_state, steps, dt=0.1):
    """
    Compute the path cost based on deviation from the desired path.

    Args:
        controller: The NRHDGController2D instance.
        initial_state: The initial state of the drone.
        steps: Number of simulation steps.
        dt: Time step size.

    Returns:
        float: The total path cost based on deviation.
    """
    trajectory = [initial_state]
    total_deviation_cost = 0.0

    ego_state = initial_state

    for step in range(steps):
        # Compute control for the ego drone
        u = controller.compute_control(ego_state, None)

        # Update ego state
        ego_state = controller._propagate_state(ego_state, u, dt)
        trajectory.append(ego_state)

        # Compute deviation from the desired path
        desired_position = controller.role_switcher.path_function(ego_state.theta)
        actual_position = ego_state.position()
        deviation = np.linalg.norm(actual_position - desired_position)
        total_deviation_cost += deviation ** 2

    return total_deviation_cost


def optimize_nrhdg_parameters():
    """
    Optimize NRHDG parameters using CMA-ES.
    """
    initial_params = [1.0, 1.0, 1.0, 0.1]  # Updated to 4 parameters
    bounds = [[0.1, 0.1, 0.1, 0.01], [10.0, 10.0, 10.0, 1.0]]  # Updated bounds to match 4 parameters

    # Suppress internal CMA printouts by setting verb_disp to 0
    es = cma.CMAEvolutionStrategy(initial_params, 0.5, {
        'bounds': bounds,
        'popsize': 20,
        'tolx': 1e-8,
        'tolfun': 1e-8,
        'maxiter': 100,
        'verb_disp': 0  # Disable auto-printing
    })

    iteration = 0
    best_cost_so_far = float('inf')  # Initialize best cost as infinity

    while not es.stop():
        iteration += 1
        print(f"\nIteration {iteration} --------------------------")

        # Generate candidate solutions and evaluate costs
        solutions = es.ask()
        costs = [objective_function(params) for params in solutions]

        # Update the best cost so far
        current_best_cost = min(costs)
        if current_best_cost < best_cost_so_far:
            best_cost_so_far = current_best_cost
            print(f"Improved! Best cost so far: {best_cost_so_far:.4f}")
        else:
            print(f"No improvement. Best cost remains: {best_cost_so_far:.4f}")

        # Provide feedback to the optimizer
        es.tell(solutions, costs)
        print(f"Current best parameters: {es.result.xbest}")

    print("\nOptimization finished.")
    print("Optimized Parameters:", es.result.xbest)
    print("Final Cost:", es.result.fbest)


if __name__ == "__main__":
    optimize_nrhdg_parameters()
