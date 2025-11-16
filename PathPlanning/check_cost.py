from nrhdg_agent import NRHDGController2D, NRHDGConfig, DroneState, DynamicRoleSwitching2D

def check_cost(params):
    """
    Check the cost of a given parameter set.

    Parameters:
    params : list
        List of NRHDG parameters to evaluate.

    Returns:
    float
        Total cost for the given parameter set.
    """
    try:
        # Unpack parameters safely
        w_progress, w_deviation, w_control, w_interaction, drag_coeff = params[:5]

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
        total_cost = controller.compute_objective(ego_state, opp_state, u_sequence, v_sequence)
        return total_cost

    except Exception as e:
        print(f"Error evaluating params {params}: {e}")
        return 1e6  # Penalize invalid configurations

if __name__ == "__main__":
    # Example usage
    example_params = [1.0, 1.0, 1.0, 1.0, 0.1]
    cost = check_cost(example_params)
    print(f"Cost for parameters {example_params}: {cost}")