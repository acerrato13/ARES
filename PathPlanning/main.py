from core import DroneState
from control import DynamicProjections

# Define initial 2D state
initial_state = DroneState(
    x=0.0, y=0.0,    # start position
    vx=5.0, vy=0.0,  # initial velocity (5 m/s along x)
    ax=0.0, ay=0.0,  # initial acceleration
    timestamp=0.0
)

# Create projection system (2D)
projections = DynamicProjections(gravity=9.81)

# Predict state after 1 second
future_state = projections.predict_state(initial_state, dt=1.0)

print("Initial State:")
print(f"  Position: ({initial_state.x:.2f}, {initial_state.y:.2f})")
print(f"  Velocity: ({initial_state.vx:.2f}, {initial_state.vy:.2f})")

print("\nPredicted State (t=1s):")
print(f"  Position: ({future_state.x:.2f}, {future_state.y:.2f})")
print(f"  Velocity: ({future_state.vx:.2f}, {future_state.vy:.2f})")

# Predict trajectory over 5 seconds
trajectory = projections.predict_trajectory(
    initial_state, time_horizon=5.0, dt=0.5
)

print(f"\nTrajectory over 5 seconds ({len(trajectory)} points):")
for i, state in enumerate(trajectory[::2]):
    pos_str = f"pos=({state.x:.2f}, {state.y:.2f})"
    vel_str = f"vel=({state.vx:.2f}, {state.vy:.2f})"
    print(f"  t={state.timestamp:.1f}s: {pos_str} {vel_str}")
