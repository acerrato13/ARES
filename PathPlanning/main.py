from core import DroneState
from control import DynamicProjections

initial_state = DroneState(
    mass=1.5,  # 1.5 kg drone
    x=0.0, y=0.0, z=10.0,  # Starting at 10m altitude
    vx=5.0, vy=0.0, vz=0.0,  # Moving at 5 m/s in x direction
    ax=0.0, ay=0.0, az=0.0,  # No initial acceleration
    timestamp=0.0
)

# Create projections system
projections = DynamicProjections(gravity=9.81)

# Predict state after 1 second
future_state = projections.predict_state(initial_state, dt=1.0)
print("Initial State:")
print(f"  Position: ({initial_state.x:.2f}, {initial_state.y:.2f}, "
      f"{initial_state.z:.2f})")
print(f"  Velocity: ({initial_state.vx:.2f}, {initial_state.vy:.2f}, "
      f"{initial_state.vz:.2f})")
print("\nPredicted State (t=1s):")
print(f"  Position: ({future_state.x:.2f}, {future_state.y:.2f}, "
      f"{future_state.z:.2f})")
print(f"  Velocity: ({future_state.vx:.2f}, {future_state.vy:.2f}, "
      f"{future_state.vz:.2f})")

# Predict trajectory over 5 seconds
trajectory = projections.predict_trajectory(
    initial_state, time_horizon=5.0, dt=0.5
)
print(f"\nTrajectory over 5 seconds ({len(trajectory)} points):")
for i, state in enumerate(trajectory[::2]):
    pos_str = f"pos=({state.x:.2f}, {state.y:.2f}, {state.z:.2f})"
    vel_str = f"vel=({state.vx:.2f}, {state.vy:.2f}, {state.vz:.2f})"
    print(f"  t={state.timestamp:.1f}s: {pos_str} {vel_str}")
