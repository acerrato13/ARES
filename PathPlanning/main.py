import numpy as np
from core import DroneState
from core.Shape import DRONE_PRESETS
from control import DynamicProjections

# Select drone shape from presets
drone_shape = DRONE_PRESETS["Standard_Quadcopter"]

# Define initial 3D state (now with numpy arrays)
initial_state = DroneState(
    position=np.array([0.0, 0.0, 0.0]),  # start position (x, y, z)
    velocity=np.array([5.0, 0.0, 0.0]),  # initial velocity (5 m/s along x)
    acceleration=np.array([0.0, 0.0, 0.0]),  # initial acceleration
    alpha=np.array([0.0, 0.0, 0.0]),  # rotation angles
    theta=0.0,  # yaw angle
    theta_dot=0.0,  # yaw rate
    timestamp=0.0
)

# Create projection system with drone shape
projections = DynamicProjections(
    drone_shape=drone_shape,
    gravity=9.81,
    air_resistance_coeff=0.1,
    wind_velocity=np.array([0.0, 0.0, 0.0])
)

# Predict state after 1 second
future_state = projections.predict_state(initial_state, dt=1.0)

print("=" * 60)
print("DRONE STATE PREDICTION - 3D TRAJECTORY ANALYSIS")
print("=" * 60)

print("\nDrone Configuration:")
print(f"  Type: {drone_shape.form.value}")
print(f"  Mass: {drone_shape.mass:.2f} kg")
length = drone_shape.length
width = drone_shape.width
height = drone_shape.height
print(f"  Dimensions: {length:.2f}m x {width:.2f}m x {height:.2f}m")
print(f"  Max Speed: {drone_shape.max_speed:.2f} m/s")

print("\nInitial State:")
pos_init = initial_state.position
print(f"  Position: ({pos_init[0]:.2f}, {pos_init[1]:.2f}, "
      f"{pos_init[2]:.2f}) m")
vel_init = initial_state.velocity
print(f"  Velocity: ({vel_init[0]:.2f}, {vel_init[1]:.2f}, "
      f"{vel_init[2]:.2f}) m/s")
alpha_init = initial_state.alpha
print(f"  Rotation (alpha): ({alpha_init[0]:.2f}, {alpha_init[1]:.2f}, "
      f"{alpha_init[2]:.2f}) rad")
print(f"  Speed: {initial_state.speed():.2f} m/s")

print("\nPredicted State (t=1.0s):")
pos_fut = future_state.position
print(f"  Position: ({pos_fut[0]:.2f}, {pos_fut[1]:.2f}, "
      f"{pos_fut[2]:.2f}) m")
vel_fut = future_state.velocity
print(f"  Velocity: ({vel_fut[0]:.2f}, {vel_fut[1]:.2f}, "
      f"{vel_fut[2]:.2f}) m/s")
alpha_fut = future_state.alpha
print(f"  Rotation (alpha): ({alpha_fut[0]:.2f}, {alpha_fut[1]:.2f}, "
      f"{alpha_fut[2]:.2f}) rad")
print(f"  Speed: {future_state.speed():.2f} m/s")
print(f"  Altitude: {future_state.position[2]:.2f} m")

# Predict trajectory over 5 seconds
trajectory = projections.predict_trajectory(
    initial_state, time_horizon=5.0, dt=0.5
)

print(f"\nTrajectory over 5 seconds ({len(trajectory)} points):")
header = f"{'Time (s)':>8} {'X (m)':>9} {'Y (m)':>9} {'Z (m)':>9}"
header += f" {'Speed':>8} {'Alpha_X':>8}"
print(header)
print("-" * 70)
for state in trajectory[::2]:
    pos = state.position
    speed = state.speed()
    alpha_x = state.alpha[0]
    print(f"{state.timestamp:8.1f} {pos[0]:9.2f} {pos[1]:9.2f} "
          f"{pos[2]:9.2f} {speed:8.2f} {alpha_x:8.2f}")

# Demonstrate distance calculation
target = np.array([10.0, 10.0, 5.0])
dist_to_target = trajectory[-1].distance_to(target)
target_str = f"Distance to target {target}: "
print(f"\n{target_str}{dist_to_target:.2f} m")
