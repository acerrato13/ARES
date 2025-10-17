# ARES - Autonomous Robotic Exploration System

## Project Overview

ARES (Autonomous Robotic Exploration System) is a comprehensive drone path planning and control system developed for VIP 47921 at Purdue University. The project focuses on autonomous UAV navigation, obstacle avoidance, and trajectory prediction using advanced control algorithms.

## Project Structure

```
Code/
├── Path Planning/           # Main path planning module
│   ├── main.py             # Main execution script
│   ├── functions-anu.py    # Utility functions
│   ├── core/               # Core data structures
│   │   ├── __init__.py
│   │   └── State.py        # DroneState class definition
│   ├── control/            # Control and prediction algorithms
│   │   ├── __init__.py
│   │   └── DynamicProjections.py  # Trajectory prediction
│   ├── planning/           # Path planning algorithms
│   ├── config/             # Configuration files
│   ├── scripts/            # Utility scripts
│   └── tests/              # Unit tests
│
└── Simulator/              # MATLAB Simulink simulation
    ├── ObstacleAvoidanceDemo.slx
    └── UAVObstacleAvoidanceInSimulinkExample.mlx
```

## Features

### 1. Dynamic Trajectory Prediction
- **DroneState Management**: Complete state representation including:
  - Mass (kg)
  - Position (x, y, z) in meters
  - Velocity (vx, vy, vz) in m/s
  - Acceleration (ax, ay, az) in m/s²
  - Timestamp tracking

### 2. Physics-Based Modeling
- Newtonian mechanics implementation (F = ma)
- Kinematic equations for motion prediction
- Gravity compensation
- Optional air resistance/drag modeling

### 3. Prediction Algorithms
- **Euler Integration**: Fast position/velocity prediction
- **Runge-Kutta 4th Order**: High-accuracy numerical integration
- **Trajectory Generation**: Multi-step path prediction over time horizons
- **Time-to-Target Estimation**: Calculates intercept times

### 4. Simulink Integration
- MATLAB Simulink obstacle avoidance demo
- Real-time UAV simulation environment
- Hardware-in-the-loop testing capability

## Installation

### Prerequisites
```bash
Python 3.8+
numpy
```

### Setup
```bash
# Clone the repository
git clone https://github.com/acerrato13/ARES.git
cd ARES/Code

# Install Python dependencies
pip install numpy
```

## Usage

### Basic Example
```python
from core import DroneState
from control import DynamicProjections

# Create initial drone state
initial_state = DroneState(
    mass=1.5,           # 1.5 kg drone
    x=0.0, y=0.0, z=10.0,   # Starting at 10m altitude
    vx=5.0, vy=0.0, vz=0.0,  # Moving at 5 m/s in x direction
    ax=0.0, ay=0.0, az=0.0,  # No initial acceleration
    timestamp=0.0
)

# Initialize prediction system
projections = DynamicProjections(gravity=9.81)

# Predict state after 1 second
future_state = projections.predict_state(initial_state, dt=1.0)

# Generate trajectory over 5 seconds
trajectory = projections.predict_trajectory(
    initial_state, 
    time_horizon=5.0, 
    dt=0.1
)
```

### Running the Main Script
```bash
cd "Path Planning"
python main.py
```

Expected output:
```
Initial State:
  Position: (0.00, 0.00, 10.00)
  Velocity: (5.00, 0.00, 0.00)

Predicted State (t=1s):
  Position: (5.00, 0.00, 5.10)
  Velocity: (5.00, 0.00, -9.81)

Trajectory over 5 seconds (11 points):
  t=0.0s: pos=(0.00, 0.00, 10.00) vel=(5.00, 0.00, 0.00)
  ...
```

## API Reference

### DroneState Class
```python
DroneState(
    mass: float,
    x: float, y: float, z: float,
    vx: float, vy: float, vz: float,
    ax: float, ay: float, az: float,
    timestamp: float = 0.0
)
```

**Methods:**
- `position()` → np.ndarray: Returns [x, y, z]
- `velocity()` → np.ndarray: Returns [vx, vy, vz]
- `acceleration()` → np.ndarray: Returns [ax, ay, az]
- `speed()` → float: Returns magnitude of velocity
- `to_dict()` → dict: Converts state to dictionary

### DynamicProjections Class
```python
DynamicProjections(
    gravity: float = 9.81,
    air_resistance_coeff: float = 0.0
)
```

**Key Methods:**
- `predict_position(state, dt)`: Predicts future position
- `predict_velocity(state, dt)`: Predicts future velocity
- `predict_state(state, dt, applied_force=None)`: Complete state prediction
- `predict_trajectory(state, time_horizon, dt, control_forces=None)`: Multi-step trajectory
- `runge_kutta_4_step(state, dt, force_func=None)`: High-accuracy RK4 integration
- `estimate_time_to_position(state, target_pos, max_time)`: Time-to-target calculation

## Simulation

### MATLAB Simulink
Open the Simulink model for obstacle avoidance testing:
```matlab
cd Simulator
open('ObstacleAvoidanceDemo.slx')
```

## Development Guidelines

### Code Standards
- Follow PEP 8 style guide
- Maximum line length: 79 characters
- Use type hints for function signatures
- Document all public methods with docstrings

### Testing
```bash
cd "Path Planning/tests"
python -m pytest
```

## Team & Course Information

- **Course**: VIP 47921 - ARES
- **Institution**: Purdue University
- **Semester**: Fall 2025
- **Repository Owner**: acerrato13

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is part of Purdue University coursework. All rights reserved.

## References

- Newton's Laws of Motion
- Kinematic Equations
- Runge-Kutta Methods for Numerical Integration
- UAV Path Planning Algorithms

## Contact

For questions or collaboration:
- Repository: [github.com/acerrato13/ARES](https://github.com/acerrato13/ARES)
- Course: VIP 47921, Purdue University

---

**Last Updated**: October 16, 2025
