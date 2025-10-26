import numpy as np
from dataclasses import dataclass


@dataclass
class DroneState:
    mass: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    theta: float # Path parameter
    theta_dot: float # rate of progress along path
    timestamp: float = 0.0

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])

    def acceleration(self) -> np.ndarray:
        return np.array([self.ax, self.ay, self.az])

    def speed(self) -> float:
        return np.linalg.norm(self.velocity())

    def to_dict(self) -> dict:
        return {
            'mass': self.mass,
            'position': {'x': self.x, 'y': self.y, 'z': self.z},
            'velocity': {'vx': self.vx, 'vy': self.vy, 'vz': self.vz},
            'acceleration': {'ax': self.ax, 'ay': self.ay, 'az': self.az},
            'timestamp': self.timestamp
        }

@dataclass
class NRHDGConfig:
    T: float = 1.0  # receding horizon time
    dt: float = 0.1 # time step
    n_steps: int = 10 # num of discretization steps

    # Weights for our object function
    # chosen arbitrarily tune later
    w_progress: float = 1.0 # Higher it priorities progress along path
    w_deviation: float = 0.5 # higher it punishes deviation off the path
    w_control: float = 0.1  # Higher higher changes is control (thrust)

    # TODO: make these make sense
    max_thrust: float = 100.0
    mass: float = 0.5
    drag_coeff: float = 0.0
