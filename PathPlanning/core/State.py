import numpy as np
from dataclasses import dataclass


@dataclass
class DroneState:
    x: float
    y: float
    vx: float
    vy: float
    ax: float
    ay: float
    theta: float
    theta_dot: float
    timestamp: float = 0.0

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy])

    def acceleration(self) -> np.ndarray:
        return np.array([self.ax, self.ay])

    def speed(self) -> float:
        return np.linalg.norm(self.velocity())

    def to_dict(self) -> dict:
        return {
            'position': {'x': self.x, 'y': self.y},
            'velocity': {'vx': self.vx, 'vy': self.vy},
            'acceleration': {'ax': self.ax, 'ay': self.ay},
            'orientation': {'theta': self.theta, 'theta_dot': self.theta_dot},
            'timestamp': self.timestamp
        }
