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
    
