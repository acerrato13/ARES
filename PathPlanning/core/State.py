import numpy as np
from dataclasses import dataclass, field


@dataclass
class DroneState:
    """
    Optimized drone state using NumPy arrays for vector quantities.

    Stores position, velocity, and acceleration as 3D vectors for efficient
    physics calculations and linear algebra operations.
    """
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    alpha: np.ndarray = field(default_factory=lambda: np.zeros(3))
    theta: float = 0.0
    theta_dot: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self):
        """Ensure all vector fields are numpy arrays."""
        if not isinstance(self.position, np.ndarray):
            self.position = np.array(self.position, dtype=float)
        if not isinstance(self.velocity, np.ndarray):
            self.velocity = np.array(self.velocity, dtype=float)
        if not isinstance(self.acceleration, np.ndarray):
            self.acceleration = np.array(self.acceleration, dtype=float)
        if not isinstance(self.alpha, np.ndarray):
            self.alpha = np.array(self.alpha, dtype=float)

    def speed(self) -> float:
        """Calculate speed magnitude from velocity vector."""
        return np.linalg.norm(self.velocity)

    def distance_to(self, target: np.ndarray) -> float:
        """Calculate distance to target position."""
        return np.linalg.norm(self.position - target)

    def displacement(self) -> float:
        """Calculate total displacement magnitude."""
        return np.linalg.norm(self.position)

    def to_dict(self) -> dict:
        """Convert state to dictionary for serialization."""
        return {
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'acceleration': self.acceleration.tolist(),
            'orientation': {
                'alpha': self.alpha.tolist(),
                'theta': self.theta,
                'theta_dot': self.theta_dot
            },
            'timestamp': self.timestamp
        }

    @staticmethod
    def from_dict(data: dict) -> 'DroneState':
        """Create DroneState from dictionary."""
        return DroneState(
            position=np.array(data['position']),
            velocity=np.array(data['velocity']),
            acceleration=np.array(data['acceleration']),
            alpha=np.array(data['orientation']['alpha']),
            theta=data['orientation']['theta'],
            theta_dot=data['orientation']['theta_dot'],
            timestamp=data['timestamp']
        )
