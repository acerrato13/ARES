import numpy as np
from dataclasses import dataclass
from typing import Tuple
from enum import Enum


class DroneFormType(Enum):
    """Enumeration of common drone body shapes."""
    VTOL = "vtol"
    CUSTOM = "custom"
    QUADCOPTER = "quadcopter"
    HEXACOPTER = "hexacopter"
    FIXED_WING = "fixed_wing"


@dataclass
class DroneShape:
    mass: float
    length: float
    width: float
    height: float

    form: DroneFormType = DroneFormType.CUSTOM

    drag_coefficient: float = 0.5
    reference_area: float = 0.25
    lift_coefficient: float = 0.0

    com_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)  #

    inertia_xx: float = 0.1
    inertia_yy: float = 0.1
    inertia_zz: float = 0.1

    max_speed: float = 25.0
    max_acceleration: float = 10.0
    max_angular_velocity: float = 6.28

    max_thrust: float = 50.0
    min_thrust: float = 0.0

    def volume(self) -> float:
        """Calculate bounding box volume in m^3."""
        return self.length * self.width * self.height

    def frontal_area(self) -> float:
        """Calculate frontal area for drag calculations in m^2."""
        return self.width * self.height

    def lateral_area(self) -> float:
        """Calculate lateral area in m^2."""
        return self.length * self.height

    def vertical_area(self) -> float:
        """Calculate vertical projection area in m^2."""
        return self.length * self.width

    def center_of_mass(self) -> np.ndarray:
        """Get center of mass position relative to geometric center."""
        return np.array(self.com_offset)

    def inertia_matrix(self) -> np.ndarray:
        """
        Get the 3x3 inertia tensor (assuming alignment with principal axes).

        Returns:
            np.ndarray: 3x3 inertia matrix
        """
        return np.array([
            [self.inertia_xx, 0.0, 0.0],
            [0.0, self.inertia_yy, 0.0],
            [0.0, 0.0, self.inertia_zz]
        ])

    def inverse_inertia_matrix(self) -> np.ndarray:
        """Get inverse of inertia tensor for angle acceleration."""
        return np.linalg.inv(self.inertia_matrix())

    def get_drag_coefficient_adjusted(self, angle_of_attack: float) -> float:
        """
        Get drag coefficient adjusted for angle of attack.

        Args:
            angle_of_attack: Angle relative to body axis in radians

        Returns:
            float: Adjusted drag coefficient
        """
        return self.drag_coefficient * abs(np.cos(angle_of_attack))

    def is_within_limits(self, speed: float, acceleration: float,
                         angular_velocity: float) -> bool:
        """
        Check if proposed motion is within drone performance limits.

        Args:
            speed: Current speed magnitude in m/s
            acceleration: Current acceleration magnitude in m/s^2
            angular_velocity: Current angular velocity magnitude in rad/s

        Returns:
            bool: True if all parameters within limits
        """
        return (speed <= self.max_speed and
                acceleration <= self.max_acceleration and
                angular_velocity <= self.max_angular_velocity)

    def to_dict(self) -> dict:
        """Convert shape to dictionary for serialization."""
        return {
            'mass': self.mass,
            'dimensions': {
                'length': self.length,
                'width': self.width,
                'height': self.height,
                'volume': self.volume()
            },
            'form': self.form.value,
            'aerodynamics': {
                'drag_coefficient': self.drag_coefficient,
                'lift_coefficient': self.lift_coefficient,
                'reference_area': self.reference_area
            },
            'inertia': {
                'ixx': self.inertia_xx,
                'iyy': self.inertia_yy,
                'izz': self.inertia_zz
            },
            'performance': {
                'max_speed': self.max_speed,
                'max_acceleration': self.max_acceleration,
                'max_angular_velocity': self.max_angular_velocity,
                'max_thrust': self.max_thrust
            }
        }


DRONE_PRESETS = {
    "DJI_M300": DroneShape(
        mass=2.7,
        length=0.77,
        width=0.77,
        height=0.39,
        form=DroneFormType.QUADCOPTER,
        drag_coefficient=0.47,
        reference_area=0.59,
        max_speed=22.0,
        max_acceleration=8.0,
        max_thrust=54.0,
        inertia_xx=0.12,
        inertia_yy=0.12,
        inertia_zz=0.21
    ),
    "Standard_Quadcopter": DroneShape(
        mass=2.0,
        length=0.5,
        width=0.5,
        height=0.3,
        form=DroneFormType.QUADCOPTER,
        drag_coefficient=0.5,
        reference_area=0.25,
        max_speed=20.0,
        max_acceleration=10.0,
        max_thrust=40.0,
        inertia_xx=0.1,
        inertia_yy=0.1,
        inertia_zz=0.1
    ),
    "Light_Quadcopter": DroneShape(
        mass=1.2,
        length=0.34,
        width=0.34,
        height=0.18,
        form=DroneFormType.QUADCOPTER,
        drag_coefficient=0.45,
        reference_area=0.12,
        max_speed=25.0,
        max_acceleration=12.0,
        max_thrust=30.0,
        inertia_xx=0.05,
        inertia_yy=0.05,
        inertia_zz=0.08
    ),
    "Heavy_Hexacopter": DroneShape(
        mass=5.0,
        length=1.0,
        width=0.9,
        height=0.4,
        form=DroneFormType.HEXACOPTER,
        drag_coefficient=0.55,
        reference_area=0.36,
        max_speed=18.0,
        max_acceleration=6.0,
        max_thrust=100.0,
        inertia_xx=0.25,
        inertia_yy=0.25,
        inertia_zz=0.40
    ),
    "Fixed_Wing": DroneShape(
        mass=3.5,
        length=1.5,
        width=2.0,
        height=0.3,
        form=DroneFormType.FIXED_WING,
        drag_coefficient=0.03,
        lift_coefficient=0.8,
        reference_area=0.45,
        max_speed=30.0,
        max_acceleration=15.0,
        max_thrust=60.0,
        inertia_xx=0.15,
        inertia_yy=0.30,
        inertia_zz=0.40
    ),
}
