from dataclasses import dataclass


@dataclass
class NRHDGConfig:
    T: float = 1.0
    dt: float = 0.1
    n_steps: int = 10

    w_progress: float = 1.0
    w_deviation: float = 0.5
    w_control: float = 0.1

    max_thrust: float = 20.0
    mass: float = 1.0
    drag_coeff: float = 0.0
