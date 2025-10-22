import numpy as np
from typing import Tuple, Optional, List
from core import DroneState

class DynamicProjections:
    def __init__(self, gravity: float = 9.81,
                 air_resistance_coeff: float = 0.0):
        self.gravity = gravity
        self.air_resistance_coeff = air_resistance_coeff
        self.state_history: List[DroneState] = []

    def predict_position(self, state: DroneState, dt: float) -> np.ndarray:
        pos = state.position()
        vel = state.velocity()
        acc = state.acceleration()

        acc_with_gravity = acc.copy()
        acc_with_gravity[2] -= self.gravity

        predicted_pos = pos + vel * dt + 0.5 * acc_with_gravity * (dt ** 2)
        return predicted_pos

    def predict_velocity(self, state: DroneState, dt: float) -> np.ndarray:
        vel = state.velocity()
        acc = state.acceleration()

        acc_with_gravity = acc.copy()
        acc_with_gravity[2] -= self.gravity

        if self.air_resistance_coeff > 0:
            drag_force = -self.air_resistance_coeff * vel * np.linalg.norm(vel)
            drag_acc = drag_force / state.mass
            acc_with_gravity += drag_acc

        predicted_vel = vel + acc_with_gravity * dt
        return predicted_vel

    def predict_acceleration(self, state: DroneState,
                             force: Optional[np.ndarray] = None) -> np.ndarray:
        if force is None:
            return state.acceleration()

        predicted_acc = force / state.mass
        return predicted_acc

    def predict_state(self, state: DroneState, dt: float,
                      applied_force: Optional[np.ndarray] = None
                      ) -> DroneState:
        if applied_force is not None:
            new_acc = self.predict_acceleration(state, applied_force)
        else:
            new_acc = state.acceleration()

        new_vel = self.predict_velocity(state, dt)
        new_pos = self.predict_position(state, dt)

        predicted_state = DroneState(
            mass=state.mass,
            x=new_pos[0],
            y=new_pos[1],
            z=new_pos[2],
            vx=new_vel[0],
            vy=new_vel[1],
            vz=new_vel[2],
            ax=new_acc[0],
            ay=new_acc[1],
            az=new_acc[2],
            timestamp=state.timestamp + dt
        )

        return predicted_state

    def predict_trajectory(self, state: DroneState, time_horizon: float,
                           dt: float = 0.1,
                           control_forces: Optional[List[np.ndarray]] = None
                           ) -> List[DroneState]:
        trajectory = [state]
        current_state = state
        num_steps = int(time_horizon / dt)

        for i in range(num_steps):
            if control_forces and i < len(control_forces):
                force = control_forces[i]
            else:
                force = None

            next_state = self.predict_state(current_state, dt, force)
            trajectory.append(next_state)
            current_state = next_state

        return trajectory

    def derivatives(self, s: DroneState, t: float,
                    force_func: Optional[callable] = None
                    ) -> Tuple[np.ndarray, np.ndarray]:
        force = force_func(s, t) if force_func else np.zeros(3)
        acc = force / s.mass
        acc[2] -= self.gravity
        return s.velocity(), acc

    def runge_kutta_4_step(self, state: DroneState, dt: float,
                           force_func: Optional[callable] = None
                           ) -> DroneState:
        k1_vel, k1_acc = self.derivatives(state, state.timestamp, force_func)

        mid_state1 = DroneState(
            mass=state.mass,
            x=state.x + 0.5 * k1_vel[0] * dt,
            y=state.y + 0.5 * k1_vel[1] * dt,
            z=state.z + 0.5 * k1_vel[2] * dt,
            vx=state.vx + 0.5 * k1_acc[0] * dt,
            vy=state.vy + 0.5 * k1_acc[1] * dt,
            vz=state.vz + 0.5 * k1_acc[2] * dt,
            ax=state.ax, ay=state.ay, az=state.az,
            timestamp=state.timestamp + 0.5 * dt
        )

        k2_vel, k2_acc = self.derivatives(mid_state1, mid_state1.timestamp,
                                          force_func)

        mid_state2 = DroneState(
            mass=state.mass,
            x=state.x + 0.5 * k2_vel[0] * dt,
            y=state.y + 0.5 * k2_vel[1] * dt,
            z=state.z + 0.5 * k2_vel[2] * dt,
            vx=state.vx + 0.5 * k2_acc[0] * dt,
            vy=state.vy + 0.5 * k2_acc[1] * dt,
            vz=state.vz + 0.5 * k2_acc[2] * dt,
            ax=state.ax, ay=state.ay, az=state.az,
            timestamp=state.timestamp + 0.5 * dt
        )
        k3_vel, k3_acc = self.derivatives(mid_state2, mid_state2.timestamp,
                                          force_func)

        end_state = DroneState(
            mass=state.mass,
            x=state.x + k3_vel[0] * dt,
            y=state.y + k3_vel[1] * dt,
            z=state.z + k3_vel[2] * dt,
            vx=state.vx + k3_acc[0] * dt,
            vy=state.vy + k3_acc[1] * dt,
            vz=state.vz + k3_acc[2] * dt,
            ax=state.ax, ay=state.ay, az=state.az,
            timestamp=state.timestamp + dt
        )
        k4_vel, k4_acc = self.derivatives(end_state, end_state.timestamp,
                                          force_func)

        new_pos = state.position() + (dt / 6.0) * (k1_vel + 2*k2_vel
                                                   + 2*k3_vel + k4_vel)
        new_vel = state.velocity() + (dt / 6.0) * (k1_acc + 2*k2_acc
                                                   + 2*k3_acc + k4_acc)

        return DroneState(
            mass=state.mass,
            x=new_pos[0], y=new_pos[1], z=new_pos[2],
            vx=new_vel[0], vy=new_vel[1], vz=new_vel[2],
            ax=k4_acc[0], ay=k4_acc[1], az=k4_acc[2],
            timestamp=state.timestamp + dt
        )

    def estimate_time_to_position(self, state: DroneState,
                                  target_pos: np.ndarray,
                                  max_time: float = 10.0) -> Optional[float]:
        dt = 0.1
        current_state = state
        min_distance = np.linalg.norm(state.position() - target_pos)

        for t in np.arange(0, max_time, dt):
            current_state = self.predict_state(current_state, dt)
            distance = np.linalg.norm(current_state.position() - target_pos)

            if distance < 0.1:
                return t + dt

            if distance < min_distance:
                min_distance = distance
            elif distance > min_distance * 2:
                return None

        return None

    def add_to_history(self, state: DroneState) -> None:
        self.state_history.append(state)

    def get_state_history(self) -> List[DroneState]:
        return self.state_history

    def clear_history(self) -> None:
        self.state_history.clear()
