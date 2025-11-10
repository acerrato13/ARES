import numpy as np
from typing import Tuple, Optional, List
from core import DroneState
from core.Shape import DroneShape


class DynamicProjections:
    def __init__(self, drone_shape: DroneShape,
                 air_resistance_coeff: float = 0.0,
                 gravity: float = 9.81,
                 wind_velocity: np.ndarray = np.array([0.0, 0.0, 0.0])):
        """
        Initialize the DynamicProjections system for 3D drone trajectory
        planning

        Args:
            drone_shape: DroneShape instance defining the drone's physical
            properties
            air_resistance_coeff: Air resistance coefficient, set to 0 to
            disable air resistance.
            gravity: Gravitational acceleration in m/s^2 (default: 9.81)
            wind_velocity: Wind velocity vector [vx, vy, vz] in m/s
            (default: [0, 0, 0])
        """
        self.shape = drone_shape
        self.air_resistance_coeff = air_resistance_coeff
        self.gravity = gravity
        self.wind_velocity = np.array(wind_velocity)
        self.state_history: List[DroneState] = []

    def predict_position(self, state: DroneState, dt: float) -> np.ndarray:
        """
        Predict future position using kinematic equations:
        https://en.wikipedia.org/wiki/Equations_of_motion
        pos = pos_0 + vel_0 * dt + 1/2 * acc * dt^2

        Args:
            state: current drone state containing pos, vel, & acc
            dt: Time step (seconds) for prediction
        Returns:
           np.ndarry: Predicted position vector [x y z] (meters)

        """
        predicted_pos = (state.position + state.velocity * dt +
                         0.5 * state.acceleration * (dt ** 2))
        return predicted_pos

    def predict_velocity(self, state: DroneState, dt: float) -> np.ndarray:
        """
        Predict future velocity considering air resistance, wind, and gravity
        https://en.wikipedia.org/wiki/Equations_of_motion
        vel = vel_0 + acc * dt

        Args:
            state: current drone state containing vel, acc
            dt: Time step (seconds) for prediction
        Returns:
           np.ndarry: Predicted velocity vector [vx vy vz] (m/s)

        """
        gravity_acc = np.array([0.0, 0.0, -self.gravity])
        acc = state.acceleration + gravity_acc

        relative_vel = state.velocity - self.wind_velocity

        if self.air_resistance_coeff > 0:
            drag_magnitude = (self.air_resistance_coeff *
                              np.linalg.norm(relative_vel))
            drag_force = -drag_magnitude * relative_vel
            acc = acc + drag_force

        predicted_vel = state.velocity + acc * dt
        return predicted_vel

    def predict_acceleration(self, state: DroneState,
                             force: Optional[np.ndarray] = None) -> np.ndarray:

        """
        Predict future acc from applied forces including gravity and drag
        (f = ma)

        Args:
            state: current drone state containing vel, acc
            force: Optional external force applied to vehicle
                if None, returns zero acceleration with gravity and drag only

        Returns:
           np.ndarry: Predicted acceleration vector [ax ay az] m/s^2

        """
        # Use provided force or zero
        if force is None:
            force = np.zeros(3)

        # Add gravity (negative z-direction)
        gravity_force = np.array([0.0, 0.0, -self.gravity])

        # Calculate relative velocity for drag
        relative_vel = state.velocity - self.wind_velocity

        # Add air resistance based on relative velocity
        if self.air_resistance_coeff > 0:
            drag_magnitude = (self.air_resistance_coeff *
                              np.linalg.norm(relative_vel))
            drag_force = -drag_magnitude * relative_vel
        else:
            drag_force = np.zeros(3)

        # Total acceleration (assuming unit mass or forces account for mass)
        predicted_acc = force + gravity_force + drag_force
        return predicted_acc

    def predict_state(self, state: DroneState, dt: float,
                      applied_force: Optional[np.ndarray] = None
                      ) -> DroneState:
        """
        Predict complete drone state after a time step dt.
        Integrates position, velocity, and acceleration forward in time
        using Euler integration method. Maintains drone shape properties
        and rotation angle.

        Args:
            state: Current drone state to predict from
            dt: Time step in seconds for prediction
            applied_force: Optional external force vector [Fx, Fy, Fz] to
            apply during the time step

        Returns:
            DroneState: New drone state at time t + dt with updated position,
                       velocity, acceleration, and timestamp
        """
        if applied_force is not None:
            new_acc = self.predict_acceleration(state, applied_force)
        else:
            new_acc = state.acceleration.copy()

        new_vel = self.predict_velocity(state, dt)
        new_pos = self.predict_position(state, dt)

        predicted_state = DroneState(
            position=new_pos,
            velocity=new_vel,
            acceleration=new_acc,
            alpha=state.alpha.copy(),
            theta=state.theta,
            theta_dot=state.theta_dot,
            timestamp=state.timestamp + dt
        )

        return predicted_state

    def predict_trajectory(self, state: DroneState, time_horizon: float,
                           dt: float = 0.1,
                           control_forces: Optional[List[np.ndarray]] = None
                           ) -> List[DroneState]:
        """
        Generate a trajectory prediction over a specified time horizon.

        Simulates the drone's motion by iteratively applying predict_state
        for each time step. Supports time-varying control forces.

        Args:
            state: Initial drone state to start trajectory from
            time_horizon: Total time in seconds to simulate
            dt: Time step in seconds for simulation resolution (default 100ms)
            control_forces: Optional list of force vectors to apply at each
            time step. If provided, should have length = time_horizon/dt.
            If shorter than required, no force is applied after list ends.

        Returns:
            List[DroneState]: Sequence of drone states representing the
            predicted trajectory, including the initial state
        """
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
        """
        Calculate time derivatives of position and velocity for ODE
        integration. This function provides the right-hand side for
        the system of ODEs: dx/dt = v, dv/dt = a = F + gravity

        Args:
            s: Current drone state
            t: Current time in seconds
            force_func: Optional function that takes (state, time) and returns
            force vector [Fx, Fy, Fz]. If None, zero force is assumed.

        Returns:
            Tuple[np.ndarray, np.ndarray]: (velocity, acceleration) where:
                - velocity: current velocity vector [vx, vy, vz]
                - acceleration: acceleration vector [ax, ay, az]

        """
        # Get control force
        force = force_func(s, t) if force_func else np.zeros(3)

        # Add gravity (negative z-direction)
        gravity_force = np.array([0.0, 0.0, -self.gravity])

        # Calculate relative velocity for drag
        relative_vel = s.velocity - self.wind_velocity

        # Add air resistance based on relative velocity
        if self.air_resistance_coeff > 0:
            drag_force = (-self.air_resistance_coeff * relative_vel *
                          np.linalg.norm(relative_vel))
        else:
            drag_force = np.zeros(3)

        # Total acceleration
        acc = force + gravity_force + drag_force

        return s.velocity, acc

    def runge_kutta_4_step(self, state: DroneState, dt: float,
                           force_func: Optional[callable] = None
                           ) -> DroneState:
        """
        Perform a single 4th-order Runge-Kutta integration step.

        Args:
            state: Current drone state to integrate from
            dt: Time step in seconds for integration
            force_func: Optional function that takes (state, time) and returns
                force vector [Fx, Fy, Fz]. If None, zero force is assumed.

        Returns:
            DroneState: New drone state after RK4 integration step
        """
        k1_vel, k1_acc = self.derivatives(state, state.timestamp, force_func)

        mid_state1 = DroneState(
            position=state.position + 0.5 * k1_vel * dt,
            velocity=state.velocity + 0.5 * k1_acc * dt,
            acceleration=state.acceleration.copy(),
            alpha=state.alpha.copy(),
            theta=state.theta,
            theta_dot=state.theta_dot,
            timestamp=state.timestamp + 0.5 * dt
        )

        k2_vel, k2_acc = self.derivatives(mid_state1, mid_state1.timestamp,
                                          force_func)

        mid_state2 = DroneState(
            position=state.position + 0.5 * k2_vel * dt,
            velocity=state.velocity + 0.5 * k2_acc * dt,
            acceleration=state.acceleration.copy(),
            alpha=state.alpha.copy(),
            theta=state.theta,
            theta_dot=state.theta_dot,
            timestamp=state.timestamp + 0.5 * dt
        )
        k3_vel, k3_acc = self.derivatives(mid_state2, mid_state2.timestamp,
                                          force_func)

        end_state = DroneState(
            position=state.position + k3_vel * dt,
            velocity=state.velocity + k3_acc * dt,
            acceleration=state.acceleration.copy(),
            alpha=state.alpha.copy(),
            theta=state.theta,
            theta_dot=state.theta_dot,
            timestamp=state.timestamp + dt
        )
        k4_vel, k4_acc = self.derivatives(end_state, end_state.timestamp,
                                          force_func)

        new_pos = (state.position +
                   (dt / 6.0) * (k1_vel + 2*k2_vel + 2*k3_vel + k4_vel))
        new_vel = (state.velocity +
                   (dt / 6.0) * (k1_acc + 2*k2_acc + 2*k3_acc + k4_acc))

        return DroneState(
            position=new_pos,
            velocity=new_vel,
            acceleration=k4_acc,
            alpha=state.alpha.copy(),
            theta=state.theta,
            theta_dot=state.theta_dot,
            timestamp=state.timestamp + dt
        )

    def estimate_time_to_position(self, state: DroneState,
                                  target_pos: np.ndarray,
                                  max_time: float = 10.0) -> Optional[float]:
        """
        Estimate the time required to reach a target position.

        Simulates forward in time until the drone comes within 0.1 meters
        of the target position. Stops early if moving away from target.

        Args:
            state: Starting drone state
            target_pos: Target position vector [x, y, z] in meters
            max_time: Maximum time in seconds to simulate (default: 10.0)

        Returns:
            Optional[float]: Estimated time in seconds to reach target, or None
                           if target not reached within max_time or if moving
                           away from target
        """

        dt = 0.1
        current_state = state
        min_distance = np.linalg.norm(state.position - target_pos)

        for t in np.arange(0, max_time, dt):
            current_state = self.predict_state(current_state, dt)
            distance = np.linalg.norm(current_state.position - target_pos)

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
