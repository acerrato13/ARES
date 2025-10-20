import numpy as np
from typing import Tuple, Optional, List

from core import DroneState


class DynamicProjections:
    def __init__(self, gravity: float = 9.81,
                 air_resistance_coeff: float = 0.0):
        """
        Initialize the DynamicProjections system for drone trajectory planning

        Args:
            gravity: Gravitational acceleration constant 
            air_resistance_coeff: Air resistance coefficient, set to 0 to 
                disable air resistance. 
        """
        self.gravity = gravity
        self.air_resistance_coeff = air_resistance_coeff
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
        pos = state.position()
        vel = state.velocity()
        acc = state.acceleration()

        acc_with_gravity = acc.copy()
        acc_with_gravity[2] -= self.gravity

        predicted_pos = pos + vel * dt + 0.5 * acc_with_gravity * (dt ** 2)
        return predicted_pos

    def predict_velocity(self, state: DroneState, dt: float) -> np.ndarray:
        """
        Predict future velocity using considering gravity and air resistance
        https://en.wikipedia.org/wiki/Equations_of_motion
        vel = vel_0 + acc * dt

        Args:
            state: current drone state containing vel, acc, mass
            dt: Time step (seconds) for prediction
        Returns:
           np.ndarry: Predicted velocity vector [x y z] (meters)
            
        """

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

        """
        Predict future acc from applied forces (f = ma)

        Args:
            state: current drone state containing vel, acc, mass
            force: Optional external force applied to vehicle
                if None, current acceleration is returned 
        Returns:
           np.ndarry: Predicted acceleration vector [ax ay az] m/s^2
            
        """
        if force is None:
            return state.acceleration()

        predicted_acc = force / state.mass
        return predicted_acc

    def predict_state(self, state: DroneState, dt: float,
                      applied_force: Optional[np.ndarray] = None
                      ) -> DroneState: 
        """
        Predict complete drone state after a time step dt. 
        Integrates position, velocity, and acceleration forward in time
        using Euler integration method. Includes gravity effects and
        optional external forces.
        
        Args:
            state: Current drone state to predict from
            dt: Time step in seconds for prediction
            applied_force: Optional external force vector [Fx, Fy, Fz] in Newtons
                         to apply during the time step
                         
        Returns:
            DroneState: New drone state at time t + dt with updated position,
                       velocity, acceleration, and timestamp
        """
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
        """
        Generate a trajectory prediction over a specified time horizon.
        
        Simulates the drone's motion by iteratively applying predict_state
        for each time step. Supports time-varying control forces.
        
        Args:
            state: Initial drone state to start trajectory from
            time_horizon: Total time in seconds to simulate
            dt: Time step in seconds for simulation resolution (default 100ms)
            control_forces: Optional list of force vectors to apply at each time step.
                          If provided, should have length = time_horizon/dt.
                          If shorter than required, no force is applied after list ends.
                          
        Returns:
            List[DroneState]: Sequence of drone states representing the predicted
                             trajectory, including the initial state
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
        Calculate time derivatives of position and velocity for ODE integration.
        This function provides the right-hand side for the system of ODEs:
        dx/dt = v, dv/dt = a = F/m - g
        
        Args:
            s: Current drone state
            t: Current time in seconds
            force_func: Optional function that takes (state, time) and returns
                       force vector [Fx, Fy, Fz]. If None, zero force is assumed.
                       
        Returns:
            Tuple[np.ndarray, np.ndarray]: (velocity, acceleration) where:
                - velocity: current velocity vector [vx, vy, vz]
                - acceleration: acceleration vector [ax, ay, az] including gravity
        """
        force = force_func(s, t) if force_func else np.zeros(3)
        acc = force / s.mass
        acc[2] -= self.gravity
        return s.velocity(), acc

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
