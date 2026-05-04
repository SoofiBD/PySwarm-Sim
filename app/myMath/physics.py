"""
Classical mechanics force models for a quadcopter UAV.

Mathematical model
==================
Newton's second law:   F_net = m · a   ⟹   a = F_net / m

Forces modelled
---------------
1. Thrust   (F_T)  — PD controller output, clamped to motor limits
2. Gravity  (F_g)  — constant downward acceleration g = 9.81 m/s²
3. Drag     (F_d)  — quadratic (Newtonian) drag, opposes velocity

Thrust — PD controller
-----------------------
A proportional-derivative controller drives the drone toward its target
while damping velocity overshoot:

    F_T = m · (k_p · (x_goal − x) − k_d · v) + m·g·ẑ

The gravity-compensation term (+m·g·ẑ) keeps the drone airborne at rest;
the PD terms steer it toward the goal.  The total is clamped to F_max so
motor saturation is respected.

Drag — quadratic (Newtonian) model
-----------------------------------
At drone flight speeds (< 30 m/s) the dominant aerodynamic drag is:

    F_d = −½ · ρ · C_d · A · |v|² · v̂

where
    ρ  = 1.225 kg/m³  (ISA sea-level air density at 15 °C)
    C_d = 0.47         (sphere approximation for a quadcopter body)
    A   = 0.04 m²      (reference cross-section, ~20 cm × 20 cm)
    v̂  = v / |v|       (unit velocity direction)

Integration — first-order Euler
---------------------------------
    a(t)      = F_net(t) / m
    v(t + dt) = v(t) + a(t) · dt
    x(t + dt) = x(t) + v(t + dt) · dt

Euler integration is O(1) per step and accurate enough for dt ≤ 0.05 s
at the speed scales of this simulation.  Switch to RK4 if you need
long-horizon energy accuracy.
"""

import numpy as np

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------
AIR_DENSITY: float = 1.225      # kg/m³  ISA sea level
GRAVITY: float = 9.81           # m/s²
DRAG_COEFF: float = 0.47        # C_d — sphere approximation
REF_AREA: float = 0.04          # m²    ~20 cm × 20 cm cross-section


class DronePhysics:
    """
    Stateless collection of force-calculation functions.

    All functions operate on raw numpy arrays (shape (3,)) for speed.
    The UAV class calls `step()` to obtain updated (position, velocity).
    """

    @staticmethod
    def gravity_force(mass: float) -> np.ndarray:
        """F_g = −m·g·ẑ  (downward in simulation Z-up convention)."""
        return np.array([0.0, 0.0, -mass * GRAVITY], dtype=np.float64)

    @staticmethod
    def drag_force(velocity: np.ndarray) -> np.ndarray:
        """
        Quadratic aerodynamic drag.

            F_d = −½·ρ·C_d·A·|v|²·v̂

        Returns zero vector when |v| < ε (avoids division by zero at rest).
        """
        speed_sq = float(np.dot(velocity, velocity))
        if speed_sq < 1e-9:
            return np.zeros(3, dtype=np.float64)
        speed = np.sqrt(speed_sq)
        unit_vel = velocity / speed
        magnitude = 0.5 * AIR_DENSITY * DRAG_COEFF * REF_AREA * speed_sq
        return -magnitude * unit_vel

    @staticmethod
    def thrust_force(
        desired_pos: np.ndarray,
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        mass: float,
        max_thrust: float,
        kp: float = 3.0,
        kd: float = 2.0,
    ) -> np.ndarray:
        """
        PD controller thrust.

            F_T = m·(k_p·(x_goal−x) − k_d·v) + m·g·ẑ
                  clamped to |F_T| ≤ max_thrust

        k_p  proportional gain — stiffness toward goal
        k_d  derivative gain   — velocity damping (prevents overshoot)

        The gravity-compensation term (+m·g·ẑ) is added so that when
        position error is zero the drone hovers without falling.
        """
        error = desired_pos - current_pos
        gravity_comp = np.array([0.0, 0.0, mass * GRAVITY], dtype=np.float64)
        raw = mass * (kp * error - kd * current_vel) + gravity_comp
        norm = float(np.linalg.norm(raw))
        if norm > max_thrust:
            raw = raw * (max_thrust / norm)
        return raw

    @staticmethod
    def step(
        pos: np.ndarray,
        vel: np.ndarray,
        desired_pos: np.ndarray,
        mass: float,
        max_thrust: float,
        dt: float,
        kp: float = 3.0,
        kd: float = 2.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Euler-integrate one physics timestep.

        Returns
        -------
        pos_new : np.ndarray  shape (3,)
        vel_new : np.ndarray  shape (3,)
        """
        f_thrust = DronePhysics.thrust_force(desired_pos, pos, vel, mass, max_thrust, kp, kd)
        f_gravity = DronePhysics.gravity_force(mass)
        f_drag = DronePhysics.drag_force(vel)

        f_net = f_thrust + f_gravity + f_drag
        accel = f_net / mass            # a = F / m

        vel_new = vel + accel * dt      # v(t+dt) = v(t) + a·dt
        pos_new = pos + vel_new * dt    # x(t+dt) = x(t) + v(t+dt)·dt

        return pos_new, vel_new
