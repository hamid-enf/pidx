"""Setpoint trajectory shaping. Mirrors pid_shaper.h / pid_shaper.c.

A step change in the setpoint asks the plant for infinite acceleration. The
controller responds with whatever the actuator can deliver, saturates, winds up
and overshoots. Shaping the setpoint into something the plant can actually
follow fixes the problem at its source instead of detuning the controller to
survive a command that was never physical.

Deceleration must begin when the remaining distance equals

    d_brake = v^2 / (2 * a_decel)

which comes from integrating v*dv = a*dx. Starting later guarantees overshoot;
starting earlier just wastes time. The check runs every sample against the
current velocity, so the profile is self-correcting and a mid-flight target
change needs no replanning.

There is no jerk limit and no multi-axis coordination: those belong in a motion
planner, not a PID library.
"""

from . import mathutil as m
from .types import Status


def profile_step(pos, vel, target, rate_max, accel, decel, dt):
    """Advance a trapezoidal/rate profile one step.

    The single implementation shared by the controller's built-in shaper and
    the standalone Shaper object, exactly as pids_profile_step() is in C.

    Returns (new_pos, new_vel, moving).
    """
    dist = target - pos
    v = vel

    if rate_max <= 0.0:
        return target, 0.0, False
    if dist == 0.0:
        return pos, 0.0, False

    if accel <= 0.0:
        v = rate_max if dist > 0.0 else -rate_max
    else:
        d = decel if decel > 0.0 else accel
        brake = (v * v) / (2.0 * d)

        if dist > 0.0:
            if v < 0.0:
                v += d * dt            # travelling the wrong way
            elif brake >= dist:
                v -= d * dt            # time to start braking
            else:
                v += accel * dt        # speed up
        else:
            if v > 0.0:
                v -= d * dt
            elif brake >= -dist:
                v += d * dt
            else:
                v -= accel * dt
        v = m.clamp(v, -rate_max, rate_max)

    step = v * dt
    if m.fabs(step) >= m.fabs(dist):
        # Landing sample: snap to the target rather than overshoot it.
        return target, 0.0, False
    return pos + step, v, True


class Shaper:
    """A reusable trajectory generator.

    The controller has this built in (set_setpoint_ramp); use this type when
    you want to shape a signal that is not a PID setpoint, or to run a profile
    ahead of time.
    """

    __slots__ = ("position", "velocity", "target", "rate_max", "accel",
                 "decel", "moving")

    def __init__(self, rate_max=0.0, accel=0.0, decel=0.0):
        self.position = 0.0
        self.velocity = 0.0
        self.target = 0.0
        self.rate_max = rate_max
        self.accel = accel
        self.decel = decel
        self.moving = False

    def init(self, rate_max, accel, decel):
        if (not m.isfinite(rate_max) or rate_max < 0.0
                or not m.isfinite(accel) or accel < 0.0
                or not m.isfinite(decel) or decel < 0.0):
            return Status.ERR_INVALID_PARAM
        self.position = 0.0
        self.velocity = 0.0
        self.target = 0.0
        self.rate_max = rate_max
        self.accel = accel
        self.decel = decel
        self.moving = False
        return Status.OK

    def set_target(self, target):
        """Command a new destination. Does not reset velocity - it blends."""
        if not m.isfinite(target):
            return Status.ERR_INVALID_PARAM
        self.target = target
        self.moving = (target != self.position)
        return Status.OK

    def reset(self, position=0.0):
        """Teleport to `position` with zero velocity, e.g. after homing."""
        if not m.isfinite(position):
            return Status.ERR_INVALID_PARAM
        self.position = position
        self.target = position
        self.velocity = 0.0
        self.moving = False
        return Status.OK

    def update(self, dt):
        """Advance one timestep and return the new shaped position."""
        if not m.isfinite(dt) or dt <= 0.0:
            return self.position
        if self.rate_max <= 0.0:
            self.position = self.target      # shaping disabled: pass through
            self.velocity = 0.0
            self.moving = False
            return self.position

        self.position, self.velocity, self.moving = profile_step(
            self.position, self.velocity, self.target,
            self.rate_max, self.accel, self.decel, dt)
        return self.position

    def is_moving(self):
        return self.moving

    def estimate_time(self):
        """Seconds to reach the current target from rest.

        Trapezoidal: if the distance is long enough to reach cruise speed,
        t = (d - d_ramp)/v + v/a + v/b; otherwise the profile is triangular and
        peaks at v_peak = sqrt(2*d*a*b/(a+b)). Ignores the current velocity by
        design - it answers "how long is this move", not "how long is left".
        """
        d = m.fabs(self.target - self.position)
        if d == 0.0 or self.rate_max <= 0.0:
            return 0.0

        v = self.rate_max
        a = self.accel
        if a <= 0.0:
            return d / v                      # rate-only profile
        b = self.decel if self.decel > 0.0 else a

        d_ramp = ((v * v) / (2.0 * a)) + ((v * v) / (2.0 * b))
        if d >= d_ramp:
            return ((d - d_ramp) / v) + (v / a) + (v / b)

        vp = m.sqrt((2.0 * d * a * b) / (a + b))
        return (vp / a) + (vp / b)
