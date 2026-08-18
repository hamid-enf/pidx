"""Cascade control coordinator. Mirrors src/pid_cascade.c.

The whole module is one idea: run the loops outermost-to-innermost, then push
the truth about what the actuator actually did back the other way so that no
outer integrator can wind up against a saturated inner loop.

Measured benefit against a single loop on the same plant: IAE 0.00628 vs
0.05289, an 8.4x improvement (tests/test_cascade in the C tree).
"""

from enum import IntEnum

from . import mathutil as m
from .types import (FLAG_SATURATED_HIGH, FLAG_SATURATED_LOW, Mode,
                    PIDX_CASCADE_MAX_LOOPS, Status)


class CascadeAntiWindup(IntEnum):
    NONE = 0
    BACK_CALC = 1
    FREEZE = 2


class _Level:
    __slots__ = ("pid", "decimation", "sp_min", "sp_max")

    def __init__(self, pid):
        self.pid = pid
        self.decimation = 1
        self.sp_min = 0.0
        self.sp_max = 0.0      # min >= max means the clamp is disabled


class Cascade:
    """A chain of controllers, outermost first."""

    def __init__(self, loops=None):
        self.level = []
        self.tick = []
        self.command = []
        self.count = 0
        self.aw_mode = int(CascadeAntiWindup.BACK_CALC)
        self.aw_gain = 1.0
        self.mode = int(Mode.AUTOMATIC)
        self.output = 0.0
        self.last_error = int(Status.OK)
        self.initialised = False
        if loops is not None:
            rc = self.init(loops)
            if rc != Status.OK:
                raise ValueError("cascade init failed: %d" % int(rc))

    # ------------------------------------------------------------------

    def _set_error(self, e):
        """Sticky, first-wins: the first failure in a cycle is informative."""
        if e != Status.OK and self.last_error == Status.OK:
            self.last_error = int(e)

    def init(self, loops):
        n = len(loops)
        if n < 2 or n > PIDX_CASCADE_MAX_LOOPS:
            return Status.ERR_INVALID_PARAM
        for lp in loops:
            if lp is None:
                return Status.ERR_NULL
            # A controller that never went through init() has no usable dt, so
            # the cascade could not compute per-level periods. Catch it here
            # rather than producing silent nonsense at the first update.
            if lp.get_sample_time() <= 0.0:
                return Status.ERR_NOT_INIT

        self.level = [_Level(lp) for lp in loops]
        self.tick = [0] * n
        self.command = [0.0] * n
        self.count = n
        self.aw_mode = int(CascadeAntiWindup.BACK_CALC)
        self.mode = int(loops[0].get_mode())
        self.output = 0.0
        self.last_error = int(Status.OK)
        self.initialised = True

        # Derive Kt_c from the outer loop: unwind at roughly the rate that loop
        # winds up, i.e. 1/Ti = Ki/Kp. Falls back to 1 1/s when the outer loop
        # has no integral action yet.
        self.aw_gain = 1.0
        rc, kp, ki, _ = loops[0].get_gains()
        if rc == Status.OK and ki > 0.0 and kp > 0.0:
            self.aw_gain = ki / kp
        return Status.OK

    def config_level(self, index, decimation=1, sp_min=0.0, sp_max=0.0):
        """Set a level's execution rate and the range its command is clamped
        to. decimation=N runs that level once every N cascade updates."""
        if not self.initialised or index >= self.count:
            return Status.ERR_INVALID_PARAM
        if not m.isfinite(sp_min) or not m.isfinite(sp_max):
            return Status.ERR_INVALID_LIMIT
        self.level[index].decimation = 1 if decimation == 0 else decimation
        self.level[index].sp_min = sp_min
        self.level[index].sp_max = sp_max
        self.tick[index] = 0
        return Status.OK

    def set_anti_windup(self, mode, aw_gain=0.0):
        if mode > CascadeAntiWindup.FREEZE:
            return Status.ERR_INVALID_PARAM
        if not m.isfinite(aw_gain):
            return Status.ERR_INVALID_PARAM
        self.aw_mode = int(mode)
        if aw_gain > 0.0:
            self.aw_gain = aw_gain
        return Status.OK

    # ------------------------------------------------------------------

    def update(self, measurements, setpoint, dt):
        """One cascade cycle. `measurements[i]` belongs to level i."""
        if not self.initialised or self.count < 2:
            self._set_error(Status.ERR_NOT_INIT)
            return self.output
        if measurements is None:
            self._set_error(Status.ERR_NULL)
            return self.output
        if not m.isfinite(dt) or dt <= 0.0:
            self._set_error(Status.ERR_INVALID_DT)
            return self.output
        if not m.isfinite(setpoint):
            self._set_error(Status.ERR_NAN_INPUT)
            return self.output

        ran = [False] * self.count
        sp = setpoint

        # ---------------- Forward pass: outer -> inner ------------------
        for i in range(self.count):
            lv = self.level[i]
            dec = lv.decimation if lv.decimation != 0 else 1

            self.tick[i] += 1
            if self.tick[i] >= dec:
                # This level integrates over the whole interval since it last
                # ran, not over the caller's dt - otherwise a decimated loop
                # would under-integrate by exactly its decimation factor.
                level_dt = dt * float(dec)
                self.tick[i] = 0
                ran[i] = True
                lv.pid.set_setpoint_immediate(sp)
                self.command[i] = lv.pid.update_dt(measurements[i], level_dt)
            # else: hold the previous command - a zero-order hold, which is
            # what the child physically experiences anyway.

            sp = self.command[i]
            if lv.sp_min < lv.sp_max:
                sp = m.clamp(sp, lv.sp_min, lv.sp_max)

        self.output = self.command[self.count - 1]

        # ---------------- Backward pass: inner -> outer -----------------
        # For each parent, ask whether its child could actually deliver what
        # was requested; if not, correct the parent so its integrator stops
        # accumulating against a wall. Same-cycle, like the core's own
        # back-calculation.
        #
        # HOLD means "the integrator does not move" and MANUAL means "the
        # integrator is owned by the tracking back-solve". Both write the
        # parent's integrator directly, bypassing the core's stage-10 mode
        # guard, so the mode has to be honoured here as well.
        if (self.aw_mode != CascadeAntiWindup.NONE
                and self.mode == Mode.AUTOMATIC):
            for i in range(self.count - 1, 0, -1):
                p = i - 1
                child = self.level[i].pid
                parent = self.level[p].pid
                requested = self.command[p]

                if not ran[p] or not m.isfinite(requested):
                    continue

                child_high = (child.flags & FLAG_SATURATED_HIGH) != 0
                child_low = (child.flags & FLAG_SATURATED_LOW) != 0

                clipped_high = (self.level[p].sp_min < self.level[p].sp_max
                                and requested > self.level[p].sp_max)
                clipped_low = (self.level[p].sp_min < self.level[p].sp_max
                               and requested < self.level[p].sp_min)

                # Establish what was actually achievable downstream, and only
                # act when the parent is pushing FURTHER into the obstruction.
                # Direction matters: a child pinned at its upper rail must
                # still let its parent integrate downwards - that is how the
                # pair escapes saturation. Correcting both directions would
                # turn anti-windup into a lock-up.
                if clipped_high:
                    achievable = self.level[p].sp_max
                elif clipped_low:
                    achievable = self.level[p].sp_min
                elif child_high and requested > measurements[i]:
                    achievable = measurements[i]
                elif child_low and requested < measurements[i]:
                    achievable = measurements[i]
                else:
                    continue        # child is keeping up

                if not m.isfinite(achievable):
                    continue

                dec_p = (self.level[p].decimation
                         if self.level[p].decimation != 0 else 1)
                parent_dt = dt * float(dec_p)

                if self.aw_mode == CascadeAntiWindup.BACK_CALC:
                    #   I_parent += Kt_c * (u_achievable - u_requested) * dt
                    # Identical in form to the core's back-calculation, with
                    # the child standing in for the actuator. Signed and
                    # proportional to the shortfall, so it fades smoothly to
                    # zero as the child recovers and carries no state.
                    corr = self.aw_gain * (achievable - requested) * parent_dt
                    if m.isfinite(corr):
                        parent.set_integrator(parent.get_integrator() + corr)
                else:
                    # FREEZE: undo just this cycle's accumulation, and only
                    # when the parent's error would drive it deeper into the
                    # blocked direction. Whatever it had already banked stays.
                    e = parent.get_error()
                    digging = ((achievable < requested and e > 0.0)
                               or (achievable > requested and e < 0.0))
                    if digging:
                        rc, kp, ki, kd = parent.get_gains()
                        if rc == Status.OK:
                            step = ki * e * parent_dt
                            if m.isfinite(step):
                                parent.set_integrator(
                                    parent.get_integrator() - step)

        # Surface the first per-loop failure so the user does not have to poll
        # every controller to notice a dead sensor.
        for i in range(self.count):
            e = self.level[i].pid.peek_last_error()
            if e != Status.OK:
                self._set_error(e)
                break

        return self.output

    # ------------------------------------------------------------------

    def set_mode(self, mode):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if mode > Mode.HOLD:
            return Status.ERR_INVALID_MODE

        rc = Status.OK
        if mode == Mode.MANUAL:
            # Make the chain self-consistent before freezing it. Each outer
            # loop is told to hold the setpoint its child is currently
            # following, so every level's tracking back-solve lands on a value
            # that is actually true. Skip this and only the innermost loop is
            # bumpless. Innermost first, so each parent reads a child that has
            # already been placed in manual with a settled setpoint.
            for k in range(self.count - 1, -1, -1):
                r = self.level[k].pid.set_mode(Mode.MANUAL)
                if r != Status.OK:
                    rc = r
                if k > 0:
                    held = self.level[k].pid.get_setpoint()
                    r = self.level[k - 1].pid.set_manual_output(held)
                    if r != Status.OK:
                        rc = r
                    self.command[k - 1] = held
        else:
            # Outermost first on the way back: by the time a child switches to
            # AUTOMATIC its parent is already producing a live setpoint.
            for i in range(self.count):
                r = self.level[i].pid.set_mode(mode)
                if r != Status.OK:
                    rc = r

        self.mode = int(mode)
        self._set_error(rc)
        return rc

    def set_manual_output(self, output):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(output):
            return Status.ERR_INVALID_PARAM
        # The manual value is an actuator command, so it belongs to the
        # innermost loop. Outer loops keep tracking their children.
        self.command[self.count - 1] = output
        self.output = output
        return self.level[self.count - 1].pid.set_manual_output(output)

    def reset(self):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        rc = Status.OK
        for i in range(self.count):
            r = self.level[i].pid.reset()
            if r != Status.OK:
                rc = r
            self.tick[i] = 0
            self.command[i] = 0.0
        self.output = 0.0
        self.last_error = int(Status.OK)
        return rc

    def get_output(self):
        return self.output

    def get_last_error(self):
        code = self.last_error
        self.last_error = int(Status.OK)
        return code
