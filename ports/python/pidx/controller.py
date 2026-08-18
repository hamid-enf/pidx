"""The PIDX control law. A faithful port of src/pid.c.

CONTROL LAW
-----------
    u = Kp*(beta*r - y) + Ki*integral(r - y) + (Kd*s/(1 + s*Tf))*(gamma*r - y)
        + u_ff

The integral always acts on the UNWEIGHTED error (r - y). Weighting it would
leave a steady-state offset of (1 - beta)*r, which is not a tuning choice, it
is a bug.

INTEGRATOR UNITS
----------------
`self.integrator` holds Ki*integral(e) - the integral TERM in output units, not
the raw integral of the error. Three consequences, all of them wanted:

  * changing Ki at runtime does not step the output, so gain scheduling and
    auto-tune application are bumpless for free;
  * integral limits are expressed in the same units as the output limits, so
    one can inherit from the other;
  * `set_gains_rescale_integral()` exists for callers who want the classic
    behaviour where the accumulated integral(e) is preserved instead.

DERIVATIVE
----------
Discretised as one filtered block rather than a difference followed by a
low-pass:

    D_k = c_da*D_{k-1} - c_db*(x_k - x_{k-1})
    c_da = Tf/(Tf + dt)        c_db = Kd/(Tf + dt)

The pole c_da lies in [0,1) for every Tf >= 0 and dt > 0, so the term can never
diverge. The naive form has pole (1 - dt/Tf), which leaves the unit circle as
soon as Tf < dt/2 - i.e. exactly when a user picks a light filter.

STAGE ORDER
-----------
0 guards, 1 timing, 2 sensor validation, 3 input filter, 4 setpoint,
5 gain scheduling, 6 error and P, 7 derivative, 8 feedforward,
9 manual/hold, 10 integral, 11 sum, 12 output saturation,
13 back-calculation/tracking/conditional rollback, 14 output slew.

The order is not arbitrary and the port must not "tidy" it. Anti-windup in
stage 13 runs in the SAME sample as the saturation it corrects; deferring it
one cycle inserts a delay into the anti-windup loop that shows up as extra
overshoot on recovery.
"""

import math

from . import mathutil as m
from .filters import LPF1
from .shaper import profile_step
from .types import (
    AntiWindup, Config, Direction, DerivativeMode, Input, IntegrationMethod,
    Mode, Status, StatusSnapshot,
    PID_HUGE_F, PIDX_CONFIG_ABI_VERSION, PIDX_VERSION_STRING,
    FEAT_ADVANCED_MASK, FEAT_DERIVATIVE, FEAT_D_FILTER, FEAT_DIAGNOSTICS,
    FEAT_FEEDFORWARD, FEAT_GAIN_SCHED, FEAT_INPUT_FILTER, FEAT_INTEGRAL,
    FEAT_INTEGRAL_LIMIT, FEAT_OUTPUT_LIMIT, FEAT_OUT_SHAPER, FEAT_SAFETY,
    FEAT_SP_SHAPER, FEAT_TELEMETRY,
    FLAG_DT_VIOLATION, FLAG_FAULT, FLAG_INTEGRAL_ACTIVE, FLAG_INTEGRAL_LIMITED,
    FLAG_MANUAL, FLAG_OUTPUT_SLEWING, FLAG_SATURATED, FLAG_SATURATED_HIGH,
    FLAG_SATURATED_LOW, FLAG_SENSOR_INVALID, FLAG_SP_RAMPING, FLAG_TUNING,
)

_NAN = float("nan")


def config_default():
    """A Config with every field at its documented default."""
    return Config()


def _gain_ok(g):
    """A gain must be finite and non-negative; sign lives in `direction`."""
    return m.isfinite(g) and g >= 0.0


class PID:
    """A PID controller. The analogue of PID_Handle plus its API."""

    def __init__(self, cfg=None):
        self._zero_state()
        if cfg is not None:
            rc = self.init(cfg)
            if rc != Status.OK:
                raise ValueError("PID init failed: %d" % int(rc))

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _zero_state(self):
        """Put every field in a defined state, as PID_Init's memset does."""
        self.initialised = False

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        # dt-dependent coefficients, rebuilt by _recompute()
        self.c_i = 0.0
        self.c_da = 0.0
        self.c_db = 0.0
        self.c_aw = 0.0
        self.dt_last = 0.0
        self.dt_nominal = 0.0
        self.dt_min = 0.0
        self.dt_max = 0.0

        self.setpoint = 0.0
        self.setpoint_target = 0.0
        self.sp_velocity = 0.0

        self.integrator = 0.0
        self.d_state = 0.0
        self.d_prev_in = 0.0
        self.e_prev = 0.0
        self.output = 0.0
        self.manual_output = 0.0
        self.tracking_input = 0.0

        self.out_min = -PID_HUGE_F
        self.out_max = PID_HUGE_F
        self.i_min = -PID_HUGE_F
        self.i_max = PID_HUGE_F

        self.tf = 0.0
        self.n_filter = 0.0
        self.kt = 0.0
        self.i_separation = 0.0
        self.i_deadband = 0.0
        self.beta = 1.0
        self.gamma = 0.0

        self.ff_fn = None
        self.ff_ctx = None
        self.ff_value = 0.0
        self.ff_gain = 1.0

        self.sp_rate_max = 0.0
        self.sp_accel = 0.0
        self.sp_decel = 0.0
        self.out_slew_max = 0.0

        self.in_lpf = LPF1()

        self.meas_min = 0.0
        self.meas_max = 0.0
        self.meas_rate_max = 0.0
        self.failsafe_output = 0.0
        self.fault_persist_n = 1
        self.auto_recover = False
        self.fault_count = 0
        self.meas_prev = 0.0
        self.meas_prev_valid = False

        self.sched = None
        self.sched_var_ext = 0.0

        self.dir_sign = 1
        self.mode = int(Mode.AUTOMATIC)
        self.integ_method = int(IntegrationMethod.BACKWARD_EULER)
        self.aw_mode = int(AntiWindup.CLAMP)
        self.d_mode = int(DerivativeMode.ON_MEASUREMENT)

        self.features = 0
        self.flags = 0
        self.last_error = int(Status.OK)
        self.status = StatusSnapshot()
        self.telemetry = None

    def init(self, cfg):
        """Validate `cfg` and configure this controller.

        Validation happens BEFORE any field is written, so a rejected config
        leaves an already-working controller untouched.
        """
        if cfg is None:
            return Status.ERR_NULL
        if cfg.abi_version != PIDX_CONFIG_ABI_VERSION:
            return Status.ERR_INVALID_CONFIG
        if (not _gain_ok(cfg.core.kp) or not _gain_ok(cfg.core.ki)
                or not _gain_ok(cfg.core.kd)):
            return Status.ERR_INVALID_GAIN
        if not m.isfinite(cfg.core.sample_time) or cfg.core.sample_time <= 0.0:
            return Status.ERR_INVALID_DT
        if cfg.limits.use_output_limits and (
                not m.isfinite(cfg.limits.output_min)
                or not m.isfinite(cfg.limits.output_max)
                or cfg.limits.output_min >= cfg.limits.output_max):
            return Status.ERR_INVALID_LIMIT
        if (cfg.limits.use_integral_limits
                and cfg.limits.integral_min >= cfg.limits.integral_max):
            return Status.ERR_INVALID_LIMIT
        if (cfg.integral.mode == AntiWindup.BACK_CALCULATION
                and not cfg.limits.use_output_limits
                and not cfg.limits.use_integral_limits):
            # u_sat would always equal u_raw, so the correction term is
            # identically zero: the user almost certainly forgot the limits.
            return Status.ERR_INVALID_LIMIT
        if (not m.isfinite(cfg.weight.beta) or not m.isfinite(cfg.weight.gamma)
                or cfg.weight.beta < 0.0 or cfg.weight.beta > 2.0
                or cfg.weight.gamma < 0.0 or cfg.weight.gamma > 2.0):
            return Status.ERR_INVALID_CONFIG

        self._zero_state()

        self.kp = cfg.core.kp
        self.ki = cfg.core.ki
        self.kd = cfg.core.kd
        self.dt_nominal = cfg.core.sample_time
        self.dt_min = cfg.limits.dt_min
        self.dt_max = cfg.limits.dt_max
        self.dir_sign = -1 if cfg.core.direction == Direction.REVERSE else 1
        self.mode = int(cfg.core.mode)
        self.integ_method = int(cfg.core.integration)

        self.out_min = cfg.limits.output_min
        self.out_max = cfg.limits.output_max

        # i_min/i_max always hold the EFFECTIVE bounds, resolved once here.
        # Without explicit integral limits they inherit the output limits: an
        # integrator that can demand more than the actuator can deliver is
        # just windup waiting to happen. Resolving here rather than per-cycle
        # also lets the fast path clamp against these fields directly.
        if cfg.limits.use_integral_limits:
            self.i_min = cfg.limits.integral_min
            self.i_max = cfg.limits.integral_max
        elif cfg.limits.use_output_limits:
            self.i_min = cfg.limits.output_min
            self.i_max = cfg.limits.output_max
        else:
            self.i_min = -PID_HUGE_F
            self.i_max = PID_HUGE_F

        self.d_mode = int(cfg.filter.derivative_mode)
        self.tf = cfg.filter.tf
        self.n_filter = cfg.filter.n_filter

        self.aw_mode = int(cfg.integral.mode)
        self.kt = cfg.integral.kt
        self.i_separation = cfg.integral.separation_threshold
        self.i_deadband = cfg.integral.deadband

        self.beta = cfg.weight.beta
        self.gamma = cfg.weight.gamma

        feat = FEAT_DERIVATIVE | FEAT_D_FILTER
        if cfg.integral.enabled:
            feat |= FEAT_INTEGRAL
        if cfg.limits.use_output_limits:
            feat |= FEAT_OUTPUT_LIMIT
        if cfg.limits.use_integral_limits:
            feat |= FEAT_INTEGRAL_LIMIT

        if cfg.feedforward.enabled:
            feat |= FEAT_FEEDFORWARD
        self.ff_fn = cfg.feedforward.fn
        self.ff_ctx = cfg.feedforward.ctx
        self.ff_value = cfg.feedforward.value
        self.ff_gain = cfg.feedforward.gain if cfg.feedforward.gain != 0.0 else 1.0

        self.sp_rate_max = cfg.shaper.sp_rate_max
        self.sp_accel = cfg.shaper.sp_accel
        self.sp_decel = cfg.shaper.sp_decel
        self.out_slew_max = cfg.shaper.out_slew_max
        if self.sp_rate_max > 0.0:
            feat |= FEAT_SP_SHAPER
        if self.out_slew_max > 0.0:
            feat |= FEAT_OUT_SHAPER

        self.in_lpf.tau = cfg.filter.input_lpf_tau
        if self.in_lpf.tau > 0.0:
            feat |= FEAT_INPUT_FILTER

        self.meas_min = cfg.safety.meas_min
        self.meas_max = cfg.safety.meas_max
        self.meas_rate_max = cfg.safety.meas_rate_max
        self.failsafe_output = cfg.safety.failsafe_output
        self.fault_persist_n = (1 if cfg.safety.fault_persist_n == 0
                                else cfg.safety.fault_persist_n)
        self.auto_recover = cfg.safety.auto_recover
        if cfg.safety.enabled:
            feat |= FEAT_SAFETY

        feat |= FEAT_DIAGNOSTICS

        self.features = feat
        self.tracking_input = 0.0
        self.last_error = int(Status.OK)
        self.initialised = True

        self._recompute(self.dt_nominal)
        return Status.OK

    def init_default(self):
        return self.init(config_default())

    def deinit(self):
        """Invalidate the controller. A later update reports ERR_NOT_INIT."""
        self._zero_state()
        return Status.OK

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_error(self, code):
        """Record an error without ever overwriting it with success."""
        if code != Status.OK:
            self.last_error = int(code)

    def _effective_tf(self):
        """Resolve the derivative filter time constant.

        An explicit tf always wins. Otherwise, if the user expressed the filter
        as the ratio N, Tf = Td/N = Kd/(N*Kp). That needs a non-zero Kp; with
        Kp == 0 (a pure ID controller - unusual but legal) the ratio is
        undefined and we fall back to an unfiltered derivative rather than
        inventing a value.
        """
        if (self.features & FEAT_D_FILTER) == 0:
            return 0.0
        if self.tf > 0.0:
            return self.tf
        if self.n_filter > 0.0 and self.kp > 0.0 and self.kd > 0.0:
            return self.kd / (self.n_filter * self.kp)
        return 0.0

    def _effective_kt(self):
        """Back-calculation gain when the user passed 0.

        Astrom & Hagglund: Tt = sqrt(Ti*Td) with derivative action, Tt = Ti
        without. In parallel-form gains Ti = Kp/Ki and Td = Kd/Kp, so
        Ti*Td = Kd/Ki, giving Kt = sqrt(Ki/Kd) and Kt = Ki/Kp respectively.
        """
        kt = self.kt
        if kt <= 0.0:
            if self.ki > 0.0 and self.kd > 0.0:
                kt = m.sqrt(self.ki / self.kd)
            elif self.ki > 0.0 and self.kp > 0.0:
                kt = self.ki / self.kp
            else:
                kt = 0.0
        return kt

    def _recompute(self, dt):
        """Rebuild every dt-dependent coefficient.

        The only place in the library that divides on behalf of the control
        law, which is what keeps the update path division-free.
        """
        tf = self._effective_tf()
        den = tf + dt

        if self.integ_method == IntegrationMethod.TRAPEZOIDAL:
            self.c_i = self.ki * dt * 0.5
        else:
            self.c_i = self.ki * dt

        # den >= dt > 0, so this division is always safe.
        self.c_da = tf / den
        self.c_db = self.kd / den

        self.c_aw = self._effective_kt() * dt

        self.in_lpf.coeff(self.in_lpf.tau, dt)
        self.dt_last = dt

    def _integral_bounds(self):
        return self.i_min, self.i_max

    def _back_solve(self, desired, p, d, ff):
        """Force the integrator so P + I + D + FF reproduces `desired`.

        This one operation implements bumpless manual->auto transfer, bumpless
        fault recovery and integrator preloading.

        When the clamp bites the transfer CANNOT be bumpless: the requested
        output is not reachable from the current P/D/FF with a legal
        integrator. That is flagged rather than silently accepted, because a
        "bumpless transfer" that quietly steps the actuator is worse than one
        that tells you it could not.

        Returns True when the exact value was representable.
        """
        lo, hi = self._integral_bounds()
        want = desired - p - d - ff
        got = m.clamp(want, lo, hi)
        self.integrator = got

        if got != want:
            self.flags |= FLAG_INTEGRAL_LIMITED
            # Also sticky: FLAG_INTEGRAL_LIMITED is rebuilt every cycle, so a
            # caller who switches mode and reads the flags after the next
            # update would never see it. The sticky channel survives until the
            # application clears it, which is what makes "did my transfer
            # bump?" an answerable question.
            self._set_error(Status.ERR_INVALID_LIMIT)
            return False
        return True

    def _shape_setpoint(self, dt):
        self.setpoint, self.sp_velocity, moving = profile_step(
            self.setpoint, self.sp_velocity, self.setpoint_target,
            self.sp_rate_max, self.sp_accel, self.sp_decel, dt)
        if moving:
            self.flags |= FLAG_SP_RAMPING
        else:
            self.flags &= ~FLAG_SP_RAMPING

    def _check_sensor(self, y, dt):
        """Range then slew plausibility. Returns OK when y may be trusted."""
        if self.meas_max > self.meas_min and (y < self.meas_min
                                              or y > self.meas_max):
            return Status.ERR_SENSOR_RANGE
        if self.meas_rate_max > 0.0 and self.meas_prev_valid:
            if m.fabs(y - self.meas_prev) > (self.meas_rate_max * dt):
                return Status.ERR_SENSOR_RATE
        return Status.OK

    # ------------------------------------------------------------------
    # The update path
    # ------------------------------------------------------------------

    def _run(self, meas, dt, inp=None):
        """One control cycle. Returns (output, status)."""
        y = meas
        ff = 0.0
        i_pre = 0.0
        i_stepped = False
        recover_to = 0.0
        recovering = False
        rc = Status.OK

        # -------- Stage 0: guards --------------------------------------
        if not self.initialised:
            return 0.0, Status.ERR_NOT_INIT

        if not m.isfinite(y):
            rc = Status.ERR_NAN_INPUT if m.isnan(y) else Status.ERR_INF_INPUT
            self._set_error(rc)
            self.flags |= FLAG_SENSOR_INVALID
            if (self.features & FEAT_SAFETY) != 0:
                self.fault_count += 1
                if self.fault_count >= self.fault_persist_n:
                    self.flags |= FLAG_FAULT
                    self.output = self.failsafe_output
            # Hold the previous output: one bad sample must not command a jump.
            return self.output, rc

        # Transient flags are rebuilt every cycle; FAULT is latched separately.
        self.flags &= (FLAG_FAULT | FLAG_TUNING | FLAG_SP_RAMPING)

        # -------- Stage 1: timing --------------------------------------
        if dt <= 0.0:
            rc = Status.ERR_INVALID_DT
            self._set_error(rc)
            self.flags |= FLAG_DT_VIOLATION
            dt = self.dt_nominal
        elif ((self.dt_min > 0.0 and dt < self.dt_min)
                or (self.dt_max > 0.0 and dt > self.dt_max)):
            rc = Status.ERR_INVALID_DT
            self._set_error(rc)
            self.flags |= FLAG_DT_VIOLATION
            dt = m.clamp(dt,
                         self.dt_min if self.dt_min > 0.0 else dt,
                         self.dt_max if self.dt_max > 0.0 else dt)

        if dt != self.dt_last:
            self._recompute(dt)

        # -------- Stage 2: sensor validation ---------------------------
        if (self.features & FEAT_SAFETY) != 0:
            sc = self._check_sensor(y, dt)

            if sc != Status.OK:
                self._set_error(sc)
                self.flags |= FLAG_SENSOR_INVALID
                self.fault_count += 1
                if self.fault_count >= self.fault_persist_n:
                    self.flags |= FLAG_FAULT
            elif self.fault_count > 0:
                if self.auto_recover:
                    self.fault_count = 0
                    if (self.flags & FLAG_FAULT) != 0:
                        # Bumpless re-entry. The back-solve is DEFERRED to
                        # stage 10 because P, D and FF for this sample do not
                        # exist yet: solving now with zeros sets I to the
                        # failsafe output and then the real P term is added on
                        # top, which is a measurable step exactly where this
                        # code exists to prevent one.
                        self.flags &= ~FLAG_FAULT
                        self.d_prev_in = y
                        recover_to = self.output
                        recovering = True
                else:
                    self.fault_count = 0   # sample fine; latch stays put

            self.meas_prev = y
            self.meas_prev_valid = True

            if (self.flags & FLAG_FAULT) != 0:
                self.output = self.failsafe_output
                return self.output, (rc if rc != Status.OK
                                     else Status.ERR_SENSOR_RANGE)

        # -------- Stage 3: input filter --------------------------------
        if (self.features & FEAT_INPUT_FILTER) != 0:
            y = self.in_lpf.step(y)

        # -------- Stage 4: setpoint ------------------------------------
        if inp is not None and m.isfinite(inp.setpoint):
            self.setpoint_target = inp.setpoint

        if (self.features & FEAT_SP_SHAPER) != 0:
            self._shape_setpoint(dt)
        else:
            self.setpoint = self.setpoint_target
        sp = self.setpoint

        # -------- Stage 5: gain scheduling -----------------------------
        if (self.features & FEAT_GAIN_SCHED) != 0 and self.sched is not None:
            if inp is not None and m.isfinite(inp.schedule_var):
                var = inp.schedule_var
            else:
                src = self.sched.source
                if src == 0:      # SETPOINT
                    var = sp
                elif src == 1:    # MEASUREMENT
                    var = y
                elif src == 2:    # ERROR
                    var = sp - y
                elif src == 3:    # ABS_ERROR
                    var = m.fabs(sp - y)
                elif src == 4:    # OUTPUT
                    var = self.output
                else:             # EXTERNAL
                    var = self.sched_var_ext

            ok, nkp, nki, nkd = self.sched.evaluate(var)
            if ok == Status.OK:
                if nkp != self.kp or nki != self.ki or nkd != self.kd:
                    self.kp = nkp
                    self.ki = nki
                    self.kd = nkd
                    self._recompute(dt)

        # -------- Stage 6: error and P ---------------------------------
        dsign = float(self.dir_sign)
        e = dsign * (sp - y)
        p_term = self.kp * dsign * ((self.beta * sp) - y)

        # -------- Stage 7: derivative ----------------------------------
        # All three derivative modes are the same expression with a different
        # setpoint weight, so there is one code path instead of three:
        #   x = dir*(y - gamma_eff*r), D = -Kd/(Tf+dt)*dx, filtered.
        #   gamma_eff = 0 -> on measurement, 1 -> on error, gamma -> 2DOF.
        if self.d_mode == DerivativeMode.ON_ERROR:
            gamma_eff = 1.0
        elif self.d_mode == DerivativeMode.ON_WEIGHTED_ERROR:
            gamma_eff = self.gamma
        else:
            gamma_eff = 0.0
        d_src = dsign * (y - (gamma_eff * sp))

        if (self.features & FEAT_DERIVATIVE) != 0:
            self.d_state = ((self.c_da * self.d_state)
                            - (self.c_db * (d_src - self.d_prev_in)))
        else:
            self.d_state = 0.0
        self.d_prev_in = d_src

        # -------- Stage 8: feedforward ---------------------------------
        if (self.features & FEAT_FEEDFORWARD) != 0:
            if inp is not None and m.isfinite(inp.feedforward):
                ff = inp.feedforward * self.ff_gain
            elif self.ff_fn is not None:
                ff = self.ff_fn(sp, y, self.ff_ctx) * self.ff_gain
            else:
                ff = self.ff_value * self.ff_gain
            if not m.isfinite(ff):
                # A misbehaving user callback must not poison the controller.
                ff = 0.0
                self._set_error(Status.ERR_NAN_INPUT)

        i_lo, i_hi = self._integral_bounds()

        # Deferred from stage 2: now that p_term, d_state and ff exist for this
        # sample, the integrator can be solved so the sum reproduces the
        # fail-safe output exactly.
        if recovering:
            self._back_solve(recover_to, p_term, self.d_state, ff)

        # -------- Stage 9: manual / hold -------------------------------
        if self.mode == Mode.MANUAL:
            u = self.manual_output
            if (self.features & FEAT_OUTPUT_LIMIT) != 0:
                u = m.clamp(u, self.out_min, self.out_max)
            # Track continuously so a switch to AUTOMATIC at any instant is
            # bumpless without a special case in set_mode().
            self._back_solve(u, p_term, self.d_state, ff)
            self.output = u
            self.flags |= FLAG_MANUAL
            self.e_prev = e
            self._fill_status(meas, y, sp, e, p_term, self.d_state, ff,
                              self.output, dt)
            return u, rc

        # -------- Stage 10: integral -----------------------------------
        integrate = ((self.features & FEAT_INTEGRAL) != 0
                     and self.mode != Mode.HOLD)

        if integrate:
            ae = m.fabs(e)
            if self.i_separation > 0.0 and ae > self.i_separation:
                # Integral separation: during a large excursion the integrator
                # would charge far beyond what the steady state needs,
                # guaranteeing overshoot. P and D handle the transient; I
                # re-engages near target.
                integrate = False
            elif self.i_deadband > 0.0 and ae < self.i_deadband:
                # Deadband: stop hunting against a quantised actuator.
                integrate = False
            # Conditional integration is NOT decided here: whether this
            # sample's accumulation is admissible depends on whether the output
            # it produces saturates, which is only known after the sum in
            # stage 11. Testing the flags now would test the PREVIOUS cycle -
            # and they were just cleared. The decision is made, and undone if
            # necessary, in stage 13.

        if integrate:
            i_pre = self.integrator
            if self.integ_method == IntegrationMethod.TRAPEZOIDAL:
                self.integrator += self.c_i * (e + self.e_prev)
            else:
                self.integrator += self.c_i * e
            i_stepped = True
            self.flags |= FLAG_INTEGRAL_ACTIVE

        if self.aw_mode == AntiWindup.CLAMP:
            clamped = m.clamp(self.integrator, i_lo, i_hi)
            if clamped != self.integrator:
                self.integrator = clamped
                self.flags |= FLAG_INTEGRAL_LIMITED

        self.e_prev = e

        # -------- Stage 11: sum ----------------------------------------
        u_raw = p_term + self.integrator + self.d_state + ff

        # -------- Stage 12: output saturation --------------------------
        u = u_raw
        if (self.features & FEAT_OUTPUT_LIMIT) != 0:
            if u > self.out_max:
                u = self.out_max
                self.flags |= FLAG_SATURATED_HIGH
            elif u < self.out_min:
                u = self.out_min
                self.flags |= FLAG_SATURATED_LOW

        # -------- Stage 13: back-calculation / tracking ----------------
        # Applied in the SAME sample as the saturation it corrects. Deferring
        # it to the next cycle inserts a one-sample delay into the anti-windup
        # loop, which shows up as extra overshoot on recovery.
        if self.mode != Mode.HOLD:
            if self.aw_mode == AntiWindup.BACK_CALCULATION:
                if u != u_raw:
                    self.integrator += self.c_aw * (u - u_raw)
                    self.integrator = m.clamp(self.integrator, i_lo, i_hi)

            elif self.aw_mode == AntiWindup.CONDITIONAL:
                # Conditional integration in the Astrom sense: an increment is
                # admissible unless the output saturates AND the error would
                # drive it further past the same limit.
                #
                # The test uses u_raw - the unsaturated sum - because that is
                # what says how far past the limit the controller is asking to
                # go. The increment is UNDONE rather than merely skipped, so
                # the decision uses this sample's saturation state instead of
                # the previous one; a one-cycle-late test is the classic way
                # this strategy quietly degrades into no protection at all.
                if i_stepped and ((u_raw > self.out_max and e > 0.0)
                                  or (u_raw < self.out_min and e < 0.0)):
                    self.integrator = i_pre
                    self.flags &= ~FLAG_INTEGRAL_ACTIVE
                    self.flags |= FLAG_INTEGRAL_LIMITED

                    # Recompute: removing the increment may pull the output
                    # back inside the limits, and holding it at the limit
                    # anyway would throw away authority the controller has.
                    u_raw = p_term + self.integrator + self.d_state + ff
                    u = u_raw
                    self.flags &= ~FLAG_SATURATED
                    if (self.features & FEAT_OUTPUT_LIMIT) != 0:
                        if u > self.out_max:
                            u = self.out_max
                            self.flags |= FLAG_SATURATED_HIGH
                        elif u < self.out_min:
                            u = self.out_min
                            self.flags |= FLAG_SATURATED_LOW

            elif self.aw_mode == AntiWindup.TRACKING:
                track = self.tracking_input
                if inp is not None and m.isfinite(inp.tracking):
                    track = inp.tracking
                if m.isfinite(track):
                    self.integrator += self.c_aw * (track - u_raw)
                    self.integrator = m.clamp(self.integrator, i_lo, i_hi)

        # -------- Stage 14: output slew --------------------------------
        if (self.features & FEAT_OUT_SHAPER) != 0 and self.out_slew_max > 0.0:
            max_step = self.out_slew_max * dt
            delta = u - self.output
            if delta > max_step:
                u = self.output + max_step
                self.flags |= FLAG_OUTPUT_SLEWING
            elif delta < -max_step:
                u = self.output - max_step
                self.flags |= FLAG_OUTPUT_SLEWING

        # Final numeric guard. If anything went non-finite despite the checks
        # above - a pathological gain, a denormal cascade - fall back to the
        # last good output rather than propagating NaN into an actuator.
        if not m.isfinite(u):
            self._set_error(Status.ERR_NAN_INPUT)
            self.integrator = 0.0
            self.d_state = 0.0
            u = self.output if m.isfinite(self.output) else 0.0

        self.output = u

        self._fill_status(meas, y, sp, e, p_term, self.d_state, ff, u_raw, dt)
        return self.output, rc

    def _fill_status(self, meas, y, sp, e, p_term, d_term, ff, unsat, dt):
        s = self.status
        s.setpoint_raw = self.setpoint_target
        s.setpoint_shaped = sp
        s.measurement_raw = meas
        s.measurement_filtered = y
        s.error = e
        s.p_term = p_term
        s.i_term = self.integrator
        s.d_term = d_term
        s.ff_term = ff
        s.output_unsat = self.output if self.mode == Mode.MANUAL else unsat
        s.output = self.output
        s.dt_used = dt
        s.kp_active = self.kp
        s.ki_active = self.ki
        s.kd_active = self.kd
        s.update_count += 1
        if (self.flags & FLAG_SATURATED) != 0:
            s.saturation_count += 1
        s.flags = self.flags
        s.last_error = self.last_error

        if (self.features & FEAT_TELEMETRY) != 0 and self.telemetry is not None:
            self.telemetry.push(s)

    # ------------------------------------------------------------------
    # Level 1 - basic API
    # ------------------------------------------------------------------

    def update(self, measurement):
        """One cycle at the nominal sample time. The five-line-API entry."""
        out, _ = self._run(measurement, self.dt_nominal, None)
        return out

    def update_dt(self, measurement, dt):
        """One cycle with a measured dt. Use this when the loop jitters."""
        out, _ = self._run(measurement, dt, None)
        return out

    def update_ex(self, inp):
        """Full-control update. Returns (output, status).

        NaN fields in `inp` mean "keep the current handle state".
        """
        if inp is None:
            return 0.0, Status.ERR_NULL
        meas = inp.measurement if m.isfinite(inp.measurement) else _NAN
        dt = inp.dt if m.isfinite(inp.dt) and inp.dt > 0.0 else self.dt_nominal
        return self._run(meas, dt, inp)

    def update_fast(self, measurement):
        """Minimal-overhead update.

        Executes exactly: P with beta, backward-Euler I, filtered D on
        measurement, sum, clamp, integrator clamp. It deliberately IGNORES the
        shaper, safety, gain scheduling, feedforward, input filter,
        diagnostics and mode handling - and does not test for them. If any
        ignored feature is enabled the result silently differs from update();
        update_fast_is_safe() lets you assert against that in development.
        """
        if not self.initialised:
            return 0.0

        dsign = float(self.dir_sign)
        e = dsign * (self.setpoint - measurement)
        p = self.kp * dsign * ((self.beta * self.setpoint) - measurement)

        x = dsign * measurement
        self.d_state = (self.c_da * self.d_state) - (self.c_db * (x - self.d_prev_in))
        self.d_prev_in = x

        self.integrator += self.c_i * e
        self.integrator = m.clamp(self.integrator, self.i_min, self.i_max)

        u = m.clamp(p + self.integrator + self.d_state,
                    self.out_min, self.out_max)
        self.output = u
        return u

    def update_fast_is_safe(self):
        """True when update_fast() would produce the same output as update().

        Output limits must be in force because the fast path clamps
        unconditionally. Explicit INTEGRAL_LIMIT is not required: i_min/i_max
        always hold the effective bounds, inherited from the output limits when
        the user did not set their own. DIAGNOSTICS is excluded from the
        advanced mask because the fast path simply does not fill the snapshot -
        it produces the same OUTPUT, which is what "safe" means here.
        """
        if not self.initialised:
            return False
        return ((self.features & FEAT_ADVANCED_MASK & ~FEAT_DIAGNOSTICS) == 0
                and (self.features & FEAT_OUTPUT_LIMIT) != 0
                and (self.features & FEAT_INTEGRAL) != 0
                and self.aw_mode == AntiWindup.CLAMP
                and self.integ_method == IntegrationMethod.BACKWARD_EULER
                and self.d_mode == DerivativeMode.ON_MEASUREMENT
                and self.mode == Mode.AUTOMATIC
                and self.i_separation <= 0.0
                and self.i_deadband <= 0.0)

    def reset(self):
        """Clear all dynamic state, keeping the configuration."""
        if not self.initialised:
            return Status.ERR_NOT_INIT
        self.integrator = 0.0
        self.d_state = 0.0
        self.d_prev_in = 0.0
        self.e_prev = 0.0
        self.output = 0.0
        self.flags = 0
        self.last_error = int(Status.OK)
        self.sp_velocity = 0.0
        self.setpoint = self.setpoint_target
        self.in_lpf.reset()
        self.fault_count = 0
        self.meas_prev = 0.0
        self.meas_prev_valid = False
        self.status = StatusSnapshot()
        return Status.OK

    def set_gains(self, kp, ki, kd):
        """Change gains without touching the integral TERM - bumpless."""
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not _gain_ok(kp) or not _gain_ok(ki) or not _gain_ok(kd):
            return Status.ERR_INVALID_GAIN
        self.kp, self.ki, self.kd = kp, ki, kd
        self._recompute(self.dt_last)
        return Status.OK

    def set_gains_rescale_integral(self, kp, ki, kd):
        """Change gains preserving integral(e) rather than the term.

        term_new = Ki_new * integral(e) = term_old * (Ki_new / Ki_old).
        This is the classic semantics; it DOES step the output when Ki changes.
        """
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not _gain_ok(kp) or not _gain_ok(ki) or not _gain_ok(kd):
            return Status.ERR_INVALID_GAIN
        old_ki = self.ki
        if old_ki > 0.0:
            self.integrator = self.integrator * (ki / old_ki)
        self.kp, self.ki, self.kd = kp, ki, kd
        self._recompute(self.dt_last)
        return Status.OK

    def set_kp(self, kp):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not _gain_ok(kp):
            return Status.ERR_INVALID_GAIN
        self.kp = kp
        self._recompute(self.dt_last)
        return Status.OK

    def set_ki(self, ki):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not _gain_ok(ki):
            return Status.ERR_INVALID_GAIN
        self.ki = ki
        self._recompute(self.dt_last)
        return Status.OK

    def set_kd(self, kd):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not _gain_ok(kd):
            return Status.ERR_INVALID_GAIN
        self.kd = kd
        self._recompute(self.dt_last)
        return Status.OK

    def get_gains(self):
        """Returns (status, kp, ki, kd). Values are 0 on failure, never junk."""
        if not self.initialised:
            return Status.ERR_NOT_INIT, 0.0, 0.0, 0.0
        return Status.OK, self.kp, self.ki, self.kd

    def set_setpoint(self, setpoint):
        """Command a new setpoint. Goes through the shaper when enabled."""
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(setpoint):
            return Status.ERR_INVALID_PARAM
        self.setpoint_target = setpoint
        if (self.features & FEAT_SP_SHAPER) == 0:
            self.setpoint = setpoint
        return Status.OK

    def set_setpoint_immediate(self, sp):
        """Bypass the shaper: both target and effective setpoint jump."""
        self.setpoint_target = sp
        self.setpoint = sp

    def get_setpoint(self):
        return self.setpoint

    def get_output(self):
        return self.output

    def get_manual_output(self):
        return self.manual_output

    # ------------------------------------------------------------------
    # Level 2 - intermediate API
    # ------------------------------------------------------------------

    def set_sample_time(self, dt):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(dt) or dt <= 0.0:
            return Status.ERR_INVALID_DT
        self.dt_nominal = dt
        self._recompute(dt)
        return Status.OK

    def get_sample_time(self):
        """Nominal sample time, or 0 when the controller is unusable.

        Validated rather than merely present: this getter is the only evidence
        a cascade has that a member loop was ever initialised.
        """
        return self.dt_nominal if self.initialised else 0.0

    def set_output_limits(self, lo, hi):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(lo) or not m.isfinite(hi) or lo >= hi:
            return Status.ERR_INVALID_LIMIT
        self.out_min = lo
        self.out_max = hi
        self.features |= FEAT_OUTPUT_LIMIT
        # Keep existing state consistent with the new envelope. Without
        # explicit integral limits the integrator inherits these.
        self.output = m.clamp(self.output, lo, hi)
        if (self.features & FEAT_INTEGRAL_LIMIT) == 0:
            self.i_min = lo
            self.i_max = hi
            self.integrator = m.clamp(self.integrator, lo, hi)
        return Status.OK

    def clear_output_limits(self):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        self.features &= ~FEAT_OUTPUT_LIMIT
        self.out_min = -PID_HUGE_F
        self.out_max = PID_HUGE_F
        # An inherited integral bound has nothing left to inherit from.
        if (self.features & FEAT_INTEGRAL_LIMIT) == 0:
            self.i_min = -PID_HUGE_F
            self.i_max = PID_HUGE_F
        return Status.OK

    def set_integral_limits(self, lo, hi):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(lo) or not m.isfinite(hi) or lo >= hi:
            return Status.ERR_INVALID_LIMIT
        self.i_min = lo
        self.i_max = hi
        self.features |= FEAT_INTEGRAL_LIMIT
        self.integrator = m.clamp(self.integrator, lo, hi)
        return Status.OK

    def set_anti_windup(self, mode, kt=0.0):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if mode > AntiWindup.TRACKING:
            return Status.ERR_INVALID_PARAM
        if not m.isfinite(kt) or kt < 0.0:
            return Status.ERR_INVALID_PARAM
        if (mode == AntiWindup.BACK_CALCULATION
                and (self.features & (FEAT_OUTPUT_LIMIT
                                      | FEAT_INTEGRAL_LIMIT)) == 0):
            return Status.ERR_INVALID_LIMIT
        self.aw_mode = int(mode)
        self.kt = kt
        self._recompute(self.dt_last)
        return Status.OK

    def set_derivative_mode(self, mode):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if mode > DerivativeMode.ON_WEIGHTED_ERROR:
            return Status.ERR_INVALID_PARAM
        self.d_mode = int(mode)
        # The derivative source changes meaning; re-prime on the next sample
        # to avoid differentiating across the discontinuity.
        self.d_prev_in = 0.0
        self.d_state = 0.0
        return Status.OK

    def set_derivative_filter(self, tf):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(tf) or tf < 0.0:
            return Status.ERR_INVALID_PARAM
        self.tf = tf
        if tf > 0.0:
            self.features |= FEAT_D_FILTER
        self._recompute(self.dt_last)
        return Status.OK

    def set_derivative_filter_n(self, n):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(n) or n <= 0.0:
            return Status.ERR_INVALID_PARAM
        self.n_filter = n
        self.tf = 0.0            # explicit tf no longer overrides N
        self.features |= FEAT_D_FILTER
        self._recompute(self.dt_last)
        return Status.OK

    def set_direction(self, direction):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        self.dir_sign = -1 if direction == Direction.REVERSE else 1
        self.d_prev_in = -self.d_prev_in   # keep the stored source consistent
        return Status.OK

    def set_mode(self, mode):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if mode > Mode.HOLD:
            return Status.ERR_INVALID_MODE
        if self.mode != Mode.MANUAL and mode == Mode.MANUAL:
            # Entering manual: start from where the controller already is.
            self.manual_output = self.output
        # Leaving manual needs no work: _run() back-solves the integrator on
        # every manual sample, so the automatic law already reproduces output.
        self.mode = int(mode)
        return Status.OK

    def get_mode(self):
        return Mode(self.mode)

    def set_manual_output(self, output):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(output):
            return Status.ERR_INVALID_PARAM
        self.manual_output = output
        return Status.OK

    def set_setpoint_ramp(self, rate_max, accel=0.0, decel=0.0):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if (not m.isfinite(rate_max) or rate_max < 0.0
                or not m.isfinite(accel) or accel < 0.0
                or not m.isfinite(decel) or decel < 0.0):
            return Status.ERR_INVALID_PARAM
        self.sp_rate_max = rate_max
        self.sp_accel = accel
        self.sp_decel = decel
        if rate_max > 0.0:
            self.features |= FEAT_SP_SHAPER
        else:
            self.features &= ~FEAT_SP_SHAPER
            self.sp_velocity = 0.0
        return Status.OK

    def set_output_slew_rate(self, slew_max):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(slew_max) or slew_max < 0.0:
            return Status.ERR_INVALID_PARAM
        self.out_slew_max = slew_max
        if slew_max > 0.0:
            self.features |= FEAT_OUT_SHAPER
        else:
            self.features &= ~FEAT_OUT_SHAPER
        return Status.OK

    def set_input_filter(self, tau):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(tau) or tau < 0.0:
            return Status.ERR_INVALID_PARAM
        self.in_lpf.tau = tau
        if tau > 0.0:
            self.features |= FEAT_INPUT_FILTER
        else:
            self.features &= ~FEAT_INPUT_FILTER
            self.in_lpf.primed = False
        self._recompute(self.dt_last)
        return Status.OK

    # ------------------------------------------------------------------
    # Level 3 - advanced API
    # ------------------------------------------------------------------

    def set_weights(self, beta, gamma):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if (not m.isfinite(beta) or not m.isfinite(gamma)
                or beta < 0.0 or beta > 2.0
                or gamma < 0.0 or gamma > 2.0):
            return Status.ERR_INVALID_PARAM
        self.beta = beta
        self.gamma = gamma
        return Status.OK

    def set_feedforward(self, ff):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(ff):
            return Status.ERR_INVALID_PARAM
        self.ff_value = ff
        self.features |= FEAT_FEEDFORWARD
        return Status.OK

    def set_feedforward_fn(self, fn, ctx=None, gain=1.0):
        """Install u_ff = gain * fn(setpoint, measurement, ctx)."""
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(gain):
            return Status.ERR_INVALID_PARAM
        self.ff_fn = fn
        self.ff_ctx = ctx
        self.ff_gain = gain if gain != 0.0 else 1.0
        if fn is not None:
            self.features |= FEAT_FEEDFORWARD
        return Status.OK

    def set_integral_separation(self, threshold):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(threshold) or threshold < 0.0:
            return Status.ERR_INVALID_PARAM
        self.i_separation = threshold
        return Status.OK

    def set_integral_deadband(self, db):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(db) or db < 0.0:
            return Status.ERR_INVALID_PARAM
        self.i_deadband = db
        return Status.OK

    def enable_integral(self, enable):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if enable:
            self.features |= FEAT_INTEGRAL
        else:
            self.features &= ~FEAT_INTEGRAL
        return Status.OK

    def set_integrator(self, value):
        """Preload the integral TERM, in output units."""
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(value):
            return Status.ERR_INVALID_PARAM
        self.integrator = m.clamp(value, self.i_min, self.i_max)
        return Status.OK

    def get_integrator(self):
        return self.integrator

    def set_tracking_input(self, u_track):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(u_track):
            return Status.ERR_INVALID_PARAM
        self.tracking_input = u_track
        return Status.OK

    def set_integration_method(self, method):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if method > IntegrationMethod.TRAPEZOIDAL:
            return Status.ERR_INVALID_PARAM
        self.integ_method = int(method)
        self._recompute(self.dt_last)
        return Status.OK

    def set_safety(self, sc):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if sc is None:
            return Status.ERR_NULL
        if sc.meas_max < sc.meas_min:
            return Status.ERR_INVALID_LIMIT
        self.meas_min = sc.meas_min
        self.meas_max = sc.meas_max
        self.meas_rate_max = sc.meas_rate_max
        self.failsafe_output = sc.failsafe_output
        self.fault_persist_n = 1 if sc.fault_persist_n == 0 else sc.fault_persist_n
        self.auto_recover = sc.auto_recover
        if sc.enabled:
            self.features |= FEAT_SAFETY
        else:
            self.features &= ~FEAT_SAFETY
        return Status.OK

    def set_fault_output(self, output):
        if not self.initialised:
            return Status.ERR_NOT_INIT
        if not m.isfinite(output):
            return Status.ERR_INVALID_PARAM
        self.failsafe_output = output
        return Status.OK

    def clear_fault(self):
        """Drop the latched fault. The next healthy sample resumes control."""
        if not self.initialised:
            return Status.ERR_NOT_INIT
        self.flags &= ~(FLAG_FAULT | FLAG_SENSOR_INVALID)
        self.fault_count = 0
        self.meas_prev_valid = False
        # Re-seed the integrator so control resumes from the fail-safe output
        # rather than from whatever the integrator held before the fault.
        # P/D/FF are not knowable from application context, so they enter as
        # zero here - see docs/24_port_comparison.md for why that is a weaker
        # guarantee than the auto-recovery path's deferred back-solve.
        self._back_solve(self.output, 0.0, 0.0, 0.0)
        return Status.OK

    def get_error(self):
        return self.status.error

    def get_last_error(self):
        """Read and clear the sticky error. Returns the code."""
        code = self.last_error
        self.last_error = int(Status.OK)
        return code

    def peek_last_error(self):
        """Read the sticky error without clearing it."""
        return self.last_error

    def clear_error(self):
        self.last_error = int(Status.OK)
        return Status.OK

    def get_status(self):
        """The per-cycle diagnostic snapshot, or None when uninitialised."""
        return self.status if self.initialised else None

    def get_flags(self):
        return self.flags

    def get_features(self):
        return self.features


def get_version():
    return PIDX_VERSION_STRING
