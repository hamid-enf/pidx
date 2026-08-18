classdef PID < handle
    % PIDX.PID  Production-grade PID controller. Port of src/pid.c.
    %
    % CONTROL LAW
    %   u = Kp*(beta*r - y) + Ki*integral(r - y)
    %       + (Kd*s/(1 + s*Tf))*(gamma*r - y) + u_ff
    %
    % The integral always acts on the UNWEIGHTED error (r - y). Weighting it
    % would leave a steady-state offset of (1 - beta)*r, which is not a
    % tuning choice but a bug.
    %
    % INTEGRATOR UNITS
    %   `integrator` holds Ki*integral(e) - the integral TERM in output
    %   units, not the raw integral of the error. So changing Ki at runtime
    %   does not step the output (gain scheduling and auto-tune application
    %   are bumpless for free), and integral limits share units with the
    %   output limits. setGainsRescaleIntegral() gives the classic semantics.
    %
    % DERIVATIVE
    %   Discretised as one filtered block, not a difference then a low-pass:
    %       D_k  = c_da*D_{k-1} - c_db*(x_k - x_{k-1})
    %       c_da = Tf/(Tf + dt),   c_db = Kd/(Tf + dt)
    %   The pole c_da lies in [0,1) for every Tf >= 0 and dt > 0, so the term
    %   can never diverge. The naive form has pole (1 - dt/Tf), which leaves
    %   the unit circle as soon as Tf < dt/2 - exactly when a user picks a
    %   light filter.
    %
    % A handle class, so a PID behaves like the C pointer it mirrors: pass it
    % to a function and the callee mutates the same controller.
    %
    % Tested under GNU Octave 9.4; it uses no toolbox functions, so it also
    % runs under MathWorks MATLAB, but the numbers in docs/24_port_comparison
    % were produced with Octave.

    properties (Access = public)
        initialised = false;

        kp = 0.0; ki = 0.0; kd = 0.0;

        % dt-dependent coefficients, rebuilt by recompute()
        c_i = 0.0; c_da = 0.0; c_db = 0.0; c_aw = 0.0;
        dt_last = 0.0; dt_nominal = 0.0; dt_min = 0.0; dt_max = 0.0;

        setpoint = 0.0; setpoint_target = 0.0; sp_velocity = 0.0;

        integrator = 0.0; d_state = 0.0; d_prev_in = 0.0; e_prev = 0.0;
        output = 0.0; manual_output = 0.0; tracking_input = 0.0;

        out_min = -1.0e30; out_max = 1.0e30;
        i_min   = -1.0e30; i_max   = 1.0e30;

        tf = 0.0; n_filter = 0.0; kt = 0.0;
        i_separation = 0.0; i_deadband = 0.0;
        beta = 1.0; gamma = 0.0;

        ff_fn = []; ff_value = 0.0; ff_gain = 1.0;

        sp_rate_max = 0.0; sp_accel = 0.0; sp_decel = 0.0;
        out_slew_max = 0.0;

        % input low-pass state
        lpf_a = 0.0; lpf_state = 0.0; lpf_tau = 0.0; lpf_primed = false;

        meas_min = 0.0; meas_max = 0.0; meas_rate_max = 0.0;
        failsafe_output = 0.0; fault_persist_n = 1; auto_recover = false;
        fault_count = 0; meas_prev = 0.0; meas_prev_valid = false;

        sched = []; sched_var_ext = 0.0;

        dir_sign = 1;
        mode = 1;            % Const.MODE_AUTOMATIC
        integ_method = 0;    % BACKWARD_EULER
        aw_mode = 1;         % AW_CLAMP
        d_mode = 0;          % DERIV_ON_MEASUREMENT

        features = 0;
        flags = 0;
        last_error = 0;
        status;
    end

    methods
        function o = PID(cfg)
            o.zeroState();
            if nargin > 0 && ~isempty(cfg)
                rc = o.init(cfg);
                if rc ~= pidx.Const.OK
                    error('pidx:PID:initFailed', ...
                          'PID init failed: %d', rc);
                end
            end
        end

        % ==============================================================
        % Lifecycle
        % ==============================================================

        function zeroState(o)
            % Put every field in a defined state, as PID_Init's memset does.
            K = pidx.Const;
            o.initialised = false;
            o.kp = 0; o.ki = 0; o.kd = 0;
            o.c_i = 0; o.c_da = 0; o.c_db = 0; o.c_aw = 0;
            o.dt_last = 0; o.dt_nominal = 0; o.dt_min = 0; o.dt_max = 0;
            o.setpoint = 0; o.setpoint_target = 0; o.sp_velocity = 0;
            o.integrator = 0; o.d_state = 0; o.d_prev_in = 0; o.e_prev = 0;
            o.output = 0; o.manual_output = 0; o.tracking_input = 0;
            o.out_min = -K.HUGE_F; o.out_max = K.HUGE_F;
            o.i_min = -K.HUGE_F;   o.i_max = K.HUGE_F;
            o.tf = 0; o.n_filter = 0; o.kt = 0;
            o.i_separation = 0; o.i_deadband = 0;
            o.beta = 1; o.gamma = 0;
            o.ff_fn = []; o.ff_value = 0; o.ff_gain = 1;
            o.sp_rate_max = 0; o.sp_accel = 0; o.sp_decel = 0;
            o.out_slew_max = 0;
            o.lpf_a = 0; o.lpf_state = 0; o.lpf_tau = 0; o.lpf_primed = false;
            o.meas_min = 0; o.meas_max = 0; o.meas_rate_max = 0;
            o.failsafe_output = 0; o.fault_persist_n = 1;
            o.auto_recover = false;
            o.fault_count = 0; o.meas_prev = 0; o.meas_prev_valid = false;
            o.sched = []; o.sched_var_ext = 0;
            o.dir_sign = 1;
            o.mode = K.MODE_AUTOMATIC;
            o.integ_method = K.INTEGRATION_BACKWARD_EULER;
            o.aw_mode = K.AW_CLAMP;
            o.d_mode = K.DERIV_ON_MEASUREMENT;
            o.features = 0; o.flags = 0; o.last_error = K.OK;
            o.status = pidx.PID.emptyStatus();
        end

        function rc = init(o, cfg)
            % Validate cfg and configure. Validation happens BEFORE any field
            % is written, so a rejected config leaves a working controller
            % untouched.
            K = pidx.Const;

            if isempty(cfg)
                rc = K.ERR_NULL; return;
            end
            if cfg.abi_version ~= K.CONFIG_ABI_VERSION
                rc = K.ERR_INVALID_CONFIG; return;
            end
            if ~pidx.PID.gainOk(cfg.core.kp) || ...
               ~pidx.PID.gainOk(cfg.core.ki) || ...
               ~pidx.PID.gainOk(cfg.core.kd)
                rc = K.ERR_INVALID_GAIN; return;
            end
            if ~isfinite(cfg.core.sample_time) || cfg.core.sample_time <= 0
                rc = K.ERR_INVALID_DT; return;
            end
            if cfg.limits.use_output_limits && ...
               (~isfinite(cfg.limits.output_min) || ...
                ~isfinite(cfg.limits.output_max) || ...
                cfg.limits.output_min >= cfg.limits.output_max)
                rc = K.ERR_INVALID_LIMIT; return;
            end
            if cfg.limits.use_integral_limits && ...
               cfg.limits.integral_min >= cfg.limits.integral_max
                rc = K.ERR_INVALID_LIMIT; return;
            end
            if cfg.integral.mode == K.AW_BACK_CALCULATION && ...
               ~cfg.limits.use_output_limits && ...
               ~cfg.limits.use_integral_limits
                % u_sat would always equal u_raw, so the correction term is
                % identically zero: the user forgot to set limits.
                rc = K.ERR_INVALID_LIMIT; return;
            end
            if ~isfinite(cfg.weight.beta) || ~isfinite(cfg.weight.gamma) || ...
               cfg.weight.beta < 0 || cfg.weight.beta > 2 || ...
               cfg.weight.gamma < 0 || cfg.weight.gamma > 2
                rc = K.ERR_INVALID_CONFIG; return;
            end

            o.zeroState();

            o.kp = cfg.core.kp; o.ki = cfg.core.ki; o.kd = cfg.core.kd;
            o.dt_nominal = cfg.core.sample_time;
            o.dt_min = cfg.limits.dt_min;
            o.dt_max = cfg.limits.dt_max;
            if cfg.core.direction == K.REVERSE
                o.dir_sign = -1;
            else
                o.dir_sign = 1;
            end
            o.mode = cfg.core.mode;
            o.integ_method = cfg.core.integration;

            o.out_min = cfg.limits.output_min;
            o.out_max = cfg.limits.output_max;

            % i_min/i_max always hold the EFFECTIVE bounds, resolved once
            % here. Without explicit integral limits they inherit the output
            % limits: an integrator that can demand more than the actuator
            % can deliver is just windup waiting to happen.
            if cfg.limits.use_integral_limits
                o.i_min = cfg.limits.integral_min;
                o.i_max = cfg.limits.integral_max;
            elseif cfg.limits.use_output_limits
                o.i_min = cfg.limits.output_min;
                o.i_max = cfg.limits.output_max;
            else
                o.i_min = -K.HUGE_F;
                o.i_max = K.HUGE_F;
            end

            o.d_mode = cfg.filter.derivative_mode;
            o.tf = cfg.filter.tf;
            o.n_filter = cfg.filter.n_filter;

            o.aw_mode = cfg.integral.mode;
            o.kt = cfg.integral.kt;
            o.i_separation = cfg.integral.separation_threshold;
            o.i_deadband = cfg.integral.deadband;

            o.beta = cfg.weight.beta;
            o.gamma = cfg.weight.gamma;

            feat = bitor(K.FEAT_DERIVATIVE, K.FEAT_D_FILTER);
            if cfg.integral.enabled
                feat = bitor(feat, K.FEAT_INTEGRAL);
            end
            if cfg.limits.use_output_limits
                feat = bitor(feat, K.FEAT_OUTPUT_LIMIT);
            end
            if cfg.limits.use_integral_limits
                feat = bitor(feat, K.FEAT_INTEGRAL_LIMIT);
            end

            if cfg.feedforward.enabled
                feat = bitor(feat, K.FEAT_FEEDFORWARD);
            end
            o.ff_fn = cfg.feedforward.fn;
            o.ff_value = cfg.feedforward.value;
            if cfg.feedforward.gain ~= 0
                o.ff_gain = cfg.feedforward.gain;
            else
                o.ff_gain = 1.0;
            end

            o.sp_rate_max = cfg.shaper.sp_rate_max;
            o.sp_accel = cfg.shaper.sp_accel;
            o.sp_decel = cfg.shaper.sp_decel;
            o.out_slew_max = cfg.shaper.out_slew_max;
            if o.sp_rate_max > 0
                feat = bitor(feat, K.FEAT_SP_SHAPER);
            end
            if o.out_slew_max > 0
                feat = bitor(feat, K.FEAT_OUT_SHAPER);
            end

            o.lpf_tau = cfg.filter.input_lpf_tau;
            if o.lpf_tau > 0
                feat = bitor(feat, K.FEAT_INPUT_FILTER);
            end

            o.meas_min = cfg.safety.meas_min;
            o.meas_max = cfg.safety.meas_max;
            o.meas_rate_max = cfg.safety.meas_rate_max;
            o.failsafe_output = cfg.safety.failsafe_output;
            if cfg.safety.fault_persist_n == 0
                o.fault_persist_n = 1;
            else
                o.fault_persist_n = cfg.safety.fault_persist_n;
            end
            o.auto_recover = cfg.safety.auto_recover;
            if cfg.safety.enabled
                feat = bitor(feat, K.FEAT_SAFETY);
            end

            feat = bitor(feat, K.FEAT_DIAGNOSTICS);

            o.features = feat;
            o.tracking_input = 0;
            o.last_error = K.OK;
            o.initialised = true;

            o.recompute(o.dt_nominal);
            rc = K.OK;
        end

        function rc = initDefault(o)
            rc = o.init(pidx.config());
        end

        function rc = deinit(o)
            o.zeroState();
            rc = pidx.Const.OK;
        end

        % ==============================================================
        % Internal helpers
        % ==============================================================

        function setError(o, code)
            % Record an error without ever overwriting it with success.
            if code ~= pidx.Const.OK
                o.last_error = code;
            end
        end

        function tf_eff = effectiveTf(o)
            % An explicit tf always wins. Otherwise Tf = Td/N = Kd/(N*Kp).
            % That needs a non-zero Kp; with Kp == 0 (a pure ID controller -
            % unusual but legal) the ratio is undefined and we fall back to an
            % unfiltered derivative rather than inventing a value.
            tf_eff = 0.0;
            if bitand(o.features, pidx.Const.FEAT_D_FILTER) ~= 0
                if o.tf > 0
                    tf_eff = o.tf;
                elseif o.n_filter > 0 && o.kp > 0 && o.kd > 0
                    tf_eff = o.kd / (o.n_filter * o.kp);
                end
            end
        end

        function kt_eff = effectiveKt(o)
            % Astrom & Hagglund: Tt = sqrt(Ti*Td) with derivative action,
            % Tt = Ti without. In parallel gains Ti = Kp/Ki, Td = Kd/Kp, so
            % Ti*Td = Kd/Ki, giving Kt = sqrt(Ki/Kd) and Kt = Ki/Kp.
            kt_eff = o.kt;
            if kt_eff <= 0
                if o.ki > 0 && o.kd > 0
                    kt_eff = sqrt(o.ki / o.kd);
                elseif o.ki > 0 && o.kp > 0
                    kt_eff = o.ki / o.kp;
                else
                    kt_eff = 0.0;
                end
            end
        end

        function recompute(o, dt)
            % Rebuild every dt-dependent coefficient. The only place that
            % divides on behalf of the control law, which keeps the update
            % path division-free.
            tf_eff = o.effectiveTf();
            den = tf_eff + dt;

            if o.integ_method == pidx.Const.INTEGRATION_TRAPEZOIDAL
                o.c_i = o.ki * dt * 0.5;
            else
                o.c_i = o.ki * dt;
            end

            % den >= dt > 0, so this division is always safe.
            o.c_da = tf_eff / den;
            o.c_db = o.kd / den;

            o.c_aw = o.effectiveKt() * dt;

            % input LPF pole, exact backward-Euler discretisation
            if o.lpf_tau > 0
                o.lpf_a = o.lpf_tau / (o.lpf_tau + dt);
            else
                o.lpf_a = 0.0;
            end

            o.dt_last = dt;
        end

        function ok = backSolve(o, desired, p, d, ff)
            % Force the integrator so P + I + D + FF reproduces `desired`.
            % One operation implementing bumpless manual->auto transfer,
            % bumpless fault recovery and integrator preloading.
            %
            % When the clamp bites the transfer CANNOT be bumpless: the
            % requested output is not reachable from the current P/D/FF with a
            % legal integrator. Flagged rather than silently accepted, because
            % a "bumpless transfer" that quietly steps the actuator is worse
            % than one that says it could not.
            K = pidx.Const;
            want = desired - p - d - ff;
            got = pidx.PID.clamp(want, o.i_min, o.i_max);
            o.integrator = got;

            if got ~= want
                o.flags = bitor(o.flags, K.FLAG_INTEGRAL_LIMITED);
                % Also sticky: the flag is rebuilt every cycle, so a caller
                % who switches mode and reads flags after the next update
                % would never see it.
                o.setError(K.ERR_INVALID_LIMIT);
                ok = false;
            else
                ok = true;
            end
        end

        function shapeSetpoint(o, dt)
            K = pidx.Const;
            [o.setpoint, o.sp_velocity, moving] = pidx.profileStep( ...
                o.setpoint, o.sp_velocity, o.setpoint_target, ...
                o.sp_rate_max, o.sp_accel, o.sp_decel, dt);
            if moving
                o.flags = bitor(o.flags, K.FLAG_SP_RAMPING);
            else
                o.flags = bitand(o.flags, pidx.PID.notBits(K.FLAG_SP_RAMPING));
            end
        end

        function rc = checkSensor(o, y, dt)
            % Range then slew plausibility.
            K = pidx.Const;
            rc = K.OK;
            if o.meas_max > o.meas_min && (y < o.meas_min || y > o.meas_max)
                rc = K.ERR_SENSOR_RANGE;
            elseif o.meas_rate_max > 0 && o.meas_prev_valid
                if abs(y - o.meas_prev) > (o.meas_rate_max * dt)
                    rc = K.ERR_SENSOR_RATE;
                end
            end
        end

        % ==============================================================
        % The update path
        % ==============================================================

        function [u, rc] = run(o, meas, dt, inp)
            % One control cycle. Stage order matches src/pid.c exactly and
            % must not be "tidied": anti-windup in stage 13 runs in the SAME
            % sample as the saturation it corrects.
            K = pidx.Const;
            y = meas;
            ff = 0.0;
            i_pre = 0.0;
            i_stepped = false;
            recover_to = 0.0;
            recovering = false;
            rc = K.OK;

            % -------- Stage 0: guards ----------------------------------
            if ~o.initialised
                u = 0.0; rc = K.ERR_NOT_INIT; return;
            end

            if ~isfinite(y)
                if isnan(y)
                    rc = K.ERR_NAN_INPUT;
                else
                    rc = K.ERR_INF_INPUT;
                end
                o.setError(rc);
                o.flags = bitor(o.flags, K.FLAG_SENSOR_INVALID);
                if bitand(o.features, K.FEAT_SAFETY) ~= 0
                    o.fault_count = o.fault_count + 1;
                    if o.fault_count >= o.fault_persist_n
                        o.flags = bitor(o.flags, K.FLAG_FAULT);
                        o.output = o.failsafe_output;
                    end
                end
                % Hold the previous output: one bad sample must not command
                % a jump.
                u = o.output; return;
            end

            % Transient flags are rebuilt every cycle; FAULT is latched.
            o.flags = bitand(o.flags, ...
                bitor(bitor(K.FLAG_FAULT, K.FLAG_TUNING), K.FLAG_SP_RAMPING));

            % -------- Stage 1: timing ----------------------------------
            if dt <= 0
                rc = K.ERR_INVALID_DT;
                o.setError(rc);
                o.flags = bitor(o.flags, K.FLAG_DT_VIOLATION);
                dt = o.dt_nominal;
            elseif (o.dt_min > 0 && dt < o.dt_min) || ...
                   (o.dt_max > 0 && dt > o.dt_max)
                rc = K.ERR_INVALID_DT;
                o.setError(rc);
                o.flags = bitor(o.flags, K.FLAG_DT_VIOLATION);
                lo = dt; hi = dt;
                if o.dt_min > 0, lo = o.dt_min; end
                if o.dt_max > 0, hi = o.dt_max; end
                dt = pidx.PID.clamp(dt, lo, hi);
            end

            if dt ~= o.dt_last
                o.recompute(dt);
            end

            % -------- Stage 2: sensor validation -----------------------
            if bitand(o.features, K.FEAT_SAFETY) ~= 0
                sc = o.checkSensor(y, dt);

                if sc ~= K.OK
                    o.setError(sc);
                    o.flags = bitor(o.flags, K.FLAG_SENSOR_INVALID);
                    o.fault_count = o.fault_count + 1;
                    if o.fault_count >= o.fault_persist_n
                        o.flags = bitor(o.flags, K.FLAG_FAULT);
                    end
                elseif o.fault_count > 0
                    if o.auto_recover
                        o.fault_count = 0;
                        if bitand(o.flags, K.FLAG_FAULT) ~= 0
                            % Bumpless re-entry. The back-solve is DEFERRED
                            % to stage 10 because P, D and FF for this sample
                            % do not exist yet: solving now with zeros sets I
                            % to the failsafe output and the real P term is
                            % then added on top, which is a measurable step
                            % exactly where this code prevents one.
                            o.flags = bitand(o.flags, pidx.PID.notBits(K.FLAG_FAULT));
                            o.d_prev_in = y;
                            recover_to = o.output;
                            recovering = true;
                        end
                    else
                        o.fault_count = 0;   % sample fine; latch stays put
                    end
                end

                o.meas_prev = y;
                o.meas_prev_valid = true;

                if bitand(o.flags, K.FLAG_FAULT) ~= 0
                    o.output = o.failsafe_output;
                    if rc == K.OK
                        rc = K.ERR_SENSOR_RANGE;
                    end
                    u = o.output; return;
                end
            end

            % -------- Stage 3: input filter ----------------------------
            if bitand(o.features, K.FEAT_INPUT_FILTER) ~= 0
                if ~o.lpf_primed
                    o.lpf_state = y;
                    o.lpf_primed = true;
                else
                    o.lpf_state = (o.lpf_a * o.lpf_state) + ...
                                  ((1.0 - o.lpf_a) * y);
                end
                y = o.lpf_state;
            end

            % -------- Stage 4: setpoint --------------------------------
            if ~isempty(inp) && isfinite(inp.setpoint)
                o.setpoint_target = inp.setpoint;
            end

            if bitand(o.features, K.FEAT_SP_SHAPER) ~= 0
                o.shapeSetpoint(dt);
            else
                o.setpoint = o.setpoint_target;
            end
            sp = o.setpoint;

            % -------- Stage 5: gain scheduling -------------------------
            if bitand(o.features, K.FEAT_GAIN_SCHED) ~= 0 && ~isempty(o.sched)
                if ~isempty(inp) && isfinite(inp.schedule_var)
                    var = inp.schedule_var;
                else
                    switch o.sched.source
                        case K.SCHED_SRC_SETPOINT,    var = sp;
                        case K.SCHED_SRC_MEASUREMENT, var = y;
                        case K.SCHED_SRC_ERROR,       var = sp - y;
                        case K.SCHED_SRC_ABS_ERROR,   var = abs(sp - y);
                        case K.SCHED_SRC_OUTPUT,      var = o.output;
                        otherwise,                    var = o.sched_var_ext;
                    end
                end

                [ok, nkp, nki, nkd] = o.sched.evaluate(var);
                if ok == K.OK
                    if nkp ~= o.kp || nki ~= o.ki || nkd ~= o.kd
                        o.kp = nkp; o.ki = nki; o.kd = nkd;
                        o.recompute(dt);
                    end
                end
            end

            % -------- Stage 6: error and P -----------------------------
            dsign = o.dir_sign;
            e = dsign * (sp - y);
            p_term = o.kp * dsign * ((o.beta * sp) - y);

            % -------- Stage 7: derivative ------------------------------
            % All three modes are the same expression with a different
            % setpoint weight, so there is one code path instead of three:
            %   x = dir*(y - gamma_eff*r), D = -Kd/(Tf+dt)*dx, filtered.
            switch o.d_mode
                case K.DERIV_ON_ERROR,          gamma_eff = 1.0;
                case K.DERIV_ON_WEIGHTED_ERROR, gamma_eff = o.gamma;
                otherwise,                      gamma_eff = 0.0;
            end
            d_src = dsign * (y - (gamma_eff * sp));

            if bitand(o.features, K.FEAT_DERIVATIVE) ~= 0
                o.d_state = (o.c_da * o.d_state) - ...
                            (o.c_db * (d_src - o.d_prev_in));
            else
                o.d_state = 0.0;
            end
            o.d_prev_in = d_src;

            % -------- Stage 8: feedforward -----------------------------
            if bitand(o.features, K.FEAT_FEEDFORWARD) ~= 0
                if ~isempty(inp) && isfinite(inp.feedforward)
                    ff = inp.feedforward * o.ff_gain;
                elseif ~isempty(o.ff_fn)
                    ff = o.ff_fn(sp, y) * o.ff_gain;
                else
                    ff = o.ff_value * o.ff_gain;
                end
                if ~isfinite(ff)
                    % A misbehaving user callback must not poison the loop.
                    ff = 0.0;
                    o.setError(K.ERR_NAN_INPUT);
                end
            end

            i_lo = o.i_min; i_hi = o.i_max;

            % Deferred from stage 2: now that p_term, d_state and ff exist,
            % the integrator can be solved so the sum reproduces the fail-safe
            % output exactly.
            if recovering
                o.backSolve(recover_to, p_term, o.d_state, ff);
            end

            % -------- Stage 9: manual / hold ---------------------------
            if o.mode == K.MODE_MANUAL
                u = o.manual_output;
                if bitand(o.features, K.FEAT_OUTPUT_LIMIT) ~= 0
                    u = pidx.PID.clamp(u, o.out_min, o.out_max);
                end
                % Track continuously so a switch to AUTOMATIC at any instant
                % is bumpless without a special case in setMode().
                o.backSolve(u, p_term, o.d_state, ff);
                o.output = u;
                o.flags = bitor(o.flags, K.FLAG_MANUAL);
                o.e_prev = e;
                o.fillStatus(meas, y, sp, e, p_term, o.d_state, ff, ...
                             o.output, dt);
                return;
            end

            % -------- Stage 10: integral -------------------------------
            integrate = bitand(o.features, K.FEAT_INTEGRAL) ~= 0 && ...
                        o.mode ~= K.MODE_HOLD;

            if integrate
                ae = abs(e);
                if o.i_separation > 0 && ae > o.i_separation
                    % Integral separation: during a large excursion the
                    % integrator would charge far beyond what the steady state
                    % needs, guaranteeing overshoot. P and D handle the
                    % transient; I re-engages near target.
                    integrate = false;
                elseif o.i_deadband > 0 && ae < o.i_deadband
                    % Deadband: stop hunting against a quantised actuator.
                    integrate = false;
                end
                % Conditional integration is NOT decided here: admissibility
                % depends on whether the output saturates, known only after
                % the sum in stage 11. The decision is made, and undone if
                % necessary, in stage 13.
            end

            if integrate
                i_pre = o.integrator;
                if o.integ_method == K.INTEGRATION_TRAPEZOIDAL
                    o.integrator = o.integrator + o.c_i * (e + o.e_prev);
                else
                    o.integrator = o.integrator + o.c_i * e;
                end
                i_stepped = true;
                o.flags = bitor(o.flags, K.FLAG_INTEGRAL_ACTIVE);
            end

            if o.aw_mode == K.AW_CLAMP
                clamped = pidx.PID.clamp(o.integrator, i_lo, i_hi);
                if clamped ~= o.integrator
                    o.integrator = clamped;
                    o.flags = bitor(o.flags, K.FLAG_INTEGRAL_LIMITED);
                end
            end

            o.e_prev = e;

            % -------- Stage 11: sum ------------------------------------
            u_raw = p_term + o.integrator + o.d_state + ff;

            % -------- Stage 12: output saturation ----------------------
            u = u_raw;
            if bitand(o.features, K.FEAT_OUTPUT_LIMIT) ~= 0
                if u > o.out_max
                    u = o.out_max;
                    o.flags = bitor(o.flags, K.FLAG_SATURATED_HIGH);
                elseif u < o.out_min
                    u = o.out_min;
                    o.flags = bitor(o.flags, K.FLAG_SATURATED_LOW);
                end
            end

            % -------- Stage 13: back-calculation / tracking ------------
            % Applied in the SAME sample as the saturation it corrects.
            if o.mode ~= K.MODE_HOLD
                if o.aw_mode == K.AW_BACK_CALCULATION
                    if u ~= u_raw
                        o.integrator = o.integrator + o.c_aw * (u - u_raw);
                        o.integrator = pidx.PID.clamp(o.integrator, i_lo, i_hi);
                    end

                elseif o.aw_mode == K.AW_CONDITIONAL
                    % Conditional integration in the Astrom sense: an
                    % increment is admissible unless the output saturates AND
                    % the error would drive it further past the same limit.
                    % The test uses u_raw - the unsaturated sum - because that
                    % says how far past the limit the controller is asking to
                    % go. The increment is UNDONE rather than merely skipped,
                    % so the decision uses this sample's saturation state; a
                    % one-cycle-late test is the classic way this strategy
                    % quietly degrades into no protection at all.
                    if i_stepped && ((u_raw > o.out_max && e > 0) || ...
                                     (u_raw < o.out_min && e < 0))
                        o.integrator = i_pre;
                        o.flags = bitand(o.flags, ...
                                         pidx.PID.notBits(K.FLAG_INTEGRAL_ACTIVE));
                        o.flags = bitor(o.flags, K.FLAG_INTEGRAL_LIMITED);

                        % Recompute: removing the increment may pull the
                        % output back inside the limits, and holding it at the
                        % limit anyway would throw away authority the
                        % controller actually has.
                        u_raw = p_term + o.integrator + o.d_state + ff;
                        u = u_raw;
                        o.flags = bitand(o.flags, pidx.PID.notBits(K.FLAG_SATURATED));
                        if bitand(o.features, K.FEAT_OUTPUT_LIMIT) ~= 0
                            if u > o.out_max
                                u = o.out_max;
                                o.flags = bitor(o.flags, K.FLAG_SATURATED_HIGH);
                            elseif u < o.out_min
                                u = o.out_min;
                                o.flags = bitor(o.flags, K.FLAG_SATURATED_LOW);
                            end
                        end
                    end

                elseif o.aw_mode == K.AW_TRACKING
                    track = o.tracking_input;
                    if ~isempty(inp) && isfinite(inp.tracking)
                        track = inp.tracking;
                    end
                    if isfinite(track)
                        o.integrator = o.integrator + o.c_aw * (track - u_raw);
                        o.integrator = pidx.PID.clamp(o.integrator, i_lo, i_hi);
                    end
                end
            end

            % -------- Stage 14: output slew ----------------------------
            if bitand(o.features, K.FEAT_OUT_SHAPER) ~= 0 && o.out_slew_max > 0
                max_step = o.out_slew_max * dt;
                delta = u - o.output;
                if delta > max_step
                    u = o.output + max_step;
                    o.flags = bitor(o.flags, K.FLAG_OUTPUT_SLEWING);
                elseif delta < -max_step
                    u = o.output - max_step;
                    o.flags = bitor(o.flags, K.FLAG_OUTPUT_SLEWING);
                end
            end

            % Final numeric guard: fall back to the last good output rather
            % than propagating NaN into an actuator.
            if ~isfinite(u)
                o.setError(K.ERR_NAN_INPUT);
                o.integrator = 0.0;
                o.d_state = 0.0;
                if isfinite(o.output)
                    u = o.output;
                else
                    u = 0.0;
                end
            end

            o.output = u;
            o.fillStatus(meas, y, sp, e, p_term, o.d_state, ff, u_raw, dt);
        end

        function fillStatus(o, meas, y, sp, e, p_term, d_term, ff, unsat, dt)
            K = pidx.Const;
            s = o.status;
            s.setpoint_raw = o.setpoint_target;
            s.setpoint_shaped = sp;
            s.measurement_raw = meas;
            s.measurement_filtered = y;
            s.error = e;
            s.p_term = p_term;
            s.i_term = o.integrator;
            s.d_term = d_term;
            s.ff_term = ff;
            if o.mode == K.MODE_MANUAL
                s.output_unsat = o.output;
            else
                s.output_unsat = unsat;
            end
            s.output = o.output;
            s.dt_used = dt;
            s.kp_active = o.kp;
            s.ki_active = o.ki;
            s.kd_active = o.kd;
            s.update_count = s.update_count + 1;
            if bitand(o.flags, K.FLAG_SATURATED) ~= 0
                s.saturation_count = s.saturation_count + 1;
            end
            s.flags = o.flags;
            s.last_error = o.last_error;
            o.status = s;
        end

        % ==============================================================
        % Level 1 - basic API
        % ==============================================================

        function u = update(o, measurement)
            % One cycle at the nominal sample time. The five-line-API entry.
            [u, ~] = o.run(measurement, o.dt_nominal, []);
        end

        function u = updateDt(o, measurement, dt)
            % One cycle with a measured dt. Use this when the loop jitters.
            [u, ~] = o.run(measurement, dt, []);
        end

        function [u, rc] = updateEx(o, inp)
            % Full-control update. NaN fields mean "keep current state".
            if isempty(inp)
                u = 0.0; rc = pidx.Const.ERR_NULL; return;
            end
            if isfinite(inp.dt) && inp.dt > 0
                dt = inp.dt;
            else
                dt = o.dt_nominal;
            end
            [u, rc] = o.run(inp.measurement, dt, inp);
        end

        function u = updateFast(o, measurement)
            % Minimal-overhead update: P with beta, backward-Euler I, filtered
            % D on measurement, sum, clamp, integrator clamp. Deliberately
            % IGNORES the shaper, safety, gain scheduling, feedforward, input
            % filter, diagnostics and mode handling - and does not test for
            % them. Use updateFastIsSafe() to assert in development.
            if ~o.initialised
                u = 0.0; return;
            end
            dsign = o.dir_sign;
            e = dsign * (o.setpoint - measurement);
            p = o.kp * dsign * ((o.beta * o.setpoint) - measurement);

            x = dsign * measurement;
            o.d_state = (o.c_da * o.d_state) - (o.c_db * (x - o.d_prev_in));
            o.d_prev_in = x;

            o.integrator = o.integrator + o.c_i * e;
            o.integrator = pidx.PID.clamp(o.integrator, o.i_min, o.i_max);

            u = pidx.PID.clamp(p + o.integrator + o.d_state, ...
                               o.out_min, o.out_max);
            o.output = u;
        end

        function tf_ = updateFastIsSafe(o)
            % True when updateFast() would produce the same output as
            % update(). Output limits must be in force because the fast path
            % clamps unconditionally; explicit INTEGRAL_LIMIT is not required
            % since i_min/i_max always hold the effective bounds.
            K = pidx.Const;
            adv = bitand(K.FEAT_ADVANCED_MASK, pidx.PID.notBits(K.FEAT_DIAGNOSTICS));
            tf_ = o.initialised && ...
                  bitand(o.features, adv) == 0 && ...
                  bitand(o.features, K.FEAT_OUTPUT_LIMIT) ~= 0 && ...
                  bitand(o.features, K.FEAT_INTEGRAL) ~= 0 && ...
                  o.aw_mode == K.AW_CLAMP && ...
                  o.integ_method == K.INTEGRATION_BACKWARD_EULER && ...
                  o.d_mode == K.DERIV_ON_MEASUREMENT && ...
                  o.mode == K.MODE_AUTOMATIC && ...
                  o.i_separation <= 0 && o.i_deadband <= 0;
        end

        function rc = reset(o)
            % Clear all dynamic state, keeping the configuration.
            K = pidx.Const;
            if ~o.initialised
                rc = K.ERR_NOT_INIT; return;
            end
            o.integrator = 0; o.d_state = 0; o.d_prev_in = 0;
            o.e_prev = 0; o.output = 0;
            o.flags = 0; o.last_error = K.OK;
            o.sp_velocity = 0;
            o.setpoint = o.setpoint_target;
            o.lpf_state = 0; o.lpf_primed = false;
            o.fault_count = 0; o.meas_prev = 0; o.meas_prev_valid = false;
            o.status = pidx.PID.emptyStatus();
            rc = K.OK;
        end

        function rc = setGains(o, kp, ki, kd)
            % Change gains without touching the integral TERM - bumpless.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~pidx.PID.gainOk(kp) || ~pidx.PID.gainOk(ki) || ...
               ~pidx.PID.gainOk(kd)
                rc = K.ERR_INVALID_GAIN; return;
            end
            o.kp = kp; o.ki = ki; o.kd = kd;
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        function rc = setGainsRescaleIntegral(o, kp, ki, kd)
            % Change gains preserving integral(e) rather than the term:
            % term_new = term_old * (Ki_new / Ki_old). The classic semantics;
            % it DOES step the output when Ki changes.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~pidx.PID.gainOk(kp) || ~pidx.PID.gainOk(ki) || ...
               ~pidx.PID.gainOk(kd)
                rc = K.ERR_INVALID_GAIN; return;
            end
            old_ki = o.ki;
            if old_ki > 0
                o.integrator = o.integrator * (ki / old_ki);
            end
            o.kp = kp; o.ki = ki; o.kd = kd;
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        function rc = setKp(o, kp)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~pidx.PID.gainOk(kp), rc = K.ERR_INVALID_GAIN; return; end
            o.kp = kp; o.recompute(o.dt_last); rc = K.OK;
        end

        function rc = setKi(o, ki)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~pidx.PID.gainOk(ki), rc = K.ERR_INVALID_GAIN; return; end
            o.ki = ki; o.recompute(o.dt_last); rc = K.OK;
        end

        function rc = setKd(o, kd)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~pidx.PID.gainOk(kd), rc = K.ERR_INVALID_GAIN; return; end
            o.kd = kd; o.recompute(o.dt_last); rc = K.OK;
        end

        function [rc, kp, ki, kd] = getGains(o)
            % Values are 0 on failure, never junk.
            K = pidx.Const;
            if ~o.initialised
                rc = K.ERR_NOT_INIT; kp = 0; ki = 0; kd = 0; return;
            end
            rc = K.OK; kp = o.kp; ki = o.ki; kd = o.kd;
        end

        function rc = setSetpoint(o, sp)
            % Command a new setpoint. Goes through the shaper when enabled.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(sp), rc = K.ERR_INVALID_PARAM; return; end
            o.setpoint_target = sp;
            if bitand(o.features, K.FEAT_SP_SHAPER) == 0
                o.setpoint = sp;
            end
            rc = K.OK;
        end

        function setSetpointImmediate(o, sp)
            % Bypass the shaper: both target and effective setpoint jump.
            o.setpoint_target = sp;
            o.setpoint = sp;
        end

        function v = getSetpoint(o),      v = o.setpoint;       end
        function v = getOutput(o),        v = o.output;         end
        function v = getManualOutput(o),  v = o.manual_output;  end

        % ==============================================================
        % Level 2 - intermediate API
        % ==============================================================

        function rc = setSampleTime(o, dt)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(dt) || dt <= 0, rc = K.ERR_INVALID_DT; return; end
            o.dt_nominal = dt;
            o.recompute(dt);
            rc = K.OK;
        end

        function v = getSampleTime(o)
            % Nominal sample time, or 0 when unusable. Validated rather than
            % merely present: this getter is the only evidence a cascade has
            % that a member loop was ever initialised.
            if o.initialised
                v = o.dt_nominal;
            else
                v = 0.0;
            end
        end

        function rc = setOutputLimits(o, lo, hi)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(lo) || ~isfinite(hi) || lo >= hi
                rc = K.ERR_INVALID_LIMIT; return;
            end
            o.out_min = lo; o.out_max = hi;
            o.features = bitor(o.features, K.FEAT_OUTPUT_LIMIT);
            % Keep existing state consistent with the new envelope.
            o.output = pidx.PID.clamp(o.output, lo, hi);
            if bitand(o.features, K.FEAT_INTEGRAL_LIMIT) == 0
                o.i_min = lo; o.i_max = hi;
                o.integrator = pidx.PID.clamp(o.integrator, lo, hi);
            end
            rc = K.OK;
        end

        function rc = clearOutputLimits(o)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            o.features = bitand(o.features, pidx.PID.notBits(K.FEAT_OUTPUT_LIMIT));
            o.out_min = -K.HUGE_F; o.out_max = K.HUGE_F;
            % An inherited integral bound has nothing left to inherit from.
            if bitand(o.features, K.FEAT_INTEGRAL_LIMIT) == 0
                o.i_min = -K.HUGE_F; o.i_max = K.HUGE_F;
            end
            rc = K.OK;
        end

        function rc = setIntegralLimits(o, lo, hi)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(lo) || ~isfinite(hi) || lo >= hi
                rc = K.ERR_INVALID_LIMIT; return;
            end
            o.i_min = lo; o.i_max = hi;
            o.features = bitor(o.features, K.FEAT_INTEGRAL_LIMIT);
            o.integrator = pidx.PID.clamp(o.integrator, lo, hi);
            rc = K.OK;
        end

        function rc = setAntiWindup(o, mode, kt)
            K = pidx.Const;
            if nargin < 3, kt = 0.0; end
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if mode > K.AW_TRACKING, rc = K.ERR_INVALID_PARAM; return; end
            if ~isfinite(kt) || kt < 0, rc = K.ERR_INVALID_PARAM; return; end
            lim = bitor(K.FEAT_OUTPUT_LIMIT, K.FEAT_INTEGRAL_LIMIT);
            if mode == K.AW_BACK_CALCULATION && bitand(o.features, lim) == 0
                rc = K.ERR_INVALID_LIMIT; return;
            end
            o.aw_mode = mode; o.kt = kt;
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        function rc = setDerivativeMode(o, mode)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if mode > K.DERIV_ON_WEIGHTED_ERROR
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.d_mode = mode;
            % The derivative source changes meaning; re-prime on the next
            % sample to avoid differentiating across the discontinuity.
            o.d_prev_in = 0; o.d_state = 0;
            rc = K.OK;
        end

        function rc = setDerivativeFilter(o, tf_)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(tf_) || tf_ < 0, rc = K.ERR_INVALID_PARAM; return; end
            o.tf = tf_;
            if tf_ > 0
                o.features = bitor(o.features, K.FEAT_D_FILTER);
            end
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        function rc = setDerivativeFilterN(o, n)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(n) || n <= 0, rc = K.ERR_INVALID_PARAM; return; end
            o.n_filter = n;
            o.tf = 0;             % explicit tf no longer overrides N
            o.features = bitor(o.features, K.FEAT_D_FILTER);
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        function rc = setDirection(o, dir)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if dir == K.REVERSE
                o.dir_sign = -1;
            else
                o.dir_sign = 1;
            end
            o.d_prev_in = -o.d_prev_in;   % keep the stored source consistent
            rc = K.OK;
        end

        function rc = setMode(o, mode)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if mode > K.MODE_HOLD, rc = K.ERR_INVALID_MODE; return; end
            if o.mode ~= K.MODE_MANUAL && mode == K.MODE_MANUAL
                % Entering manual: start from where the controller already is.
                o.manual_output = o.output;
            end
            % Leaving manual needs no work: run() back-solves the integrator
            % on every manual sample.
            o.mode = mode;
            rc = K.OK;
        end

        function v = getMode(o), v = o.mode; end

        function rc = setManualOutput(o, out)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(out), rc = K.ERR_INVALID_PARAM; return; end
            o.manual_output = out;
            rc = K.OK;
        end

        function rc = setSetpointRamp(o, rate_max, accel, decel)
            K = pidx.Const;
            if nargin < 3, accel = 0; end
            if nargin < 4, decel = 0; end
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(rate_max) || rate_max < 0 || ...
               ~isfinite(accel) || accel < 0 || ...
               ~isfinite(decel) || decel < 0
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.sp_rate_max = rate_max; o.sp_accel = accel; o.sp_decel = decel;
            if rate_max > 0
                o.features = bitor(o.features, K.FEAT_SP_SHAPER);
            else
                o.features = bitand(o.features, pidx.PID.notBits(K.FEAT_SP_SHAPER));
                o.sp_velocity = 0;
            end
            rc = K.OK;
        end

        function rc = setOutputSlewRate(o, slew_max)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(slew_max) || slew_max < 0
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.out_slew_max = slew_max;
            if slew_max > 0
                o.features = bitor(o.features, K.FEAT_OUT_SHAPER);
            else
                o.features = bitand(o.features, pidx.PID.notBits(K.FEAT_OUT_SHAPER));
            end
            rc = K.OK;
        end

        function rc = setInputFilter(o, tau)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(tau) || tau < 0, rc = K.ERR_INVALID_PARAM; return; end
            o.lpf_tau = tau;
            if tau > 0
                o.features = bitor(o.features, K.FEAT_INPUT_FILTER);
            else
                o.features = bitand(o.features, pidx.PID.notBits(K.FEAT_INPUT_FILTER));
                o.lpf_primed = false;
            end
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        % ==============================================================
        % Level 3 - advanced API
        % ==============================================================

        function rc = setWeights(o, beta, gamma)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(beta) || ~isfinite(gamma) || ...
               beta < 0 || beta > 2 || gamma < 0 || gamma > 2
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.beta = beta; o.gamma = gamma;
            rc = K.OK;
        end

        function rc = setFeedforward(o, ff)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(ff), rc = K.ERR_INVALID_PARAM; return; end
            o.ff_value = ff;
            o.features = bitor(o.features, K.FEAT_FEEDFORWARD);
            rc = K.OK;
        end

        function rc = setFeedforwardFn(o, fn, gain)
            % Install u_ff = gain * fn(setpoint, measurement).
            K = pidx.Const;
            if nargin < 3, gain = 1.0; end
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(gain), rc = K.ERR_INVALID_PARAM; return; end
            o.ff_fn = fn;
            if gain ~= 0
                o.ff_gain = gain;
            else
                o.ff_gain = 1.0;
            end
            if ~isempty(fn)
                o.features = bitor(o.features, K.FEAT_FEEDFORWARD);
            end
            rc = K.OK;
        end

        function rc = setIntegralSeparation(o, threshold)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(threshold) || threshold < 0
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.i_separation = threshold;
            rc = K.OK;
        end

        function rc = setIntegralDeadband(o, db)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(db) || db < 0, rc = K.ERR_INVALID_PARAM; return; end
            o.i_deadband = db;
            rc = K.OK;
        end

        function rc = enableIntegral(o, enable)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if enable
                o.features = bitor(o.features, K.FEAT_INTEGRAL);
            else
                o.features = bitand(o.features, pidx.PID.notBits(K.FEAT_INTEGRAL));
            end
            rc = K.OK;
        end

        function rc = setIntegrator(o, value)
            % Preload the integral TERM, in output units.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(value), rc = K.ERR_INVALID_PARAM; return; end
            o.integrator = pidx.PID.clamp(value, o.i_min, o.i_max);
            rc = K.OK;
        end

        function v = getIntegrator(o), v = o.integrator; end

        function rc = setTrackingInput(o, u_track)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(u_track), rc = K.ERR_INVALID_PARAM; return; end
            o.tracking_input = u_track;
            rc = K.OK;
        end

        function rc = setIntegrationMethod(o, method)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if method > K.INTEGRATION_TRAPEZOIDAL
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.integ_method = method;
            o.recompute(o.dt_last);
            rc = K.OK;
        end

        function rc = setFaultOutput(o, out)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(out), rc = K.ERR_INVALID_PARAM; return; end
            o.failsafe_output = out;
            rc = K.OK;
        end

        function rc = clearFault(o)
            % Drop the latched fault. The next healthy sample resumes control.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            o.flags = bitand(o.flags, ...
                pidx.PID.notBits(bitor(K.FLAG_FAULT, K.FLAG_SENSOR_INVALID)));
            o.fault_count = 0;
            o.meas_prev_valid = false;
            % Re-seed the integrator so control resumes from the fail-safe
            % output rather than from whatever it held before the fault.
            o.backSolve(o.output, 0.0, 0.0, 0.0);
            rc = K.OK;
        end

        function v = isFaulted(o)
            v = bitand(o.flags, pidx.Const.FLAG_FAULT) ~= 0;
        end

        function v = getError(o),  v = o.status.error; end

        function code = getLastError(o)
            % Read AND clear the sticky error.
            code = o.last_error;
            o.last_error = pidx.Const.OK;
        end

        function code = peekLastError(o)
            % Read the sticky error without clearing it.
            code = o.last_error;
        end

        function rc = clearError(o)
            o.last_error = pidx.Const.OK;
            rc = pidx.Const.OK;
        end

        function s = getStatus(o)
            if o.initialised
                s = o.status;
            else
                s = [];
            end
        end

        function v = getFlags(o),    v = o.flags;    end
        function v = getFeatures(o), v = o.features; end
    end

    methods (Static)
        function y = notBits(x)
            % Bitwise complement inside a 32-bit mask, for clearing flags:
            % BITAND(flags, notBits(bit)) clears that bit.
            %
            % A static method rather than a private function on purpose: a
            % private/ subfunction is not visible from inside a classdef that
            % lives in a +package directory under Octave, and hunting that
            % down at runtime is not a trap worth leaving for a user.
            %
            % Subtracting from 2^32-1 is the one spelling that behaves
            % identically in MATLAB and Octave (BITCMP's signature differs
            % between them), and it is exact: every flag value is a
            % non-negative integer far inside the 2^53 range where a double
            % represents integers without loss.
            y = 4294967295 - x;   % 2^32 - 1
        end

        function y = clamp(x, lo, hi)
            % Ordered lo-then-hi exactly as the C macro is, so an inverted
            % range resolves to hi in both languages.
            if x < lo
                y = lo;
            elseif x > hi
                y = hi;
            else
                y = x;
            end
        end

        function ok = gainOk(g)
            % A gain must be finite and non-negative; sign lives in direction.
            ok = isfinite(g) && g >= 0;
        end

        function s = emptyStatus()
            s = struct( ...
                'setpoint_raw', 0, 'setpoint_shaped', 0, ...
                'measurement_raw', 0, 'measurement_filtered', 0, ...
                'error', 0, 'p_term', 0, 'i_term', 0, 'd_term', 0, ...
                'ff_term', 0, 'output_unsat', 0, 'output', 0, ...
                'dt_used', 0, 'kp_active', 0, 'ki_active', 0, ...
                'kd_active', 0, 'update_count', 0, 'saturation_count', 0, ...
                'flags', 0, 'last_error', 0);
        end
    end
end
