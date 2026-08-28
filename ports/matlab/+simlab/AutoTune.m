classdef AutoTune < handle
    % SIMLAB.AUTOTUNE  Non-blocking relay / step identification + tuning rule.
    % A line-by-line port of src/pid_autotune.c.
    %
    % WHY PORT IT RATHER THAN CALL A TOOLBOX
    %   The gains this produces are the gains you will flash to the STM32,
    %   where src/pid_autotune.c is what runs. A toolbox autotune would give
    %   you numbers that no longer correspond to anything on the target.
    %   Everything here - the hysteresis correction in the describing
    %   function, the midpoint moment arm in the step fit, the quality score,
    %   the validation gates - is the C code's behaviour, stage by stage.
    %
    %   [stage, name] = o.state  matches PID_TuneState in the same order, so
    %   a log from MATLAB and a log from the target are directly comparable.
    %
    % ARCHITECTURE (unchanged from C)
    %
    %   IDENTIFICATION            PLANT MODEL              TUNING RULE
    %   relay feedback     ->   FREQ  {Ku, Pu}      ->   ZN, TL, Pessen, ...
    %   open-loop step     ->   FOPDT {K, T, L}     ->   Cohen-Coon, AMIGO, IMC
    %
    % A rule that needs FOPDT parameters cannot be fed from relay data: one
    % point on the Nyquist curve does not determine three model parameters.
    % Asking for that pairing returns ERR_TUNE_MODEL_MISMATCH and names the
    % identification method that would work. No (Ku,Pu) -> (K,T,L) conversion
    % is invented anywhere in this module, exactly as in C.
    %
    % USAGE
    %   cfg = simlab.AutoTune.configDefault(pidx.Const.IDENT_RELAY);
    %   cfg.output_step = 20;  cfg.hysteresis = 0.3;
    %   cfg.output_min = 0;    cfg.output_max = 100;
    %   at = simlab.AutoTune(cfg);
    %   at.start(pid, 60);
    %   while at.isRunning()
    %       u = at.update(y, dt);
    %       y = plant.update(u, dt);
    %   end
    %   res = at.getResult();          % res.model, res.gains
    %   at.apply(pid);                 % bumpless
    %
    % simlab.Sim does all of the above for you when you pass 'tuner'.
    %
    % KNOWN BEHAVIOUR, INHERITED DELIBERATELY FROM C
    %   The relay test UNDER-estimates Ku, by up to 25% even with zero noise:
    %   describing-function theory wants the amplitude of the fundamental, and
    %   every practical implementation measures the peak. The error is in the
    %   safe direction - gains come out low, so the loop is sluggish rather
    %   than unstable. See docs/14_autotune.md and sim/sim_autotune.c.

    properties
        cfg;                  % configuration struct
        sp = 0;               % operating point of the experiment
        state = 0;            % PID_TuneState
        err = 0;              % sticky PID_StatusCode
        result;               % PID_AutoTuneResult
        h = [];               % controller being tuned (borrowed)
        ruleFn = [];          % custom rule: f(model, structure) -> gains
    end

    properties (Access = private)
        magic = 0;

        % experiment bookkeeping
        output = 0;
        elapsed = 0;
        state_time = 0;

        % saved controller state, restored on finish/abort
        saved_mode = 1;
        saved_manual = 0;
        saved_sp = 0;
        restored = false;

        % relay engine
        relay_high = true;
        y_prev = 0;
        y_min = 0; y_max = 0;
        t_last_cross = 0;
        cycle_count = 0; cycles_kept = 0;
        per_sum = 0; per_sq = 0;
        amp_sum = 0; amp_pos_sum = 0; amp_neg_sum = 0;
        per_min = 0; per_max = 0; amp_min = 0; amp_max = 0;

        % step engine
        y0 = 0; u0 = 0;
        y_settled = 0;
        t_283 = 0; t_632 = 0;
        got_283 = false; got_632 = false;
        area1 = 0; moment1 = 0;
        t_end = 0;
        y_acc = 0; y_acc_n = 0;
        y_slow = 0; y_slow_prev = 0;
        settle_timer = 0;
        y_peak = 0;

        % noise / steady-state detection
        noise_acc = 0; noise_n = 0;
        stab_timer = 0;
    end

    properties (Constant)
        TUNE_MAX_CYCLES = 12;      % PIDX_TUNE_MAX_CYCLES
        TUNE_MAGIC = 1414811973;   % 0x54554E45, 'TUNE'

        % PID_TuneState, in the C enum's order
        IDLE = 0; STABILIZING = 1; RELAY_WARMUP = 2; RELAY_OSC = 3;
        STEP_APPLY = 4; STEP_RECORD = 5; ANALYZING = 6; COMPUTING = 7;
        VALIDATING = 8; COMPLETE = 9; FAILED = 10;
    end

    methods
        function o = AutoTune(cfg)
            K = pidx.Const;
            o.state = o.IDLE;
            o.err = K.OK;
            o.result = simlab.AutoTune.emptyResult();
            if nargin < 1 || isempty(cfg)
                return;
            end
            rc = o.init(cfg);
            if rc ~= K.OK
                error('simlab:AutoTune:badConfig', ...
                      'invalid auto-tune config: %s', K.statusToString(rc));
            end
        end

        function rc = init(o, cfg)
            % Copy and validate the configuration, as PID_AutoTune_Init does.
            K = pidx.Const;
            rc = simlab.AutoTune.checkCfg(cfg);
            if rc ~= K.OK
                return;
            end
            o.cfg = cfg;
            o.magic = o.TUNE_MAGIC;
            o.state = o.IDLE;
            o.err = K.OK;
            o.result = simlab.AutoTune.emptyResult();
            o.result.model.kind = K.MODEL_NONE;
            % When the rule cannot be fed by this experiment, say up front
            % which experiment would have worked.
            if pidx.ruleRequiredModel(cfg.rule) == K.MODEL_FOPDT
                o.result.suggested_ident = K.IDENT_STEP;
            else
                o.result.suggested_ident = K.IDENT_RELAY;
            end
        end

        function rc = registerRule(o, fn)
            % fn(model, structure) -> gains struct, for cfg.rule = RULE_CUSTOM.
            o.ruleFn = fn;
            rc = pidx.Const.OK;
        end

        % ==============================================================
        % Start / abort
        % ==============================================================

        function rc = start(o, h, sp)
            K = pidx.Const;
            if ~o.isValid() || isempty(h)
                rc = K.ERR_NULL; return;
            end
            if o.state ~= o.IDLE && o.state ~= o.COMPLETE && o.state ~= o.FAILED
                rc = K.ERR_BUSY; return;
            end
            if ~isfinite(sp)
                rc = K.ERR_NAN_INPUT; return;
            end
            if o.cfg.rule == K.RULE_CUSTOM && isempty(o.ruleFn)
                rc = K.ERR_INVALID_PARAM; return;
            end

            o.h = h;
            o.sp = sp;
            o.saved_mode = h.getMode();
            % Seed the bias from the commanded manual level, not from the last
            % completed update: a caller that has just done setMode(MANUAL) +
            % setManualOutput() without stepping would otherwise hand the
            % tuner a stale zero and the relay would swing about the wrong
            % operating point. Same reasoning as in C.
            o.saved_manual = h.getManualOutput();
            o.saved_sp = h.getSetpoint();
            o.restored = false;

            % The tuner drives the actuator directly; the controller must not
            % fight it. MANUAL also keeps the core back-solving its integrator
            % every sample, so whatever the tune leaves behind is a bumpless
            % starting point.
            h.setMode(K.MODE_MANUAL);

            o.elapsed = 0; o.state_time = 0; o.stab_timer = 0;
            o.noise_acc = 0; o.noise_n = 0;
            o.cycle_count = 0; o.cycles_kept = 0;
            o.per_sum = 0; o.per_sq = 0;
            o.amp_sum = 0; o.amp_pos_sum = 0; o.amp_neg_sum = 0;
            o.per_min = 0; o.per_max = 0; o.amp_min = 0; o.amp_max = 0;
            o.got_283 = false; o.got_632 = false;
            o.area1 = 0; o.moment1 = 0; o.t_end = 0;
            o.y_acc = 0; o.y_acc_n = 0;
            o.y_slow = 0; o.y_slow_prev = 0; o.settle_timer = 0;
            o.y_prev = 0;
            o.err = K.OK;

            if o.cfg.auto_bias
                o.u0 = o.saved_manual;
            else
                o.u0 = o.cfg.bias;
            end

            % Start centred on the bias so the plant is not disturbed before
            % the experiment proper begins.
            o.output = o.clampOut(o.u0);
            o.relay_high = true;

            if o.cfg.skip_stabilize
                if o.cfg.ident == K.IDENT_RELAY
                    o.state = o.RELAY_WARMUP;
                else
                    o.state = o.STEP_APPLY;
                end
            else
                o.state = o.STABILIZING;
            end

            o.result.code = K.ERR_BUSY;
            o.result.model.kind = K.MODEL_NONE;
            rc = K.OK;
        end

        function rc = abort(o)
            K = pidx.Const;
            if ~o.isValid(), rc = K.ERR_NULL; return; end
            if o.state == o.IDLE || o.state == o.COMPLETE
                rc = K.ERR_BUSY; return;
            end
            o.fail(K.ERR_TUNE_ABORTED);
            rc = K.OK;
        end

        % ==============================================================
        % The update path - one sample
        % ==============================================================

        function u = update(o, measurement, dt)
            K = pidx.Const;
            if ~o.isValid()
                u = 0; return;
            end
            if o.state == o.IDLE || o.state == o.COMPLETE || o.state == o.FAILED
                u = o.output; return;
            end
            if ~(dt > 0) || ~isfinite(dt)
                % A bad dt would corrupt every period and crossing
                % measurement, so it ends the run rather than being ignored.
                o.fail(K.ERR_INVALID_DT);
                u = o.output; return;
            end

            o.elapsed = o.elapsed + dt;
            o.state_time = o.state_time + dt;

            if o.safetyCheck(measurement, dt)
                u = o.output; return;
            end

            % Noise estimate: mean absolute sample-to-sample change. Used to
            % report sigma and to justify the hysteresis the user chose.
            if o.noise_n > 0
                o.noise_acc = o.noise_acc + abs(measurement - o.y_prev);
            end
            if o.noise_n < 4294967295
                o.noise_n = o.noise_n + 1;
            end

            switch o.state
                case o.STABILIZING
                    o.y_settled = measurement;
                    o.doStabilize(measurement, dt);
                case {o.RELAY_WARMUP, o.RELAY_OSC}
                    o.doRelay(measurement);
                case {o.STEP_APPLY, o.STEP_RECORD}
                    o.doStep(measurement, dt);
                otherwise
                    % nothing
            end

            % The three analysis states are one-shot: entering one runs it to
            % completion within this same call, so the caller never sees the
            % tuner stall in a state that produces no output.
            if o.state == o.ANALYZING
                if o.cfg.ident == K.IDENT_RELAY
                    o.analyzeRelay();
                else
                    o.analyzeStep();
                end
            end
            if o.state == o.COMPUTING
                o.compute();
            end
            if o.state == o.VALIDATING
                o.validate();
            end

            o.y_prev = measurement;
            u = o.output;
        end

        % ==============================================================
        % Queries
        % ==============================================================

        function ok = isComplete(o),  ok = o.isValid() && o.state == o.COMPLETE; end

        function ok = isRunning(o)
            ok = o.isValid() && o.state ~= o.IDLE && ...
                 o.state ~= o.COMPLETE && o.state ~= o.FAILED;
        end

        function s = getState(o)
            if o.isValid(), s = o.state; else, s = o.IDLE; end
        end

        function code = getError(o)
            if o.isValid(), code = o.err; else, code = pidx.Const.ERR_NULL; end
        end

        function pct = getProgress(o)
            if ~o.isValid(), pct = 0; return; end
            switch o.state
                case o.IDLE,          pct = 0;
                case o.STABILIZING,   pct = 5;
                case o.RELAY_WARMUP,  pct = 20;
                case o.STEP_APPLY,    pct = 20;
                case o.RELAY_OSC
                    done = floor(o.cycles_kept * 50 / o.cfg.eval_cycles);
                    pct = 25 + min(done, 50);
                case o.STEP_RECORD
                    if o.got_632, pct = 60; else, pct = 40; end
                case o.ANALYZING,     pct = 80;
                case o.COMPUTING,     pct = 90;
                case o.VALIDATING,    pct = 95;
                case o.COMPLETE,      pct = 100;
                case o.FAILED,        pct = 100;
                otherwise,            pct = 0;
            end
        end

        function [rc, r] = getResult(o)
            K = pidx.Const;
            if ~o.isValid()
                rc = K.ERR_NULL; r = o.result; return;
            end
            if o.state ~= o.COMPLETE && o.state ~= o.FAILED
                rc = K.ERR_BUSY; r = o.result; return;
            end
            r = o.result;
            if o.state == o.COMPLETE
                rc = K.OK;
            else
                rc = o.err;
            end
        end

        function rc = apply(o, h)
            % Write the tuned gains into a handle, bumplessly.
            % setGainsRescaleIntegral keeps Ki*integral constant across the
            % change, so the output does not step when the new tuning lands.
            K = pidx.Const;
            if ~o.isValid(), rc = K.ERR_NULL; return; end
            if o.state ~= o.COMPLETE, rc = K.ERR_BUSY; return; end
            if nargin < 2 || isempty(h), h = o.h; end
            if isempty(h), rc = K.ERR_NULL; return; end

            g = o.result.gains;
            rc = h.setGainsRescaleIntegral(g.kp, g.ki, g.kd);
            if rc ~= K.OK, return; end
            if g.tf > 0
                h.setDerivativeFilter(g.tf);
            end
        end

        function rc = retune(o, rule, structure)
            % Re-run only the tuning rule on an already identified model,
            % without touching the plant. Lets you try a different rule on
            % data you already paid for.
            K = pidx.Const;
            if ~o.isValid(), rc = K.ERR_NULL; return; end
            if o.result.model.kind == K.MODEL_NONE, rc = K.ERR_BUSY; return; end
            if rule == K.RULE_CUSTOM
                if isempty(o.ruleFn), rc = K.ERR_INVALID_PARAM; return; end
                [rc, g] = o.ruleFn(o.result.model, structure);
            else
                [rc, g] = pidx.ruleApply(rule, o.result.model, structure, ...
                                         o.cfg.lambda);
            end
            if rc ~= K.OK, return; end
            o.result.gains = g;
            o.cfg.rule = rule;
            o.cfg.structure = structure;
            o.state = o.COMPLETE;
            o.err = K.OK;
            o.result.code = K.OK;
        end
    end

    % ==================================================================
    % The states
    % ==================================================================

    methods (Access = private)

        function ok = isValid(o)
            ok = (o.magic == o.TUNE_MAGIC);
        end

        function u = clampOut(o, u)
            if o.cfg.output_max > o.cfg.output_min
                u = min(max(u, o.cfg.output_min), o.cfg.output_max);
            end
        end

        function stop = safetyCheck(o, y, dt)
            % Applied on every sample regardless of state.
            K = pidx.Const;
            stop = false;
            if ~isfinite(y)
                o.fail(K.ERR_NAN_INPUT); stop = true; return;
            end
            if o.cfg.meas_max > o.cfg.meas_min
                if y < o.cfg.meas_min || y > o.cfg.meas_max
                    o.fail(K.ERR_SENSOR_RANGE); stop = true; return;
                end
            end
            if o.cfg.rate_max > 0
                rate = abs(y - o.y_prev) / dt;
                % The first sample has no previous value to compare against.
                if o.noise_n > 0 && rate > o.cfg.rate_max
                    o.fail(K.ERR_SENSOR_RATE); stop = true; return;
                end
            end
            if o.cfg.timeout_s > 0 && o.elapsed > o.cfg.timeout_s
                o.fail(K.ERR_TUNE_TIMEOUT); stop = true;
            end
        end

        function doStabilize(o, y, dt)
            % Wait until the process is quiet enough that the experiment
            % starts from a defined operating point.
            K = pidx.Const;
            thr = o.cfg.stab_rate;
            rate = abs(y - o.y_prev) / dt;

            if ~(thr > 0)
                if o.cfg.stab_time > 0
                    dwell = o.cfg.stab_time;
                else
                    dwell = 1.0;
                end
                if o.cfg.hysteresis > 0
                    thr = o.cfg.hysteresis / dwell;
                else
                    thr = abs(y) * 0.01 / dwell;
                end
                if ~(thr > 0)
                    thr = 1e-4;
                end
            end

            % The default threshold above is derived from the relay
            % hysteresis, which says nothing about how noisy the sensor is -
            % and on a step test there is no hysteresis to derive it from at
            % all. Hold the threshold at three times the measured noise rate,
            % or stabilisation would never be declared and every tune would
            % end in a timeout.
            if o.noise_n > 8
                noise_rate = (o.noise_acc / o.noise_n) / dt;
                floor_rate = 3.0 * noise_rate;
                if thr < floor_rate
                    thr = floor_rate;
                end
            end

            o.output = o.clampOut(o.u0);

            if rate <= thr
                o.stab_timer = o.stab_timer + dt;
            else
                o.stab_timer = 0;
            end

            if o.stab_timer >= o.cfg.stab_time
                o.y0 = y;
                o.y_min = y; o.y_max = y; o.y_peak = y;
                o.y_settled = y;
                o.t_last_cross = o.elapsed;
                o.state_time = 0;
                if o.cfg.ident == K.IDENT_RELAY
                    o.state = o.RELAY_WARMUP;
                else
                    o.state = o.STEP_APPLY;
                end
            end
        end

        function doRelay(o, y)
            % One relay sample. The relay switches on the hysteresis band
            % around the setpoint, and a full period is measured between two
            % successive switches of the SAME direction.
            eps_ = o.cfg.hysteresis;
            e = o.sp - y;
            switched_up = false;

            if y > o.y_max, o.y_max = y; end
            if y < o.y_min, o.y_min = y; end

            % Switch only outside the eps band, so noise inside the band
            % cannot chatter the actuator.
            if o.relay_high
                if e < -eps_
                    o.relay_high = false;
                end
            else
                if e > eps_
                    o.relay_high = true;
                    switched_up = true;
                end
            end

            if o.relay_high
                o.output = o.clampOut(o.u0 + o.cfg.output_step);
            else
                o.output = o.clampOut(o.u0 - o.cfg.output_step);
            end

            if ~switched_up
                return;
            end

            % A rising switch closes one full period.
            period = o.elapsed - o.t_last_cross;
            a_pos = o.y_max - o.sp;
            a_neg = o.sp - o.y_min;
            amp = 0.5 * (o.y_max - o.y_min);

            o.t_last_cross = o.elapsed;
            o.y_max = y;
            o.y_min = y;
            o.cycle_count = o.cycle_count + 1;

            if o.state == o.RELAY_WARMUP
                % Early cycles are transient: the limit cycle has not formed
                % yet and including them biases both Pu and a.
                if o.cycle_count >= o.cfg.warmup_cycles
                    o.state = o.RELAY_OSC;
                    o.cycle_count = 0;
                end
                return;
            end

            % Oscillation sanity, checked per cycle.
            if o.cfg.osc_max > 0 && (2.0 * amp > o.cfg.osc_max)
                o.fail(pidx.Const.ERR_TUNE_UNSTABLE);
                return;
            end

            if o.cycles_kept == 0
                o.per_min = period; o.per_max = period;
                o.amp_min = amp;    o.amp_max = amp;
            else
                if period < o.per_min, o.per_min = period; end
                if period > o.per_max, o.per_max = period; end
                if amp < o.amp_min,    o.amp_min = amp;    end
                if amp > o.amp_max,    o.amp_max = amp;    end
            end
            o.per_sum = o.per_sum + period;
            o.per_sq = o.per_sq + period * period;
            o.amp_sum = o.amp_sum + amp;
            o.amp_pos_sum = o.amp_pos_sum + a_pos;
            o.amp_neg_sum = o.amp_neg_sum + a_neg;
            o.cycles_kept = o.cycles_kept + 1;

            if o.cycles_kept >= o.cfg.eval_cycles
                o.state = o.ANALYZING;
            end
        end

        function doStep(o, y, dt)
            % Open-loop step. The end of the experiment is detected by the
            % response going flat, not by a fixed timer, so a slow plant is
            % not cut off early and a fast one does not waste time.
            K = pidx.Const;
            if o.state == o.STEP_APPLY
                o.y0 = o.y_settled;
                o.output = o.clampOut(o.u0 + o.cfg.output_step);
                o.state = o.STEP_RECORD;
                o.state_time = 0;
                o.y_peak = y;
                o.y_slow = y;
                o.y_slow_prev = y;
                return;
            end

            o.output = o.clampOut(o.u0 + o.cfg.output_step);

            % Low-pass the response to get a stable y_infinity estimate. The
            % time constant is a fifth of the elapsed test time, so it adapts
            % to the plant instead of needing to be configured.
            if o.state_time > 0
                tau = o.state_time * 0.2;
            else
                tau = dt;
            end
            a = tau / (tau + dt);
            o.y_settled = a * o.y_settled + (1.0 - a) * y;

            % Accumulate the moments of the raw response (y - y0). Both
            % integrals use the trapezoidal rule evaluated at the interval
            % MIDPOINT (t - dt/2). This is not cosmetic: the analytic
            % subtraction uses integral of t dt = te^2/2, and only the
            % midpoint sum matches that exactly. Sampling the moment arm at
            % the right endpoint instead leaves the first moment short by
            % te*dt/2 per unit of dy - a deficit that GROWS with test length,
            % so a longer and more careful experiment would produce a WORSE
            % model.
            y_avg = 0.5 * ((y - o.y0) + (o.y_prev - o.y0));
            t_mid = o.state_time - 0.5 * dt;
            o.area1 = o.area1 + y_avg * dt;
            o.moment1 = o.moment1 + t_mid * y_avg * dt;
            o.t_end = o.state_time;

            if abs(y - o.y0) > abs(o.y_peak - o.y0)
                o.y_peak = y;
            end

            % ---- flatness test ----
            % Both moments integrate the residual (y_inf - y), so the
            % experiment must not stop while that residual is still
            % significant: a truncated tail biases A1 low and M1 much lower,
            % which shows up as an underestimated T and an overestimated L.
            if o.state_time > 0
                tau_s = o.state_time * 0.05;
            else
                tau_s = dt;
            end
            as = tau_s / (tau_s + dt);
            total = abs(o.y_settled - o.y0);

            o.y_slow_prev = o.y_slow;
            o.y_slow = as * o.y_slow + (1.0 - as) * y;

            slope = abs(o.y_slow - o.y_slow_prev) / dt;
            remaining = abs(o.y_settled - o.y_slow);
            if total > 0
                slope_thr = total * 0.001;
            else
                slope_thr = 1e-6;
            end
            near_thr = total * 0.005;

            % Both thresholds must stay above the noise floor, or the test can
            % never pass and the only possible outcome is a timeout.
            if o.noise_n > 0
                noise = o.noise_acc / o.noise_n;
                jitter = (1.0 - as) * noise / dt;
                floor_slope = 3.0 * jitter;
                floor_near = 2.0 * noise;
                if slope_thr < floor_slope, slope_thr = floor_slope; end
                if near_thr < floor_near,   near_thr = floor_near;   end
            end

            if total > 0 && slope < slope_thr && remaining < near_thr
                o.settle_timer = o.settle_timer + dt;
                % Average the raw measurement across the flat window. This,
                % not the lagging low-pass estimate, is what the fit will use
                % as y_infinity.
                o.y_acc = o.y_acc + y;
                if o.y_acc_n < 65535
                    o.y_acc_n = o.y_acc_n + 1;
                end
            else
                o.settle_timer = 0;
                o.y_acc = 0;
                o.y_acc_n = 0;
            end

            % Guard against declaring victory before the plant has even
            % reacted. Right after the step, total is still tiny, so both
            % relative thresholds are tiny too and noise alone can satisfy
            % them - the tune would "settle" during the dead time and fit a
            % model to nothing.
            moved = abs(o.y_slow - o.y0);
            move_min = 0;
            if o.noise_n > 8
                move_min = 10.0 * (o.noise_acc / o.noise_n);
            end
            if moved < move_min
                o.settle_timer = 0;
                o.y_acc = 0;
                o.y_acc_n = 0;
            end

            if o.settle_timer > (o.state_time * 0.25) && ...
               o.settle_timer > (20.0 * dt) && o.y_acc_n > 0
                % Commit the settle-window mean as the final value.
                o.y_settled = o.y_acc / o.y_acc_n;
                o.state = o.ANALYZING;
                return;
            end

            % Record the two crossings against the running final-value
            % estimate. Cross-check only; the fit uses the moments.
            total = o.y_settled - o.y0;
            now = y - o.y0;
            if abs(total) > 0
                frac = now / total;
                if ~o.got_283 && frac >= 0.283
                    o.t_283 = o.state_time;
                    o.got_283 = true;
                end
                if o.got_283 && ~o.got_632 && frac >= 0.632
                    o.t_632 = o.state_time;
                    o.got_632 = true;
                end
            end
        end

        function analyzeRelay(o)
            K = pidx.Const;
            n = o.cycles_kept;
            pu = o.per_sum / n;
            a = o.amp_sum / n;
            eps_ = o.cfg.hysteresis;
            a_pos = o.amp_pos_sum / n;
            a_neg = o.amp_neg_sum / n;

            % Reject a "limit cycle" that is really just noise.
            if o.cfg.osc_min > 0
                amp_min_ok = o.cfg.osc_min;
            elseif eps_ > 0
                amp_min_ok = 2.0 * eps_;
            else
                amp_min_ok = 0.0;
            end
            if a <= 0 || a < amp_min_ok
                o.fail(K.ERR_TUNE_NO_OSCILLATION); return;
            end
            if ~(pu > 0)
                o.fail(K.ERR_TUNE_NO_OSCILLATION); return;
            end

            % Describing-function inversion with the hysteresis correction:
            %   Ku = 4h / (pi * sqrt(a^2 - eps^2))
            % If the amplitude does not clear the hysteresis band, the relay
            % never really exercised the plant and the formula is undefined.
            radicand = a * a - eps_ * eps_;
            if radicand <= 0
                o.fail(K.ERR_TUNE_NO_OSCILLATION); return;
            end
            ku = (4.0 * o.cfg.output_step) / (pi * sqrt(radicand));

            o.result.model.kind = K.MODEL_FREQ;
            o.result.model.ku = ku;
            o.result.model.pu = pu;
            o.result.model.k = 0; o.result.model.t = 0; o.result.model.l = 0;

            % Spread of the kept cycles is the repeatability measure; it is
            % what "quality" means here, rather than an invented score.
            o.result.period_spread = (o.per_max - o.per_min) / pu;
            o.result.amp_spread = (o.amp_max - o.amp_min) / a;
            o.result.amplitude = a;
            o.result.cycles_used = o.cycles_kept;

            denom = a_pos + a_neg;
            if denom > 0
                o.result.asymmetry = abs(a_pos - a_neg) / denom;
            else
                o.result.asymmetry = 0;
            end
            % Above 0.30 the plant is nonlinear or the bias is wrong at this
            % operating point. The result is still usable, but the user must
            % be told rather than silently trusting it.
            o.result.asymmetric = o.result.asymmetry > 0.30;

            % Quality: start at 100 and subtract for period spread, amplitude
            % spread and asymmetry, each against its acceptance threshold
            % from the design doc (10%, 15%, 30%).
            q = 100.0;
            q = q - (o.result.period_spread / 0.10) * 25.0;
            q = q - (o.result.amp_spread / 0.15) * 15.0;
            q = q - (o.result.asymmetry / 0.30) * 10.0;
            q = min(max(q, 0), 100);
            o.result.model.quality = uint8(round(q));

            if o.noise_n > 0
                o.result.model.noise_sigma = o.noise_acc / o.noise_n;
            else
                o.result.model.noise_sigma = 0;
            end

            o.state = o.COMPUTING;
        end

        function analyzeStep(o)
            % Area / moment FOPDT fit. area1 and moment1 hold the integrals of
            % (y - y0); subtracting them from the enclosing rectangles dy*te
            % and dy*te^2/2 gives the integrals of the residual (y_inf - y),
            % and dividing by dy gives the moments of the UNIT step response,
            % which is what the closed form is derived for:
            %
            %   A1 = L + T
            %   M1 = L^2/2 + L*T + T^2
            %   T  = sqrt(2*M1 - A1^2),   L = A1 - T
            K = pidx.Const;
            dy = o.y_settled - o.y0;
            if abs(dy) <= 0
                o.fail(K.ERR_TUNE_NO_OSCILLATION); return;
            end
            k = dy / o.cfg.output_step;

            te = o.t_end;
            a1 = (dy * te - o.area1) / dy;
            m1 = (dy * te * te * 0.5 - o.moment1) / dy;
            rad = 2.0 * m1 - a1 * a1;

            if ~(rad > 0) || ~(a1 > 0)
                % The moments are inconsistent with a first-order model - the
                % response was too short, too noisy, or not first order.
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end
            tt = sqrt(rad);
            l = a1 - tt;

            if ~(tt > 0)
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end
            if l < 0
                % A negative dead time is physically impossible; it means the
                % model has essentially no transport delay. Floor it at zero
                % and let the quality score reflect the strained fit.
                l = 0;
            end

            o.result.model.kind = K.MODEL_FOPDT;
            o.result.model.k = k;
            o.result.model.t = tt;
            o.result.model.l = l;
            o.result.model.ku = 0;
            o.result.model.pu = 0;
            o.result.amplitude = abs(dy);
            o.result.cycles_used = 1;

            % Quality from two independent indicators: the normalised dead
            % time L/T, and how much of the transient the experiment actually
            % captured.
            ratio = l / tt;
            q = 100.0;
            if ratio < 0.05
                q = q - 40.0;      % dead time barely resolvable
            elseif ratio > 2.0
                q = q - 45.0;      % dead-time dominant, FOPDT strained
            end
            span = l + tt;
            if span > 0
                covered = o.t_end / span;
                if covered < 5.0
                    d = (5.0 - covered) * 10.0;
                    if d > 45.0, d = 45.0; end
                    q = q - d;
                end
            end
            q = min(max(q, 0), 100);
            o.result.model.quality = uint8(round(q));

            if o.noise_n > 0
                o.result.model.noise_sigma = o.noise_acc / o.noise_n;
            else
                o.result.model.noise_sigma = 0;
            end

            o.state = o.COMPUTING;
        end

        function compute(o)
            K = pidx.Const;
            if o.cfg.rule == K.RULE_CUSTOM
                if isempty(o.ruleFn)
                    o.fail(K.ERR_INVALID_PARAM); return;
                end
                [rc, g] = o.ruleFn(o.result.model, o.cfg.structure);
            else
                [rc, g] = pidx.ruleApply(o.cfg.rule, o.result.model, ...
                                         o.cfg.structure, o.cfg.lambda);
            end
            if rc ~= K.OK
                if rc == K.ERR_TUNE_MODEL_MISMATCH
                    % Tell the caller which experiment would have worked
                    % instead of fabricating the missing model parameters.
                    if pidx.ruleRequiredModel(o.cfg.rule) == K.MODEL_FOPDT
                        o.result.suggested_ident = K.IDENT_STEP;
                    else
                        o.result.suggested_ident = K.IDENT_RELAY;
                    end
                end
                o.fail(rc); return;
            end
            o.result.gains = g;

            % Cohen-Coon is only valid on a band of normalised dead time;
            % outside it the formula still evaluates but the answer is not
            % trustworthy.
            if o.cfg.rule == K.RULE_COHEN_COON && ...
               o.result.model.kind == K.MODEL_FOPDT
                ratio = o.result.model.l / o.result.model.t;
                if ratio < 0.05 || ratio > 1.5
                    o.fail(K.ERR_TUNE_MODEL_MISMATCH); return;
                end
            end

            o.state = o.VALIDATING;
        end

        function validate(o)
            K = pidx.Const;
            g = o.result.gains;
            dt = o.h.getSampleTime();

            if ~isfinite(g.kp) || ~isfinite(g.ki) || ~isfinite(g.kd) || ...
               ~isfinite(g.tf)
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end
            if ~(g.kp > 0)
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end
            if g.ki < 0 || g.kd < 0
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end

            % Discretisation floor. The relay can only switch on a sample
            % boundary, so each half period is quantised to a whole number of
            % samples and the measured Pu carries an error of order +/-2*dt.
            % Requiring 20 samples per period holds the quantisation
            % contribution near 10%, comparable to the intrinsic error of the
            % describing-function method, so neither term dominates.
            %
            % This bounds the SAMPLING error only. It cannot bound the error
            % of the relay method itself: on a lag-dominated plant (L/T ~
            % 0.05) the limit cycle runs about 19% slower than the true
            % ultimate period no matter how finely it is sampled. A step test
            % is the right experiment there.
            if o.result.model.kind == K.MODEL_FREQ && dt > 0 && ...
               o.result.model.pu < 20.0 * dt
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end
            if o.result.model.kind == K.MODEL_FOPDT && dt > 0 && ...
               o.result.model.t < 4.0 * dt
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end

            % Repeatability gate from the design doc: below 50 the experiment
            % did not produce a consistent limit cycle.
            if double(o.result.model.quality) < 50
                o.fail(K.ERR_TUNE_VALIDATION); return;
            end

            o.result.elapsed_s = o.elapsed;
            o.succeed();
        end

        function restore(o)
            % Put the controller back exactly as it was found. Called on
            % success, on failure and on abort - a tuner that leaves the plant
            % in MANUAL after an aborted experiment is a safety hazard.
            if ~isempty(o.h) && ~o.restored
                o.h.setManualOutput(o.saved_manual);
                o.h.setSetpoint(o.saved_sp);
                o.h.setMode(o.saved_mode);
                o.restored = true;
            end
        end

        function fail(o, code)
            o.state = o.FAILED;
            o.err = code;
            o.result.code = code;
            o.output = o.saved_manual;
            o.restore();
        end

        function succeed(o)
            o.state = o.COMPLETE;
            o.err = pidx.Const.OK;
            o.result.code = pidx.Const.OK;
            o.output = o.saved_manual;
            o.restore();
        end
    end

    % ==================================================================
    % Configuration and helpers
    % ==================================================================

    methods (Static)
        function cfg = configDefault(ident)
            % CONFIGDEFAULT(IDENT) - the same safe defaults as
            % PID_AutoTune_ConfigDefault(). Relay defaults to Tyreus-Luyben,
            % step to AMIGO-step, for the reason documented on those rules:
            % they are the ones that survive a model that is 30% wrong.
            %
            % The caller must at least set output_step and the output limits.
            K = pidx.Const;
            if nargin < 1, ident = K.IDENT_RELAY; end

            cfg = struct();
            cfg.ident = ident;
            if ident == K.IDENT_RELAY
                cfg.rule = K.RULE_TYREUS_LUYBEN;
            else
                cfg.rule = K.RULE_AMIGO_STEP;
            end
            cfg.structure = K.STRUCT_PID;
            cfg.lambda = 0.0;            % derived from the model

            cfg.output_step = 0.0;       % caller must set this
            cfg.hysteresis = 0.0;
            cfg.bias = 0.0;
            cfg.auto_bias = true;

            cfg.output_min = 0.0;
            cfg.output_max = 0.0;        % max <= min disables clamp
            cfg.meas_min = 0.0;
            cfg.meas_max = 0.0;          % max <= min disables check
            cfg.osc_max = 0.0;
            cfg.osc_min = 0.0;
            cfg.rate_max = 0.0;
            cfg.timeout_s = 120.0;

            cfg.warmup_cycles = 2;
            cfg.eval_cycles = 4;
            cfg.stab_time = 1.0;
            cfg.stab_rate = 0.0;         % derived from hysteresis
            cfg.skip_stabilize = false;
        end

        function rc = checkCfg(c)
            % The ident/rule pairing is checked FIRST, ahead of the numeric
            % limits. It is a structural mistake - the chosen rule
            % mathematically cannot be evaluated from the model the chosen
            % experiment produces - and the caller is best served by that
            % specific diagnosis rather than a generic INVALID_PARAM from
            % some amplitude they had not filled in yet.
            K = pidx.Const;
            if c.ident ~= K.IDENT_RELAY && c.ident ~= K.IDENT_STEP
                rc = K.ERR_INVALID_PARAM; return;
            end
            if c.rule < 0 || c.rule > K.RULE_CUSTOM
                rc = K.ERR_INVALID_PARAM; return;
            end
            if c.rule ~= K.RULE_CUSTOM
                need = pidx.ruleRequiredModel(c.rule);
                if c.ident == K.IDENT_RELAY
                    have = K.MODEL_FREQ;
                else
                    have = K.MODEL_FOPDT;
                end
                if need ~= have
                    rc = K.ERR_TUNE_MODEL_MISMATCH; return;
                end
            end
            if ~(c.output_step > 0) || ~isfinite(c.output_step)
                rc = K.ERR_INVALID_PARAM; return;
            end
            if c.hysteresis < 0 || ~isfinite(c.hysteresis)
                rc = K.ERR_INVALID_PARAM; return;
            end
            if c.structure < 0 || c.structure > K.STRUCT_PID
                rc = K.ERR_INVALID_PARAM; return;
            end
            if c.eval_cycles == 0
                rc = K.ERR_INVALID_PARAM; return;
            end
            if c.eval_cycles > simlab.AutoTune.TUNE_MAX_CYCLES
                rc = K.ERR_INVALID_PARAM; return;
            end
            rc = K.OK;
        end

        function r = emptyResult()
            r = struct();
            r.model = pidx.plantModel();
            r.gains = struct('kp', 0, 'ki', 0, 'kd', 0, 'ti', 0, ...
                             'td', 0, 'tf', 0);
            r.code = 0;
            r.elapsed_s = 0;
            r.amplitude = 0;
            r.period_spread = 0;
            r.amp_spread = 0;
            r.asymmetry = 0;
            r.cycles_used = 0;
            r.asymmetric = false;
            r.suggested_ident = 0;
        end

        function s = stateToString(s_)
            names = {'IDLE', 'STABILIZING', 'RELAY_WARMUP', 'RELAY_OSC', ...
                     'STEP_APPLY', 'STEP_RECORD', 'ANALYZING', 'COMPUTING', ...
                     'VALIDATING', 'COMPLETE', 'FAILED'};
            if s_ >= 0 && s_ < numel(names)
                s = names{s_ + 1};
            else
                s = '?';
            end
        end
    end
end
