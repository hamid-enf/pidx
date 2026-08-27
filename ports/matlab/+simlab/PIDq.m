classdef PIDq < handle
    % SIMLAB.PIDQ  Standalone fixed-point PID. Port of src/pid_fixed.c.
    %
    % WHY PORT IT
    %   A fast current loop on a Cortex-M0 has no FPU, and PIDX's answer is
    %   this controller rather than a soft-float build of the main one. If you
    %   are going to flash it, you need to know what the quantisation costs
    %   BEFORE you flash it - and "the gains are the same but in Q16.16" is
    %   not an answer, because the integrator increment is where fixed point
    %   quietly breaks.
    %
    % THE FAILURE MODE THIS EXISTS TO SHOW
    %   Integral resolution death. The per-sample increment is Ki*dt*e. With
    %   Ki = 0.5, dt = 1 ms and an error of one LSB, that is 1.5e-8 in real
    %   units - 0.0005 of a Q15 LSB. Rounded into a Q15 accumulator it is
    %   exactly zero, so the integrator never moves and the loop keeps a
    %   permanent steady-state error that no amount of tuning removes.
    %
    %   The C controller holds the integrator (and the EMA state) at Q30,
    %   32768x finer than the output LSB. This port does the same, with the
    %   same rounding, the same saturating arithmetic and the same arithmetic
    %   right shift, so a trace from here and a trace from the target match
    %   sample for sample. simlab_tests/test_fixed.m compares against
    %   PIDq_Update running in C.
    %
    % FORMATS (identical to pid_fixed.h)
    %   signals  Q15      int16, 32768 == 1.0, so the range is [-1, +0.99997]
    %   gains    Q16.16   int32, 65536 == 1.0
    %   internal Q30      integrator, derivative and EMA state
    %
    % SUPPORTED, exactly as in C: P, I, D-on-measurement, derivative filter,
    % shift-based EMA input filter, output and integral clamping, anti-windup
    % (clamp / conditional / back-calculation with a power-of-two
    % coefficient), integral deadband and separation, DIRECT/REVERSE, and
    % manual/automatic with bumpless transfer.
    %
    % NOT SUPPORTED, and deliberately absent rather than stubbed: auto-tuning,
    % interpolated gain scheduling, setpoint weighting, setpoint shaping,
    % feedforward, cascade. Those need division or wide dynamic range and
    % belong to the floating-point core.
    %
    % EXAMPLE
    %   cfg = simlab.PIDq.configDefault();
    %   cfg.kp_q16 = simlab.PIDq.fToQ16(1.5);
    %   cfg.ki_q16 = simlab.PIDq.fToQ16(0.5);
    %   cfg.dt_us  = 1000;
    %   q = simlab.PIDq(cfg);
    %   q.setSetpoint(int16(16384));            % 0.5 full scale
    %   u = q.update(int16(0));                 % Q15 out
    %
    %   See simlab.compareFixed for the float-versus-fixed study.

    properties (SetAccess = private)
        % Precomputed coefficients. Recomputed only by init/setGains, so the
        % update path contains no division at all.
        kp_q16 = 0;
        ci_q30 = 0;
        cb_q16 = 0;
        ca_q30 = 0;

        dt_us = 0;
        tf_us = 0;

        out_min_q30 = 0; out_max_q30 = 0;
        i_min_q30 = 0;   i_max_q30 = 0;

        integ_q30 = 0;
        d_q30 = 0;
        meas_filt_q30 = 0;
        meas_prev_q30 = 0;
        out_q30 = 0;

        setpoint_q15 = 0;
        manual_q15 = 0;
        out_q15 = 0;

        deadband_q15 = 0;
        separation_q15 = 0;

        aw_mode = 1;
        bc_shift = 4;
        lpf_shift = 0;
        direction = 0;
        mode = 1;
        flags = 0;

        magic = 0;
    end

    properties (Constant)
        MAGIC = 20951;            % 0x51D7
        ONE = 32768;              % 1.0 in Q15
        QMAX = 32767;
        QMIN = -32768;
        FRAC = 30;
        SIG_FRAC = 15;
        GAIN_FRAC = 16;
        MICRO = 1000000;

        FLAG_SATURATED = 1;
        FLAG_PRIMED = 2;

        DIRECT = 0;  REVERSE = 1;
        MODE_MANUAL = 0;  MODE_AUTOMATIC = 1;
        AW_NONE = 0;  AW_CLAMP = 1;  AW_CONDITIONAL = 2;  AW_BACK_CALC = 3;
    end

    methods
        function o = PIDq(cfg)
            if nargin > 0 && ~isempty(cfg)
                rc = o.init(cfg);
                if rc ~= pidx.Const.OK
                    error('simlab:PIDq:init', 'PIDq init failed: %s', ...
                        pidx.Const.statusToString(rc));
                end
            end
        end

        function rc = init(o, cfg)
            % Validate, fold dt and Tf into the coefficients, clear all state.
            K = pidx.Const;
            if cfg.dt_us == 0
                rc = K.ERR_INVALID_DT; return;
            end
            % Gains carry no sign: the sense of the loop is `direction`. A
            % negative gain here would silently fight that setting.
            if cfg.kp_q16 < 0 || cfg.ki_q16 < 0 || cfg.kd_q16 < 0
                rc = K.ERR_INVALID_GAIN; return;
            end
            if cfg.out_max_q15 <= cfg.out_min_q15
                rc = K.ERR_INVALID_LIMIT; return;
            end
            if cfg.i_max_q15 <= cfg.i_min_q15
                rc = K.ERR_INVALID_LIMIT; return;
            end
            if cfg.aw_mode > o.AW_BACK_CALC
                rc = K.ERR_INVALID_PARAM; return;
            end
            % Back-calculation coefficient is 2^-bc_shift; shift 0 would feed
            % the full saturation error back in one sample and ring.
            if cfg.aw_mode == o.AW_BACK_CALC && ...
               (cfg.bc_shift == 0 || cfg.bc_shift > 15)
                rc = K.ERR_INVALID_PARAM; return;
            end
            if cfg.lpf_shift > 15
                rc = K.ERR_INVALID_PARAM; return;
            end
            if cfg.direction > o.REVERSE
                rc = K.ERR_INVALID_PARAM; return;
            end
            if cfg.mode > o.MODE_AUTOMATIC
                rc = K.ERR_INVALID_MODE; return;
            end
            % Separation must sit above the deadband, otherwise the two
            % windows overlap and the integrator can never run at all.
            if cfg.separation_q15 ~= 0 && cfg.separation_q15 <= cfg.deadband_q15
                rc = K.ERR_INVALID_PARAM; return;
            end

            o.magic = 0;                       % invalid until fully built
            [rc, ci, cb, ca] = simlab.PIDq.recompute(cfg.dt_us, cfg.tf_us, ...
                cfg.ki_q16, cfg.kd_q16);
            if rc ~= K.OK
                return;
            end
            o.ci_q30 = ci; o.cb_q16 = cb; o.ca_q30 = ca;

            o.kp_q16 = cfg.kp_q16;
            o.dt_us = cfg.dt_us;
            o.tf_us = cfg.tf_us;
            o.out_min_q30 = simlab.PIDq.q15ToQ30(cfg.out_min_q15);
            o.out_max_q30 = simlab.PIDq.q15ToQ30(cfg.out_max_q15);
            o.i_min_q30 = simlab.PIDq.q15ToQ30(cfg.i_min_q15);
            o.i_max_q30 = simlab.PIDq.q15ToQ30(cfg.i_max_q15);
            o.deadband_q15 = cfg.deadband_q15;
            o.separation_q15 = cfg.separation_q15;
            o.aw_mode = cfg.aw_mode;
            o.bc_shift = cfg.bc_shift;
            o.lpf_shift = cfg.lpf_shift;
            o.direction = cfg.direction;
            o.mode = cfg.mode;

            o.integ_q30 = 0; o.d_q30 = 0;
            o.meas_filt_q30 = 0; o.meas_prev_q30 = 0; o.out_q30 = 0;
            o.setpoint_q15 = 0; o.manual_q15 = 0; o.out_q15 = 0;
            o.flags = 0;

            o.magic = o.MAGIC;
            rc = K.OK;
        end

        function rc = deinit(o)
            o.magic = 0;
            rc = pidx.Const.OK;
        end

        function rc = reset(o)
            K = pidx.Const;
            if ~o.isValid()
                rc = K.ERR_NOT_INIT; return;
            end
            o.integ_q30 = 0; o.d_q30 = 0;
            o.meas_filt_q30 = 0; o.meas_prev_q30 = 0;
            o.out_q30 = 0; o.out_q15 = 0; o.flags = 0;
            rc = K.OK;
        end

        function u = update(o, measurement_q15)
            % One control step at the configured sample rate.
            %
            % Stage order matches PIDq_Update exactly and must not be tidied:
            % back-calculation runs in the SAME sample as the saturation it
            % corrects, and the conditional-integration test reads LAST
            % sample's saturation flag, not this one's.
            if ~o.isValid()
                u = int16(0);
                return;
            end

            % ---- input filter ----
            x_q30 = simlab.PIDq.q15ToQ30(measurement_q15);
            if bitand(o.flags, o.FLAG_PRIMED) == 0
                % First sample: seed the filter and the derivative history
                % with the current measurement. Without this the first
                % derivative sees a step from zero and kicks the output to a
                % limit.
                o.meas_filt_q30 = x_q30;
                o.meas_prev_q30 = x_q30;
                o.flags = bitor(o.flags, o.FLAG_PRIMED);
            elseif o.lpf_shift ~= 0
                % EMA in the WIDE domain: y += (x - y) >> shift.
                %
                % The state must be Q30, not Q15. At Q15 the increment
                % (x-y)>>shift truncates to zero as soon as |x-y| < 2^shift,
                % so the filter output sticks one dead-band short of the
                % input and never converges.
                o.meas_filt_q30 = simlab.PIDq.addSat(o.meas_filt_q30, ...
                    simlab.PIDq.ashift( ...
                        simlab.PIDq.wrap32(x_q30 - o.meas_filt_q30), ...
                        o.lpf_shift));
            else
                o.meas_filt_q30 = x_q30;
            end

            % ---- error ----
            e_q30 = simlab.PIDq.wrap32( ...
                simlab.PIDq.q15ToQ30(o.setpoint_q15) - o.meas_filt_q30);
            if o.direction == o.REVERSE
                e_q30 = simlab.PIDq.wrap32(-e_q30);
            end
            e_q15 = simlab.PIDq.ashift(e_q30, o.FRAC - o.SIG_FRAC);
            abs_e_q15 = abs(e_q15);

            % ---- manual ----
            if o.mode == o.MODE_MANUAL
                % Track the manual output so a later switch to automatic is
                % bumpless, and keep the derivative history current so it
                % does not see a jump.
                man_q30 = simlab.PIDq.clamp32( ...
                    simlab.PIDq.q15ToQ30(o.manual_q15), ...
                    o.out_min_q30, o.out_max_q30);
                o.out_q30 = man_q30;
                o.out_q15 = simlab.PIDq.q30ToQ15(man_q30);
                o.meas_prev_q30 = o.meas_filt_q30;
                o.d_q30 = 0;
                u = o.out_q15;
                return;
            end

            % ---- proportional ----
            % P = Kp*e. Kp is Q16.16 and e is Q30, so the product is Q46;
            % shifting by the gain's 16 fractional bits brings it to Q30.
            p_q30 = simlab.PIDq.mulShift(o.kp_q16, e_q30, o.GAIN_FRAC);

            % ---- derivative on measurement ----
            % D_k = ca*D_{k-1} - cb*(x_k - x_{k-1}), with ca = Tf/(Tf+dt) and
            % cb = Kd/(Tf+dt). Differentiating the measurement rather than the
            % error removes the impulse a setpoint step would produce. When
            % Tf = 0 the pole ca is 0 and this collapses to the unfiltered
            % backward difference.
            if o.cb_q16 ~= 0
                dx_q30 = simlab.PIDq.wrap32(o.meas_filt_q30 - o.meas_prev_q30);
                o.d_q30 = simlab.PIDq.wrap32( ...
                    simlab.PIDq.mulShift(o.ca_q30, o.d_q30, o.FRAC) - ...
                    simlab.PIDq.mulShift(o.cb_q16, dx_q30, o.GAIN_FRAC));
                if o.direction == o.REVERSE
                    % Reverse action already flipped the error; the
                    % measurement difference has to follow, otherwise D
                    % fights P.
                    o.d_q30 = simlab.PIDq.wrap32(-o.d_q30);
                end
            else
                o.d_q30 = 0;
            end
            o.meas_prev_q30 = o.meas_filt_q30;

            % ---- decide whether to integrate this sample ----
            integrate = (o.ci_q30 ~= 0);

            if integrate && o.deadband_q15 ~= 0 && ...
               abs_e_q15 < o.deadband_q15
                % Inside the deadband: stop integrating so sensor noise
                % around the setpoint does not slowly walk the actuator.
                integrate = false;
            end
            if integrate && o.separation_q15 ~= 0 && ...
               abs_e_q15 > o.separation_q15
                % Integral separation: far from the setpoint the integrator
                % only accumulates windup, so run pure PD until it returns.
                integrate = false;
            end
            if integrate && o.aw_mode == o.AW_CONDITIONAL && ...
               bitand(o.flags, o.FLAG_SATURATED) ~= 0
                % Conditional integration: keep integrating only if the error
                % would pull the output back out of the limit it is stuck
                % against. Note this reads LAST sample's saturation flag, as
                % the C does - a one-cycle-late test, which is the standard
                % way this strategy quietly degrades.
                at_high = (o.out_q30 >= o.out_max_q30);
                if (at_high && e_q30 > 0) || (~at_high && e_q30 < 0)
                    integrate = false;
                end
            end

            if integrate
                % I += Ki*dt*e, accumulated in OUTPUT units at Q30. Holding
                % the integrator in output units rather than accumulating raw
                % error and multiplying by Ki later is what makes a runtime
                % gain change bumpless.
                o.integ_q30 = simlab.PIDq.addSat(o.integ_q30, ...
                    simlab.PIDq.mulShift(o.ci_q30, e_q30, o.FRAC));
            end

            if o.aw_mode == o.AW_CLAMP
                o.integ_q30 = simlab.PIDq.clamp32(o.integ_q30, ...
                    o.i_min_q30, o.i_max_q30);
            end

            % ---- sum and saturate ----
            u_q30 = simlab.PIDq.addSat( ...
                simlab.PIDq.addSat(p_q30, o.integ_q30), o.d_q30);
            u_sat_q30 = simlab.PIDq.clamp32(u_q30, o.out_min_q30, o.out_max_q30);

            if u_sat_q30 ~= u_q30
                o.flags = bitor(o.flags, o.FLAG_SATURATED);
                if o.aw_mode == o.AW_BACK_CALC
                    % Back-calculation, applied in the same sample as the
                    % saturation: I += (u_sat - u) * 2^-bc_shift. Restricting
                    % the coefficient to a power of two keeps this a shift, so
                    % the hot path stays division-free.
                    o.integ_q30 = simlab.PIDq.addSat(o.integ_q30, ...
                        simlab.PIDq.ashift( ...
                            simlab.PIDq.wrap32(u_sat_q30 - u_q30), ...
                            o.bc_shift));
                end
            else
                o.flags = bitand(o.flags, ...
                    simlab.PIDq.notBits8(o.FLAG_SATURATED));
            end

            o.out_q30 = u_sat_q30;
            o.out_q15 = simlab.PIDq.q30ToQ15(u_sat_q30);
            u = o.out_q15;
        end

        function rc = setGains(o, kp_q16, ki_q16, kd_q16)
            % Bumpless: the integrator holds output units, so it is not
            % rescaled here - that is exactly what makes the change bumpless.
            K = pidx.Const;
            if ~o.isValid()
                rc = K.ERR_NOT_INIT; return;
            end
            if kp_q16 < 0 || ki_q16 < 0 || kd_q16 < 0
                rc = K.ERR_INVALID_GAIN; return;
            end
            % Snapshot and roll back: if the new gains do not fit their
            % format, a half-applied tuning is worse than a rejected one.
            save_ci = o.ci_q30;
            save_cb = o.cb_q16;
            save_ca = o.ca_q30;

            [rc, ci, cb, ca] = simlab.PIDq.recompute(o.dt_us, o.tf_us, ...
                ki_q16, kd_q16);
            if rc ~= K.OK
                o.ci_q30 = save_ci; o.cb_q16 = save_cb; o.ca_q30 = save_ca;
                return;
            end
            o.ci_q30 = ci; o.cb_q16 = cb; o.ca_q30 = ca;
            o.kp_q16 = kp_q16;
            rc = K.OK;
        end

        function rc = setSetpoint(o, sp_q15)
            if ~o.isValid(), rc = pidx.Const.ERR_NOT_INIT; return; end
            o.setpoint_q15 = sp_q15;
            rc = pidx.Const.OK;
        end

        function v = getSetpoint(o), v = o.setpoint_q15; end

        function rc = setMode(o, mode)
            K = pidx.Const;
            if ~o.isValid(), rc = K.ERR_NOT_INIT; return; end
            if mode > o.MODE_AUTOMATIC, rc = K.ERR_INVALID_MODE; return; end

            if o.mode == o.MODE_MANUAL && mode == o.MODE_AUTOMATIC
                % Bumpless manual -> automatic. The next automatic output
                % will be P + I + D, so choosing I = u_manual - P - D makes
                % that sum exactly u_manual and the actuator does not step.
                % D is zero because manual mode kept the derivative history
                % aligned with the measurement.
                e_q30 = simlab.PIDq.wrap32( ...
                    simlab.PIDq.q15ToQ30(o.setpoint_q15) - o.meas_filt_q30);
                if o.direction == o.REVERSE
                    e_q30 = simlab.PIDq.wrap32(-e_q30);
                end
                p_q30 = simlab.PIDq.mulShift(o.kp_q16, e_q30, o.GAIN_FRAC);
                o.integ_q30 = simlab.PIDq.clamp32(simlab.PIDq.wrap32( ...
                    simlab.PIDq.q15ToQ30(o.manual_q15) - p_q30 - o.d_q30), ...
                    o.i_min_q30, o.i_max_q30);
            end
            o.mode = mode;
            rc = K.OK;
        end

        function m = getMode(o), m = o.mode; end

        function rc = setManualOutput(o, u_q15)
            if ~o.isValid(), rc = pidx.Const.ERR_NOT_INIT; return; end
            o.manual_q15 = u_q15;
            rc = pidx.Const.OK;
        end

        function v = getManualOutput(o), v = o.manual_q15; end

        function rc = setOutputLimits(o, min_q15, max_q15)
            K = pidx.Const;
            if ~o.isValid(), rc = K.ERR_NOT_INIT; return; end
            if max_q15 <= min_q15, rc = K.ERR_INVALID_LIMIT; return; end
            o.out_min_q30 = simlab.PIDq.q15ToQ30(min_q15);
            o.out_max_q30 = simlab.PIDq.q15ToQ30(max_q15);
            o.integ_q30 = simlab.PIDq.clamp32(o.integ_q30, ...
                o.i_min_q30, o.i_max_q30);
            rc = K.OK;
        end

        function v = getOutput(o), v = o.out_q15; end
        function v = getIntegral(o), v = simlab.PIDq.q30ToQ15(o.integ_q30); end
        function v = isSaturated(o)
            v = bitand(o.flags, o.FLAG_SATURATED) ~= 0;
        end
        function ok = isValid(o)
            ok = (o.magic == o.MAGIC);
        end
    end

    methods (Static)
        function cfg = configDefault()
            % The same safe defaults as PIDq_ConfigDefault(): unity
            % proportional gain, no integral or derivative action, full-scale
            % limits, clamp anti-windup, DIRECT, AUTOMATIC.
            o = simlab.PIDq;
            cfg = struct();
            cfg.kp_q16 = 65536;              % 1.0
            cfg.ki_q16 = 0;
            cfg.kd_q16 = 0;
            cfg.dt_us = 1000;                % 1 kHz
            cfg.tf_us = 0;
            cfg.out_min_q15 = int16(o.QMIN);
            cfg.out_max_q15 = int16(o.QMAX);
            cfg.i_min_q15 = int16(o.QMIN);
            cfg.i_max_q15 = int16(o.QMAX);
            cfg.deadband_q15 = 0;
            cfg.separation_q15 = 0;
            cfg.aw_mode = o.AW_CLAMP;
            cfg.bc_shift = 4;
            cfg.lpf_shift = 0;
            cfg.direction = o.DIRECT;
            cfg.mode = o.MODE_AUTOMATIC;
        end

        function q = fToQ15(x)
            % Round-to-nearest, away from zero on both sides - the same
            % expression as the PIDQ_F_TO_Q15 macro, so a literal converted
            % here and one converted by the compiler agree.
            q = int16(roundAway(x * 32768.0));
        end

        function q = fToQ16(x)
            q = int32(roundAway(x * 65536.0));
        end

        function f = q15ToF(q)
            f = double(q) / 32768.0;
        end

        function f = q16ToF(q)
            f = double(q) / 65536.0;
        end

        function v = q15ToQ30(v_q15)
            % Promote a Q15 signal into the wide Q30 domain. Always exact.
            v = double(v_q15) * 32768;      % 2^(FRAC - SIG_FRAC)
        end

        function r = q30ToQ15(v_q30)
            % Saturating conversion from Q30 to a Q15 signal, with
            % round-to-nearest.
            %
            % A plain arithmetic shift truncates toward negative infinity,
            % which biases the output down by up to one LSB on every sample.
            % In a loop that closes around this output the bias behaves like a
            % small constant disturbance, so the rounding term is not
            % cosmetic.
            half = 2^(30 - 15 - 1);
            if v_q30 >= 0
                if v_q30 > 2147483647 - half
                    r = int16(32767);
                    return;
                end
                r32 = bitshift(v_q30 + half, -15);
            else
                if v_q30 < -2147483648 + half
                    r = int16(-32768);
                    return;
                end
                r32 = bitshift(v_q30 - half, -15);
            end
            if r32 > 32767, r32 = 32767; end
            if r32 < -32768, r32 = -32768; end
            r = int16(r32);
        end

        function y = ashift(v, n)
            % Arithmetic right shift: floor(v / 2^n), which is what C's >>
            % does on a negative signed value on every compiler this library
            % targets. PIDq_SelfTest() verifies the assumption on the target;
            % floor() is the assumption stated as arithmetic.
            if n <= 0
                y = v;
            else
                y = floor(v / 2^n);
            end
        end

        function y = wrap32(v)
            % Two's-complement wrap into int32.
            %
            % MATLAB's int32 class SATURATES instead of wrapping, and it does
            % so silently - which would turn a real overflow bug into a
            % plausible-looking trace. State is therefore held in doubles and
            % wrapped explicitly here, so this port overflows exactly where
            % the C does.
            y = v;
            if y >= 2147483648 || y <= -2147483649
                y = mod(y + 2147483648, 4294967296) - 2147483648;
            end
        end

        function y = mulShift(a, b, shift)
            % Multiply two fixed-point values and shift back, in 64-bit, with
            % saturation rather than wrap: a wrapped controller output flips
            % sign, which on a real actuator means full reverse.
            p = a * b;
            p = simlab.PIDq.ashift(p, shift);
            if p > 2147483647, p = 2147483647; end
            if p < -2147483648, p = -2147483648; end
            y = p;
        end

        function y = addSat(a, b)
            s = a + b;
            if s > 2147483647, s = 2147483647; end
            if s < -2147483648, s = -2147483648; end
            y = s;
        end

        function y = clamp32(v, lo, hi)
            if v < lo
                y = lo;
            elseif v > hi
                y = hi;
            else
                y = v;
            end
        end

        function y = notBits8(x)
            % Bitwise complement within the 8-bit flags word.
            y = 255 - x;
        end

        function [rc, ci, cb, ca] = recompute(dt_us, tf_us, ki_q16, kd_q16)
            % Fold the sample period and filter constant into the update-path
            % coefficients. The only divisions in the whole controller.
            %
            %   ci = Ki*dt        [Q30]   = ki_q16 * dt_us * 2^14 / 1e6
            %   cb = Kd/(Tf+dt)   [Q16]   = kd_q16 * 1e6 / (tf_us + dt_us)
            %   ca = Tf/(Tf+dt)   [Q30]
            %
            % Times arrive in microseconds. The numerator is built in 64 bits
            % so the 2^14 scaling is applied BEFORE the division and no
            % precision is thrown away first. Every product below stays inside
            % 2^53, so a double represents it exactly.
            K = pidx.Const;
            num = ki_q16 * dt_us;
            num = num * 16384;                       % 2^14
            num = truncDiv(num, 1000000);
            if num > 2147483647 || num < -2147483647
                rc = K.ERR_INVALID_GAIN; ci = 0; cb = 0; ca = 0; return;
            end
            ci = num;

            den = tf_us + dt_us;                     % > 0, dt_us validated
            num = kd_q16 * 1000000;
            num = truncDiv(num, den);
            if num > 2147483647 || num < -2147483647
                % Kd huge relative to Tf+dt: the derivative would dominate
                % entirely. Refuse rather than silently saturate every sample.
                rc = K.ERR_INVALID_GAIN; ci = 0; cb = 0; ca = 0; return;
            end
            cb = num;

            num = tf_us * 1073741824;                % 2^30
            ca = truncDiv(num, den);
            rc = K.OK;
        end
    end
end

function y = roundAway(x)
% Round half away from zero, matching the C macros' +/- 0.5 before truncation.
% MATLAB's round() already does this, but stating it as arithmetic keeps the
% correspondence with PIDQ_F_TO_Q15 visible and does not depend on it.
    if x < 0
        y = -floor(-x + 0.5);
    else
        y = floor(x + 0.5);
    end
end

function q = truncDiv(num, den)
% Integer division truncating toward zero, as C's / does for signed operands.
    q = fix(num / den);
end
