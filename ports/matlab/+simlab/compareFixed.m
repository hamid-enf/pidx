function r = compareFixed(plant, gains, opt)
%SIMLAB.COMPAREFIXED  What does Q15 cost you, on your plant, with your gains?
%
%   R = SIMLAB.COMPAREFIXED(PLANT, GAINS)
%   R = SIMLAB.COMPAREFIXED(PLANT, GAINS, 'measRange', [0 300], ...
%                           'outRange', [0 100], 'dt', 0.001)
%
% GAINS is a struct with fields kp, ki, kd (engineering units, the same ones
% the floating-point controller uses) and optionally tf.
%
% The comparison is run through simlab.Sim twice - once with pidx.PID, once
% with simlab.PIDq wrapped in the Q15 scaling - on the SAME plant and the
% SAME scenario. Same random seed, so the noise realisation is identical and
% any difference in the traces is the number format and nothing else.
%
% WHAT IT REPORTS
%   r.floatResult / r.fixedResult   the two full simulation logs
%   r.diff                          max |u_float - u_fixed|, in engineering
%                                   units AND in output LSBs
%   r.ssErrorFloat / .ssErrorFixed  steady-state error of each. This is where
%                                   integral resolution death shows up: if the
%                                   fixed-point loop has a steady-state error
%                                   the floating-point one does not, the
%                                   integrator is not moving.
%   r.iaeFloat / .iaeFixed          integral of |e| for each
%   r.tvFloat / .tvFixed            total variation. Quantisation makes the
%                                   output stair-step; TV is how you see it.
%   r.scaling                       the resolution actually used, as a sentence
%   r.diagnosis                     plain language about what to do
%
% WHY THE SCALING IS AN INPUT AND NOT GUESSED
%   A plant whose measurement spans 0..300 degC and whose output spans 0..100%
%   needs a different map on each side, and the gains carry the ratio between
%   them. Guessing that ratio would tune a different loop from the one you
%   designed, silently.

    if nargin < 3, opt = struct(); end
    o = fillOpt(opt, 'measRange', []);
    o = fillOpt(opt, 'outRange', []);
    o = fillOpt(opt, 'dt', []);
    o = fillOpt(opt, 'scenario', []);
    o = fillOpt(opt, 'tf', 0);
    o = fillOpt(opt, 'bcShift', 4);
    o = fillOpt(opt, 'lpfShift', 0);
    o = fillOpt(opt, 'verbose', true);

    % ---- ranges: from the plant where it knows, otherwise asked for ----
    if isempty(o.outRange)
        [lo, hi] = plant.actuatorLimits();
        if isfinite(lo) && isfinite(hi)
            o.outRange = [lo, hi];
        else
            error('simlab:compareFixed:outRange', ...
                  ['the plant has no actuator limits, so the output scaling ' ...
                   'cannot be inferred. Pass ''outRange'', [lo hi].']);
        end
    end
    if isempty(o.measRange)
        % The measurement range is a property of the SENSOR, not the process,
        % and the plant does not always carry one. The ADC range is the
        % honest default when it is configured, because that is the range the
        % converter actually spans.
        bits = plant.sensorParam('bits');
        if bits > 0
            o.measRange = [plant.sensorParam('qmin'), ...
                           plant.sensorParam('qmax')];
        else
            error('simlab:compareFixed:measRange', ...
                  ['the plant has no ADC range configured, so the ' ...
                   'measurement scaling cannot be inferred. Pass ' ...
                   '''measRange'', [lo hi], or call setAdcBits on the plant.']);
        end
    end

    sy = simlab.Scaling(o.measRange(1), o.measRange(2), 'measurement');
    su = simlab.Scaling(o.outRange(1), o.outRange(2), 'output');

    dt = o.dt;
    if isempty(dt), dt = plant.dt; end
    if ~(dt > 0), dt = 0.001; end

    sc = o.scenario;
    if isempty(sc)
        sc = simlab.Scenario.presets('stepResponse', ...
            'sp', 0.5 * (o.measRange(1) + o.measRange(2)));
    end

    % ---- the floating-point loop ----
    cfgF = pidx.config('kp', gains.kp, 'ki', gains.ki, 'kd', gains.kd, ...
        'dt', dt);
    cfgF.limits.use_output_limits = true;
    cfgF.limits.output_min = o.outRange(1);
    cfgF.limits.output_max = o.outRange(2);
    if o.tf > 0, cfgF.filter.tf = o.tf; end
    ctrlF = pidx.PID(cfgF);

    rF = simlab.Sim(plant, ctrlF, sc).run();

    % ---- the fixed-point loop, on the same plant and the same noise ----
    gq = simlab.Scaling.fromFloatGains(gains.kp, gains.ki, gains.kd, sy, su);

    cfgQ = simlab.PIDq.configDefault();
    cfgQ.kp_q16 = gq.kp_q16;
    cfgQ.ki_q16 = gq.ki_q16;
    cfgQ.kd_q16 = gq.kd_q16;
    cfgQ.dt_us = uint32(round(dt * 1e6));
    cfgQ.tf_us = uint32(round(o.tf * 1e6));
    cfgQ.out_min_q15 = su.toQ15(o.outRange(1));
    cfgQ.out_max_q15 = su.toQ15(o.outRange(2));
    cfgQ.i_min_q15 = su.toQ15(o.outRange(1));
    cfgQ.i_max_q15 = su.toQ15(o.outRange(2));
    cfgQ.aw_mode = simlab.PIDq.AW_BACK_CALC;
    cfgQ.bc_shift = o.bcShift;
    cfgQ.lpf_shift = o.lpfShift;

    % The setpoint has to be reachable in Q15 or the comparison is measuring
    % the clamp rather than the controller.
    spEng = scSetpoint(sc);
    if ~sy.fitsQ15(spEng)
        error('simlab:compareFixed:spRange', ...
              ['the scenario setpoint %.6g is outside the measurement range ' ...
               '[%.6g, %.6g] - it would clamp to full scale and the ' ...
               'comparison would measure the clamp, not the format.'], ...
              spEng, o.measRange(1), o.measRange(2));
    end

    wrapper = simlab.Q15Loop(simlab.PIDq(cfgQ), sy, su, spEng);
    rQ = simlab.Sim(plant, wrapper, sc).run();

    % ---- the comparison ----
    r = struct();
    r.floatResult = rF;
    r.fixedResult = rQ;
    r.scaling = simlab.Scaling.describe(sy, su);
    r.gainsQ16 = gq;
    r.sy = sy;
    r.su = su;
    r.dt = dt;

    du = abs(rF.u(:) - rQ.u(:));
    r.diffMax = max(du);
    r.diffLsb = r.diffMax / su.resolution();
    r.diffRms = sqrt(mean(du.^2));

    r.ssErrorFloat = rF.metrics.ssError;
    r.ssErrorFixed = rQ.metrics.ssError;
    r.iaeFloat = rF.metrics.iae;
    r.iaeFixed = rQ.metrics.iae;
    r.tvFloat = rF.metrics.tv;
    r.tvFixed = rQ.metrics.tv;
    r.overshootFloat = rF.metrics.overshoot;
    r.overshootFixed = rQ.metrics.overshoot;

    r.diagnosis = diagnose(r, gains, sy, su, dt);

    if o.verbose
        printReport(r);
    end
end

% ---------------------------------------------------------------------------

function s = diagnose(r, gains, sy, su, dt)
% Plain language, because "IAE 1.204 vs 1.231" does not tell anyone what to
% do. The three things that actually go wrong in a fixed-point port are
% checked explicitly, in the order they bite.
    bits = {};

    % 1. Integral resolution death. The increment per sample is Ki*dt*e; if
    %    that is far below one Q30 LSB of the OUTPUT... it never is, because
    %    Q30 is 32768x finer than the output LSB. The real question is how
    %    long it takes to accumulate to one output LSB.
    if gains.ki > 0
        incPerLsb = gains.ki * dt * sy.resolution() / su.resolution();
        % In Q30 output units, one output LSB is 2^15. The increment in those
        % units is incPerLsb * 2^15.
        incQ30 = incPerLsb * 32768;
        if incQ30 < 1
            n = ceil(1 / incQ30);
            bits{end + 1} = sprintf( ...
                ['INTEGRAL RESOLUTION: one measurement LSB of error moves the ' ...
                 'integrator by %.3g of an output LSB, so it takes %d samples ' ...
                 '(%.3g s) to accumulate one. A Q15 accumulator would round ' ...
                 'this to zero and never move.'], incPerLsb, n, n * dt);
        else
            bits{end + 1} = sprintf( ...
                ['integral resolution is fine: one measurement LSB moves the ' ...
                 'integrator by %.3g output LSB per sample.'], incPerLsb);
        end
    end

    % 2. Steady-state error the float loop does not have.
    if abs(r.ssErrorFixed) > 2 * max(abs(r.ssErrorFloat), sy.resolution())
        bits{end + 1} = sprintf( ...
            ['STEADY-STATE ERROR: %.6g in Q15 against %.6g in float. The ' ...
             'integrator is not reaching the value it needs - check the ' ...
             'integral limits and the deadband.'], ...
            r.ssErrorFixed, r.ssErrorFloat);
    else
        bits{end + 1} = sprintf( ...
            'steady-state error matches the float loop (%.6g vs %.6g).', ...
            r.ssErrorFixed, r.ssErrorFloat);
    end

    % 3. Output chatter from quantisation.
    if r.tvFixed > 1.5 * max(r.tvFloat, 1e-12)
        bits{end + 1} = sprintf( ...
            ['OUTPUT CHATTER: TV(u) is %.4g in Q15 against %.4g in float. ' ...
             'The derivative is amplifying converter steps - raise tf, or ' ...
             'set lpf_shift on the Q15 controller.'], r.tvFixed, r.tvFloat);
    else
        bits{end + 1} = sprintf( ...
            'output activity matches the float loop (TV %.4g vs %.4g).', ...
            r.tvFixed, r.tvFloat);
    end

    % 4. The overall gap, in the units that matter: output LSBs.
    if r.diffLsb > 4
        bits{end + 1} = sprintf( ...
            ['the two loops differ by up to %.1f output LSB. Above a few LSB ' ...
             'this is usually the gain conversion, not the arithmetic - ' ...
             'check the span ratio %.4g.'], r.diffLsb, r.gainsQ16.ratio);
    else
        bits{end + 1} = sprintf( ...
            'the two loops agree to within %.2f output LSB.', r.diffLsb);
    end

    s = strjoin(bits, sprintf('\n'));
    %#ok<NASGU>
    su;
end

function printReport(r)
    fprintf('\n================ float vs Q15 ================\n');
    fprintf('  %s\n', r.scaling);
    fprintf('  gains Q16.16: Kp = %d, Ki = %d, Kd = %d  (span ratio %.6g)\n', ...
        r.gainsQ16.kp_q16, r.gainsQ16.ki_q16, r.gainsQ16.kd_q16, ...
        r.gainsQ16.ratio);
    fprintf('\n  %-22s %14s %14s\n', '', 'float', 'Q15');
    fprintf('  %-22s %14.6g %14.6g\n', 'steady-state error', ...
        r.ssErrorFloat, r.ssErrorFixed);
    fprintf('  %-22s %14.6g %14.6g\n', 'IAE', r.iaeFloat, r.iaeFixed);
    fprintf('  %-22s %14.6g %14.6g\n', 'overshoot %', ...
        r.overshootFloat, r.overshootFixed);
    fprintf('  %-22s %14.6g %14.6g\n', 'TV(u)', r.tvFloat, r.tvFixed);
    fprintf('\n  max |u_float - u_Q15| = %.6g  (%.2f output LSB)\n', ...
        r.diffMax, r.diffLsb);
    fprintf('\n%s\n', r.diagnosis);
    fprintf('==============================================\n\n');
end

function sp = scSetpoint(sc)
% The largest setpoint the scenario commands. A scenario is a list of events,
% so this reads them rather than assuming there is one step.
    sp = 0;
    for i = 1:sc.nEvents
        e = sc.getEvent(i);
        if strcmp(e.type, 'setpoint') && ~isempty(e.a)
            if abs(e.a) > abs(sp)
                sp = e.a;
            end
        end
    end
end

function o = fillOpt(opt, name, default)
    if isfield(opt, name) && ~isempty(opt.(name))
        o.(name) = opt.(name);
    else
        o.(name) = default;
    end
end
