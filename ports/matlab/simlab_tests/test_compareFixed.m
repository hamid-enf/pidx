function T = test_compareFixed(T)
%SIMLAB_TESTS.TEST_COMPAREFIXED  Does the fixed-point study measure anything?
%
% Three things are checked:
%   1. the Q15 scaling round-trips, because every other number rests on it;
%   2. the study runs end to end on a plant that fits Q15, and the two loops
%      agree to within a few output LSB - which is what "the port is correct"
%      looks like in engineering units;
%   3. the diagnosis detects integral resolution death when it is present.
%
% The plant is chosen to fit Q15 naturally. A heater at 0..300 degC does not,
% and pretending otherwise would measure the clamp rather than the format -
% compareFixed() refuses that case, and the refusal is checked too.

    % ---- 1. scaling round-trips ----
    sy = simlab.Scaling(0, 1);
    for v = [0, 0.25, 0.5, 0.75, 0.999]
        q = sy.toQ15(v);
        T = simlab_tests.ok(T, abs(sy.toEng(q) - v) < sy.resolution(), ...
            'Q15 round trip of %.4f is %.6f (one LSB is %.2e)', ...
            v, sy.toEng(q), sy.resolution());
    end
    T = simlab_tests.eq(T, double(sy.toQ15(0)), 0, '0 maps to code 0');
    T = simlab_tests.eq(T, double(sy.toQ15(1)), 32767, 'full scale maps to 32767');
    T = simlab_tests.eq(T, double(sy.toQ15(5)), 32767, 'and above full scale CLAMPS, not wraps');

    % A bipolar range uses the whole signed domain.
    sb = simlab.Scaling(-1, 1);
    T = simlab_tests.eq(T, double(sb.toQ15(-1)), -32768, 'bipolar: -1 maps to -32768');
    T = simlab_tests.near(T, sb.resolution(), 2 / 32768, 1e-9, ...
        'bipolar resolution is twice the unipolar one, as it must be');

    % A unipolar range does not waste half the codes.
    T = simlab_tests.near(T, sy.resolution(), 1 / 32767, 1e-9, ...
        'a 0..1 signal uses the positive half only, so its resolution is 1/32767');

    % ---- 2. the gains carry the span ratio ----
    su = simlab.Scaling(0, 1);
    sy2 = simlab.Scaling(0, 10);      % measurement spans 10x the output
    g = simlab.Scaling.fromFloatGains(2.0, 0.5, 0.01, sy2, su);
    T = simlab_tests.near(T, g.ratio, 0.1, 1e-12, 'the span ratio is su/sy');
    T = simlab_tests.near(T, simlab.PIDq.q16ToF(g.kp_q16), 0.2, 1e-4, ...
        'Kp is scaled by the ratio: 2.0 * 0.1 = 0.2');
    T = simlab_tests.near(T, simlab.PIDq.q16ToF(g.ki_q16), 0.05, 1e-4, ...
        'Ki is scaled by the same ratio');

    % ---- 3. the study runs, on a plant that fits ----
    plant = simlab.Plant('fopdt', 'name', 'normalised', 'k', 1.0, ...
        'tau', 0.5, 'l', 0.05);
    plant.setActuatorLimits(-1, 1);
    plant.setAdcBits(12, -1, 1);
    dt = 0.002;

    r = simlab.compareFixed(plant, struct('kp', 3.0, 'ki', 6.0, 'kd', 0.02), ...
        struct('dt', dt, 'tf', 0.01, 'verbose', false, ...
               'scenario', simlab.Scenario.presets('stepResponse', ...
                   'sp', 0.5, 'tEnd', 4)));

    T = simlab_tests.ok(T, ~isempty(r.fixedResult), 'the fixed-point loop ran');
    T = simlab_tests.ok(T, ~isempty(r.floatResult), 'and so did the float one');
    T = simlab_tests.ok(T, logical(r.fixedResult.metrics.stable), ...
        'the Q15 loop is stable');
    T = simlab_tests.ok(T, logical(r.floatResult.metrics.stable), ...
        'the float loop is stable');

    % The two must agree. Not exactly - the output is quantised - but within a
    % handful of LSB. Anything more means the conversion, not the arithmetic.
    T = simlab_tests.ok(T, r.diffLsb < 8, ...
        'the two loops agree to within %.2f output LSB', r.diffLsb);
    T = simlab_tests.ok(T, abs(r.ssErrorFixed) < 0.02, ...
        'the Q15 steady-state error is %.5f', r.ssErrorFixed);
    T = simlab_tests.ok(T, abs(r.iaeFixed - r.iaeFloat) / r.iaeFloat < 0.10, ...
        'IAE differs by %.1f%% (%.5g against %.5g)', ...
        100 * abs(r.iaeFixed - r.iaeFloat) / r.iaeFloat, r.iaeFixed, r.iaeFloat);

    % ---- 4. the diagnosis names the three failure modes ----
    T = simlab_tests.ok(T, ~isempty(strfind(lower(r.diagnosis), 'steady-state')), ...
        'the diagnosis reports on steady-state error');
    T = simlab_tests.ok(T, ~isempty(strfind(lower(r.diagnosis), 'integral')), ...
        'and on integral resolution');
    T = simlab_tests.ok(T, ~isempty(strfind(lower(r.diagnosis), 'lsb')), ...
        'and states the agreement in output LSB');

    % ---- 5. a plant that does not fit Q15 is refused, not scaled silently ----
    heater = simlab.Plant.presets('heater');      % 0..300 degC
    threw = false;
    try
        simlab.compareFixed(heater, struct('kp', 3, 'ki', 0.08, 'kd', 0), ...
            struct('dt', 0.1, 'measRange', [-1 1], 'verbose', false));
    catch err
        threw = ~isempty(strfind(err.identifier, 'spRange')); %#ok<STREMP>
    end
    T = simlab_tests.ok(T, threw, ...
        'a setpoint outside the declared Q15 range is refused rather than clamped');

    % And with a range that fits, the same plant is fine.
    rH = simlab.compareFixed(heater, struct('kp', 3, 'ki', 0.08, 'kd', 0), ...
        struct('dt', 0.5, 'measRange', [0 300], 'outRange', [0 100], ...
               'tf', 1.0, 'verbose', false, ...
               'scenario', simlab.Scenario.presets('stepResponse', ...
                   'sp', 150, 'tEnd', 600)));
    T = simlab_tests.ok(T, ~isempty(rH.gainsQ16), ...
        'a 0..300 degC plant works when the range is declared: Kp_q16 = %d', ...
        rH.gainsQ16.kp_q16);
    T = simlab_tests.near(T, rH.gainsQ16.ratio, 100 / 300, 1e-9, ...
        'and the span ratio is 100/300');
end
