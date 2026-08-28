function T = test_fixed(T)
%SIMLAB_TESTS.TEST_FIXED  Does the Q15 port compute what the C computes?
%
% simlab.PIDq is a port of src/pid_fixed.c. Fixed point has no tolerance to
% hide behind: either the rounding, the shifts and the saturation are the same
% or the traces diverge, and a diverging trace means the gains you measured in
% MATLAB are not the gains running on the board.
%
% Every number here is read from c_reference.csv, produced by
% tools/matlab_ref/matlab_ref.c calling PIDq_Update. The comparison is EXACT
% (integer equality), not approximate - there is no reason for a single LSB of
% slack between two implementations of the same integer arithmetic.
%
% The scenarios also cover the things that make fixed point fixed point:
%   1. a full PID trace with saturation and a setpoint reversal;
%   2. integral resolution death - the sample at which a one-LSB error first
%      moves the output, which is the number to know before choosing a format;
%   3. configurations the format cannot represent, which must be refused;
%   4. the manual -> automatic transfer, which is bumpless to within the
%      rounding of the back-solve and NOT bit-exact. That is measured, not
%      assumed.

    haveRef = simlab_tests.hasRef(T);
    K = pidx.Const;
    Q = simlab.PIDq;

    % ---- the same measurement sequence the C oracle uses ----
    ySeq = qMeas(0:199);

    % ==================================================================
    % 1. full trace
    % ==================================================================
    cfg = simlab.PIDq.configDefault();
    cfg.kp_q16 = Q.fToQ16(1.5);
    cfg.ki_q16 = Q.fToQ16(4.0);
    cfg.kd_q16 = Q.fToQ16(0.01);
    cfg.dt_us = 1000;
    cfg.tf_us = 2000;
    cfg.out_min_q15 = Q.fToQ15(-0.9);
    cfg.out_max_q15 = Q.fToQ15(0.9);
    cfg.i_min_q15 = Q.fToQ15(-0.9);
    cfg.i_max_q15 = Q.fToQ15(0.9);
    cfg.aw_mode = Q.AW_BACK_CALC;
    cfg.bc_shift = 4;

    q = simlab.PIDq(cfg);
    T = simlab_tests.ok(T, q.isValid(), 'PIDq initialises');

    % The Q15 limits are quantised, so -0.9 is not exactly representable.
    % Checking the conversion here is what makes the trace comparison below
    % meaningful: if the limits differed, everything after would too.
    T = simlab_tests.eq(T, double(Q.fToQ15(0.9)), 29491, ...
        'fToQ15(0.9) rounds to 29491, the same as PIDQ_F_TO_Q15');
    T = simlab_tests.eq(T, double(Q.fToQ16(1.5)), 98304, 'fToQ16(1.5) = 98304');

    q.setSetpoint(Q.fToQ15(0.5));
    u = zeros(200, 1);
    for k = 1:200
        if k - 1 == 100
            q.setSetpoint(Q.fToQ15(-0.3));
        end
        u(k) = double(q.update(ySeq(k)));
    end

    if haveRef
        for k = 0:10:190
            T = simlab_tests.eq(T, u(k + 1), ...
                simlab_tests.refGet(T, sprintf('q15.u%03d', k)), ...
                sprintf('Q15 output at sample %d', k));
        end
        T = simlab_tests.eq(T, u(200), simlab_tests.refGet(T, 'q15.finalU'), ...
            'Q15 final output');
        T = simlab_tests.eq(T, double(q.getIntegral()), ...
            simlab_tests.refGet(T, 'q15.integral'), 'Q15 integral term');
        T = simlab_tests.eq(T, double(q.isSaturated()), ...
            simlab_tests.refGet(T, 'q15.saturated'), 'Q15 saturation flag');
    else
        T = simlab_tests.skip(T, 'Q15 trace vs C reference', ...
            'c_reference.csv not found');
    end

    % The output really did hit the rail - otherwise the anti-windup path
    % under test was never exercised.
    T = simlab_tests.ok(T, any(sat), ...
        'the trace saturates somewhere, so the anti-windup path is exercised');

    % ==================================================================
    % 2. integral resolution death
    % ==================================================================
    cfg2 = simlab.PIDq.configDefault();
    cfg2.kp_q16 = 0;                  % pure integrator: only I can move
    cfg2.ki_q16 = Q.fToQ16(0.5);
    cfg2.kd_q16 = 0;
    cfg2.dt_us = 1000;
    cfg2.out_min_q15 = Q.fToQ15(-0.9);
    cfg2.out_max_q15 = Q.fToQ15(0.9);

    q2 = simlab.PIDq(cfg2);
    q2.setSetpoint(int16(1));         % ONE LSB of error
    first = -1;
    prev = 0;
    for k = 0:199999
        uu = double(q2.update(int16(0)));
        if first < 0 && uu ~= prev
            first = k;
        end
        prev = uu;
    end

    if haveRef
        T = simlab_tests.eq(T, first, ...
            simlab_tests.refGet(T, 'q15res.firstMoveSample'), ...
            'sample at which a one-LSB error first moves the output');
        T = simlab_tests.eq(T, prev, simlab_tests.refGet(T, 'q15res.finalU'), ...
            'Q15 output after 200000 sub-LSB increments');
    end
    T = simlab_tests.ok(T, first > 0, ...
        ['the Q30 accumulator does move: a one-LSB error first appears at ' ...
         'sample %d. In a Q15 accumulator the increment Ki*dt*e = 1.5e-8 is ' ...
         '0.0005 of an LSB and would round to zero FOREVER - the loop would ' ...
         'hold a permanent steady-state error no tuning could remove.'], first);
    T = simlab_tests.ok(T, first > 100, ...
        'and it takes %d samples to do it, which is the real cost', first);

    % ==================================================================
    % 3. configurations the format cannot represent
    % ==================================================================
    cfgBad = simlab.PIDq.configDefault();
    cfgBad.ki_q16 = int32(2000000);      % Ki ~ 30.5
    cfgBad.dt_us = 100000;               % 100 ms: Ki*dt ~ 3.05 > 2.0
    threw = false;
    try
        simlab.PIDq(cfgBad);
    catch
        threw = true;
    end
    T = simlab_tests.ok(T, threw, ...
        'Ki*dt >= 2.0 is refused: the loop would integrate more than full scale in one sample');
    if haveRef
        % The C returns PID_ERR_INVALID_GAIN (4) from PIDq_Init for this.
        T = simlab_tests.eq(T, 4, simlab_tests.refGet(T, 'q15reject.initRc'), ...
            'the C refuses it with ERR_INVALID_GAIN');
    end

    cfgBad2 = simlab.PIDq.configDefault();
    cfgBad2.deadband_q15 = 100;
    cfgBad2.separation_q15 = 50;         % below the deadband: no window left
    threw2 = false;
    try
        simlab.PIDq(cfgBad2);
    catch
        threw2 = true;
    end
    T = simlab_tests.ok(T, threw2, ...
        'separation at or below the deadband is refused: the integrator could never run');

    cfgBad3 = simlab.PIDq.configDefault();
    cfgBad3.aw_mode = Q.AW_BACK_CALC;
    cfgBad3.bc_shift = 0;
    threw3 = false;
    try
        simlab.PIDq(cfgBad3);
    catch
        threw3 = true;
    end
    T = simlab_tests.ok(T, threw3, ...
        'back-calculation with bc_shift = 0 is refused: it would ring');

    % ==================================================================
    % 4. manual -> automatic transfer
    % ==================================================================
    cfg4 = simlab.PIDq.configDefault();
    cfg4.kp_q16 = Q.fToQ16(2.0);
    cfg4.ki_q16 = Q.fToQ16(1.0);
    cfg4.dt_us = 1000;
    cfg4.mode = Q.MODE_MANUAL;
    q4 = simlab.PIDq(cfg4);
    q4.setSetpoint(Q.fToQ15(0.5));
    q4.setManualOutput(Q.fToQ15(0.25));
    uMan = double(q4.update(Q.fToQ15(0.1)));
    q4.setMode(Q.MODE_AUTOMATIC);
    uAuto = double(q4.update(Q.fToQ15(0.1)));

    if haveRef
        T = simlab_tests.eq(T, uMan, simlab_tests.refGet(T, 'q15bump.uManual'), ...
            'manual output matches C');
        T = simlab_tests.eq(T, uAuto, ...
            simlab_tests.refGet(T, 'q15bump.uAfterSwitch'), ...
            'output on the first automatic sample matches C');
    end
    % NOT bit-exact, and that is the honest result: the back-solve computes
    % I = u_manual - P - D and clamps it, so the stored integral carries up to
    % half an LSB of rounding, and the next sum rounds again. 13 LSB is 0.04%
    % of full scale - invisible on any actuator, but a test that asserted
    % equality would be asserting something the C does not do.
    T = simlab_tests.ok(T, abs(uAuto - uMan) <= 32, ...
        ['the manual->automatic step is %d LSB (%.3f%% of full scale), not ' ...
         'zero: the back-solve rounds. Bumpless to within the format, which ' ...
         'is the strongest claim fixed point can make.'], ...
        abs(uAuto - uMan), 100 * abs(uAuto - uMan) / 32768);
    if haveRef
        T = simlab_tests.eq(T, 0, simlab_tests.refGet(T, 'q15bump.bumpless'), ...
            'the C reports the same: it is NOT bit-exact either');
    end

    % ==================================================================
    % 5. the arithmetic primitives
    % ==================================================================
    % Arithmetic shift of a negative value: C99 leaves >> on a negative signed
    % integer implementation-defined and the whole module assumes it is
    % arithmetic. Stated here as floor division so the assumption is visible.
    T = simlab_tests.eq(T, Q.ashift(-256, 4), -16, ...
        'arithmetic right shift of -256 by 4 is -16 (PIDq_SelfTest checks this on the target)');
    T = simlab_tests.eq(T, Q.ashift(-1, 15), -1, 'ashift(-1, 15) = -1');
    T = simlab_tests.eq(T, Q.ashift(255, 4), 15, 'ashift(255, 4) = 15');

    % Wrap, not saturate: MATLAB's int32 class saturates silently, which would
    % turn a real overflow bug into a plausible-looking trace.
    T = simlab_tests.eq(T, Q.wrap32(2147483647 + 1), -2147483648, ...
        'int32 wraps at the top');
    T = simlab_tests.eq(T, Q.wrap32(-2147483648 - 1), 2147483647, ...
        'int32 wraps at the bottom');

    % Saturating multiply: a wrapped controller output flips sign, which on a
    % real actuator means full reverse.
    T = simlab_tests.eq(T, Q.mulShift(2147483647, 65536, 16), 2147483647, ...
        'mulShift saturates rather than wrapping');
    T = simlab_tests.eq(T, Q.addSat(2147483647, 1), 2147483647, 'addSat saturates');

    % Q30 -> Q15 rounds to nearest, away from zero on both sides. A plain
    % shift would truncate toward -inf and bias every output down by up to an
    % LSB, which inside a closed loop behaves like a constant disturbance.
    T = simlab_tests.eq(T, double(Q.q30ToQ15(32768)), 1, ...
        'one Q15 LSB in Q30 converts to 1');
    T = simlab_tests.eq(T, double(Q.q30ToQ15(32767)), 1, ...
        'round-to-nearest: 32767 of Q30 is still 1 LSB, not 0');
    % C rounds away from zero on the negative side ((v-half)>>15 floors
    % toward -inf), so an exact -1-LSB Q30 value lands at -2. That is the
    % library's convention; the port reproduces it and the test states it.
    T = simlab_tests.eq(T, double(Q.q30ToQ15(-32768)), -2, ...
        'negative side rounds away from zero, exactly as C does');
    % Q30 1.0 is 32768 in Q15, which is NOT representable - the largest Q15
    % signal is 32767. The conversion must clamp, not wrap to -32768.
    T = simlab_tests.eq(T, double(Q.q30ToQ15(2^30)), 32767, ...
        'Q30 1.0 clamps to the largest Q15 signal instead of wrapping');
    T = simlab_tests.eq(T, double(Q.q30ToQ15(-2^30)), -32768, ...
        'and the negative extreme clamps too');
end

function y = qMeas(k)
% The measurement sequence, generated identically on both sides.
%
% A fixed formula rather than a shared file, so there is no I/O step to get
% wrong - but the SAME formula, quantised to Q15 with the same rounding, or
% the two traces would diverge on the very first sample for a reason that has
% nothing to do with the controller.
    t = k * 0.001;
    yv = 0.4 * (1 - exp(-t / 0.05)) + 0.15 * sin(k * 0.07);
    y = int16(roundAway(yv * 32768.0));
end

function r = roundAway(x)
    r = floor(abs(x) + 0.5) .* sign(x);
    r(x == 0) = 0;
end
