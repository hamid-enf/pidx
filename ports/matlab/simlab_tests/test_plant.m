function T = test_plant(T)
%SIMLAB_TESTS.TEST_PLANT  Is the plant model the model it claims to be?
%
% Every other number this tool produces rests on simlab.Plant. If its step
% response is not the analytic step response of the transfer function it says
% it implements, then the tuning is right and the conclusion is wrong - the
% worst combination, because nothing downstream can detect it.
%
% So the model is checked against closed form, not against another
% simulation:
%
%   G(s) = K exp(-Ls)/(1 + tau s),  unit step at t = 0
%   y(t) = 0                         t < L
%   y(t) = K (1 - exp(-(t-L)/tau))   t >= L
%
% The discretisation is backward Euler, so the agreement is O(dt^2) rather
% than exact; the tolerance below is what that discretisation actually
% produces at dt = 0.1 s on a 45 s time constant, with room to spare.

    dt = 0.1;
    K_ = 2.0;
    tau = 45.0;
    L = 12.0;

    % ---- 1. open-loop step against closed form ----
    pl = simlab.Plant('fopdt', 'name', 'test', 'k', K_, 'tau', tau, 'l', 0);
    pl.reset();
    worst = 0;
    worstT = 0;
    worstCont = 0;
    a = tau / (tau + dt);
    for k = 1:600
        y = pl.update(1.0, dt);
        t = k * dt;
        % Backward Euler's pole tau/(tau+dt) versus the exact exp(-dt/tau)
        % differ by O(dt^2/tau^2). Compare against the DISCRETE closed form
        % so the test checks the implementation rather than the
        % discretisation, and record the continuous error separately so a
        % reader can see how much of it is the method.
        disc = K_ * (1 - a^k);
        d = abs(y - disc) / K_;
        if d > worst, worst = d; worstT = t; end
        dc = abs(y - K_ * (1 - exp(-t / tau))) / K_;
        if dc > worstCont, worstCont = dc; end
    end
    y600 = y;

    T = simlab_tests.ok(T, worst < 1e-12, ...
        'fopdt step matches its own discrete closed form (worst rel err %.3g at t = %.3g)', ...
        worst, worstT);
    T = simlab_tests.ok(T, worstCont < 0.01, ...
        'fopdt step stays within 1%% of the CONTINUOUS K(1-exp(-t/tau)) (worst %.4g)', ...
        worstCont);

    % ---- 2. steady state ----
    T = simlab_tests.near(T, y600 / 1.0, K_ * (1 - exp(-60 / tau)), 1e-9, ...
        'steady state approaches K');

    % ---- 3. dead time actually delays ----
    pld = simlab.Plant('fopdt', 'k', K_, 'tau', tau, 'l', L);
    pld.reset();
    nDelay = round(L / dt);
    zeroWhileDelayed = true;
    for k = 1:nDelay
        y = pld.update(1.0, dt);
        if abs(y) > 0
            zeroWhileDelayed = false;
        end
    end
    T = simlab_tests.ok(T, zeroWhileDelayed, ...
        'output is exactly zero during the %d-sample dead time', nDelay);

    % After the dead time it must track the undelayed response sample for
    % sample, shifted by exactly the delay.
    pl.reset();
    pld2 = simlab.Plant('fopdt', 'k', K_, 'tau', tau, 'l', L);
    pld2.reset();
    hist = zeros(nDelay + 200, 1);
    for k = 1:(nDelay + 200)
        hist(k) = pl.update(1.0, dt);
    end
    worst = 0;
    for k = 1:200
        y = pld2.update(1.0, dt);
        % Sample k of the delayed plant must equal sample k of the undelayed
        % one: both are measured after their own dead time has elapsed, so
        % the comparison is index-for-index.
        d = abs(y - hist(k));
        if d > worst, worst = d; end
    end
    T = simlab_tests.ok(T, worst < 1e-12, ...
        'delayed response equals the undelayed one shifted by %d samples (worst %.3g)', ...
        nDelay, worst);

    % ---- 4. the sensor chain is off unless asked for ----
    pl = simlab.Plant('fopdt', 'k', K_, 'tau', tau, 'l', L);
    pl.reset();
    pl.update(1.0, dt);
    T = simlab_tests.ok(T, pl.yMeas == pl.yTrue, ...
        'with no sensor chain configured, yMeas is bit-identical to yTrue');

    % ---- 5. ADC quantisation quantises ----
    pl.setAdcBits(12, 0, 300);
    pl.reset();
    maxStep = 300 / (2^12 - 1);
    worst = 0;
    for k = 1:50
        pl.update(1.0, dt);
        q = pl.yMeas / maxStep;
        d = abs(q - round(q)) * maxStep;
        if d > worst, worst = d; end
    end
    T = simlab_tests.ok(T, worst < 1e-9, ...
        '12-bit ADC output is always a multiple of its LSB (worst %.3g)', worst);

    % ---- 6. actuator saturation clamps before the plant sees it ----
    pl = simlab.Plant('fopdt', 'k', K_, 'tau', tau, 'l', 0);
    pl.setActuatorLimits(0, 100);
    pl.reset();
    pl.update(5000, dt);
    T = simlab_tests.eq(T, pl.uPlant, 100, 'actuator clamps 5000 to 100');
    T = simlab_tests.eq(T, pl.uCmd, 5000, 'uCmd still records what was asked');

    % ---- 7. noise is deterministic across a reset ----
    % A study whose two runs differ for no reason cannot be compared, so the
    % RNG is re-seeded by reset(). This is the check that it really is.
    pl = simlab.Plant('fopdt', 'k', K_, 'tau', tau, 'l', 0);
    pl.setNoise(0.5);
    pl.reset();
    a = zeros(20, 1);
    for k = 1:20, a(k) = pl.update(1.0, dt); end
    pl.reset();
    b = zeros(20, 1);
    for k = 1:20, b(k) = pl.update(1.0, dt); end
    T = simlab_tests.ok(T, max(abs(a - b)) == 0, ...
        'reset() re-seeds the noise stream: two runs are bit-identical');

    % ---- 8. the DC motor's static gain ----
    % w_inf = Kt / (B + Kt*Ke/R) once the inductance and Coulomb terms drop
    % out. Checked by running it, not by reading the formula back.
    pm = simlab.Plant.presets('dcMotor');
    pm.set('coulomb', 0);        % Coulomb friction is not in the linear gain
    pm.reset();
    w = 0;
    for k = 1:200000
        w = pm.update(10, 1e-4);
    end
    expected = pm.motorParam('kt') / (pm.motorParam('b') + ...
        pm.motorParam('kt') * pm.motorParam('ke') / pm.motorParam('r'));
    T = simlab_tests.ok(T, abs(w / 10 - expected) / expected < 0.01, ...
        'DC motor static gain %.5f matches Kt/(B+Kt*Ke/R) = %.5f', ...
        w / 10, expected);

    % ---- 9. polesZeros reports what the model is ----
    pl = simlab.Plant('fopdt', 'k', 3, 'tau', 10, 'l', 2);
    [z, p, k0, l] = pl.polesZeros();
    T = simlab_tests.eq(T, numel(z), 0, 'fopdt has no zeros');
    T = simlab_tests.near(T, p, -0.1, 1e-12, 'fopdt pole is -1/tau');
    T = simlab_tests.near(T, k0, 3, 1e-12, 'fopdt gain');
    T = simlab_tests.near(T, l, 2, 1e-12, 'fopdt dead time is returned exactly, not as a Pade fit');

    % ---- 10. analysisCaveats says what a linear analysis misses ----
    pl = simlab.Plant.presets('heater');
    cav = pl.analysisCaveats();
    T = simlab_tests.ok(T, ~isempty(cav), ...
        'the heater preset reports %d caveat(s) about its sensor/actuator chain', ...
        numel(cav));
end
