function T = test_metrics(T)
%SIMLAB_TESTS.TEST_METRICS  Are the reported numbers the numbers they claim?
%
% Overshoot, rise time and settling time get quoted in datasheets and then
% believed. Each is checked here against a signal built by hand, where the
% right answer can be written down before the code runs.
%
% The synthetic response is a standard second-order step:
%     y(t) = 1 - exp(-z w t)/sqrt(1-z^2) * sin(w_d t + phi)
% with z = 0.5, w = 2. For that:
%     overshoot = exp(-z*pi/sqrt(1-z^2)) = 16.30%
%     rise 10-90% and settling time are read off the sampled signal by the
%     same rules the function documents, so the test states the rule it
%     expects rather than trusting the implementation's definition.

    dt = 0.001;
    z = 0.5;
    wn = 2.0;
    wd = wn * sqrt(1 - z^2);
    n = 20000;
    t = (0:n - 1) * dt;

    y = 1 - exp(-z * wn * t) / sqrt(1 - z^2) .* sin(wd * t + atan(sqrt(1 - z^2) / z));

    r = struct();
    r.dt = dt;
    r.t = t;
    r.r = zeros(1, n);
    r.r(round(1 / dt) + 1:end) = 1;      % unit step at t = 1 s
    r.y = y;
    r.y(r.t < 1) = 0;
    r.u = zeros(1, n);
    r.flags = zeros(1, n, 'uint32');

    m = simlab.metrics(r);

    % ---- overshoot ----
    osExpected = 100 * exp(-z * pi / sqrt(1 - z^2));
    T = simlab_tests.ok(T, abs(m.overshoot - osExpected) < 0.05, ...
        'overshoot %.4f%% matches the analytic exp(-z*pi/sqrt(1-z^2)) = %.4f%%', ...
        m.overshoot, osExpected);

    % ---- rise time, 10% to 90% of the step ----
    i10 = find(r.y >= 0.1, 1, 'first');
    i90 = find(r.y >= 0.9, 1, 'first');
    T = simlab_tests.near(T, m.riseTime, t(i90) - t(i10), 2e-3, ...
        'rise time is the 10%%-to-90%% interval it documents');

    % ---- settling time: the LAST exit from the 2% band ----
    out = abs(r.y - 1) > 0.02;
    iLast = find(out, 1, 'last');
    T = simlab_tests.near(T, m.settlingTime, t(iLast + 1) - 1.0, 2e-3, ...
        'settling time is the last exit from the band, not the first entry');

    % A response that dips back out late must NOT be reported as settled
    % early. Build one and check.
    r2 = r;
    r2.y = r.y;
    kLate = round(6 / dt);
    r2.y(kLate:kLate + 200) = 1.5;       % a late excursion
    m2 = simlab.metrics(r2);
    T = simlab_tests.ok(T, m2.settlingTime > 5.9, ...
        'a late excursion moves the settling time to %.3f s, not to the first entry', ...
        m2.settlingTime);

    % ---- IAE ----
    T = simlab_tests.near(T, m.iae, trapz(t, abs(r.r - r.y)), 1e-12, 'IAE is trapz(|e|)');
    T = simlab_tests.near(T, m.ise, trapz(t, (r.r - r.y).^2), 1e-9, 'ISE is trapz(e^2)');
    T = simlab_tests.near(T, m.itae, trapz(t, t .* abs(r.r - r.y)), 1e-9, 'ITAE is trapz(t|e|)');

    % ---- total variation ----
    u = sin(2 * pi * t);
    r3 = r; r3.u = u;
    m3 = simlab.metrics(r3);
    T = simlab_tests.near(T, m3.tv, sum(abs(diff(u))), 1e-12, 'TV is sum |du|');

    % ---- saturation fraction comes from the FLAGS, not from the data ----
    r4 = r;
    r4.u = ones(1, n) * 5;
    r4.flags = zeros(1, n, 'uint32');
    r4.flags(1:1000) = uint32(pidx.Const.FLAG_SATURATED_HIGH);
    m4 = simlab.metrics(r4);
    T = simlab_tests.near(T, m4.satFraction, 1000 / n, 1e-12, ...
        'saturation fraction is read from PIDX flags');

    % ---- stability ----
    T = simlab_tests.ok(T, m.stable, 'a settled second-order step is reported stable');
    rDiv = r;
    rDiv.y = exp(0.5 * t);
    mDiv = simlab.metrics(rDiv);
    T = simlab_tests.ok(T, ~mDiv.stable, 'a diverging response is reported NOT stable');
    rNan = r;
    rNan.y(5000) = NaN;
    mNan = simlab.metrics(rNan);
    T = simlab_tests.ok(T, ~mNan.stable, 'a NaN in the trace is reported NOT stable');

    % ---- no step in the scenario -> transient metrics are NaN, not zero ----
    rFlat = r;
    rFlat.r = ones(1, n);
    mFlat = simlab.metrics(rFlat);
    T = simlab_tests.ok(T, isnan(mFlat.overshoot) && isnan(mFlat.riseTime), ...
        'with no setpoint step the transient metrics are NaN rather than a fabricated 0');
    T = simlab_tests.ok(T, ~isnan(mFlat.iae), ...
        'but the integral metrics are still computed');

    % ---- a step DOWN is handled with the right sign ----
    rDown = r;
    rDown.r = ones(1, n);
    rDown.r(round(1 / dt) + 1:end) = 0;
    rDown.y = 1 - r.y;
    rDown.y(rDown.t < 1) = 1;
    mDown = simlab.metrics(rDown);
    T = simlab_tests.ok(T, abs(mDown.overshoot - osExpected) < 0.05, ...
        'overshoot on a step DOWN is %.3f%%, the same magnitude', mDown.overshoot);
end
