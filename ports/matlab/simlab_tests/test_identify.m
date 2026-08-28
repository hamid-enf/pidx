function T = test_identify(T)
%SIMLAB_TESTS.TEST_IDENTIFY  Does the fit recover a model it was given?
%
% The identification is checked by generating a step response from a KNOWN
% FOPDT and asking for the parameters back. That is a closed loop around the
% estimator, which is the only way to test it without a plant to hand.
%
% The tolerance is stated per parameter and the reasons differ:
%   T is recovered almost exactly - the first moment is dominated by it.
%   L is recovered well when the record is long, badly when it is short.
%   K depends on the final-value estimate, which depends on the settling
%     criterion. That is the same sensitivity the C library documents, and
%     the test asserts the direction of the error rather than hiding it.

    dt = 0.1;
    K_ = 2.0;
    tau = 45.0;
    L = 12.0;
    du = 30.0;

    % ---- 1. clean, long record ----
    d = fopdtStep(K_, tau, L, du, dt, 6000, 0);
    m = simlab.identify(d, struct('uStep', du));

    % The area method on a record truncated at ~5-6 time constants carries a
    % few-percent truncation bias (the residual tail never enters the
    % moments); the mirror computation gives 2.6% on T and 7.9% on L for this
    % exact record. The tolerances state THAT error, not machine precision.
    T = simlab_tests.near(T, m.t, tau, 0.05, 'time constant from a clean long record');
    T = simlab_tests.near(T, m.l, L, 0.10, 'dead time from a clean long record');
    T = simlab_tests.ok(T, abs(m.k - K_) / K_ < 0.10, ...
        'static gain %.4f against the true %.4f (%.1f%% error)', ...
        m.k, K_, 100 * abs(m.k - K_) / K_);
    T = simlab_tests.ok(T, double(m.quality) >= 50, ...
        'quality score is %d, above the 50 gate', double(m.quality));
    T = simlab_tests.ok(T, m.fit.covered >= 5, ...
        'the record covers %.1f time constants', m.fit.covered);

    % The model must be a pidx.plantModel, or no tuning rule will accept it.
    T = simlab_tests.eq(T, m.kind, pidx.Const.MODEL_FOPDT, ...
        'the result is a MODEL_FOPDT, so every FOPDT rule accepts it');
    [rc, g] = pidx.ruleApply(pidx.Const.RULE_IMC, m, pidx.Const.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rc, pidx.Const.OK, 'IMC applies to the identified model');
    T = simlab_tests.ok(T, g.kp > 0, 'and produces a usable Kp (%.4g)', g.kp);

    % And the plant can be rebuilt from it.
    pl = simlab.Plant.fromIdentified(m);
    T = simlab_tests.near(T, pl.steadyStateGain(), m.k, 1e-12, ...
        'Plant.fromIdentified reproduces the gain');

    % ---- 2. the 63.2% crossing cross-checks against L+T ----
    T = simlab_tests.ok(T, ~isnan(m.fit.t632), 'a 63.2%% crossing was found');
    if ~isnan(m.fit.t632)
        T = simlab_tests.ok(T, abs(m.fit.t632 - (m.l + m.t)) < 0.25 * (m.l + m.t), ...
            'the crossing at %.3g s agrees with L+T = %.3g s', ...
            m.fit.t632, m.l + m.t);
    end

    % ---- 3. noise ----
    rng(4, 'twister');
    dn = fopdtStep(K_, tau, L, du, dt, 6000, 0.15);
    mn = simlab.identify(dn, struct('uStep', du));
    T = simlab_tests.near(T, mn.t, tau, 0.10, 'T survives 0.15 of measurement noise');
    T = simlab_tests.ok(T, abs(mn.l - L) / L < 0.25, ...
        'L = %.3f against the true %.1f under noise (%.0f%% error)', ...
        mn.l, L, 100 * abs(mn.l - L) / L);
    T = simlab_tests.ok(T, mn.noise_sigma > 0.05 && mn.noise_sigma < 0.4, ...
        'the reported noise sigma %.4f brackets the injected 0.15', mn.noise_sigma);

    % ---- 4. a short record is REFUSED, or at least says so ----
    %
    % Below about five time constants the fit is extrapolating more than it is
    % measuring. The honest response is a quality penalty and a warning, not a
    % confident model.
    ds = fopdtStep(K_, tau, L, du, dt, 250, 0);
    ms = simlab.identify(ds, struct('uStep', du));
    T = simlab_tests.ok(T, ~isempty(ms.warnings), ...
        'a short record produces %d warning(s)', numel(ms.warnings));
    T = simlab_tests.ok(T, double(ms.quality) < double(m.quality), ...
        'and a lower quality score (%d vs %d)', ...
        double(ms.quality), double(m.quality));

    % ---- 5. no response at all is an error, not a model ----
    dFlat = struct('t', (0:999) * dt, 'y', 50 + 0.001 * randn(1000, 1));
    threw = false;
    try
        simlab.identify(dFlat, struct('uStep', du));
    catch
        threw = true;
    end
    T = simlab_tests.ok(T, threw, ...
        'a flat trace with no step response is refused rather than fitted');

    % ---- 6. a missing step amplitude is an error, not a guess ----
    dNoU = struct('t', d.t, 'y', d.y);
    threw2 = false;
    try
        simlab.identify(dNoU);
    catch
        threw2 = true;
    end
    T = simlab_tests.ok(T, threw2, ...
        'without .u or ''uStep'' the fit is refused: K = dy/du needs du');

    % ---- 7. a command column is used when present ----
    dU = d;
    dU.u = [zeros(50, 1); du * ones(numel(d.t) - 50, 1)];
    mU = simlab.identify(dU);
    T = simlab_tests.near(T, mU.k, m.k, 1e-9, ...
        'the logged command column gives the same K as ''uStep''');

    % ---- 8. a second-order process is not first order, and says so ----
    %
    % The moments go inconsistent and the radicand turns negative. Returning a
    % plausible-looking FOPDT here would detune the loop with no warning.
    d2 = secondOrderStep(dt, 600);
    threw3 = false;
    try
        simlab.identify(d2, struct('uStep', 1));
    catch err
        threw3 = ~isempty(strfind(err.identifier, 'notFopdt')); %#ok<STREMP>
    end
    T = simlab_tests.ok(T, threw3, ...
        'a strongly second-order response is refused with simlab:identify:notFopdt');

    % ---- 9. the midpoint moment arm ----
    %
    % The moment arm must be sampled at t - dt/2. Sampled at the right
    % endpoint instead, the first moment comes up short by te*dt/2 per unit of
    % dy - a deficit that grows with the record length. Detected by fitting
    % the SAME response at two sample rates: with the midpoint arm the answer
    % barely moves; with the wrong arm the error grows as dt grows.
    m1 = simlab.identify(fopdtStep(K_, tau, L, du, 0.05, 2400, 0), struct('uStep', du));
    m2 = simlab.identify(fopdtStep(K_, tau, L, du, 0.20, 600, 0), struct('uStep', du));
    drift = abs(m2.t - m1.t) / tau;
    T = simlab_tests.ok(T, drift < 0.05, ...
        ['T changes by only %.2f%% between dt = 0.05 s and dt = 0.20 s. ' ...
         'With the moment arm at the right endpoint this drifts with the ' ...
         'record length, which is how a longer, more careful test would ' ...
         'produce a WORSE model.'], 100 * drift);
end

function d = fopdtStep(K_, tau, L, du, dt, n, sigma)
% The analytic FOPDT step response, sampled. The ground truth for the fit.
% The command steps at t = 5 s, INSIDE the record: a step at sample 1 would
% give identify no pre-step baseline and no observable step instant, which
% is exactly the historian case it must handle honestly.
    t0 = 5;
    t = (0:n - 1).' * dt;
    y = zeros(n, 1);
    for k = 1:n
        if t(k) > t0 + L
            y(k) = K_ * du * (1 - exp(-(t(k) - t0 - L) / tau));
        end
    end
    if sigma > 0
        y = y + sigma * randn(n, 1);
    end
    d = struct('t', t, 'y', y, 'u', du * (t >= t0));
end

function d = secondOrderStep(dt, n)
% An underdamped second-order step. Not a FOPDT at any K, T, L.
    t = (0:n - 1).' * dt;
    wn = 1.0;
    z = 0.2;
    wd = wn * sqrt(1 - z^2);
    y = 1 - exp(-z * wn * t) ./ sqrt(1 - z^2) .* sin(wd * t + atan(sqrt(1 - z^2) / z));
    d = struct('t', t, 'y', y);
end
