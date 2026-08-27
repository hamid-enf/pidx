function T = test_sensitivity(T)
%SIMLAB_TESTS.TEST_SENSITIVITY  Are the margins the margins?
%
% Gain margin and phase margin have exact answers on a loop you can write
% down, so they are checked against arithmetic rather than against another
% implementation. If these are wrong, every "Ms = 1.4, safe to flash" the
% tool prints is wrong with it.
%
% LOOP 1 - pure gain through a first-order lag, no delay:
%     L(s) = Kp / (1 + s)
%   Phase is -90 deg at every frequency, so it never reaches -180: the gain
%   margin must come out infinite, and the phase margin is 180 - atan(wc)
%   where wc is the frequency at which |L| = 1.
%
% LOOP 2 - the same loop with a known dead time. Adding L seconds of delay
%   rotates the phase by -w*L without touching the magnitude, so the gain
%   crossover does not move and the phase margin drops by exactly wc*L
%   radians. That is a checkable identity, and it is the one that catches a
%   sign error or a missing hold delay.

    % ---------------- LOOP 1 ----------------
    plant = simlab.Plant('fopdt', 'k', 1.0, 'tau', 1.0, 'l', 0.0);
    gains = struct('kp', 2.0, 'ki', 0.0, 'kd', 0.0);
    s1 = simlab.sensitivity(plant, gains, struct('dt', 0.001));

    % Crossover of 2/(1+jw): |L| = 2/sqrt(1+w^2) = 1  ->  wc = sqrt(3).
    wc = sqrt(3);
    T = simlab_tests.near(T, s1.wc, wc, 2e-3, ...
        'gain crossover of 2/(1+s) is sqrt(3)');
    pmExpected = 180 - atan(wc) * 180 / pi;
    T = simlab_tests.near(T, s1.pm, pmExpected, 0.5, ...
        'phase margin is 180 - atan(wc) = %.2f deg', pmExpected);
    T = simlab_tests.ok(T, isinf(s1.gm), ...
        'a first-order loop with no delay has an infinite gain margin');
    T = simlab_tests.near(T, s1.Ms, 1 / sin(atan(wc)), 2e-2, ...
        'Ms = 1/sin(pm) for this loop');
    % Ms = 1/|1+L| at the point closest to -1; for a first-order loop the
    % closest approach is at the crossover, giving 1/sin(pm).

    % The half-sample hold delay must be in there, and it must be tiny at
    % dt = 0.001 s: 0.0005 s at 1.73 rad/s is 0.05 deg. Removing it entirely
    % would change the answer by less than the tolerance, so the presence of
    % the term is checked on a loop where it is NOT negligible - below.

    % ---------------- LOOP 2: dead time rotates the phase, nothing else ----
    Ld = 0.5;
    plant2 = simlab.Plant('fopdt', 'k', 1.0, 'tau', 1.0, 'l', Ld);
    s2 = simlab.sensitivity(plant2, gains, struct('dt', 0.001));

    % Same magnitude curve, so the crossover is the same frequency...
    T = simlab_tests.near(T, s2.wc, wc, 2e-3, ...
        'adding dead time does not move the gain crossover');
    % ...and the phase margin drops by exactly wc*Ld radians.
    pm2Expected = pmExpected - (wc * Ld) * 180 / pi;
    T = simlab_tests.near(T, s2.pm, pm2Expected, 0.6, ...
        'dead time lowers the phase margin by exactly wc*L = %.2f deg', ...
        (wc * Ld) * 180 / pi);

    % ---------------- the hold delay is really applied ----------------
    % At dt = 0.5 s on a loop crossing near 1.73 rad/s, half a sample is
    % 0.25 s, i.e. 24.8 deg of phase. Omit the term and this fails loudly.
    s3 = simlab.sensitivity(plant2, gains, struct('dt', 0.5));
    T = simlab_tests.ok(T, s3.pm < s2.pm - 10, ...
        'dt = 0.5 s costs %.1f deg of phase margin through the ZOH half-sample delay', ...
        s2.pm - s3.pm);
    T = simlab_tests.near(T, s3.holdDelay, 0.25, 1e-12, ...
        'the hold delay is dt/2');

    % ---------------- delay margin ----------------
    % The extra dead time the loop survives is the remaining phase budget
    % divided by the crossover frequency.
    T = simlab_tests.near(T, s2.delayMargin, (s2.pm * pi / 180) / s2.wc, ...
        1e-9, 'delay margin is pm/wc');
    T = simlab_tests.ok(T, s2.delayMargin > Ld * 0.5, ...
        'the loop tolerates %.3g s more dead time than the %.3g s it has', ...
        s2.delayMargin, Ld);

    % ---------------- sensitivity identity ----------------
    % S + T = 1 at every frequency. It is the definition, and it is the
    % cheapest way to catch an indexing error between the two curves.
    resid = max(abs(s2.S + s2.T - 1));
    T = simlab_tests.ok(T, resid < 1e-9, 'S + T = 1 at every frequency (residual %.3g)', resid);

    % ---------------- Ms really is the peak of |S| ----------------
    [mx, ~] = max(abs(s2.S));
    T = simlab_tests.near(T, s2.Ms, mx, 1e-12, 'Ms is max |S| on the grid');

    % ---------------- a loop that is too aggressive is flagged ----------------
    sHot = simlab.sensitivity(plant2, struct('kp', 20, 'ki', 5, 'kd', 0), ...
        struct('dt', 0.01));
    T = simlab_tests.ok(T, sHot.Ms > 2.0, ...
        'an aggressive tuning gives Ms = %.2f and the verdict says so', sHot.Ms);
    T = simlab_tests.ok(T, ~isempty(strfind(lower(sHot.verdict), 'fragile')), ...
        'the verdict names the loop FRAGILE');

    % ---------------- a gentle loop is flagged as comfortable ----------------
    sCalm = simlab.sensitivity(plant, struct('kp', 0.3, 'ki', 0.02, 'kd', 0), ...
        struct('dt', 0.01));
    T = simlab_tests.ok(T, sCalm.Ms < 1.4, ...
        'a gentle tuning gives Ms = %.2f', sCalm.Ms);
    T = simlab_tests.ok(T, ~isempty(strfind(lower(sCalm.verdict), 'comfortable')), ...
        'the verdict names the loop comfortable');

    % ---------------- caveats are reported, not swallowed ----------------
    plSat = simlab.Plant.presets('heater');
    sSat = simlab.sensitivity(plSat, struct('kp', 3, 'ki', 0.08, 'kd', 0), ...
        struct('dt', 0.1));
    T = simlab_tests.ok(T, ~isempty(sSat.warnings), ...
        'a plant with saturation and a 12-bit ADC reports %d caveat(s)', ...
        numel(sSat.warnings));

    % ---------------- the derivative filter is resolved the way the core does it
    % Tf = Td/N with N = 10 unless an explicit Tf is given.
    g = struct('kp', 1.0, 'ki', 0.1, 'kd', 2.0);
    sD = simlab.sensitivity(plant, g, struct('dt', 0.01));
    T = simlab_tests.near(T, sD.tf, 2.0 / (10.0 * 1.0), 1e-12, ...
        'Tf is derived as Kd/(N*Kp) with N = 10');
    sD2 = simlab.sensitivity(plant, g, struct('dt', 0.01, 'tf', 0.75));
    T = simlab_tests.near(T, sD2.tf, 0.75, 1e-12, 'an explicit Tf overrides N');

    % ---------------- a custom plant is refused, not guessed ----------------
    pc = simlab.Plant('custom', 'f', @(x, u, h) -x + u, 'n', 1, 'x0', 0);
    threw = false;
    try
        simlab.sensitivity(pc, struct('kp', 1, 'ki', 0, 'kd', 0));
    catch
        threw = true;
    end
    T = simlab_tests.ok(T, threw, ...
        'a custom plant with no declared transfer function is refused rather than approximated');
end
