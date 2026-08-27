%DEMO_DESIGN  Design to a specification instead of picking a rule.
%
%   simlab_demos.demo_design
%
% A tuning rule tells you what a table says. This demo asks for what you
% actually want - "overshoot under 5%, as fast as the dead time allows" - and
% searches the one-parameter family that spans the trade-off.
%
% Four things, in the order you would do them on a real project:
%   1. identify the plant from a recorded step (no auto-tuner, no disturbance)
%   2. design to a specification and see WHY the answer is what it is
%   3. find out what the specification costs in robustness
%   4. export it, and check what the fixed-point version would cost
%
% Requires no toolbox.

    K = pidx.Const;
    dt = 0.25;

    % ==================================================================
    fprintf('\n=== 1. identify from a recorded step ======================\n');
    % Suppose this is a historian export: an open-loop step of 30% on a
    % process with K = 2, tau = 45 s, L = 12 s, plus sensor noise.
    truePlant = simlab.Plant('fopdt', 'k', 2.0, 'tau', 45.0, 'l', 12.0);
    tRec = (0:0.1:1200).';
    yRec = zeros(numel(tRec), 1);
    uRec = zeros(numel(tRec), 1);
    rng(11, 'twister');
    for k = 1:numel(tRec)
        if tRec(k) >= 50
            uRec(k) = 20 + 30;
            yRec(k) = 2.0 * 30 * (1 - exp(-(tRec(k) - 50 - 12) / 45));
        else
            uRec(k) = 20;
        end
        yRec(k) = yRec(k) + 0.1 * randn();
    end
    data = struct('t', tRec, 'y', yRec, 'u', uRec);

    m = simlab.identify(data);
    fprintf('  fitted   K = %.4f   T = %.4f s   L = %.4f s   quality %d\n', ...
        m.k, m.t, m.l, double(m.quality));
    fprintf('  true     K = %.4f   T = %.4f s   L = %.4f s\n', 2.0, 45.0, 12.0);
    fprintf('  errors   K %+.1f%%   T %+.1f%%   L %+.1f%%\n', ...
        100 * (m.k - 2.0) / 2.0, 100 * (m.t - 45.0) / 45.0, ...
        100 * (m.l - 12.0) / 12.0);
    fprintf('  63.2%% crossing at %.3f s, L+T = %.3f s (a cross-check, not\n', ...
        m.fit.t632, m.l + m.t);
    fprintf('  part of the fit - see the area/moment note in simlab.identify)\n');
    for i = 1:numel(m.warnings)
        fprintf('  ! %s\n', m.warnings{i});
    end

    % Build the plant the design will be checked against: the identified
    % model plus the sensor and actuator reality.
    plant = simlab.Plant.fromIdentified(m);
    plant.setActuatorLimits(0, 100);
    plant.setAdcBits(12, 0, 300);
    plant.setNoise(0.15);

    % ==================================================================
    fprintf('\n=== 2. design to a specification ==========================\n');
    sc = simlab.Scenario.presets('stepResponse', 'sp', 100, 'tEnd', 1200);

    goal = struct('maxOvershoot', 5, 'maxMs', 1.6, 'minDelayMargin', 5, ...
                  'objective', 'iae');
    fprintf('  goal: overshoot < %.0f%%, Ms < %.1f, delay margin > %.0f s,\n', ...
        goal.maxOvershoot, goal.maxMs, goal.minDelayMargin);
    fprintf('        fastest IAE\n');

    d = simlab.designByGoal(plant, goal, struct('model', m, 'dt', dt, ...
        'scenario', sc, 'nLambda', 20, 'verbose', true));

    if ~d.feasible
        fprintf('\n  the specification is not reachable on this plant.\n');
        fprintf('  %s\n', d.diagnosis);
        return;
    end

    fprintf('\n  chosen: Kp = %.5f, Ki = %.5f, Kd = %.5f\n', ...
        d.gains.kp, d.gains.ki, d.gains.kd);
    fprintf('          Tf = %.5f s, beta = %.2f, lambda = %.5f s\n', ...
        d.gains.tf, d.gains.beta, d.gains.lambda);

    % ==================================================================
    fprintf('\n=== 3. verify independently, then ask what it costs =======\n');
    cfg = pidx.config('kp', d.gains.kp, 'ki', d.gains.ki, 'kd', d.gains.kd, ...
        'dt', dt);
    cfg.filter.tf = d.gains.tf;
    cfg.weight.beta = d.gains.beta;
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    cfg.safety.enabled = true;
    cfg.safety.meas_min = -10;
    cfg.safety.meas_max = 350;
    cfg.safety.meas_rate_max = 25;

    r = simlab.Sim(plant, pidx.PID(cfg), sc).run();
    simlab.plot(r, struct('fig', 51, 'title', 'designed loop'));
    fprintf('  independent run: rise %.4g s, overshoot %.2f%%, settling %.4g s\n', ...
        r.metrics.riseTime, r.metrics.overshoot, r.metrics.settlingTime);
    fprintf('                   IAE %.5g, steady error %.5g\n', ...
        r.metrics.iae, r.metrics.ssError);

    s = simlab.sensitivity(plant, struct('kp', d.gains.kp, 'ki', d.gains.ki, ...
        'kd', d.gains.kd), struct('dt', dt, 'tf', d.gains.tf, ...
        'beta', d.gains.beta));
    simlab.plotSensitivity(s, struct('fig', 52));
    fprintf('  margins: Ms %.3f, PM %.1f deg, GM %.2fx, delay margin %.4g s\n', ...
        s.Ms, s.pm, s.gm, s.delayMargin);

    gains = struct('kp', d.gains.kp, 'ki', d.gains.ki, 'kd', d.gains.kd, ...
        'dt', dt, 'tf', d.gains.tf, 'outMin', 0, 'outMax', 100);
    mc = simlab.monteCarlo(plant, gains, struct('nRuns', 40, 'spread', 2, ...
        'scenario', sc, 'verbose', false));
    fprintf('  %.0f%% of 40 plants with K/T/L each 0.5x..2x stayed stable\n', ...
        100 * mc.share);
    if mc.share < 0.9
        fprintf('  BELOW 90%%: the specification bought speed with margin.\n');
        fprintf('  Raise minDelayMargin in the goal and re-run - that is the\n');
        fprintf('  dial, and it is the one the search respects.\n');
    end

    % ==================================================================
    fprintf('\n=== 4. export, and price the fixed-point version ==========\n');
    out = simlab.exportSTM32(plant, cfg, struct('symbol', 'designedLoop', ...
        'profile', 'PROCESS', 'result', r, 'sens', s, ...
        'dir', fullfile(pwd, 'simlab_export')));
    fprintf('  %s\n', out.source);

    % The Q15 study needs a plant whose signals fit the Q15 range, so it is
    % run on the normalised version rather than on 0..300 degC.
    norm = simlab.Plant('fopdt', 'k', 1.0, 'tau', m.t, 'l', m.l);
    norm.setActuatorLimits(-1, 1);
    norm.setAdcBits(12, -1, 1);
    cf = simlab.compareFixed(norm, struct('kp', d.gains.kp * m.k, ...
        'ki', d.gains.ki * m.k, 'kd', d.gains.kd * m.k), ...
        struct('dt', dt, 'tf', d.gains.tf, ...
               'scenario', simlab.Scenario.presets('stepResponse', ...
                   'sp', 0.5, 'tEnd', 1200)));
    fprintf('  fixed-point study on the normalised plant is in the output above.\n');
    fprintf('\n  next: flash it, then run simlab.hilRun against the same\n');
    fprintf('        scenario and simlab.hilCompare the two traces.\n\n');
