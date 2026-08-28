function T = test_monteCarlo(T)
%SIMLAB_TESTS.TEST_MONTECARLO  Does the robustness study measure what it claims?
%
% A robustness study that is not reproducible is noise, and one whose
% perturbation does not stay inside the box it advertises is a lie. The
% checks here are on the MECHANICS - determinism, the perturbation range, the
% accounting - plus one SEMANTIC check: on a dead-time-dominated plant, a
% gentle tuning must survive at least as often as Ziegler-Nichols. That
% ordering is the finding the whole study exists to reproduce (README.md,
% sim/sim_robust.c: ZN is fast on a perfect model and among the first to
% break on a wrong one), so if the study cannot see it on an L/T = 2.4 plant,
% the study is not working.

    % ---- a fast, deterministic plant ----
    plant = simlab.Plant('fopdt', 'name', 'study', 'k', 2.0, ...
        'tau', 5.0, 'l', 12.0);            % L/T = 2.4: dead-time dominated
    plant.setActuatorLimits(0, 100);
    dt = 0.25;
    sc = simlab.Scenario.presets('stepResponse', 'sp', 100, 'tEnd', 400);

    gGentle = gainsFromIMC(plant, 12.0, dt);    % lambda = one full dead time
    gZn = gainsFromZn(plant, dt);               % the aggressive reference

    % ==================================================================
    % 1. determinism: same seed, same plants, same answer
    % ==================================================================
    a = simlab.monteCarlo(plant, gGentle, struct('nRuns', 12, 'spread', 2, ...
        'scenario', sc, 'seed', 7, 'verbose', false));
    b = simlab.monteCarlo(plant, gGentle, struct('nRuns', 12, 'spread', 2, ...
        'scenario', sc, 'seed', 7, 'verbose', false));

    T = simlab_tests.ok(T, isequal(a.factors, b.factors), ...
        'the same seed draws the same %d plants', a.nRuns);
    T = simlab_tests.ok(T, isequal(a.stable, b.stable), ...
        'and scores them identically');
    T = simlab_tests.eq(T, a.share, b.share, 'same survival share');

    c2 = simlab.monteCarlo(plant, gGentle, struct('nRuns', 12, 'spread', 2, ...
        'scenario', sc, 'seed', 8, 'verbose', false));
    T = simlab_tests.ok(T, ~isequal(a.factors, c2.factors), ...
        'a different seed draws different plants, so the seed is not decorative');

    % ==================================================================
    % 2. the perturbation stays inside the advertised box
    % ==================================================================
    f = a.factors;
    T = simlab_tests.eq(T, size(f, 2), 3, 'three factors per plant: K, tau, L');
    T = simlab_tests.ok(T, all(f(:) >= 0.5 - 1e-9) && all(f(:) <= 2 + 1e-9), ...
        'every factor lies in [0.5, 2] as ''spread'' = 2 promises');
    T = simlab_tests.ok(T, max(f(:)) > 1.5 && min(f(:)) < 0.7, ...
        'and the box is actually explored, not just respected');

    % The accounting: share is the mean of the stable flags, and the table's
    % fourth column is exactly those flags.
    T = simlab_tests.near(T, a.share, mean(a.stable), 1e-12, ...
        'share = mean(stable)');
    T = simlab_tests.eq(T, double(a.table(:, 4)), double(a.stable), ...
        'the table carries the same stability flags');

    % The nominal run is part of the result and is stable by construction -
    % the gains were built for this exact plant.
    T = simlab_tests.ok(T, logical(a.nominal.stable), ...
        'the nominal plant runs stable under its own tuning');

    % The worst plant is reported, lies inside the box, and its full log is
    % carried so it can be re-run and plotted.
    T = simlab_tests.ok(T, all(a.worstPlant >= 0.5) && all(a.worstPlant <= 2), ...
        'the worst plant is inside the box (%.2f, %.2f, %.2f)', ...
        a.worstPlant(1), a.worstPlant(2), a.worstPlant(3));
    T = simlab_tests.ok(T, isstruct(a.worstLog) && ~isempty(a.worstLog.t), ...
        'and its full trace is carried for re-plotting');

    % ==================================================================
    % 3. the semantic check: gentle beats ZN on a dead-time plant
    % ==================================================================
    mG = simlab.monteCarlo(plant, gGentle, struct('nRuns', 30, 'spread', 2, ...
        'scenario', sc, 'seed', 7, 'verbose', false));
    mZ = simlab.monteCarlo(plant, gZn, struct('nRuns', 30, 'spread', 2, ...
        'scenario', sc, 'seed', 7, 'verbose', false));

    T = simlab_tests.ok(T, mG.share >= 0.7, ...
        'the gentle tuning survives %.0f%% of plants with K/tau/L in [0.5, 2]', ...
        100 * mG.share);
    T = simlab_tests.ok(T, mG.share >= mZ.share, ...
        'and it survives at least as often as Ziegler-Nichols (%.0f%% vs %.0f%%)', ...
        100 * mG.share, 100 * mZ.share);

    % The IAE ordering goes the other way ON THE NOMINAL PLANT: that is the
    % anti-correlation. Not on the perturbed median, which is deliberately not
    % asserted - with 30 plants it can flip; the survival share is the firmer
    % evidence, exactly as docs/14 says.
    rG = simlab.Sim(plant, pidx.PID(pidx.config('kp', gGentle.kp, ...
        'ki', gGentle.ki, 'kd', gGentle.kd, 'dt', dt)), sc).run();
    rZ = simlab.Sim(plant, pidx.PID(pidx.config('kp', gZn.kp, ...
        'ki', gZn.ki, 'kd', gZn.kd, 'dt', dt)), sc).run();
    if logical(rZ.metrics.stable)
        T = simlab_tests.ok(T, rZ.metrics.iae < rG.metrics.iae, ...
            'ZN is faster on the PERFECT model (IAE %.4g vs %.4g) - the ' ...
            'trade-off the study is built around', ...
            rZ.metrics.iae, rG.metrics.iae);
    end

    % ==================================================================
    % 4. gains without output limits still run
    % ==================================================================
    % monteCarlo attaches the plant's limits when the gains carry them; when
    % they do not, the runs must still complete rather than error.
    gBare = struct('kp', gGentle.kp, 'ki', gGentle.ki, 'kd', gGentle.kd, ...
        'dt', dt);
    mB = simlab.monteCarlo(plant, gBare, struct('nRuns', 6, 'spread', 2, ...
        'scenario', sc, 'verbose', false));
    T = simlab_tests.eq(T, mB.nRuns, 6, 'a limit-free gain set still runs');
end

% ---------------------------------------------------------------------------

function g = gainsFromIMC(plant, lambda, dt)
% IMC-PID on the TRUE model: the gentle end of the family.
    K = pidx.Const;
    m = pidx.plantModel(K.MODEL_FOPDT, plant.steadyStateGain(), ...
        plant.tau(), plant.transportDelay());
    [rc, gg] = pidx.ruleApply(K.RULE_IMC, m, K.STRUCT_PI, lambda);
    if rc ~= K.OK
        error('test:imc', 'IMC failed: %d', rc);
    end
    g = struct('kp', gg.kp, 'ki', gg.ki, 'kd', gg.kd, 'dt', dt, ...
        'outMin', 0, 'outMax', 100);
end

function g = gainsFromZn(plant, dt)
% Ziegler-Nichols PID from the EXACT ultimate point: the aggressive end.
    K = pidx.Const;
    w = fzero(@(w) -atan(w * plant.tau()) - w * plant.transportDelay() + pi, 1e-3);
    ku = sqrt(1 + (w * plant.tau())^2) / plant.steadyStateGain();
    pu = 2 * pi / w;
    m = pidx.plantModel(K.MODEL_FREQ, ku, pu);
    [rc, gg] = pidx.ruleApply(K.RULE_ZN, m, K.STRUCT_PID, 0);
    if rc ~= K.OK
        error('test:zn', 'ZN failed: %d', rc);
    end
    g = struct('kp', gg.kp, 'ki', gg.ki, 'kd', gg.kd, 'dt', dt, ...
        'tf', gg.tf, 'outMin', 0, 'outMax', 100);
end
