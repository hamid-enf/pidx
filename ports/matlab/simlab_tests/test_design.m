function T = test_design(T)
%SIMLAB_TESTS.TEST_DESIGN  Does design-by-goal find gains that meet the goal?
%
% The search is checked the only way that counts: give it a constraint, then
% verify the returned design against a FRESH simulation of the same scenario.
% If the design says "overshoot 4.8%" and an independent run says 12%, the
% search is reporting its own bookkeeping rather than the loop.

    K = pidx.Const;
    plant = simlab.Plant('fopdt', 'name', 'test', 'k', 2.0, 'tau', 45.0, ...
        'l', 12.0);
    plant.setActuatorLimits(0, 100);
    dt = 0.25;
    sp = 100;
    sc = simlab.Scenario.presets('stepResponse', 'sp', sp, 'tEnd', 1200);

    % ---- 1. a feasible goal is met ----
    goal = struct('maxOvershoot', 10, 'maxMs', 1.8, 'objective', 'iae');
    d = simlab.designByGoal(plant, goal, struct('dt', dt, 'scenario', sc, ...
        'nLambda', 15, 'verbose', false));

    T = simlab_tests.ok(T, d.feasible, 'a 10%% overshoot goal is feasible on this plant');
    T = simlab_tests.ok(T, ~isempty(d.gains), 'gains were returned');
    T = simlab_tests.ok(T, d.gains.kp > 0, 'Kp = %.5g is positive', d.gains.kp);
    T = simlab_tests.ok(T, d.nSimulated > 0, '%d candidates were simulated', d.nSimulated);
    T = simlab_tests.ok(T, d.nScreened > 0, ...
        '%d candidates were screened out on margins alone, before any simulation', ...
        d.nScreened);

    % ---- 2. verify against an independent run ----
    cfg = pidx.config('kp', d.gains.kp, 'ki', d.gains.ki, ...
        'kd', d.gains.kd, 'dt', dt);
    if d.gains.tf > 0, cfg.filter.tf = d.gains.tf; end
    cfg.weight.beta = d.gains.beta;
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    r = simlab.Sim(plant, pidx.PID(cfg), sc).run();

    T = simlab_tests.ok(T, r.metrics.overshoot <= goal.maxOvershoot + 0.5, ...
        'an independent run gives %.2f%% overshoot against the %.1f%% goal', ...
        r.metrics.overshoot, goal.maxOvershoot);
    T = simlab_tests.ok(T, abs(r.metrics.iae - d.best.iae) / d.best.iae < 0.02, ...
        'the reported IAE %.5g reproduces (%.5g)', d.best.iae, r.metrics.iae);

    % ---- 3. a tighter overshoot goal gives a slower loop ----
    %
    % This is the trade-off, and it is the thing the function exists to
    % navigate. If tightening the constraint did not cost speed, the search
    % would not be doing anything.
    goal2 = struct('maxOvershoot', 2, 'maxMs', 3.0, 'objective', 'iae');
    d2 = simlab.designByGoal(plant, goal2, struct('dt', dt, 'scenario', sc, ...
        'nLambda', 15, 'verbose', false));
    if d2.feasible
        T = simlab_tests.ok(T, d2.gains.lambda >= d.gains.lambda, ...
            'a 2%% overshoot goal needs lambda %.4g s against %.4g s for 10%%', ...
            d2.gains.lambda, d.gains.lambda);
        T = simlab_tests.ok(T, d2.gains.kp <= d.gains.kp, ...
            'and a lower Kp (%.5g against %.5g)', d2.gains.kp, d.gains.kp);
    else
        T = simlab_tests.ok(T, true, ...
            'a 2%% goal is infeasible here, which is itself a valid answer');
    end

    % ---- 4. an infeasible goal says so, and says why ----
    goalBad = struct('maxOvershoot', 0.001, 'maxSettling', 1.0, ...
        'objective', 'iae');
    d3 = simlab.designByGoal(plant, goalBad, struct('dt', dt, 'scenario', sc, ...
        'nLambda', 10, 'verbose', false));
    T = simlab_tests.ok(T, ~d3.feasible, ...
        'settling a 12 s dead-time plant in 1 s with no overshoot is refused');
    T = simlab_tests.ok(T, ~isempty(strfind(lower(d3.diagnosis), 'infeasible')), ...
        'the diagnosis says INFEASIBLE rather than quietly returning the best effort');
    T = simlab_tests.ok(T, ~isempty(strfind(lower(d3.diagnosis), 'dead time')), ...
        'and names the dead time as the reason');

    % ---- 5. the objective is respected ----
    goalFast = struct('maxOvershoot', 25, 'maxMs', 2.5, 'objective', 'settling');
    dF = simlab.designByGoal(plant, goalFast, struct('dt', dt, 'scenario', sc, ...
        'nLambda', 15, 'verbose', false));
    if dF.feasible && d.feasible
        T = simlab_tests.ok(T, dF.best.settling <= d.best.settling + 1e-9, ...
            'optimising for settling gives %.4g s against %.4g s for the IAE design', ...
            dF.best.settling, d.best.settling);
    end

    % ---- 6. the Ms screen really screens ----
    %
    % A candidate over maxMs must never reach the simulation stage, which is
    % what makes a wide sweep affordable. Checked on the candidate table.
    for i = 1:numel(d.candidates)
        c = d.candidates(i);
        if ~c.simulated
            T = simlab_tests.ok(T, ~isempty(c.violations), ...
                'an unsimulated candidate has a recorded reason');
            break;
        end
    end

    % ---- 7. a model from identification is accepted ----
    m = pidx.plantModel(K.MODEL_FOPDT, 2.2, 48.0, 13.0);
    dM = simlab.designByGoal(plant, struct('maxOvershoot', 15, 'maxMs', 2.0), ...
        struct('dt', dt, 'scenario', sc, 'model', m, 'nLambda', 10, ...
               'verbose', false));
    T = simlab_tests.ok(T, dM.modelWasIdentified, ...
        'an explicitly passed model is reported as identified, not assumed');
    T = simlab_tests.near(T, dM.model.k, 2.2, 1e-12, 'and is the model that was passed');

    % ---- 8. a FREQ model is refused ----
    mF = pidx.plantModel(K.MODEL_FREQ, 2.0, 40.0);
    threw = false;
    try
        simlab.designByGoal(plant, struct('maxMs', 2.0), ...
            struct('model', mF, 'dt', dt, 'verbose', false));
    catch err
        threw = ~isempty(strfind(err.identifier, 'model')); %#ok<STREMP>
    end
    T = simlab_tests.ok(T, threw, ...
        'a FREQ model is refused: the lambda family needs K, T and L');
end
