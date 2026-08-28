function T = test_sim(T)
%SIMLAB_TESTS.TEST_SIM  Does simlab.Sim reproduce the C library, sample by sample?
%
% This is the test that makes the rest of the tool trustworthy. It runs the
% identical loop - same plant, same gains, same sample order - through
% simlab.Sim and compares against tools/matlab_ref/matlab_ref.c, which ran it
% through src/pid.c in double precision.
%
% The comparison is at 1e-9 relative, which is orders of magnitude above the
% double-precision noise this loop produces and orders of magnitude below any
% logic difference. If a future change to either side moves these numbers,
% this test says so instead of the board saying so.
%
% Regenerate the reference after changing either side:
%     cd ../../../tools/matlab_ref && make run

    haveRef = simlab_tests.hasRef(T);

    % ---- the scenario, matching matlab_ref.c scenario_step() exactly ----
    dt = 0.1;
    n = 300;
    kStep = 10;                 % setpoint steps at k >= 10 (C: k >= 10)
    spVal = 100.0;

    plant = simlab.Plant('fopdt', 'name', 'ref heater', ...
        'k', 2.0, 'tau', 45.0, 'l', 12.0);
    cfg = pidx.config('kp', 3.0, 'ki', 0.08, 'kd', 0.0, 'dt', dt);
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    ctrl = pidx.PID(cfg);

    sc = simlab.Scenario('closed-loop step, saturating', n * dt);
    sc.setpoint(0, 0);
    sc.setpoint(spVal, kStep * dt);

    r = simlab.Sim(plant, ctrl, sc).run();

    T = simlab_tests.eq(T, numel(r.t), n, 'simulation ran %d samples', n);
    T = simlab_tests.near(T, r.dt, dt, 1e-15, 'dt is the controller dt');

    if haveRef
        keys = [0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275];
        for i = 1:numel(keys)
            k = keys(i) + 1;                       % MATLAB is 1-based
            wantY = simlab_tests.refGet(T, sprintf('step.t%03d.y', keys(i)));
            wantU = simlab_tests.refGet(T, sprintf('step.t%03d.u', keys(i)));
            T = simlab_tests.near(T, r.y(k), wantY, 1e-9, ...
                sprintf('y at sample %d', keys(i)));
            T = simlab_tests.near(T, r.u(k), wantU, 1e-9, ...
                sprintf('u at sample %d', keys(i)));
        end
        T = simlab_tests.near(T, r.y(end), simlab_tests.refGet(T, 'step.finalY'), ...
            1e-9, 'final measurement');
        T = simlab_tests.near(T, r.u(end), simlab_tests.refGet(T, 'step.finalU'), ...
            1e-9, 'final command');
        T = simlab_tests.near(T, ctrl.getIntegrator(), ...
            simlab_tests.refGet(T, 'step.integrator'), 1e-9, ...
            'integrator term after the run');
    else
        T = simlab_tests.skip(T, 'closed-loop trajectory vs C reference', ...
            'c_reference.csv not found');
    end

    % ---- the sample order is the ISR order, not a shifted one ----
    %
    % The plant must advance under the PREVIOUS command. If it advanced under
    % the current one the loop would have a full sample less dead time than
    % the target has, and every gain would come out high. Detected here by
    % comparing against a hand-built loop that states the order explicitly.
    plant2 = simlab.Plant('fopdt', 'name', 'x', 'k', 2.0, 'tau', 45.0, 'l', 12.0);
    ctrl2 = pidx.PID(cfg);
    plant2.reset();
    ctrl2.reset();
    uPrev = 0;
    yHand = zeros(n, 1);
    for k = 1:n
        if k - 1 >= kStep
            ctrl2.setSetpoint(spVal);
        end
        plant2.update(uPrev, dt);
        y = plant2.yMeas;
        u = ctrl2.update(y);
        yHand(k) = y;
        uPrev = u;
    end
    T = simlab_tests.ok(T, max(abs(yHand - r.y(:))) == 0, ...
        'simlab.Sim is bit-identical to a hand-written loop with the ISR sample order');

    % (An earlier version also ran the loop with the read BEFORE the plant
    % step and demanded a difference. For a strictly proper plant both
    % orderings produce the same sequence, so that check could only fail.
    % The C-reference trajectory above is the ordering pin.)

    % ---- the log is complete ----
    T = simlab_tests.ok(T, any(r.uRaw ~= r.u), ...
        'uRaw and u differ somewhere: the run saturates, as designed');
    T = simlab_tests.ok(T, r.metrics.satFraction > 0.5, ...
        'this loop is saturated %.0f%% of the time (Kp=3 on a 100%% step cannot not be)', ...
        100 * r.metrics.satFraction);
    T = simlab_tests.ok(T, ~isempty(strfind(r.scenario, 'setpoint')), ...
        'the scenario transcript is recorded in the result');

    % ---- determinism ----
    r2 = simlab.Sim(simlab.Plant('fopdt', 'k', 2.0, 'tau', 45.0, 'l', 12.0), ...
                    pidx.PID(cfg), sc).run();
    T = simlab_tests.ok(T, max(abs(r2.y - r.y)) == 0, ...
        'running the same scenario twice is bit-identical');
end
