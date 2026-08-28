function T = test_cascade(T)
%SIMLAB_TESTS.TEST_CASCADE  Does the cascade coordinator behave like the C one?
%
% simlab.Cascade ports src/pid_cascade.c. The interesting behaviour is not the
% forward pass - that is just two PID calls - it is the BACKWARD pass, where a
% saturated inner loop has to stop its parent's integrator from winding up
% against a wall. That is the part a naive cascade gets wrong, and the part
% this test pins to the C numbers.
%
% The scenario is matlab_ref.c scenario_cascade(): a 10x decimated outer loop
% commanding a setpoint the inner loop cannot reach, so the inner actuator is
% pinned for the whole run.

    haveRef = simlab_tests.hasRef(T);
    K = pidx.Const;

    dt = 0.001;
    n = 6000;

    cfgO = pidx.config('kp', 0.5, 'ki', 2.0, 'kd', 0.0, 'dt', 0.01);
    cfgO.limits.use_output_limits = true;
    cfgO.limits.output_min = -50;
    cfgO.limits.output_max = 50;
    outer = pidx.PID(cfgO);

    cfgI = pidx.config('kp', 1.0, 'ki', 20.0, 'kd', 0.0, 'dt', 0.001);
    cfgI.limits.use_output_limits = true;
    cfgI.limits.output_min = -10;
    cfgI.limits.output_max = 10;
    inner = pidx.PID(cfgI);

    cl = simlab.Cascade({outer, inner});
    cl.configLevel(0, 10, -20, 20);
    cl.configLevel(1, 1, 0, 0);        % min >= max disables the clamp

    yInner = 0.0;
    yOuter = 0.0;
    u = 0.0;
    for k = 1:n
        if k - 1 >= 500
            sp = 30.0;                  % unreachable: the inner saturates
        else
            sp = 0.0;
        end
        u = cl.update([yOuter; yInner], sp, dt);
        yInner = yInner + (0.001 / 0.01) * (u - yInner);
        yOuter = yOuter + (0.001 / 0.2) * (yInner - yOuter);
    end

    if haveRef
        T = simlab_tests.near(T, yOuter, ...
            simlab_tests.refGet(T, 'cascade.yOuter'), 1e-9, 'cascade yOuter');
        T = simlab_tests.near(T, yInner, ...
            simlab_tests.refGet(T, 'cascade.yInner'), 1e-9, 'cascade yInner');
        T = simlab_tests.near(T, u, ...
            simlab_tests.refGet(T, 'cascade.u'), 1e-9, 'cascade actuator command');
        T = simlab_tests.near(T, outer.getIntegrator(), ...
            simlab_tests.refGet(T, 'cascade.integratorOuter'), 1e-9, ...
            'outer integrator - the number the backward pass decides');
        T = simlab_tests.near(T, inner.getIntegrator(), ...
            simlab_tests.refGet(T, 'cascade.integratorInner'), 1e-9, ...
            'inner integrator');
        T = simlab_tests.eq(T, double(cl.isSaturated()), ...
            simlab_tests.refGet(T, 'cascade.saturated'), 'saturation reported');
        [rc, ratio] = cl.validate();
        T = simlab_tests.eq(T, rc, ...
            simlab_tests.refGet(T, 'cascade.validate'), 'validate() verdict matches C');
        T = simlab_tests.near(T, ratio, ...
            simlab_tests.refGet(T, 'cascade.minRatio'), 1e-9, 'timescale separation ratio');
    else
        T = simlab_tests.skip(T, 'cascade vs C reference', ...
            'c_reference.csv not found');
    end

    % ---- the backward pass is doing real work ----
    %
    % Same cascade with the anti-windup turned off. If the outer integrator
    % ends up in the same place, the correction was not happening and every
    % number above is coincidence.
    cfgO2 = cfgO; cfgI2 = cfgI;
    outer2 = pidx.PID(cfgO2);
    inner2 = pidx.PID(cfgI2);
    cl2 = simlab.Cascade({outer2, inner2});
    cl2.configLevel(0, 10, -20, 20);
    cl2.setAntiWindup(cl2.AW_NONE, 0);

    yI = 0.0; yO = 0.0;
    for k = 1:n
        if k - 1 >= 500, sp = 30.0; else, sp = 0.0; end
        u2 = cl2.update([yO; yI], sp, dt);
        yI = yI + (0.001 / 0.01) * (u2 - yI);
        yO = yO + (0.001 / 0.2) * (yI - yO);
    end
    T = simlab_tests.ok(T, outer2.getIntegrator() > outer.getIntegrator(), ...
        'without back-propagation the outer integrator winds further (%.4g vs %.4g)', ...
        outer2.getIntegrator(), outer.getIntegrator());
    T = simlab_tests.ok(T, abs(outer.getIntegrator()) <= 20 + 1e-6, ...
        'with back-propagation the outer integrator stays inside its command clamp (%.4g)', ...
        outer.getIntegrator());

    % ---- the decimation integrates over the right interval ----
    %
    % A level with decimation 10 must integrate over 10*dt, not dt, or it
    % under-integrates by exactly its decimation factor. Compared against a
    % single loop running at the outer rate on the same data.
    solo = pidx.PID(cfgO);
    solo.reset();
    solo.setSetpointImmediate(30);
    uSolo = solo.updateDt(yOuter, 0.01);
    outerS = pidx.PID(cfgO);
    innerS = pidx.PID(cfgI);
    clS = simlab.Cascade({outerS, innerS});
    clS.configLevel(0, 10, -20, 20);
    uS = 0;
    for k = 1:10
        uS = clS.update([yOuter; 0], 30, dt);
    end
    T = simlab_tests.ok(T, abs(outerS.getIntegrator() - solo.getIntegrator()) ...
        < 1e-9 * max(1, abs(solo.getIntegrator())), ...
        'a decimation-10 outer loop integrates over 10*dt, matching a loop at dt=0.01 (%.10g vs %.10g)', ...
        outerS.getIntegrator(), solo.getIntegrator());

    % ---- coordinated mode change ----
    rc = cl.setMode(K.MODE_MANUAL);
    T = simlab_tests.eq(T, rc, K.OK, 'setMode(MANUAL) on the chain returns OK');
    T = simlab_tests.eq(T, outer.getMode(), K.MODE_MANUAL, 'outer is in MANUAL');
    T = simlab_tests.eq(T, inner.getMode(), K.MODE_MANUAL, 'inner is in MANUAL');
    % The outer loop's manual output must equal what the inner loop is
    % actually following, so that re-engaging is bumpless at every level.
    T = simlab_tests.near(T, outer.getManualOutput(), inner.getSetpoint(), 1e-12, ...
        'outer manual output tracks the inner loop''s setpoint');
    rc = cl.setMode(K.MODE_AUTOMATIC);
    T = simlab_tests.eq(T, rc, K.OK, 'setMode(AUTOMATIC) returns OK');
    T = simlab_tests.eq(T, outer.getMode(), K.MODE_AUTOMATIC, 'outer back in AUTO');

    % ---- validation catches loops that are too close together ----
    fast1 = pidx.PID(pidx.config('kp', 1, 'ki', 1, 'dt', 0.01));
    fast2 = pidx.PID(pidx.config('kp', 1, 'ki', 1, 'dt', 0.008));
    clBad = simlab.Cascade({fast1, fast2});
    [rcBad, ratioBad] = clBad.validate();
    T = simlab_tests.eq(T, rcBad, K.ERR_INVALID_PARAM, ...
        'a 1.25x separation is flagged as too close');
    T = simlab_tests.near(T, ratioBad, 1.25, 1e-12, 'reported ratio');
end
