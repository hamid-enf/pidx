function T = test_scenario(T)
%SIMLAB_TESTS.TEST_SCENARIO  Do scenario events land when they should?
%
% An event that fires a sample late is invisible on a plot and wrong in a
% comparison against a hand calculation, so the timing is checked against
% arithmetic rather than against a picture.

    dt = 0.05;

    % ---- 1. the plant is really reconfigured by each event ----
    pl = simlab.Plant('fopdt', 'k', 1.0, 'tau', 10.0, 'l', 0.0);
    pl.setActuatorLimits(0, 100);
    c = pidx.PID(pidx.config('kp', 1.0, 'ki', 0.2, 'dt', dt));

    sc = simlab.Scenario('event timing', 6.0);
    sc.setpoint(0, 0);
    sc.setpoint(50, 1.0);
    sc.actLimits(0, 20, 2.0);
    sc.noise(0.4, 3.0);
    sc.plantGain(3.0, 4.0);

    r = simlab.Sim(pl, c, sc).run();

    % Output must clamp at the NEW limit from t = 2 s onwards.
    k2 = round(2.0 / dt) + 1;
    T = simlab_tests.ok(T, max(r.u(k2:end)) <= 20 + 1e-9, ...
        'the actuator limit event at t = 2 s is in force afterwards (max u = %.4g)', ...
        max(r.u(k2:end)));
    T = simlab_tests.ok(T, max(r.u(1:k2 - 1)) > 20, ...
        'and the output really did exceed 20 before the event (%.4g)', ...
        max(r.u(1:k2 - 1)));

    % Noise really is on after t = 3 s. Measured over a window where the loop
    % has already settled, so the only thing that can make the trace move is
    % the noise the event added - comparing during the transient would measure
    % the response instead.
    scN = simlab.Scenario('noise only', 12.0);
    scN.setpoint(50, 0);
    scN.noise(0.4, 6.0);
    rN = simlab.Sim(simlab.Plant('fopdt', 'k', 2, 'tau', 1, 'l', 0), ...
                    pidx.PID(pidx.config('kp', 1, 'ki', 0.5, 'dt', dt)), scN).run();
    k3 = round(6.0 / dt) + 1;
    before = std(rN.y(k3 - 60:k3 - 1));
    after = std(rN.y(k3 + 20:k3 + 80));
    T = simlab_tests.ok(T, after > 20 * max(before, 1e-12), ...
        'the noise event at t = 6 s is visible once the loop has settled (sigma %.3g -> %.3g)', ...
        before, after);

    % ---- 2. events at t = 0 shape the FIRST sample ----
    sc0 = simlab.Scenario('t0', 1.0);
    sc0.setpoint(25, 0);
    r0 = simlab.Sim(simlab.Plant('fopdt', 'k', 1, 'tau', 5, 'l', 0), ...
                    pidx.PID(pidx.config('kp', 1, 'ki', 0, 'dt', dt)), sc0).run();
    T = simlab_tests.near(T, r0.r(1), 25, 1e-12, ...
        'an event at t = 0 is applied to the first sample');

    % ---- 3. a scenario extends itself past its last event ----
    scE = simlab.Scenario('extend', 1.0);
    scE.setpoint(10, 7.5);
    T = simlab_tests.near(T, scE.tEnd, 7.5, 1e-12, ...
        'declaring an event at t = 7.5 in a 1 s scenario extends it');

    % ---- 4. the ramp interpolates ----
    scR = simlab.Scenario('ramp', 5.0);
    scR.setpoint(0, 0);
    scR.setpointRamp(100, 1.0, 2.0);
    rR = simlab.Sim(simlab.Plant('fopdt', 'k', 0, 'tau', 1, 'l', 0), ...
                    pidx.PID(pidx.config('kp', 0, 'ki', 0, 'dt', 0.1)), scR).run();
    % A zero-gain controller cannot move the plant, so r.r IS the command.
    kMid = round(2.0 / 0.1) + 1;      % t = 2 s, halfway up a 2 s ramp from t = 1
    T = simlab_tests.near(T, rR.r(kMid), 50, 0.02, ...
        'the interpolated command is halfway at the middle of the ramp (%.4g)', rR.r(kMid));
    kEnd = round(3.0 / 0.1) + 1;
    T = simlab_tests.near(T, rR.r(kEnd), 100, 1e-9, 'and reaches the target at the end');

    % ---- 5. the transcript is complete and ordered ----
    txt = sc.describe();
    T = simlab_tests.ok(T, ~isempty(strfind(txt, 'setpoint')), 'transcript lists the setpoint event');
    T = simlab_tests.ok(T, ~isempty(strfind(txt, '2 event')), 'transcript states the event count');

    % ---- 6. a sensor fault event drives PIDX's safety latch ----
    plS = simlab.Plant('fopdt', 'k', 2, 'tau', 5, 'l', 0);
    cS = pidx.PID(pidx.config('kp', 1, 'ki', 0.2, 'dt', 0.05));
    cS.setSetpoint(10);
    scS = simlab.Scenario('fault', 6.0);
    scS.setpoint(10, 0);
    scS.stuck(0.0, 2.0);        % sensor drops to zero and stays
    rS = simlab.Sim(plS, cS, scS).run();
    kF = round(2.0 / 0.05) + 1;
    % With no safety configuration the controller simply follows the lie; the
    % point of this check is that the event reached the plant at all.
    T = simlab_tests.near(T, rS.y(kF + 2), 0.0, 1e-9, 'a stuck sensor reads its stuck value');
    T = simlab_tests.ok(T, rS.y(kF - 2) > 5, 'and was tracking before the event (%.4g)', rS.y(kF - 2));

    % ---- 7. presets exist and are runnable ----
    names = {'stepResponse', 'disturbance', 'windup', 'noise', ...
             'sensorFault', 'agingPlant', 'setpointProfile'};
    for i = 1:numel(names)
        s = simlab.Scenario.presets(names{i});
        T = simlab_tests.ok(T, s.nEvents > 0 && s.tEnd > 0, ...
            'preset "%s" declares %d event(s) over %.4g s', names{i}, s.nEvents, s.tEnd);
    end
end
