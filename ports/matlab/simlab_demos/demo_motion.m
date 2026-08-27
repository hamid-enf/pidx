%DEMO_MOTION  Cascade position control on a DC motor, tuned inner-first.
%
%   simlab_demos.demo_motion
%
% The motor is the one in examples_stm32/03_motor_speed with the same
% physical constants, so a MATLAB study here and that STM32 example are
% describing the same machine.
%
%   position  <--[ outer PID, 100 Hz ]--  speed  <--[ inner PID, 1 kHz ]--  voltage
%
% THE ORDER IS NOT OPTIONAL
%   Tune the inner loop first, verify it, then tune the outer loop against
%   the CLOSED inner loop. Tuning both against the raw plant and hoping is
%   how cascades end up fighting each other. This demo does it in the right
%   order and then calls Cascade.validate(), which checks the timescale
%   separation and reports it rather than silently accepting a bad ratio.
%
% THE PART THAT IS EASY TO GET WRONG
%   When the inner loop saturates at the supply rail, the outer loop's
%   integrator keeps charging against a command that cannot be delivered.
%   Outer output limits do NOT fix this - the outer output can be well inside
%   its own limits while the inner loop is pinned. simlab.Cascade propagates
%   the saturation backwards, and the demo measures the difference.

    K = pidx.Const;
    plant = simlab.Plant.presets('dcMotor');
    fprintf('\n  motor: R = %.4g, L = %.4g H, Ke = %.4g, Kt = %.4g\n', ...
        plant.modelParam('r'), plant.modelParam('l_elec'), ...
        plant.modelParam('ke'), plant.modelParam('kt'));
    fprintf('         J = %.4g, B = %.4g, Coulomb = %.4g, supply +-%.0f V\n', ...
        plant.modelParam('j'), plant.modelParam('b'), ...
        plant.modelParam('coulomb'), plant.actuatorParam('umax'));
    fprintf('  speed-loop gain Kt/(B+Kt*Ke/R) = %.4g rad/s per volt\n', ...
        plant.steadyStateGain());

    dtInner = 0.001;          % 1 kHz current/speed loop
    decim = 10;
    dtOuter = dtInner * decim;  % 100 Hz position loop

    % ---- 1. tune the speed loop alone ----
    fprintf('\n=== 1. the inner (speed) loop, alone ======================\n');
    % A speed loop on this motor is fast; use a step test with a short dt.
    cfgI = pidx.config('kp', 0.02, 'ki', 8.0, 'kd', 0, 'dt', dtInner);
    cfgI.limits.use_output_limits = true;
    cfgI.limits.output_min = -24;
    cfgI.limits.output_max = 24;
    inner = pidx.PID(cfgI);

    scSpeed = simlab.Scenario('speed step', 0.4);
    scSpeed.setpoint(0, 0);
    scSpeed.setpoint(100, 0.05);          % 100 rad/s ~ 955 rpm
    rSpeed = simlab.Sim(plant, inner, scSpeed).run();
    fprintf('  speed step: rise %.4g ms, overshoot %.2f%%, settle %.4g ms\n', ...
        1000 * rSpeed.metrics.riseTime, rSpeed.metrics.overshoot, ...
        1000 * rSpeed.metrics.settlingTime);
    simlab.plot(rSpeed, struct('fig', 71, 'title', 'inner speed loop alone'));

    % ---- 2. wire the cascade ----
    fprintf('\n=== 2. the cascade ========================================\n');
    cfgO = pidx.config('kp', 12.0, 'ki', 30.0, 'kd', 0.4, 'dt', dtOuter);
    cfgO.limits.use_output_limits = true;
    cfgO.limits.output_min = -200;
    cfgO.limits.output_max = 200;         % rad/s it may command
    outer = pidx.PID(cfgO);

    cl = simlab.Cascade({outer, inner});
    cl.configLevel(0, decim, -200, 200);  % outer runs every 10th call
    cl.configLevel(1, 1, 0, 0);

    [rc, ratio] = cl.validate();
    if rc == K.OK
        verdict = 'acceptable, >= 3x';
    else
        verdict = 'TOO CLOSE - the loops will fight each other';
    end
    fprintf('  timescale separation: %.1fx  (%s)\n', ratio, verdict);
    per = cl.levelPeriods();
    fprintf('  effective rates: outer %.0f Hz, inner %.0f Hz\n', ...
        1 / per(1), 1 / per(end));

    % The cascade needs a per-level measurement vector: level 0 measures
    % position, level 1 measures speed. motorState() returns [i; w; theta].
    measFn = @(pl) [pl.state('theta'); pl.state('speed')];

    scPos = simlab.Scenario('position move', 1.0);
    scPos.setpoint(0, 0);
    scPos.setpoint(20, 0.05);             % 20 rad ~ 1146 deg
    scPos.loadStep(0.004, 0.5);           % load torque lands mid-move

    rC = simlab.Sim(plant, [], scPos, struct('cascade', cl, ...
        'measFn', measFn)).run();
    fprintf('  position move: rise %.4g ms, overshoot %.2f%%, settle %.4g ms\n', ...
        1000 * rC.metrics.riseTime, rC.metrics.overshoot, ...
        1000 * rC.metrics.settlingTime);
    fprintf('  peak command %.4g V, %.0f%% of samples saturated\n', ...
        rC.metrics.uPeak, 100 * rC.metrics.satFraction);
    simlab.plot(rC, struct('fig', 72, 'title', ...
        'cascade position move with a load step'));

    % ---- 3. what the backward pass buys you ----
    fprintf('\n=== 3. cascade anti-windup: with and without ==============\n');
    scSat = simlab.Scenario('demanding move', 1.0);
    scSat.setpoint(0, 0);
    scSat.setpoint(200, 0.05);            % far beyond what 24 V can do

    modes = {cl.AW_BACK_CALC, 'BACK_CALC'; cl.AW_FREEZE, 'FREEZE'; ...
             cl.AW_NONE, 'NONE'};
    fprintf('  %-12s %10s %10s %12s\n', 'strategy', 'IAE', 'peak u', ...
        'outer I term');
    for i = 1:size(modes, 1)
        pl2 = simlab.Plant.presets('dcMotor');
        o2 = pidx.PID(cfgO);
        i2 = pidx.PID(cfgI);
        c2 = simlab.Cascade({o2, i2});
        c2.configLevel(0, decim, -200, 200);
        c2.setAntiWindup(modes{i, 1}, 0);
        rr = simlab.Sim(pl2, [], scSat, struct('cascade', c2, ...
            'measFn', measFn)).run();
        fprintf('  %-12s %10.4g %10.4g %12.4g\n', modes{i, 2}, ...
            rr.metrics.iae, rr.metrics.uPeak, o2.getIntegrator());
    end
    fprintf('\n  The outer integrator is the number to watch: with NONE it\n');
    fprintf('  charges against a wall, and the overshoot on the way back is\n');
    fprintf('  the cost. BACK_CALC bleeds it off as the shortfall appears.\n');

    % ---- 4. export both loops ----
    fprintf('\n=== 4. export =============================================\n');
    outI = simlab.exportSTM32(plant, cfgI, struct('symbol', 'speedLoop', ...
        'profile', 'MOTION', 'result', rSpeed));
    outO = simlab.exportSTM32(plant, cfgO, struct('symbol', 'positionLoop', ...
        'profile', 'MOTION', 'result', rC));
    fprintf('\n  %s\n  %s\n', outI.source, outO.source);
    fprintf('\n  The exported files configure the two loops separately. The\n');
    fprintf('  cascade wiring itself is a few lines of your own: see the\n');
    fprintf('  PID_Cascade example in examples/06_cascade_pos_vel_cur/main.c.\n\n');
