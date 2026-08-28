%DEMO_QUICK  Five-minute tour of the PIDX simulation tool.
%
%   cd <repo>/ports/matlab
%   simlab_setup
%   simlab_demos.demo_quick
%
% One heater, four things you will actually do:
%   1. look at the open-loop plant, so you know what you are controlling
%   2. tune it with the same relay engine the STM32 runs
%   3. check the tuning against a plant that is wrong
%   4. write the C file for STM32CubeIDE
%
% Every line here is a plain function call, so the demo doubles as the
% reference for scripting the tool.

    fprintf('\n=== 1. the plant ==========================================\n');
    plant = simlab.Plant.presets('heater');
    fprintf('  %s: K = %.4g, tau = %.4g s, dead time = %.4g s\n', ...
        plant.name, plant.steadyStateGain(), plant.tau(), ...
        plant.transportDelay());
    fprintf('  actuator 0..100%%, 12-bit ADC, noise sigma 0.15 degC\n');
    cav = plant.analysisCaveats();
    for i = 1:numel(cav)
        fprintf('  ! %s\n', cav{i});
    end

    % An open-loop step, by holding a controller in MANUAL. That is exactly
    % what the step identification does, so this plot is what you would look
    % at on the board before trusting any model.
    dt = 0.1;
    manual = pidx.PID(pidx.config('dt', dt));
    manual.setMode(pidx.Const.MODE_MANUAL);
    manual.setManualOutput(50);
    r0 = simlab.Sim(plant, manual, ...
        simlab.Scenario('open loop', 400)).run();
    simlab.plot(r0, struct('fig', 90, 'title', ...
        'open-loop step at 50% output'));

    fprintf('\n=== 2. identify and tune ==================================\n');
    cfgT = simlab.AutoTune.configDefault(pidx.Const.IDENT_RELAY);
    cfgT.output_step = 20;          % relay half-amplitude, in output units
    cfgT.hysteresis = 0.4;          % ~3x the noise sigma
    cfgT.bias = 50;                 % the output that holds the setpoint
    cfgT.auto_bias = false;
    cfgT.output_min = 0;
    cfgT.output_max = 100;
    cfgT.timeout_s = 2000;

    tuner = simlab.AutoTune(cfgT);
    ctrl = pidx.PID(pidx.config('dt', dt));
    setpoint = 100;

    % Sim runs the tuner to completion, applies the gains, then plays the
    % scenario with them - the same order it happens on the target.
    scenario = simlab.Scenario.presets('stepResponse', 'sp', setpoint, ...
        'tEnd', 600);
    r = simlab.Sim(plant, ctrl, scenario, struct('tuner', tuner, ...
        'verbose', false)).run();

    [rc, res] = tuner.getResult();
    if rc ~= pidx.Const.OK
        error('demo:tuneFailed', 'identification failed: %s', ...
            pidx.Const.statusToString(rc));
    end
    fprintf('  identified  Ku = %.4g   Pu = %.4g s\n', res.model.ku, res.model.pu);
    fprintf('  quality     %d/100 over %d cycles\n', ...
        double(res.model.quality), res.cycles_used);
    fprintf('  gains       Kp = %.5g  Ki = %.5g  Kd = %.5g\n', ...
        res.gains.kp, res.gains.ki, res.gains.kd);
    fprintf('              Ti = %.5g s  Td = %.5g s  Tf = %.5g s\n', ...
        res.gains.ti, res.gains.td, res.gains.tf);
    fprintf('  NOTE the relay under-estimates Ku by design - see README.md.\n');

    simlab.plot(r, struct('fig', 91));
    fprintf('\n  measured: rise %.4g s, overshoot %.1f%%, settling %.4g s\n', ...
        r.metrics.riseTime, r.metrics.overshoot, r.metrics.settlingTime);
    fprintf('            IAE %.4g, saturated %.0f%% of samples\n', ...
        r.metrics.iae, 100 * r.metrics.satFraction);

    fprintf('\n=== 3. what if the plant is wrong? ========================\n');
    s = simlab.sensitivity(plant, struct('kp', res.gains.kp, ...
        'ki', res.gains.ki, 'kd', res.gains.kd), ...
        struct('dt', dt, 'tf', res.gains.tf));
    fprintf('  %s\n', s.verdict);
    simlab.plotSensitivity(s, struct('fig', 92));

    gains = struct('kp', res.gains.kp, 'ki', res.gains.ki, ...
        'kd', res.gains.kd, 'dt', dt, 'tf', res.gains.tf, ...
        'outMin', 0, 'outMax', 100);
    mc = simlab.monteCarlo(plant, gains, struct('nRuns', 40, 'spread', 2, ...
        'scenario', scenario, 'verbose', false));
    fprintf('  %.0f%% of 40 plants with K/tau/L each 0.5x..2x stayed stable\n', ...
        100 * mc.share);
    fprintf('  median IAE %.4g, worst %.4g\n', mc.iae.median, mc.iae.worst);

    fprintf('\n=== 4. export for STM32CubeIDE ============================\n');
    cfg = pidx.config('kp', res.gains.kp, 'ki', res.gains.ki, ...
        'kd', res.gains.kd, 'dt', dt);
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    cfg.filter.tf = res.gains.tf;
    cfg.safety.enabled = true;
    cfg.safety.meas_min = -10;
    cfg.safety.meas_max = 350;
    cfg.safety.meas_rate_max = 20;
    cfg.safety.auto_recover = true;

    out = simlab.exportSTM32(plant, cfg, struct('symbol', 'heaterLoop', ...
        'profile', 'FULL', 'result', r, 'sens', s, 'tune', res, ...
        'dir', fullfile(pwd, 'simlab_export')));
    simlab.exportReport(r, struct('name', 'demo_quick', 'cfg', cfg, ...
        'plant', plant, 'sens', s, 'tune', res, ...
        'dir', fullfile(pwd, 'simlab_export')));

    fprintf('\n  exported: %s\n', out.source);
    fprintf('\n  next: simlab_demos.demo_thermal for the full process study,\n');
    fprintf('        simlab_demos.demo_design to design to a specification,\n');
    fprintf('        or simlab_wizard to drive this interactively.\n\n');
