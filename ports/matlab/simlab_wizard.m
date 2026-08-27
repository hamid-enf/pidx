function simlab_wizard()
%SIMLAB_WIZARD  Guided console workflow: plant -> tune -> verify -> export.
%
%   simlab_wizard
%
% The same sequence as the graphical interface, driven by menu() and
% input(), so it works over ssh, in Octave, and in a MATLAB with no
% App Designer. Nothing here needs a toolbox.
%
% The menu is deliberately linear, because that is the order the work has to
% happen in:
%
%   1  choose the plant, and check it looks like your process
%   2  choose how the loop is disturbed - a bare step is not a scenario
%   3  set the gains: by hand, or identified from the plant with the same
%      relay/step engine the STM32 runs
%   4  run it, look at the metrics and the margins
%   5  ask what happens when the plant is wrong
%   6  write the C file for STM32CubeIDE
%
% Everything the wizard does is also one function call; the last menu entry
% prints the exact calls for the session so far, so a wizard run can always
% be turned into a script.

    S = struct();
    S.plant = [];
    S.scenario = [];
    S.cfg = [];
    S.ctrl = [];
    S.result = [];
    S.sens = [];
    S.tune = [];
    S.history = {};

    fprintf('\n');
    fprintf('=====================================================\n');
    fprintf('  PIDX simulation wizard\n');
    fprintf('  library %s   runtime %s\n', pidx.Const.VERSION_STRING, version);
    fprintf('=====================================================\n');

    while true
        fprintf('\n');
        fprintf('  plant    : %s\n', describePlant(S.plant));
        fprintf('  scenario : %s\n', describeScenario(S.scenario));
        fprintf('  gains    : %s\n', describeGains(S.cfg));
        fprintf('  result   : %s\n', describeResult(S.result));
        fprintf('\n');
        choice = menu('PIDX simulation', { ...
            '1. Choose / define the plant', ...
            '2. Choose the scenario', ...
            '3. Set the gains by hand', ...
            '4. Auto-tune (relay or step identification)', ...
            '5. Run the simulation', ...
            '6. Robustness study (Monte Carlo)', ...
            '7. Compare all nine tuning rules', ...
            '8. Export C for STM32CubeIDE', ...
            '9. Print the commands for this session', ...
            '0. Quit'});

        switch choice
            case 1, S = stepPlant(S);
            case 2, S = stepScenario(S);
            case 3, S = stepGains(S);
            case 4, S = stepTune(S);
            case 5, S = stepRun(S);
            case 6, S = stepMonteCarlo(S);
            case 7, S = stepCompareRules(S);
            case 8, S = stepExport(S);
            case 9, printHistory(S);
            otherwise
                fprintf('\nbye\n');
                return;
        end
    end
end

% ===========================================================================
% Steps
% ===========================================================================

function S = stepPlant(S)
    names = {'heater', 'extruder', 'flowValve', 'pressure', 'level', ...
             'dcMotor', 'servoPos', 'slowThermal', 'custom FOPDT'};
    c = menu('Choose a plant', [names, {'cancel'}]);
    if c > numel(names), return; end

    if c == numel(names)
        k = askNum('static gain K', 2.0);
        tau = askNum('time constant tau [s]', 45.0);
        l = askNum('dead time L [s]', 12.0);
        dt = askNum('sample time dt [s]', 0.1);
        umin = askNum('actuator min', 0);
        umax = askNum('actuator max', 100);
        S.plant = simlab.Plant('fopdt', 'name', 'custom', 'k', k, ...
            'tau', tau, 'l', l);
        S.plant.setActuatorLimits(umin, umax);
        S.history{end + 1} = sprintf( ...
            'plant = simlab.Plant(''fopdt'', ''k'', %g, ''tau'', %g, ''l'', %g);', k, tau, l);
        S.history{end + 1} = sprintf('plant.setActuatorLimits(%g, %g);', umin, umax);
        S.history{end + 1} = sprintf('dt = %g;', dt);
    else
        S.plant = simlab.Plant.presets(names{c});
        S.history{end + 1} = sprintf('plant = simlab.Plant.presets(''%s'');', names{c});
    end

    % Sensor realism. Off by default is a lie about any real board, so ask.
    if askYesNo(sprintf(['add sensor realism? (%s)'], describeSensor(S.plant)))
        sigma = askNum('measurement noise sigma', 0);
        bits = askNum('ADC bits (0 = none)', 0);
        if sigma > 0
            S.plant.setNoise(sigma);
            S.history{end + 1} = sprintf('plant.setNoise(%g);', sigma);
        end
        if bits > 0
            lo = askNum('ADC range min', 0);
            hi = askNum('ADC range max', 100);
            S.plant.setAdcBits(bits, lo, hi);
            S.history{end + 1} = sprintf('plant.setAdcBits(%d, %g, %g);', bits, lo, hi);
        end
    end

    fprintf('\n  plant: %s\n', describePlant(S.plant));
    if askYesNo('plot the open-loop step response?')
        dt = S.plant.dt;
        if ~(dt > 0), dt = 0.1; end
        c0 = pidx.PID(pidx.config('dt', dt));
        sc = simlab.Scenario('open loop', 8 * S.plant.tau());
        sc.setpoint(0, 0);
        % Drive the plant in MANUAL at a fixed level: that IS an open-loop
        % step test, and it is what the step identification would do.
        c0.setMode(pidx.Const.MODE_MANUAL);
        c0.setManualOutput(0.5 * max(1, S.plant.actuatorParam('umax')));
        r0 = simlab.Sim(S.plant, c0, sc).run();
        simlab.plot(r0, struct('fig', 90, 'title', 'open-loop step (manual)'));
        fprintf('  K = %.4g   tau = %.4g s   L = %.4g s\n', ...
            S.plant.steadyStateGain(), S.plant.tau(), S.plant.transportDelay());
    end
    S.result = [];   % the old result described a different plant
end

function S = stepScenario(S)
    names = {'stepResponse', 'disturbance', 'windup', 'noise', ...
             'sensorFault', 'agingPlant', 'setpointProfile', 'custom'};
    c = menu('Choose a scenario', [names, {'cancel'}]);
    if c > numel(names), return; end

    if c < numel(names)
        sp = askNum('setpoint', 100);
        tEnd = askNum('duration [s]', []);
        if isempty(tEnd)
            S.scenario = simlab.Scenario.presets(names{c}, 'sp', sp);
        else
            S.scenario = simlab.Scenario.presets(names{c}, 'sp', sp, 'tEnd', tEnd);
        end
        S.history{end + 1} = sprintf( ...
            'scenario = simlab.Scenario.presets(''%s'', ''sp'', %g);', names{c}, sp);
    else
        S.scenario = simlab.Scenario('custom', askNum('duration [s]', 30));
        S.scenario.setpoint(0, 0);
        S.scenario.setpoint(askNum('setpoint', 100), askNum('step at t [s]', 1));
        if askYesNo('add a load step?')
            S.scenario.loadStep(askNum('load magnitude', 0.3), ...
                                askNum('load at t [s]', 15));
        end
        if askYesNo('add measurement noise part way through?')
            S.scenario.noise(askNum('noise sigma', 0.02), ...
                             askNum('noise from t [s]', 15));
        end
        S.history{end + 1} = '% scenario built interactively - see simlab.Scenario';
    end
    fprintf('\n%s\n', S.scenario.describe());
    S.result = [];
end

function S = stepGains(S)
    if isempty(S.plant)
        fprintf('  choose a plant first\n'); return;
    end
    dt = askNum('sample time dt [s]', defaultDt(S.plant));
    kp = askNum('Kp', 1.0);
    ki = askNum('Ki', 0.1);
    kd = askNum('Kd', 0);
    tf = askNum('derivative filter Tf [s] (0 = Td/10)', 0);

    S.cfg = pidx.config('kp', kp, 'ki', ki, 'kd', kd, 'dt', dt);
    [lo, hi] = S.plant.actuatorLimits();
    if isfinite(lo) && isfinite(hi)
        S.cfg.limits.use_output_limits = true;
        S.cfg.limits.output_min = lo;
        S.cfg.limits.output_max = hi;
    end
    S.cfg.integral.mode = askAntiWindup();
    if tf > 0
        S.cfg.filter.tf = tf;
    end
    S.ctrl = pidx.PID(S.cfg);
    S.history{end + 1} = sprintf( ...
        'cfg = pidx.config(''kp'', %g, ''ki'', %g, ''kd'', %g, ''dt'', %g);', kp, ki, kd, dt);
    S.history{end + 1} = sprintf('cfg.integral.mode = %d;', S.cfg.integral.mode);
    fprintf('  gains set: Kp=%.4g Ki=%.4g Kd=%.4g dt=%.4g\n', kp, ki, kd, dt);
    S.result = [];
end

function S = stepTune(S)
    K = pidx.Const;
    if isempty(S.plant)
        fprintf('  choose a plant first\n'); return;
    end
    dt = askNum('sample time dt [s]', defaultDt(S.plant));
    c = menu('Identification method', { ...
        'relay (closed loop, keeps the process near its setpoint)', ...
        'step (open loop, gives a full FOPDT model)', 'cancel'});
    if c == 3, return; end

    if c == 1
        ident = K.IDENT_RELAY;
        rules = {'TYREUS_LUYBEN', 'ZN', 'PESSEN', 'SOME_OVERSHOOT', ...
                 'NO_OVERSHOOT', 'AMIGO_FREQ'};
    else
        ident = K.IDENT_STEP;
        rules = {'AMIGO_STEP', 'IMC', 'COHEN_COON'};
    end
    rc_ = menu('Tuning rule', [rules, {'cancel'}]);
    if rc_ > numel(rules), return; end

    cfg = simlab.AutoTune.configDefault(ident);
    cfg.rule = ruleId(rules{rc_});
    [lo, hi] = S.plant.actuatorLimits();
    span = 1;
    if isfinite(lo) && isfinite(hi), span = hi - lo; end
    cfg.output_step = askNum('excitation amplitude (relay h / step size)', ...
                             0.2 * span);
    cfg.hysteresis = askNum('relay hysteresis (2-3x the noise sigma)', 0);
    cfg.bias = askNum('bias u0 (the output that holds the setpoint)', 0.5 * span);
    cfg.auto_bias = false;
    if isfinite(lo) && isfinite(hi)
        cfg.output_min = lo;
        cfg.output_max = hi;
    end
    cfg.timeout_s = askNum('timeout [s]', 20 * max(1, S.plant.tau()));

    at = simlab.AutoTune(cfg);
    ctrl = pidx.PID(pidx.config('dt', dt));
    sp = askNum('operating point (setpoint for the experiment)', ...
                0.5 * S.plant.steadyStateGain() * span);

    S.plant.reset();
    at.start(ctrl, sp);
    y = 0;
    nMax = round(cfg.timeout_s / dt);
    lastPct = -1;
    for k = 1:nMax
        u = at.update(y, dt);
        y = S.plant.update(u, dt);
        pct = at.getProgress();
        if pct ~= lastPct
            fprintf('    %s %3d%%\r', ...
                simlab.AutoTune.stateToString(at.getState()), pct);
            lastPct = pct;
        end
        if ~at.isRunning(), break; end
    end
    fprintf('\n');

    [rc, res] = at.getResult();
    if rc ~= K.OK
        fprintf('  identification FAILED: %s\n', K.statusToString(rc));
        fprintf('  this is the tuner refusing to hand back gains it does not\n');
        fprintf('  believe - see docs/14_autotune.md for what each code means.\n');
        return;
    end

    fprintf('\n  identified model:\n');
    if res.model.kind == K.MODEL_FREQ
        fprintf('    Ku = %.6g   Pu = %.6g s\n', res.model.ku, res.model.pu);
    else
        fprintf('    K  = %.6g   T = %.6g s   L = %.6g s\n', ...
            res.model.k, res.model.t, res.model.l);
    end
    fprintf('    quality %d/100, asymmetry %.3f, %d cycle(s)\n', ...
        double(res.model.quality), res.asymmetry, res.cycles_used);
    fprintf('  gains: Kp = %.6g  Ki = %.6g  Kd = %.6g  Tf = %.6g\n', ...
        res.gains.kp, res.gains.ki, res.gains.kd, res.gains.tf);
    fprintf('  (Ti = %.6g s, Td = %.6g s)\n', res.gains.ti, res.gains.td);

    S.cfg = pidx.config('kp', res.gains.kp, 'ki', res.gains.ki, ...
        'kd', res.gains.kd, 'dt', dt);
    if isfinite(lo) && isfinite(hi)
        S.cfg.limits.use_output_limits = true;
        S.cfg.limits.output_min = lo;
        S.cfg.limits.output_max = hi;
    end
    if res.gains.tf > 0
        S.cfg.filter.tf = res.gains.tf;
    end
    S.ctrl = pidx.PID(S.cfg);
    S.tune = res;
    S.history{end + 1} = sprintf( ...
        '%% identified %s, quality %d', rules{rc_}, double(res.model.quality));
    S.history{end + 1} = sprintf( ...
        'cfg = pidx.config(''kp'', %.10g, ''ki'', %.10g, ''kd'', %.10g, ''dt'', %g);', ...
        res.gains.kp, res.gains.ki, res.gains.kd, dt);
    S.result = [];
    fprintf('\n  gains are loaded. Run the simulation to check them.\n');
end

function S = stepRun(S)
    if isempty(S.plant) || isempty(S.ctrl) || isempty(S.scenario)
        fprintf('  a plant, a controller and a scenario are all required\n');
        return;
    end
    S.plant.reset();
    S.result = simlab.Sim(S.plant, S.ctrl, S.scenario).run();
    S.sens = [];
    try
        S.sens = simlab.sensitivity(S.plant, struct('kp', S.cfg.core.kp, ...
            'ki', S.cfg.core.ki, 'kd', S.cfg.core.kd), ...
            struct('dt', S.cfg.core.sample_time, 'tf', S.cfg.filter.tf));
    catch err
        fprintf('  (frequency analysis unavailable: %s)\n', err.message);
    end
    simlab.plot(S.result, struct('fig', 91));
    if ~isempty(S.sens)
        simlab.plotSensitivity(S.sens, struct('fig', 92));
    end
    printMetrics(S.result, S.sens);
    S.history{end + 1} = 'result = simlab.Sim(plant, pidx.PID(cfg), scenario).run();';
    S.history{end + 1} = 'simlab.plot(result);';
end

function S = stepMonteCarlo(S)
    if isempty(S.plant) || isempty(S.cfg)
        fprintf('  a plant and gains are required\n'); return;
    end
    n = askNum('number of perturbed plants', 60);
    spread = askNum('parameter spread (2 = 0.5x..2x)', 2);
    g = struct('kp', S.cfg.core.kp, 'ki', S.cfg.core.ki, ...
        'kd', S.cfg.core.kd, 'dt', S.cfg.core.sample_time);
    [lo, hi] = S.plant.actuatorLimits();
    if isfinite(lo) && isfinite(hi)
        g.outMin = lo; g.outMax = hi;
    end
    sc = S.scenario;
    if isempty(sc), sc = simlab.Scenario.presets('disturbance'); end
    mc = simlab.monteCarlo(S.plant, g, struct('nRuns', n, 'spread', spread, ...
        'scenario', sc, 'dt', S.cfg.core.sample_time));
    fprintf('\n  %.0f%% of %d plants stayed stable\n', 100 * mc.share, n);
    fprintf('  median IAE %.4g, worst %.4g\n', mc.iae.median, mc.iae.worst);
    fprintf('  worst case at K x%.2f, tau x%.2f, L x%.2f\n', ...
        mc.worstPlant(1), mc.worstPlant(2), mc.worstPlant(3));
    if mc.share < 0.9
        fprintf(['  BELOW 90%%: these gains depend on the model being ' ...
                 'right. Detune, or fix the model.\n']);
    end
    S.history{end + 1} = sprintf('mc = simlab.monteCarlo(plant, gains, struct(''nRuns'', %d));', n);
end

function S = stepCompareRules(S)
    if isempty(S.plant)
        fprintf('  choose a plant first\n'); return;
    end
    c = menu('Compare the rules against...', { ...
        'a perfect model (how fast could this be)', ...
        'wrong plants, 0.5x..2x (how often will it work) - RECOMMENDED', ...
        'the perfect model with a much worse sensor', 'cancel'});
    if c == 4, return; end
    modes = {'exact', 'robust', 'noisy'};
    n = askNum('plants per rule', 20);
    dt = askNum('sample time dt [s]', defaultDt(S.plant));
    sc = S.scenario;
    if isempty(sc), sc = simlab.Scenario.presets('stepResponse'); end
    cr = simlab.compareRules(S.plant, struct('mode', modes{c}, ...
        'nRuns', n, 'dt', dt, 'scenario', sc));
    simlab.plotRules(cr, struct('fig', 93));
    fprintf('\n  %s\n', cr.recommend);
    fprintf('  %s\n', cr.rankCorrelationNote);
    S.history{end + 1} = sprintf( ...
        'cr = simlab.compareRules(plant, struct(''mode'', ''%s'', ''nRuns'', %d));', ...
        modes{c}, n);
end

function S = stepExport(S)
    if isempty(S.plant) || isempty(S.cfg)
        fprintf('  a plant and gains are required\n'); return;
    end
    dirName = askStr('output directory', fullfile(pwd, 'simlab_export'));
    sym = askStr('C symbol prefix', 'pidxLoop');
    prof = askStr('PIDX profile (MINIMAL|MOTION|PROCESS|FULL)', 'FULL');
    out = simlab.exportSTM32(S.plant, S.cfg, struct('dir', dirName, ...
        'symbol', sym, 'profile', upper(prof), 'result', S.result, ...
        'sens', S.sens, 'tune', S.tune));
    fprintf('\n  next, in your CubeIDE project:\n');
    fprintf('    1. add PIDX src/*.c to the project, include/ to the path\n');
    fprintf('    2. copy these two files into Core/Src\n');
    fprintf('    3. call %s_init() once after HAL_Init()\n', sym);
    fprintf('    4. call %s_tick(y) from the timer ISR at %.0f Hz\n', ...
        sym, 1 / S.cfg.core.sample_time);
    if askYesNo('also write the CSV/JSON report?')
        simlab.exportReport(S.result, struct('dir', dirName, ...
            'name', sym, 'cfg', S.cfg, 'plant', S.plant, ...
            'sens', S.sens, 'tune', S.tune));
    end
    S.history{end + 1} = sprintf( ...
        'simlab.exportSTM32(plant, cfg, struct(''dir'', ''%s'', ''symbol'', ''%s''));', ...
        dirName, sym);
end

% ===========================================================================
% Presentation
% ===========================================================================

function printMetrics(r, s)
    m = r.metrics;
    fprintf('\n  ---------------- metrics ----------------\n');
    fprintf('  rise 10-90%%    %12.5g s\n', m.riseTime);
    fprintf('  overshoot      %12.5g %%\n', m.overshoot);
    fprintf('  settling 2%%    %12.5g s\n', m.settlingTime);
    fprintf('  steady error   %12.5g\n', m.ssError);
    fprintf('  IAE / ITAE     %12.5g / %.5g\n', m.iae, m.itae);
    fprintf('  TV(u)          %12.5g\n', m.tv);
    fprintf('  saturated      %12.5g %% of samples\n', 100 * m.satFraction);
    fprintf('  stable         %12s\n', mat2str(logical(m.stable)));
    if ~isempty(s)
        fprintf('  ---------------- margins ---------------\n');
        fprintf('  Ms             %12.4g   (<1.4 good, >2 fragile)\n', s.Ms);
        fprintf('  gain margin    %12.4g x\n', s.gm);
        fprintf('  phase margin   %12.4g deg\n', s.pm);
        fprintf('  delay margin   %12.4g s\n', s.delayMargin);
        fprintf('  %s\n', s.verdict);
        for i = 1:numel(s.warnings)
            fprintf('  ! %s\n', s.warnings{i});
        end
    end
    fprintf('  ---------------------------------------\n');
end

function printHistory(S)
    fprintf('\n  %% ---- commands for this session ----\n');
    fprintf('  simlab_setup\n');
    for i = 1:numel(S.history)
        fprintf('  %s\n', S.history{i});
    end
    fprintf('  %% -----------------------------------\n');
end

function s = describePlant(p)
    if isempty(p)
        s = '(none)';
    else
        s = sprintf('%s [%s]  K=%.4g tau=%.4g L=%.4g  %s', ...
            p.name, p.kind, p.steadyStateGain(), p.tau(), ...
            p.transportDelay(), describeSensor(p));
    end
end

function s = describeSensor(p)
    if isempty(p)
        s = '';
        return;
    end
    bits = p.sensorParam('bits');
    sigma = p.sensorParam('sigma');
    [lo, hi] = p.actuatorLimits();
    s = sprintf('u in [%.4g, %.4g], noise %.3g, %d-bit ADC', ...
        lo, hi, sigma, bits);
end

function s = describeScenario(sc)
    if isempty(sc)
        s = '(none)';
    else
        s = sprintf('%s  (%d events, %.4g s)', sc.name, sc.nEvents, sc.tEnd);
    end
end

function s = describeGains(cfg)
    if isempty(cfg)
        s = '(none)';
    else
        s = sprintf('Kp=%.5g Ki=%.5g Kd=%.5g dt=%.4g AW=%d', ...
            cfg.core.kp, cfg.core.ki, cfg.core.kd, ...
            cfg.core.sample_time, cfg.integral.mode);
    end
end

function s = describeResult(r)
    if isempty(r)
        s = '(not run)';
    else
        s = sprintf('OS %.1f%%, settle %.4g s, IAE %.4g, stable %d', ...
            r.metrics.overshoot, r.metrics.settlingTime, r.metrics.iae, ...
            logical(r.metrics.stable));
    end
end

function dt = defaultDt(p)
    dt = p.dt;
    if ~(dt > 0)
        % A twentieth of the dominant time constant is a sane default for a
        % first run; the wizard asks, so nothing is silently assumed.
        dt = max(p.tau() / 20, 1e-4);
    end
end

% ===========================================================================
% Input helpers
% ===========================================================================

function v = askNum(prompt, default)
    if isempty(default)
        s = input(sprintf('  %s: ', prompt), 's');
    else
        s = input(sprintf('  %s [%s]: ', prompt, num2str(default, '%.6g')), 's');
    end
    if isempty(strtrim(s))
        v = default;
        return;
    end
    v = str2double(s);
    if isnan(v)
        fprintf('  not a number, using %s\n', num2str(default, '%.6g'));
        v = default;
    end
end

function s = askStr(prompt, default)
    t = input(sprintf('  %s [%s]: ', prompt, default), 's');
    if isempty(strtrim(t))
        s = default;
    else
        s = t;
    end
end

function y = askYesNo(prompt)
    t = lower(input(sprintf('  %s [y/N]: ', prompt), 's'));
    y = strcmp(t, 'y') || strcmp(t, 'yes');
end

function m = askAntiWindup()
    K = pidx.Const;
    c = menu('Anti-windup strategy', { ...
        'CLAMP (simplest, predictable) - default', ...
        'CONDITIONAL (stop integrating while saturated)', ...
        'BACK_CALCULATION (best disturbance recovery)', ...
        'NONE (only if the actuator cannot saturate)'});
    ids = [K.AW_CLAMP, K.AW_CONDITIONAL, K.AW_BACK_CALCULATION, K.AW_NONE];
    if c >= 1 && c <= numel(ids)
        m = ids(c);
    else
        m = K.AW_CLAMP;
    end
end

function r = ruleId(name)
    K = pidx.Const;
    switch name
        case 'TYREUS_LUYBEN',  r = K.RULE_TYREUS_LUYBEN;
        case 'ZN',             r = K.RULE_ZN;
        case 'PESSEN',         r = K.RULE_PESSEN;
        case 'SOME_OVERSHOOT', r = K.RULE_SOME_OVERSHOOT;
        case 'NO_OVERSHOOT',   r = K.RULE_NO_OVERSHOOT;
        case 'AMIGO_FREQ',     r = K.RULE_AMIGO_FREQ;
        case 'AMIGO_STEP',     r = K.RULE_AMIGO_STEP;
        case 'IMC',            r = K.RULE_IMC;
        case 'COHEN_COON',     r = K.RULE_COHEN_COON;
        otherwise,             r = K.RULE_AMIGO_STEP;
    end
end
