%DEMO_THERMAL  A process loop end to end: tune, verify, break it, fix it.
%
%   simlab_demos.demo_thermal
%
% The heater is the case where the anti-windup strategy, the derivative
% filter and the setpoint weight all matter, and where a loop that looks
% perfect in a step response still fails on the board. This demo runs the
% four scenarios that find those failures:
%
%   1. a bare step                       - the baseline
%   2. a step the actuator cannot reach  - windup, and the recovery time
%   3. a load disturbance                - a different problem from tracking
%   4. a sensor that sticks and recovers - PIDX's safety latch
%
% Then it compares all five anti-windup strategies on scenario 2, which is
% the only scenario that separates them.

    K = pidx.Const;
    plant = simlab.Plant.presets('heater');
    dt = 0.1;
    sp = 150;

    % ---- tune with a step test, the way a process engineer would ----
    cfgT = simlab.AutoTune.configDefault(K.IDENT_STEP);
    cfgT.output_step = 30;
    cfgT.bias = 20;
    cfgT.auto_bias = false;
    cfgT.output_min = 0;
    cfgT.output_max = 100;
    cfgT.timeout_s = 3000;
    tuner = simlab.AutoTune(cfgT);
    ctrl = pidx.PID(pidx.config('dt', dt));

    warm = simlab.Scenario('warm-up for the step test', 200);
    warm.setpoint(0, 0);
    sim = simlab.Sim(plant, ctrl, warm, struct('tuner', tuner));
    sim.run();

    [rc, res] = tuner.getResult();
    if rc ~= K.OK
        error('demo:tune', 'step identification failed: %s', ...
            K.statusToString(rc));
    end
    fprintf('\n  identified  K = %.4g   T = %.4g s   L = %.4g s   quality %d\n', ...
        res.model.k, res.model.t, res.model.l, double(res.model.quality));
    fprintf('  true values K = %.4g   T = %.4g s   L = %.4g s\n', ...
        plant.steadyStateGain(), plant.tau(), plant.transportDelay());
    fprintf('  The fit is not exact and that is the honest result: the\n');
    fprintf('  settling test stops before the response is truly flat, so K\n');
    fprintf('  comes out high. The C library does the same thing.\n');

    cfg = pidx.config('kp', res.gains.kp, 'ki', res.gains.ki, ...
        'kd', res.gains.kd, 'dt', dt);
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    cfg.filter.tf = res.gains.tf;
    cfg.integral.mode = K.AW_BACK_CALCULATION;
    cfg.safety.enabled = true;
    cfg.safety.meas_min = -10;
    cfg.safety.meas_max = 350;
    cfg.safety.meas_rate_max = 25;
    cfg.safety.auto_recover = true;

    % ---- scenario 1: the baseline ----
    fprintf('\n=== 1. bare step to %.0f =================================\n', sp);
    sc1 = simlab.Scenario.presets('stepResponse', 'sp', sp, 'tEnd', 900);
    r1 = simlab.Sim(plant, pidx.PID(cfg), sc1).run();
        m = r1.metrics;
    fprintf('  %-12s rise %8.4g s  OS %7.2f%%  settle %8.4g s  IAE %9.4g  stable %d\n', ...
        'step', m.riseTime, m.overshoot, m.settlingTime, m.iae, ...
        logical(m.stable));
    simlab.plot(r1, struct('fig', 81, 'title', '1 bare step'));

    % ---- scenario 2: windup ----
    fprintf('\n=== 2. a step the actuator cannot reach ===================\n');
    sc2 = simlab.Scenario.presets('windup', 'sp', sp, 'tStep', 1, ...
        'tDist', 400, 'tEnd', 1200);
    r2 = simlab.Sim(plant, pidx.PID(cfg), sc2).run();
        m = r2.metrics;
    fprintf('  %-12s rise %8.4g s  OS %7.2f%%  settle %8.4g s  IAE %9.4g  stable %d\n', ...
        'windup', m.riseTime, m.overshoot, m.settlingTime, m.iae, ...
        logical(m.stable));
    simlab.plot(r2, struct('fig', 82, 'title', '2 windup and recovery'));

    % ---- scenario 3: disturbance ----
    fprintf('\n=== 3. load disturbance ===================================\n');
    sc3 = simlab.Scenario('disturbance', 900);
    sc3.setpoint(0, 0);
    sc3.setpoint(sp, 1);
    sc3.disturbPulse(-25, 500, 60);      % a cold slug, 60 s long
    r3 = simlab.Sim(plant, pidx.PID(cfg), sc3).run();
        m = r3.metrics;
    fprintf('  %-12s rise %8.4g s  OS %7.2f%%  settle %8.4g s  IAE %9.4g  stable %d\n', ...
        'disturb', m.riseTime, m.overshoot, m.settlingTime, m.iae, ...
        logical(m.stable));
    simlab.plot(r3, struct('fig', 83, 'title', '3 disturbance rejection'));

    % ---- scenario 4: sensor fault ----
    fprintf('\n=== 4. sensor sticks, then recovers =======================\n');
    sc4 = simlab.Scenario.presets('sensorFault', 'sp', sp, 'tFault', 400, ...
        'tFaultEnd', 520, 'tEnd', 1000);
    r4 = simlab.Sim(plant, pidx.PID(cfg), sc4).run();
        m = r4.metrics;
    fprintf('  %-12s rise %8.4g s  OS %7.2f%%  settle %8.4g s  IAE %9.4g  stable %d\n', ...
        'fault', m.riseTime, m.overshoot, m.settlingTime, m.iae, ...
        logical(m.stable));
    simlab.plot(r4, struct('fig', 84, 'title', '4 sensor fault and recovery'));
    faulted = bitand(double(r4.flags), K.FLAG_FAULT) ~= 0;
    fprintf('  the safety latch was set for %.0f s of the run\n', ...
        dt * sum(faulted));

    % ---- the comparison that actually matters: anti-windup ----
    fprintf('\n=== 5. five anti-windup strategies on the windup scenario ==\n');
    fprintf('  %-20s %10s %10s %10s %8s\n', 'strategy', 'IAE', 'overshoot', ...
        'settle', 'stable');
    strategies = {K.AW_NONE, 'NONE'; K.AW_CLAMP, 'CLAMP'; ...
                  K.AW_CONDITIONAL, 'CONDITIONAL'; ...
                  K.AW_BACK_CALCULATION, 'BACK_CALCULATION'; ...
                  K.AW_TRACKING, 'TRACKING'};
    best = inf;
    bestName = '';
    for i = 1:size(strategies, 1)
        c2 = cfg;
        c2.integral.mode = strategies{i, 1};
        try
            rr = simlab.Sim(plant, pidx.PID(c2), sc2).run();
            fprintf('  %-20s %10.4g %10.3g %10.4g %8d\n', strategies{i, 2}, ...
                rr.metrics.iae, rr.metrics.overshoot, ...
                rr.metrics.settlingTime, logical(rr.metrics.stable));
            if rr.metrics.iae < best && logical(rr.metrics.stable)
                best = rr.metrics.iae;
                bestName = strategies{i, 2};
            end
        catch err
            fprintf('  %-20s  (%s)\n', strategies{i, 2}, err.message);
        end
    end
    if ~isempty(bestName)
        fprintf('\n  lowest IAE on this scenario: %s\n', bestName);
        fprintf('  TRACKING is not a tuning choice - it needs an external\n');
        fprintf('  reset signal (the real actuator position) to track.\n');
    end

    % ---- export ----
    out = simlab.exportSTM32(plant, cfg, struct('symbol', 'heaterLoop', ...
        'profile', 'PROCESS', 'result', r1));
    fprintf('\n  exported: %s\n\n', out.source);
