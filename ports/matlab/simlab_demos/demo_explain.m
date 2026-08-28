%DEMO_EXPLAIN  The loop as a story: what goes where, and why.
%
%   simlab_demos.demo_explain
%
% A step response shows WHAT happened. This demo shows WHERE each signal
% came from, WHERE it went, and WHY the stage between them exists - the
% answer to "هر چیزی از کجا به کجا، چرا". It tunes a heater, runs it, then
% walks the signal path with the numbers THIS run produced, and leaves the
% story as text you can hand over.

    plant = simlab.Plant.presets('heater');
    dt = 0.1;

    % tune with the relay, the way the board would
    cfgT = simlab.AutoTune.configDefault(pidx.Const.IDENT_RELAY);
    cfgT.output_step = 20; cfgT.hysteresis = 0.4;
    cfgT.auto_bias = false; cfgT.bias = 50;
    cfgT.output_min = 0; cfgT.output_max = 100;
    ctrl = pidx.PID(pidx.config('dt', dt));
    sc = simlab.Scenario.presets('stepResponse', 'sp', 150, 'tEnd', 900);
    r = simlab.Sim(plant, ctrl, sc, struct('tuner', simlab.AutoTune(cfgT))).run();

    cfg = pidx.config('kp', ctrl.kp, 'ki', ctrl.ki, 'kd', ctrl.kd, 'dt', dt);
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0; cfg.limits.output_max = 100;

    % the story, printed and drawn
    story = simlab.explain(r, plant, cfg, struct('fig', 96));

    fprintf('\n================ the story of this run ================\n');
    for k = 1:numel(story)
        fprintf('%s\n', story{k});
    end
    fprintf('========================================================\n');
    fprintf('figure 96 shows the same story as traces: the gap between\n');
    fprintf('two consecutive panels is exactly the stage named above it.\n');
