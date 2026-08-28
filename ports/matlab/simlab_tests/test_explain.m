function T = test_explain(T)
%SIMLAB_TESTS.TEST_EXPLAIN  Does the story name every stage of the loop?
%
% explain() is the "what goes where, why" view. It cannot be wrong about the
% arithmetic (it only reads the logged traces), but it CAN silently omit a
% stage, which would make the story a lie by absence. So the test demands
% every stage of the signal path appears, with the numbers of THIS run.

    plant = simlab.Plant.presets('heater');
    dt = 0.1;
    cfg = pidx.config('kp', 3, 'ki', 0.08, 'kd', 0, 'dt', dt);
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    sc = simlab.Scenario.presets('windup', 'sp', 100, 'tEnd', 600);
    r = simlab.Sim(plant, pidx.PID(cfg), sc).run();

    story = simlab.explain(r, plant, cfg, struct('fig', []));   % headless

    joined = strjoin(story, ' | ');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'SETPOINT')), ...
        'the story starts at the setpoint');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'INTEGRATOR')), ...
        'names the integrator and the offset it eats');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'saturated')), ...
        'names the saturation and why limits exist');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'ACTUATOR')), ...
        'names the actuator chain');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'PLANT')), ...
        'names the plant with K/tau/L');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'SENSOR')), ...
        'names the sensor chain and its noise');
    T = simlab_tests.ok(T, ~isempty(strfind(joined, 'CLOSE THE LOOP')), ...
        'and closes the story where the loop closes');

    % the numbers in the story must be THIS run's numbers
    satPct = 100 * mean(bitand(double(r.flags), ...
        pidx.Const.FLAG_SATURATED) ~= 0);
    T = simlab_tests.ok(T, ~isempty(strfind(joined, ...
        sprintf('%.1f%%', satPct))), ...
        'the saturation percentage quoted is measured from this run (%.1f%%)', ...
        satPct);
    T = simlab_tests.ok(T, ~isempty(strfind(joined, ...
        sprintf('%d smp', round(plant.transportDelay() / dt)))), ...
        'the dead time is quoted in samples of this run');
end
