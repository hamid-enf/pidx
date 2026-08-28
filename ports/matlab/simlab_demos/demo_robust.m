%DEMO_ROBUST  Reproduce the finding that makes rule choice non-obvious.
%
%   simlab_demos.demo_robust
%
% README.md reports that ranking the nine tuning rules by IAE against a
% PERFECT model correlates rho = -0.586 with ranking them by survival when the
% plant is wrong, over 810 runs of sim/sim_robust.c. This demo reproduces the
% study on three plants in MATLAB, so you can see whether it holds for YOURS
% before you pick a rule.
%
% It also demonstrates two things the C study says explicitly:
%
%   * "NO_OVERSHOOT" overshoots. On an exact model, with zero identification
%     error. The name comes from a 1940s table and is not a guarantee.
%   * The relay test always UNDER-estimates Ku, by up to 25% even with zero
%     noise, because a practical implementation measures the peak of a limit
%     cycle that carries harmonics while the describing function wants the
%     fundamental.
%
% Runtime: a few minutes per plant, because it simulates nine rules x N
% perturbed plants. Pass 'nRuns' to trade accuracy for time.

    K = pidx.Const;
    nRuns = 25;

    plants = { ...
        simlab.Plant('fopdt', 'name', 'lag dominated', 'k', 2, 'tau', 45, 'l', 1), ...
        simlab.Plant('fopdt', 'name', 'balanced', 'k', 2, 'tau', 10, 'l', 3), ...
        simlab.Plant('fopdt', 'name', 'dead-time dominant', 'k', 2, 'tau', 5, 'l', 12)};

    for ip = 1:numel(plants)
        plant = plants{ip};
        plant.setActuatorLimits(0, 100);
        dt = max(plant.tau() / 40, 0.01);
        L_over_T = plant.transportDelay() / plant.tau();

        fprintf('\n=========================================================\n');
        fprintf('  plant: %s   K = %.4g  T = %.4g s  L = %.4g s   L/T = %.3f\n', ...
            plant.name, plant.steadyStateGain(), plant.tau(), ...
            plant.transportDelay(), L_over_T);
        fprintf('  dt = %.4g s\n', dt);
        fprintf('=========================================================\n');

        % ---- the two rankings ----
        sc = simlab.Scenario.presets('stepResponse', 'sp', 100, ...
            'tEnd', 12 * (plant.tau() + plant.transportDelay()));
        exact = simlab.compareRules(plant, struct('mode', 'exact', ...
            'dt', dt, 'scenario', sc, 'verbose', false));
        robust = simlab.compareRules(plant, struct('mode', 'robust', ...
            'nRuns', nRuns, 'dt', dt, 'scenario', sc, 'verbose', false));

        fprintf('\n  %-16s %12s %12s %10s %10s\n', 'rule', ...
            'IAE exact', 'IAE robust', 'OS exact', 'survival');
        for i = 1:numel(exact.table.ok)
            if ~exact.table.ok(i)
                fprintf('  %-16s %s\n', exact.table.name{i}, ...
                    exact.table.note{i});
                continue;
            end
            fprintf('  %-16s %12.4g %12.4g %10.2f %10.0f%%\n', ...
                exact.table.name{i}, exact.table.iae(i), ...
                robust.table.iae(i), exact.table.overshoot(i), ...
                100 * robust.table.survival(i));
        end
        fprintf('\n  Spearman rho on this plant: %.3f  (C study: -0.586)\n', ...
            robust.spearman);
        fprintf('  %s\n', robust.recommend);

        simlab.plotRules(robust, struct('fig', 60 + ip));

        % ---- the NO_OVERSHOOT claim, checked on an exact model ----
        m = pidx.plantModel(K.MODEL_FOPDT, plant.steadyStateGain(), ...
            plant.tau(), plant.transportDelay());
        [rc, g] = pidx.ruleApply(K.RULE_NO_OVERSHOOT, m, K.STRUCT_PID, 0);
        if rc == K.OK
            cfg = pidx.config('kp', g.kp, 'ki', g.ki, 'kd', g.kd, 'dt', dt);
            cfg.filter.tf = g.tf;
            cfg.limits.use_output_limits = true;
            cfg.limits.output_min = 0;
            cfg.limits.output_max = 100;
            rr = simlab.Sim(plant, pidx.PID(cfg), sc).run();
            fprintf('\n  "NO_OVERSHOOT" on an EXACT model overshoots %.1f%%.\n', ...
                rr.metrics.overshoot);
            fprintf('  Its Ti is pinned at Pu/2 by the 1940s table, and on a\n');
            fprintf('  FOPDT plant that Ti is what produces the overshoot.\n');
            fprintf('  Stretching Ti is what fixes it, not lowering Kp:\n');
            for f = [1, 2, 4]
                cfg2 = pidx.config('kp', g.kp / f, 'ki', g.ki / (f * f), ...
                    'kd', g.kd / f, 'dt', dt);
                cfg2.filter.tf = g.tf;
                cfg2.limits.use_output_limits = true;
                cfg2.limits.output_min = 0;
                cfg2.limits.output_max = 100;
                r2 = simlab.Sim(plant, pidx.PID(cfg2), sc).run();
                fprintf('    Ti x %d: overshoot %.1f%%, IAE %.4g\n', ...
                    f, r2.metrics.overshoot, r2.metrics.iae);
            end
        else
            fprintf('\n  NO_OVERSHOOT could not be applied here: %s\n', ...
                K.statusToString(rc));
        end

        % ---- the relay's bias, measured ----
        fprintf('\n  relay identification on this plant:\n');
        cfgT = simlab.AutoTune.configDefault(K.IDENT_RELAY);
        cfgT.output_step = 20;
        cfgT.hysteresis = 0;
        cfgT.bias = 50;
        cfgT.auto_bias = false;
        cfgT.timeout_s = 60 * (plant.tau() + plant.transportDelay());
        at = simlab.AutoTune(cfgT);
        c = pidx.PID(pidx.config('dt', dt));
        plant.reset();
        at.start(c, 100);
        y = 0;
        nMax = round(cfgT.timeout_s / dt);
        for k = 1:nMax
            u = at.update(y, dt);
            y = plant.update(u, dt);
            if ~at.isRunning(), break; end
        end
        [rcA, resA] = at.getResult();
        if rcA == K.OK
            % The phase crossover of K exp(-Ls)/(1+Ts) is where
            % -atan(w*T) - w*L = -pi, and Ku is 1/|G| there:
            %     |G(jw)| = K / sqrt(1 + (w*T)^2)
            %     Ku      = sqrt(1 + (w*T)^2) / K
            wTrue = fzero(@(w) -atan(w * plant.tau()) - w * ...
                plant.transportDelay() + pi, 1e-3);
            kuTrue = sqrt(1 + (wTrue * plant.tau())^2) / ...
                plant.steadyStateGain();
            fprintf('    measured Ku = %.4g, true Ku = %.4g  (%.1f%% error)\n', ...
                resA.model.ku, kuTrue, ...
                100 * (resA.model.ku - kuTrue) / kuTrue);
            fprintf('    measured Pu = %.4g s, true Pu = %.4g s\n', ...
                resA.model.pu, 2 * pi / wTrue);
            fprintf('    The relay reads the PEAK of a limit cycle; the\n');
            fprintf('    describing function wants the FUNDAMENTAL. The error\n');
            fprintf('    is in the safe direction - gains come out low.\n');
        else
            fprintf('    refused: %s\n', K.statusToString(rcA));
        end
    end
    fprintf('\n');
