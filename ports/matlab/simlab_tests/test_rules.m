function T = test_rules(T)
%SIMLAB_TESTS.TEST_RULES  Do the tuning rules and their comparison hold together?
%
% Three things are checked:
%   1. every rule produces the published coefficients (arithmetic, exact);
%   2. the family split is enforced - a FREQ rule cannot consume a FOPDT
%      model and vice versa, because one Nyquist point does not determine
%      three model parameters;
%   3. simlab.compareRules runs end to end on a real plant and its
%      recommendation is a rule that actually survived, not the fastest one.
%
% The published coefficient tables are from src/pid_autotune_rules.c and are
% reproduced here independently, so a transcription error in either the C or
% the MATLAB table shows up as a disagreement.

    K = pidx.Const;

    % ---- 1. the frequency family: Kp = a*Ku, Ti = b*Pu, Td = c*Pu ----
    Ku = 3.0;
    Pu = 8.0;
    mF = pidx.plantModel(K.MODEL_FREQ, Ku, Pu);

    tab = [K.RULE_ZN,             0.50, 0.0,     0.0;
           K.RULE_ZN,             0.45, 1.0/1.2, 0.0;
           K.RULE_ZN,             0.60, 0.50,    0.125;
           K.RULE_TYREUS_LUYBEN,  0.50, 0.0,     0.0;
           K.RULE_TYREUS_LUYBEN,  0.31, 2.20,    0.0;
           K.RULE_TYREUS_LUYBEN,  0.45, 2.20,    1.0/6.3;
           K.RULE_PESSEN,         0.50, 0.0,     0.0;
           K.RULE_PESSEN,         0.45, 1.0/1.2, 0.0;
           K.RULE_PESSEN,         0.70, 0.40,    0.15;
           K.RULE_SOME_OVERSHOOT, 0.33, 0.0,     0.0;
           K.RULE_SOME_OVERSHOOT, 0.33, 0.50,    0.0;
           K.RULE_SOME_OVERSHOOT, 0.33, 0.50,    1.0/3.0;
           K.RULE_NO_OVERSHOOT,   0.20, 0.0,     0.0;
           K.RULE_NO_OVERSHOOT,   0.20, 0.50,    0.0;
           K.RULE_NO_OVERSHOOT,   0.20, 0.50,    1.0/3.0];

    for i = 1:size(tab, 1)
        structure = mod(i - 1, 3);          % P, PI, PID repeating
        [rc, g] = pidx.ruleApply(tab(i, 1), mF, structure, 0);
        T = simlab_tests.eq(T, rc, K.OK, 'rule %d structure %d applies', tab(i, 1), structure);
        kpWant = tab(i, 2) * Ku;
        tiWant = tab(i, 3) * Pu;
        tdWant = tab(i, 4) * Pu;
        T = simlab_tests.near(T, g.kp, kpWant, 1e-12, 'Kp = a*Ku');
        if tiWant > 0
            T = simlab_tests.near(T, g.ti, tiWant, 1e-12, 'Ti = b*Pu');
            T = simlab_tests.near(T, g.ki, kpWant / tiWant, 1e-12, 'Ki = Kp/Ti');
        else
            T = simlab_tests.near(T, g.ki, 0, 1e-15, 'no integral action means Ki = 0');
        end
        T = simlab_tests.near(T, g.kd, kpWant * tdWant, 1e-12, 'Kd = Kp*Td');
        T = simlab_tests.near(T, g.tf, tdWant * 0.1, 1e-12, ...
            'Tf = Td/10 - without a filter the D term differentiates noise without bound');
    end

    % ---- AMIGO from frequency ----
    [rc, g] = pidx.ruleApply(K.RULE_AMIGO_FREQ, mF, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rc, K.OK, 'AMIGO_FREQ applies');
    T = simlab_tests.near(T, g.kp, 0.16 * Ku, 1e-12, 'AMIGO_FREQ Kp = 0.16*Ku');
    T = simlab_tests.near(T, g.ti, 0.46 * Pu, 1e-12, 'AMIGO_FREQ Ti = 0.46*Pu');
    T = simlab_tests.near(T, g.td, 0.10 * Pu, 1e-12, 'AMIGO_FREQ Td = 0.10*Pu');

    % ---- 2. the FOPDT family ----
    mP = pidx.plantModel(K.MODEL_FOPDT, 2.0, 45.0, 12.0);

    % Cohen-Coon PID
    tauN = mP.l / mP.t;
    inv = 1.0 / (mP.k * tauN);
    [rc, g] = pidx.ruleApply(K.RULE_COHEN_COON, mP, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rc, K.OK, 'Cohen-Coon applies to a FOPDT model');
    T = simlab_tests.near(T, g.kp, inv * (4.0 / 3.0 + tauN / 4.0), 1e-12, 'Cohen-Coon Kp');
    T = simlab_tests.near(T, g.ti, mP.l * (32.0 + 6.0 * tauN) / (13.0 + 8.0 * tauN), ...
        1e-12, 'Cohen-Coon Ti');
    T = simlab_tests.near(T, g.td, mP.l * 4.0 / (11.0 + 2.0 * tauN), 1e-12, 'Cohen-Coon Td');

    % IMC PID with an explicit lambda
    lam = 5.0;
    [rc, g] = pidx.ruleApply(K.RULE_IMC, mP, K.STRUCT_PID, lam);
    half = 0.5 * mP.l;
    T = simlab_tests.near(T, g.kp, (mP.t + half) / (mP.k * (lam + half)), 1e-12, 'IMC Kp');
    T = simlab_tests.near(T, g.ti, mP.t + half, 1e-12, 'IMC Ti = T + L/2');
    T = simlab_tests.near(T, g.td, mP.t * mP.l / (2.0 * mP.t + mP.l), 1e-12, 'IMC Td');

    % IMC's robustness floor: lambda below 0.2*L is silently raised, because
    % below that the controller leans on a dead-time estimate it cannot trust.
    [rc, gTiny] = pidx.ruleApply(K.RULE_IMC, mP, K.STRUCT_PI, 0.001);
    [rc, gFloor] = pidx.ruleApply(K.RULE_IMC, mP, K.STRUCT_PI, 0.2 * mP.l);
    T = simlab_tests.eq(T, rc, K.OK, 'IMC applies with a floored lambda');
    T = simlab_tests.near(T, gTiny.kp, gFloor.kp, 1e-12, ...
        'a lambda below 0.2*L is floored at 0.2*L');

    % ---- 3. the family split is enforced ----
    [rcA, ~] = pidx.ruleApply(K.RULE_COHEN_COON, mF, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rcA, K.ERR_TUNE_MODEL_MISMATCH, 'Cohen-Coon refuses a FREQ model');
    [rcB, ~] = pidx.ruleApply(K.RULE_IMC, mF, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rcB, K.ERR_TUNE_MODEL_MISMATCH, 'IMC refuses a FREQ model');
    [rcC, ~] = pidx.ruleApply(K.RULE_ZN, mP, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rcC, K.ERR_TUNE_MODEL_MISMATCH, 'ZN refuses a FOPDT model');

    % ---- 4. garbage models are refused ----
    mBad = pidx.plantModel(K.MODEL_FOPDT, 2.0, -1.0, 12.0);
    [rcD, ~] = pidx.ruleApply(K.RULE_IMC, mBad, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rcD, K.ERR_TUNE_VALIDATION, 'a negative time constant is refused');
    mBad2 = pidx.plantModel(K.MODEL_FREQ, 0, 8.0);
    [rcE, ~] = pidx.ruleApply(K.RULE_ZN, mBad2, K.STRUCT_PID, 0);
    T = simlab_tests.eq(T, rcE, K.ERR_TUNE_VALIDATION, 'Ku = 0 is refused');

    % ---- 5. compareRules runs end to end ----
    %
    % Kept small (few plants per rule) so the suite stays quick; the point is
    % that the whole chain - identify, apply, simulate, perturb, rank -
    % completes and produces a recommendation that is defensible.
    plant = simlab.Plant('fopdt', 'name', 'test heater', 'k', 2.0, ...
        'tau', 45.0, 'l', 12.0);
    plant.setActuatorLimits(0, 100);
    c = simlab.compareRules(plant, struct('mode', 'robust', 'nRuns', 8, ...
        'verbose', false, 'seed', 7, 'dt', 0.5, ...
        'scenario', simlab.Scenario.presets('stepResponse', 'tEnd', 400)));

    T = simlab_tests.eq(T, numel(c.table.ok), 9, 'all nine rules are in the table');
    T = simlab_tests.ok(T, sum(c.table.ok) >= 6, ...
        '%d of 9 rules produced usable gains on this plant', sum(c.table.ok));
    T = simlab_tests.ok(T, ~isempty(c.best), 'a recommendation was produced');
    if ~isempty(c.best)
        T = simlab_tests.ok(T, c.best.survival >= 0.9, ...
            'the recommended rule (%s) survived %.0f%% of the perturbed plants', ...
            c.best.name, 100 * c.best.survival);
        % The recommendation must not be a rule that failed. This is the whole
        % point of the function: the fastest rule on a perfect model is often
        % the one that breaks first.
        fastest = c.table.ok & ~isnan(c.table.iae);
        [~, j] = min(c.table.iae(fastest));
        idxFast = find(fastest);
        T = simlab_tests.ok(T, c.best.survival >= c.table.survival(idxFast(j)) - 1e-12, ...
            'the recommendation is no less robust than the fastest rule');
    end
    T = simlab_tests.ok(T, ~isnan(c.spearman) || numel(find(c.table.ok)) < 3, ...
        'a rank correlation was computed (rho = %.3f)', c.spearman);
    T = simlab_tests.ok(T, c.modelWasPerfect, ...
        'with no model supplied the study says it used a perfect identification');
end
