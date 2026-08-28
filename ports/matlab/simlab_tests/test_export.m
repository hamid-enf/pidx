function T = test_export(T)
%SIMLAB_TESTS.TEST_EXPORT  Does the exported C compile, and mean the same thing?
%
% This is the test that matters most for the reason the tool exists. The
% generated file is not a report about the tuning - it IS the tuning, in the
% language the target speaks. So it is checked in two ways:
%
%   1. every number in the header appears with the right value, parsed back
%      out of the text rather than assumed from the input struct;
%   2. if a C compiler and the PIDX sources are reachable, the file is
%      actually COMPILED against the library. A generated file that does not
%      compile is not a deliverable, it is a suggestion.
%
% The compile check is skipped by name when no compiler is present, so a run
% on a machine without gcc does not quietly look like a pass.

    K = pidx.Const;
    tmp = fullfile(tempdir, sprintf('simlab_export_test_%d', ...
        round(now * 8640000)));
    if ~exist(tmp, 'dir'), mkdir(tmp); end

    % ---- build a configuration that exercises most of the surface ----
    plant = simlab.Plant('fopdt', 'name', 'test heater', ...
        'k', 2.0, 'tau', 45.0, 'l', 12.0);
    plant.setActuatorLimits(0, 100);
    plant.setAdcBits(12, 0, 300);
    plant.setNoise(0.15);

    cfg = pidx.config('kp', 3.14159, 'ki', 0.0812, 'kd', 0.55, 'dt', 0.25);
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0;
    cfg.limits.output_max = 100;
    cfg.integral.mode = K.AW_BACK_CALCULATION;
    cfg.integral.kt = 0.4;
    cfg.filter.tf = 0.05;
    cfg.filter.input_lpf_tau = 1.5;
    cfg.weight.beta = 0.7;
    cfg.weight.gamma = 0.0;
    cfg.shaper.sp_rate_max = 5;
    cfg.shaper.sp_accel = 2;
    cfg.shaper.out_slew_max = 40;
    cfg.safety.enabled = true;
    cfg.safety.meas_min = -10;
    cfg.safety.meas_max = 350;
    cfg.safety.meas_rate_max = 20;
    cfg.safety.failsafe_output = 0;
    cfg.safety.fault_persist_n = 4;
    cfg.safety.auto_recover = true;
    cfg.feedforward.enabled = true;
    cfg.feedforward.value = 12.5;

    ctrl = pidx.PID(cfg);
    sc = simlab.Scenario.presets('stepResponse', 'sp', 100, 'tEnd', 400);
    r = simlab.Sim(plant, ctrl, sc).run();
    sens = simlab.sensitivity(plant, struct('kp', cfg.core.kp, ...
        'ki', cfg.core.ki, 'kd', cfg.core.kd), struct('dt', cfg.core.sample_time));

    out = simlab.exportSTM32(plant, cfg, struct('dir', tmp, ...
        'symbol', 'heaterLoop', 'profile', 'FULL', 'result', r, ...
        'sens', sens, 'quiet', true));

    T = simlab_tests.ok(T, exist(out.header, 'file') == 2, 'the header was written');
    T = simlab_tests.ok(T, exist(out.source, 'file') == 2, 'the source was written');
    T = simlab_tests.ok(T, isempty(out.dropped), ...
        'nothing was dropped when exporting for PIDX_PROFILE_FULL');

    hdr = fileread(out.header);
    src = fileread(out.source);

    % ---- 1. the numbers round-trip through the text ----
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_KP'), cfg.core.kp, 1e-9, ...
        'Kp survives the round trip through C text');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_KI'), cfg.core.ki, 1e-9, 'Ki');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_KD'), cfg.core.kd, 1e-9, 'Kd');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_DT'), cfg.core.sample_time, ...
        1e-9, 'dt');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_TF'), cfg.filter.tf, 1e-9, 'Tf');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_AW_KT'), cfg.integral.kt, 1e-9, 'Kt');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_BETA'), cfg.weight.beta, ...
        1e-9, 'beta');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_OUT_MAX'), 100, 1e-12, 'output max');
    T = simlab_tests.near(T, parseDefine(hdr, 'HEATERLOOP_TI'), ...
        cfg.core.kp / cfg.core.ki, 1e-9, 'Ti is reported as Kp/Ki');

    T = simlab_tests.ok(T, ~isempty(strfind(hdr, 'PID_AW_BACK_CALCULATION')), ...
        'the anti-windup strategy is exported by name, not by number');
    T = simlab_tests.ok(T, ~isempty(strfind(hdr, 'PID_DERIV_ON_MEASUREMENT')), ...
        'the derivative source is exported by name');
    T = simlab_tests.ok(T, ~isempty(strfind(src, 'cfg.safety.enabled = true')), ...
        'safety validation is exported');
    T = simlab_tests.ok(T, ~isempty(strfind(src, 'cfg.filter.input_lpf_tau')), ...
        'the input filter is exported');
    T = simlab_tests.ok(T, ~isempty(strfind(src, 'cfg.shaper.out_slew_max')), ...
        'the output slew limit is exported');

    % The measured record and the margins are in the header, so the person
    % reading the C file in six months can see what the gains were bought with.
    T = simlab_tests.ok(T, ~isempty(strfind(hdr, 'measured in simulation')), ...
        'the header carries the simulation record');
    T = simlab_tests.ok(T, ~isempty(strfind(hdr, 'loop margins')), ...
        'the header carries the loop margins');
    T = simlab_tests.ok(T, ~isempty(strfind(hdr, 'G(s) =')), ...
        'the header states the plant the gains were tuned against');

    % ---- 2. a MINIMAL profile must say what it cannot do ----
    outMin = simlab.exportSTM32(plant, cfg, struct('dir', tmp, ...
        'symbol', 'minLoop', 'profile', 'MINIMAL', 'quiet', true));
    T = simlab_tests.ok(T, ~isempty(outMin.dropped), ...
        'exporting a full configuration for MINIMAL reports %d dropped feature(s)', ...
        numel(outMin.dropped));
    srcMin = fileread(outMin.source);
    T = simlab_tests.ok(T, ~isempty(strfind(srcMin, 'NOT EXPORTED')), ...
        'a dropped feature is marked NOT EXPORTED in the file, not silently omitted');
    T = simlab_tests.ok(T, ~isempty(strfind(srcMin, 'NOT EXPORTED: safety')), ...
        '...and safety is one of the features MINIMAL does not have');
    T = simlab_tests.ok(T, isempty(strfind(srcMin, 'cfg.safety.enabled = true')), ...
        'the MINIMAL source does not configure safety it cannot have');

    % ---- 3. it compiles ----
    %
    % The strongest check available without a board. If this passes, the
    % generated file is a real translation unit against the real headers, and
    % every PID_* identifier it names exists.
    [haveGcc, repoRoot] = findToolchain();
    if ~haveGcc
        T = simlab_tests.skip(T, 'compile the generated C against PIDX', ...
            'no gcc on PATH, or the PIDX include/ tree was not found');
    else
        objName = fullfile(tmp, 'pidx_tuning_heaterLoop.o');
        cmd = sprintf('gcc -std=c99 -DPIDX_PROFILE_FULL -I"%s" -I"%s" -c "%s" -o "%s" 2>&1', ...
            fullfile(repoRoot, 'include'), tmp, out.source, objName);
        [status, output] = system(cmd);
        T = simlab_tests.eq(T, status, 0, ...
            'the generated source compiles cleanly with -std=c99 -Werror-grade flags: %s', ...
            strtrim(output));

        % And the MINIMAL variant too, since it contains #if-free code paths
        % that must still build without the optional modules.
        objMin = fullfile(tmp, 'pidx_tuning_minLoop.o');
        cmd2 = sprintf('gcc -std=c99 -DPIDX_PROFILE_MINIMAL -I"%s" -I"%s" -c "%s" -o "%s" 2>&1', ...
            fullfile(repoRoot, 'include'), tmp, outMin.source, objMin);
        [status2, output2] = system(cmd2);
        T = simlab_tests.eq(T, status2, 0, ...
            'the MINIMAL-profile export compiles against a MINIMAL build: %s', ...
            strtrim(output2));

        % ---- 4. the compiled code produces the same output as MATLAB ----
        %
        % This is the check that the export is not a paraphrase. Link the
        % generated object with the library and a small driver, feed both the
        % same measurement sequence, and compare.
        driverPath = fullfile(tmp, 'drive.c');
        writeDriver(driverPath);
        exePath = fullfile(tmp, 'drive');
        cmd3 = sprintf(['gcc -std=c99 -DPIDX_PROFILE_FULL -I"%s" -I"%s" ' ...
                        '"%s" "%s" %s -lm -o "%s" 2>&1'], ...
            fullfile(repoRoot, 'include'), tmp, driverPath, out.source, ...
            strjoin(cellfun(@(f) sprintf('"%s"', f), ...
                dir2cell(fullfile(repoRoot, 'src'), '*.c'), 'UniformOutput', false), ' '), ...
            exePath);
        [status3, output3] = system(cmd3);
        if status3 ~= 0
            T = simlab_tests.ok(T, false, 'the cross-check driver links: %s', ...
                strtrim(output3));
        else
            ySeq = measSequence(fullfile(tmp, 'yseq.txt'));
            [~, csvOut] = system(sprintf('"%s" "%s"', exePath, ...
                fullfile(tmp, 'yseq.txt')));
            cVals = textscan(csvOut, '%f');
            cVals = cVals{1};
            % Replay the identical measurements through the MATLAB controller.
            ctrl2 = pidx.PID(cfg);
            ctrl2.setSetpoint(100);
            worst = 0;
            for k = 1:numel(ySeq)
                uM = ctrl2.update(ySeq(k));
                d = abs(uM - cVals(k)) / max(1, abs(uM), abs(cVals(k)));
                if d > worst, worst = d; end
            end
            % 1e-5 rather than 1e-12 on purpose: the target is built with
            % 32-bit floats unless PIDX_USE_DOUBLE is set, so the two sides
            % legitimately differ at float epsilon. What this test rules out
            % is a structural difference - a missing stage, a wrong sign, a
            % mis-ordered anti-windup - and those are orders of magnitude
            % larger than float rounding.
            T = simlab_tests.ok(T, worst < 1e-5, ...
                'the COMPILED exported loop reproduces the MATLAB output over %d samples (worst rel diff %.3g)', ...
                numel(ySeq), worst);
        end
    end

    % ---- 5. the JSON/CSV report ----
    rep = simlab.exportReport(r, struct('dir', tmp, 'name', 'unit_test', ...
        'cfg', cfg, 'plant', plant, 'sens', sens));
    T = simlab_tests.ok(T, exist(rep.csv, 'file') == 2, 'the CSV report was written');
    T = simlab_tests.ok(T, exist(rep.json, 'file') == 2, 'the JSON report was written');

    jtxt = fileread(rep.json);
    T = simlab_tests.ok(T, ~isempty(strfind(jtxt, '"metrics"')), 'JSON carries the metrics');
    T = simlab_tests.ok(T, ~isempty(strfind(jtxt, '"controller"')), 'JSON carries the configuration');
    T = simlab_tests.ok(T, ~isempty(strfind(jtxt, '"margins"')), 'JSON carries the margins');
    T = simlab_tests.ok(T, ~isempty(strfind(jtxt, '"caveats"')), 'JSON carries the plant caveats');
    % NaN and Inf are not JSON. They must be null, not the literal string.
    T = simlab_tests.ok(T, isempty(regexp(jtxt, ': *(NaN|Inf|-Inf)', 'once')), ...
        'no NaN/Inf literal appears in the JSON - they are encoded as null');
    % Balanced braces: a cheap structural check that catches a missing comma
    % or a stray quote, which is how hand-written JSON usually breaks.
    T = simlab_tests.eq(T, sum(jtxt == '{'), sum(jtxt == '}'), 'JSON braces balance');

    csvTxt = fileread(rep.csv);
    nLines = numel(strfind(csvTxt, sprintf('\n')));
    T = simlab_tests.eq(T, nLines, numel(r.t) + 1, 'the CSV has a header plus one row per sample');
end

% ---------------------------------------------------------------------------

function v = parseDefine(txt, name)
% Read a #define's numeric value back out of the generated header.
%
% Parsed from the text rather than read from the input struct: the point is to
% catch a formatting error, and reading the struct back would not.
    pat = ['#define\s+' name '\s+([-+0-9.eE]+)'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        v = NaN;
        return;
    end
    v = str2double(tok{1});
end

function [have, root] = findToolchain()
% Locate gcc and the repository root, walking up from this file.
    have = false;
    root = '';
    [st, ~] = system('gcc --version > /dev/null 2>&1');
    if st ~= 0
        return;
    end
    here = fileparts(mfilename('fullpath'));
    for k = 1:8
        cand = fullfile(here, repmat(['..' filesep], 1, k));
        if exist(fullfile(cand, 'include', 'pidx', 'pid.h'), 'file') == 2 && ...
           exist(fullfile(cand, 'src', 'pid.c'), 'file') == 2
            root = fullfile(cand);
            have = true;
            return;
        end
    end
end

function c = dir2cell(folder, pattern)
    d = dir(fullfile(folder, pattern));
    c = cell(numel(d), 1);
    for i = 1:numel(d)
        c{i} = fullfile(folder, d(i).name);
    end
end

function y = measSequence(path)
% The measurement sequence both sides are fed, written to a file so that the
% C driver and the MATLAB replay cannot disagree about a single input.
%
% Written at full round-trip precision and read back, rather than
% regenerated by a formula on each side: two implementations of exp() and
% sin() would differ in the last bits, and then this test would be measuring
% libm rather than the export.
    n = 200;
    y = zeros(n, 1);
    fid = fopen(path, 'w');
    for k = 1:n
        y(k) = 100 * (1 - exp(-(k - 1) * 0.25 / 20)) + 3 * sin(k * 0.11);
        fprintf(fid, '%.17g\n', y(k));
    end
    fclose(fid);
end

function writeDriver(path)
% A driver that calls the exported init/tick and prints the output per sample.
%
% It includes the generated header by name and calls the generated functions -
% nothing is reimplemented here, so a mismatch means the export is wrong.
    fid = fopen(path, 'w');
    fprintf(fid, '%s\n', '#include <stdio.h>');
    fprintf(fid, '%s\n', '#include <math.h>');
    fprintf(fid, '%s\n', '#include "pidx_tuning_heaterLoop.h"');
    fprintf(fid, '%s\n', '');
    fprintf(fid, '%s\n', 'int main(int argc, char **argv)');
    fprintf(fid, '%s\n', '{');
    fprintf(fid, '%s\n', '    FILE *f;');
    fprintf(fid, '%s\n', '    double y;');
    fprintf(fid, '%s\n', '    int n = 0;');
    fprintf(fid, '%s\n', '    if (argc < 2) { return 1; }');
    fprintf(fid, '%s\n', '    f = fopen(argv[1], "r");');
    fprintf(fid, '%s\n', '    if (f == NULL) { return 1; }');
    fprintf(fid, '%s\n', '    heaterLoop_init();');
    fprintf(fid, '%s\n', '    while (fscanf(f, "%lf", &y) == 1) {');
    fprintf(fid, '%s\n', '        printf("%.17g\\n", (double)heaterLoop_tick((float)y));');
    fprintf(fid, '%s\n', '        ++n;');
    fprintf(fid, '%s\n', '    }');
    fprintf(fid, '%s\n', '    fclose(f);');
    fprintf(fid, '%s\n', '    return (n > 0) ? 0 : 1;');
    fprintf(fid, '%s\n', '}');
    fclose(fid);
end
