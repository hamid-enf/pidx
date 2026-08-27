function r = hilRun(h, plant, scenario, opt)
%SIMLAB.HILRUN  Run a scenario with the controller on the board.
%
%   R = SIMLAB.HILRUN(H, PLANT, SCENARIO)
%   R = SIMLAB.HILRUN(H, PLANT, SCENARIO, 'setpoint', 100, 'spAt', 1)
%
% H is the handle from simlab.hilConnect. The plant runs HERE; the controller
% runs on the BOARD, and the two exchange one measurement and one command per
% sample. The result has the same shape as simlab.Sim.run(), so the same
% metrics, the same plots and the same report work on either - which is the
% only way a comparison between them means anything.
%
% WHAT THIS CATCHES THAT A SIMULATION CANNOT
%   The simulation runs the MATLAB port of the controller. This runs the
%   compiled C on the real core, with the real ADC, the real PWM and the real
%   interrupt latency. If the two traces diverge, one of those is doing
%   something the model does not - and that is exactly the difference worth
%   paying a board for.
%
% WHAT IT CANNOT DO
%   Run fast. Each sample is a serial round trip, so the achievable rate is
%   set by the link, not by the board. Process loops at 10-500 Hz are fine; a
%   20 kHz current loop is not, and it must not be attempted by buffering: a
%   controller whose dt depends on the host is not the controller under test.
%   Pass 'dt' to set the rate the BOARD uses, and keep it comfortably above
%   the measured round trip.
%
% OPTIONS
%   'setpoint'  the commanded setpoint. Default: taken from the scenario.
%   'spAt'      when the setpoint is commanded [s]. Default from the scenario.
%   'dt'        the board's sample time. Default: the scenario duration over
%               200 samples, floored at the measured round trip.
%   'kp','ki','kd','tf'  override the flashed tuning. NOT RECOMMENDED - the
%               point of HIL is to test the file you exported, and changing
%               the gains over the link tests something else. Provided for
%               gain-sweeping a single board session; the report says when it
%               was used.
%   'maxTimeouts' abort after this many unanswered samples. Default 5.
%   'verbose'   print progress. Default true.

    if nargin < 4, opt = struct(); end
    o = fillOpt(opt, 'setpoint', []);
    o = fillOpt(opt, 'spAt', []);
    o = fillOpt(opt, 'dt', []);
    o = fillOpt(opt, 'kp', []);
    o = fillOpt(opt, 'ki', []);
    o = fillOpt(opt, 'kd', []);
    o = fillOpt(opt, 'tf', []);
    o = fillOpt(opt, 'maxTimeouts', 5);
    o = fillOpt(opt, 'verbose', true);

    % ---- the setpoint ----
    sp = o.setpoint;
    spAt = o.spAt;
    if isempty(sp)
        [sp, spAt] = scenarioSetpoint(scenario);
    end
    if isempty(spAt), spAt = 0; end

    % ---- measure the link before choosing a rate ----
    simlab.hilFlush(h);
    t0 = tic;
    simlab.hilWrite(h, 'PING');
    ack = simlab.hilReadLine(h);
    rtt = toc(t0);
    if ~strcmp(strtrim(ack), 'PONG')
        error('simlab:hilRun:noPing', ...
              'the board did not answer PING (got "%s")', ack);
    end

    dt = o.dt;
    if isempty(dt)
        dt = max(scenario.tEnd / 200, 10 * rtt);
    end
    if dt < 5 * rtt
        warning('simlab:hilRun:slowLink', ...
                ['dt = %.4g s is only %.1f round trips (%.4g s each). The ' ...
                 'board will not see a periodic dt, and a controller tested ' ...
                 'at an irregular rate is not the controller you designed. ' ...
                 'Slow the scenario down or raise the baud rate.'], ...
                dt, dt / rtt, rtt);
    end

    n = max(1, round(scenario.tEnd / dt));

    % ---- configure the board ----
    simlab.hilFlush(h);
    simlab.hilWrite(h, sprintf('RESET %.10g', dt));
    line = simlab.hilReadLine(h);
    if isempty(strfind(line, 'OK')) %#ok<STREMP>
        error('simlab:hilRun:reset', 'board rejected RESET: %s', line);
    end
    if ~isempty(o.kp)
        simlab.hilWrite(h, sprintf('GAINS %.10g %.10g %.10g %.10g', ...
            o.kp, o.ki, o.kd, o.tf));
        line = simlab.hilReadLine(h);
        if isempty(strfind(line, 'OK')) %#ok<STREMP>
            error('simlab:hilRun:gains', 'board rejected GAINS: %s', line);
        end
    end

    % ---- the log ----
    r = struct();
    r.n = n;
    r.dt = dt;
    r.t = zeros(1, n);
    r.r = zeros(1, n);
    r.y = zeros(1, n);
    r.yTrue = zeros(1, n);
    r.u = zeros(1, n);
    r.uPlant = zeros(1, n);
    r.uRaw = zeros(1, n);
    r.p = zeros(1, n);      % the board does not report the term breakdown;
    r.i = zeros(1, n);      % zeros rather than guesses, so a plot of P+I+D
    r.d = zeros(1, n);      % visibly does not reconstruct u
    r.ff = zeros(1, n);
    r.e = zeros(1, n);
    r.flags = zeros(1, n, 'uint32');
    r.kp = zeros(1, n);
    r.ki = zeros(1, n);
    r.kd = zeros(1, n);
    r.state = zeros(1, n);
    r.timeouts = 0;
    r.roundTrip = rtt;
    r.boardId = h.boardId;
    r.gainsOverridden = ~isempty(o.kp);

    plant.reset();
    uPrev = 0;
    sc = scenario;
    spCmd = 0;

    if o.verbose
        fprintf('simlab.hilRun: %d samples at dt = %.4g s (round trip %.2f ms)\n', ...
            n, dt, 1000 * rtt);
    end

    for k = 1:n
        t = (k - 1) * dt;
        r.t(k) = t;

        % -- events, exactly as simlab.Sim applies them --
        idx = sc.dueIndex(t, dt);
        for j = 1:numel(idx)
            e = sc.getEvent(idx(j));
            switch e.type
                case 'setpoint'
                    spCmd = e.a;
                case 'loadStep',  plant.setLoad(e.a, t);
                case 'noise',     plant.setNoise(e.a);
                case 'stuck',     plant.setStuckAt(e.a);
                case 'dropout',   plant.setDropout(e.a);
                case 'actLimits', plant.setActuatorLimits(e.a, e.b);
                case 'plantGain', plant = plant.set('k', e.a);
                case 'plantDelay'
                    plant = plant.set('deadtime', e.a);
                otherwise
                    % mode/manual/disturb/custom have no meaning here: the
                    % controller is not on this machine.
                    if o.verbose && k == 1
                        fprintf('  (event "%s" is ignored in HIL mode)\n', e.type);
                    end
            end
            sc.markDone(idx(j));
        end
        if k - 1 >= round(spAt / dt)
            spCmd = sp;
        end

        % -- one round trip --
        plant.update(uPrev, dt);
        y = plant.yMeas;
        simlab.hilWrite(h, sprintf('S %.10g %.10g', y, spCmd));
        line = simlab.hilReadLine(h);
        if isempty(line)
            % An unanswered sample is recorded as such, not as zero. A zero
            % command looks like a controller that gave up; a NaN looks like
            % a link that did, which is a different problem with a different
            % fix.
            r.timeouts = r.timeouts + 1;
            r.u(k) = NaN;
            r.y(k) = y;
            r.yTrue(k) = plant.yTrue;
            r.r(k) = spCmd;
            uPrev = 0;
            if r.timeouts >= o.maxTimeouts
                warning('simlab:hilRun:timeouts', ...
                        '%d unanswered samples - aborting. Check the link.', ...
                        r.timeouts);
                r.n = k;
                break;
            end
            continue;
        end

        tok = regexp(line, '([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([0-9]+)', ...
                     'tokens', 'once');
        if isempty(tok)
            error('simlab:hilRun:parse', ...
                  'unparsable reply "%s" at sample %d', line, k);
        end
        u = str2double(tok{1});
        uSat = str2double(tok{2});
        fl = uint32(str2double(tok{3}));

        r.u(k) = u;
        r.uRaw(k) = uSat;
        r.y(k) = y;
        r.yTrue(k) = plant.yTrue;
        r.r(k) = spCmd;
        r.e(k) = spCmd - y;
        r.uPlant(k) = plant.uPlant;
        r.flags(k) = fl;
        uPrev = u;

        if o.verbose && mod(k, max(1, floor(n / 10))) == 0
            fprintf('  %d/%d  t = %.4g s  y = %.6g  u = %.6g\n', ...
                k, n, t, y, u);
        end
    end

    simlab.hilWrite(h, 'BYE');

    % ---- trim and score ----
    fn = fieldnames(r);
    for i = 1:numel(fn)
        f = fn{i};
        v = r.(f);
        if isvector(v) && numel(v) > r.n
            r.(f) = v(1:r.n);
        end
    end
    r.scenario = sc.describe();
    r.source = 'hil';
    r.metrics = simlab.metrics(r);

    if o.verbose
        fprintf('\n  board      %s\n', r.boardId);
        fprintf('  timeouts   %d of %d samples\n', r.timeouts, r.n);
        fprintf('  rise %.4g s, overshoot %.1f%%, settling %.4g s, IAE %.4g\n', ...
            r.metrics.riseTime, r.metrics.overshoot, ...
            r.metrics.settlingTime, r.metrics.iae);
        if r.gainsOverridden
            fprintf('  NOTE: gains were overridden over the link, so this\n');
            fprintf('        run did NOT test the exported tuning file.\n');
        end
    end
end

% ---------------------------------------------------------------------------

function [sp, spAt] = scenarioSetpoint(sc)
% The setpoint a scenario commands, and when. A scenario is a list, so this
% reads the events rather than assuming the preset shape.
    sp = 0;
    spAt = 0;
    for i = 1:sc.nEvents
        e = sc.getEvent(i);
        if strcmp(e.type, 'setpoint') && ~isempty(e.a) && e.a ~= 0
            sp = e.a;
            spAt = e.t;
            break;
        end
    end
end

function o = fillOpt(opt, name, default)
    if isfield(opt, name) && ~isempty(opt.(name))
        o.(name) = opt.(name);
    else
        o.(name) = default;
    end
end
