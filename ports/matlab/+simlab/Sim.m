classdef Sim < handle
    % SIMLAB.SIM  Closed-loop runner: plant + PIDX controller + scenario.
    %
    % This is the piece that produces the numbers. It owns the sample loop,
    % so the loop is written ONCE and every study - step response, windup
    % comparison, Monte Carlo, auto-tune - runs through the same code path.
    % A result you cannot reproduce with the same command is not a result.
    %
    %   pl = simlab.Plant.presets('heater');
    %   c  = pidx.PID(pidx.config('kp', 3, 'ki', 0.08, 'dt', 0.1));
    %   r  = simlab.Sim(pl, c, simlab.Scenario.presets('stepResponse')).run();
    %   simlab.plot(r);
    %   disp(r.metrics);
    %
    % WHAT IS RECORDED, every sample
    %   t, r (effective setpoint), y (what the controller saw), yTrue (what
    %   the plant really did), u (command), uPlant (what the actuator
    %   delivered), uRaw (pre-saturation), p/i/d/ff terms, error, PIDX status
    %   flags, the three active gains. Enough to explain any behaviour
    %   afterwards without re-running.
    %
    % OPTIONS (constructor name/value pairs)
    %   'dt'        override the sample time. Default: the controller's.
    %   'tuner'     a simlab.AutoTune. It runs first and drives the plant
    %               itself; when it completes, the tuned gains are applied to
    %               the controller and the scenario runs on. That is exactly
    %               the order it happens on the target.
    %   'cascade'   a simlab.Cascade; the single 'controller' is then unused.
    %   'measFn'    @(plant) returning a COLUMN VECTOR of per-level
    %               measurements, outermost first. Needed for a cascade,
    %               because each level measures a different variable.
    %   'verbose'   print a progress line. Default false.
    %
    % The random stream is seeded by plant.reset(), so two runs of the same
    % scenario differ only where the configuration differs.

    properties
        plant;
        ctrl;
        scenario;
        dt = [];
        tuner = [];
        cascade = [];
        measFn = [];
        verbose = false;
    end

    methods
        function o = Sim(plant, ctrl, scenario, varargin)
            if nargin < 2
                error('simlab:Sim:nargin', 'Sim(plant, controller[, scenario])');
            end
            if isempty(plant)
                error('simlab:Sim:noPlant', 'a plant is required');
            end
            o.plant = plant;
            o.ctrl = ctrl;
            if nargin >= 3 && ~isempty(scenario)
                o.scenario = scenario;
            else
                o.scenario = simlab.Scenario('empty', 10);
            end
            for i = 1:2:numel(varargin)
                nm = lower(varargin{i});
                switch nm
                    case 'dt',       o.dt = varargin{i + 1};
                    case 'tuner',    o.tuner = varargin{i + 1};
                    case 'cascade',  o.cascade = varargin{i + 1};
                    case 'measfn',   o.measFn = varargin{i + 1};
                    case 'verbose',  o.verbose = logical(varargin{i + 1});
                    otherwise
                        error('simlab:Sim:badOption', ...
                              'unknown option "%s"', varargin{i});
                end
            end
        end

        function dt = sampleTime(o)
            % The loop rate. For a cascade the INNERMOST level sets it, which
            % is the rate the actuator is actually driven at.
            if ~isempty(o.dt)
                dt = o.dt;
                return;
            end
            if ~isempty(o.cascade)
                dt = o.cascade.innerSampleTime();
            elseif ~isempty(o.ctrl)
                dt = o.ctrl.getSampleTime();
            else
                dt = 0.01;
            end
            if ~(dt > 0)
                error('simlab:Sim:badDt', ...
                      'no usable sample time: set the controller dt or pass ''dt''');
            end
        end

        function r = run(o)
            % Execute the scenario. Returns a result struct; see below.
            dt = o.sampleTime();
            o.plant.reset();

            if ~isempty(o.ctrl)
                o.ctrl.reset();
            end
            if ~isempty(o.cascade)
                o.cascade.reset();
            end

            % ---- phase 1: the tuner, if there is one ----
            %
            % The plant is NOT reset afterwards. A tuning session leaves the
            % process wherever the experiment put it, and pretending otherwise
            % would measure the loop from a start you cannot reproduce on the
            % board.
            tune = [];
            if ~isempty(o.tuner)
                tune = o.runTunePhase(dt);
            end

            % ---- phase 2: the scenario ----
            % Events carry a 'done' flag from the previous run; a scenario
            % object reused for a second run (the app does this on every Run
            % press) would otherwise fire nothing the second time.
            o.scenario.clearDone();
            % An Inf horizon (integrating plant) makes n Inf, and `Inf < 2`
            % is false, so the old guard let zeros(1, Inf) through to its own
            % error. Finite-ness is part of the check.
            if ~isfinite(o.scenario.tEnd) || o.scenario.tEnd <= 0
                error('simlab:Sim:badHorizon', ...
                    ['the scenario horizon is %.6g s, which is not a usable ' ...
                     'duration. For an integrating plant, build the scenario ' ...
                     'with a finite tEnd (see defaultHorizon in the app).'], ...
                    o.scenario.tEnd);
            end
            n = round(o.scenario.tEnd / dt);
            if ~isfinite(n) || n < 2
                % A one-sample "run" is not a run, and everything downstream
                % (metrics, plots) degenerates on it. Name the two numbers
                % that produced it, because that is the whole diagnosis.
                error('simlab:Sim:tooShort', ...
                    ['the scenario is %.6g s long but the sample time is ' ...
                     '%.6g s, which gives %d sample(s). Either the duration ' ...
                     'field on the Scenario tab is wrong or the controller ' ...
                     'dt is - a run needs at least 2 samples.'], ...
                    o.scenario.tEnd, dt, max(n, 0));
            end
            r = o.newResult(n, dt);

            spCmd = 0;            % commanded setpoint (pre-ramp)
            spRampTarget = [];    % active setpointRamp, if any
            disturb = 0;          % additive offset on the measurement
            uPrev = 0;            % command the plant is currently under
            sc = o.scenario;

            for k = 1:n
                t = (k - 1) * dt;
                r.t(k) = t;

                % -- events first, so an event at t = 0 shapes sample 1 --
                idx = sc.dueIndex(t, dt);
                for j = 1:numel(idx)
                    e = sc.getEvent(idx(j));
                    switch e.type
                        case 'setpoint'
                            spCmd = e.a; spRampTarget = [];
                            o.commandSetpoint(spCmd);
                        case 'spRamp'
                            spRampTarget = struct('from', spCmd, 'to', e.a, ...
                                                  't0', t, 'T', max(e.b, dt));
                        case 'loadStep',  o.plant.setLoad(e.a, t);
                        case 'noise',     o.plant.setNoise(e.a);
                        case 'stuck',     o.plant.setStuckAt(e.a);
                        case 'dropout',   o.plant.setDropout(e.a);
                        case 'actLimits', o.plant.setActuatorLimits(e.a, e.b);
                        case 'plantGain', o.plant = o.plant.set('k', e.a);
                        case 'plantDelay'
                            o.plant = o.plant.set('deadtime', e.a);
                        case 'mode'
                            if ~isempty(o.cascade)
                                o.cascade.setMode(e.a);
                            else
                                o.ctrl.setMode(e.a);
                            end
                        case 'manual'
                            if ~isempty(o.cascade)
                                o.cascade.setManualOutput(e.a);
                            else
                                o.ctrl.setManualOutput(e.a);
                            end
                        case 'disturb'
                            disturb = e.a;
                        case 'custom'
                            e.fn(o, o.plant, o.ctrl, k, dt);
                    end
                    sc.markDone(idx(j));
                end

                % -- an interpolated command ramp --
                if ~isempty(spRampTarget)
                    frac = (t - spRampTarget.t0) / spRampTarget.T;
                    if frac >= 1
                        spCmd = spRampTarget.to;
                        spRampTarget = [];
                    else
                        spCmd = spRampTarget.from + ...
                            frac * (spRampTarget.to - spRampTarget.from);
                    end
                    o.commandSetpoint(spCmd);
                end

                % -- one sample: plant advances under the command the
                %    controller issued LAST cycle, then the controller reads
                %    the result. That is what a timer ISR does, and getting
                %    this order wrong silently adds a whole sample of extra
                %    dead time to every study. --
                o.plant.update(uPrev, dt);

                if ~isempty(o.cascade)
                    yv = o.measurements();
                    yv(end) = yv(end) + disturb;
                    u = o.cascade.update(yv, spCmd, dt);
                    y = yv(end);
                    st = o.cascade.innerStatus();
                else
                    y = o.plant.yMeas + disturb;
                    u = o.ctrl.update(y);
                    st = o.ctrl.getStatus();
                end

                r = o.record(r, k, y, u, st);
                uPrev = u;
            end

            r.scenario = sc.describe();
            r.dt = dt;
            r.tune = tune;
            r.metrics = simlab.metrics(r);
            if o.verbose
                simlab.Sim.printSummary(r);
            end
        end

        function tune = runTunePhase(o, dt)
            % Run the auto-tuner to completion, driving the plant from here.
            %
            % The tuner owns the actuator while it runs - that is what
            % PID_AutoTune_Update does in C too - so this phase replaces
            % PID_Update rather than wrapping it.
            nMax = round(10 * o.tuner.cfg.timeout_s / dt);
            if nMax < 100, nMax = 100; end

            tr = o.newResult(nMax, dt);
            y = 0;
            sp = o.tuner.sp;
            o.tuner.start(o.ctrl, sp);

            for k = 1:nMax
                u = o.tuner.update(y, dt);
                y = o.plant.update(u, dt);
                tr.t(k) = k * dt;
                tr.y(k) = y;
                tr.u(k) = u;
                tr.r(k) = sp;
                tr.state(k) = o.tuner.state;
                if ~o.tuner.isRunning()
                    tr.n = k;
                    break;
                end
            end

            tune = struct();
            % getResult has TWO outputs: the status code and the result. The
            % code is what tells a caller whether the gains below are real, so
            % it is kept rather than discarded.
            [tune.rc, tune.result] = o.tuner.getResult();
            tune.trace = simlab.Sim.trim(tr);
            tune.applied = false;

            if o.tuner.isComplete()
                rc = o.tuner.apply(o.ctrl);
                tune.applied = (rc == pidx.Const.OK);
            end
        end
    end

    % ==================================================================
    % Internals
    % ==================================================================

    methods (Access = private)
        function commandSetpoint(o, sp)
            % A cascade takes its setpoint per update() call, so there is
            % nothing to push here; only a single loop has a stored target.
            if isempty(o.cascade) && ~isempty(o.ctrl)
                o.ctrl.setSetpoint(sp);
            end
        end

        function yv = measurements(o)
            % Per-level measurement vector for a cascade. Without measFn the
            % honest answer is that there is none: guessing "the outer level
            % measures the integral of the inner one" would be a plant model
            % invented inside the runner, which is exactly the sort of hidden
            % assumption this tool must not make.
            if isempty(o.measFn)
                error('simlab:Sim:needMeasFn', ...
                      ['a cascade needs ''measFn'', a function returning a ' ...
                       'column vector of per-level measurements ' ...
                       '(outermost first), e.g. @(pl) pl.motorState([3;2])']);
            end
            yv = o.measFn(o.plant);
            yv = yv(:);
            if numel(yv) ~= o.cascade.count
                error('simlab:Sim:measFnSize', ...
                      'measFn returned %d values for a %d-level cascade', ...
                      numel(yv), o.cascade.count);
            end
        end

        function r = newResult(~, n, dt) %#ok<INUSD>
            r = struct();
            r.n = n; r.dt = dt;
            r.t = zeros(1, n); r.r = zeros(1, n); r.y = zeros(1, n);
            r.yTrue = zeros(1, n); r.u = zeros(1, n);
            r.uPlant = zeros(1, n); r.uRaw = zeros(1, n);
            r.p = zeros(1, n); r.i = zeros(1, n); r.d = zeros(1, n);
            r.ff = zeros(1, n); r.e = zeros(1, n);
            r.flags = zeros(1, 'uint32');
            r.kp = zeros(1, n); r.ki = zeros(1, n); r.kd = zeros(1, n);
            r.state = zeros(1, n);
        end

        function r = record(o, r, k, y, u, st)
            r.r(k) = o.effectiveSetpoint();
            r.y(k) = y;
            r.yTrue(k) = o.plant.yTrue;
            r.u(k) = u;
            r.uPlant(k) = o.plant.uPlant;
            if isempty(st)
                r.uRaw(k) = u;
            else
                r.uRaw(k) = st.output_unsat;
                r.p(k) = st.p_term;
                r.i(k) = st.i_term;
                r.d(k) = st.d_term;
                r.ff(k) = st.ff_term;
                r.e(k) = st.error;
                r.flags(k) = uint32(st.flags);
                r.kp(k) = st.kp_active;
                r.ki(k) = st.ki_active;
                r.kd(k) = st.kd_active;
            end
        end

        function sp = effectiveSetpoint(o)
            if ~isempty(o.cascade)
                sp = o.cascade.getLevelSetpoint(0);
            elseif ~isempty(o.ctrl)
                sp = o.ctrl.getSetpoint();
            else
                sp = 0;
            end
        end
    end

    methods (Static)
        function tr = trim(tr)
            % Drop the unused tail of a preallocated log.
            n = tr.n;
            fn = fieldnames(tr);
            for i = 1:numel(fn)
                f = fn{i};
                if strcmp(f, 'n') || strcmp(f, 'dt'), continue; end
                v = tr.(f);
                if isvector(v) && numel(v) > n
                    tr.(f) = v(1:n);
                end
            end
        end

        function printSummary(r)
            m = r.metrics;
            fprintf('\n--- %s ---\n', 'simlab.Sim');
            fprintf('  rise 10-90%% : %10.4g s\n', m.riseTime);
            fprintf('  overshoot   : %10.4g %%\n', m.overshoot);
            fprintf('  settle 2%%   : %10.4g s\n', m.settlingTime);
            fprintf('  IAE / ITAE  : %10.4g / %10.4g\n', m.iae, m.itae);
            fprintf('  steady err  : %10.4g\n', m.ssError);
            fprintf('  TV(u)       : %10.4g\n', m.tv);
            fprintf('  saturated   : %10.4g %%\n', 100 * m.satFraction);
            fprintf('  stable      : %10s\n', mat2str(m.stable));
        end
    end
end
