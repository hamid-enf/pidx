classdef Scenario < handle
    % SIMLAB.SCENARIO  A timed script of things happening to a control loop.
    %
    % A scenario is not a step response. It is the list of events a real loop
    % lives through: the operator changes the setpoint, a load lands on the
    % motor, the thermocouple goes noisy, the sensor sticks, the actuator
    % limits are lowered because a fuse keeps blowing. simlab.Sim plays the
    % list against the plant and the controller and records everything.
    %
    %   sc = simlab.Scenario('setpoint ramp with a load step');
    %   sc.setpoint(0.0, 0);            % start at zero
    %   sc.setpoint(5.0, 1.0);          % command 5 at t = 1 s
    %   sc.loadStep(0.004, 4.0);        % load lands at t = 4 s
    %   sc.noise(0.02, 6.0);            % sensor gets noisy from t = 6 s
    %   r = simlab.Sim(plant, controller, sc).run();
    %
    % Every adder returns the object, so a scenario reads top to bottom as a
    % script. Events at the same time are applied in the order they were
    % added, which is the order you wrote them.
    %
    % EVENT TYPES
    %   setpoint(v, t)          new commanded setpoint
    %   setpointRamp(v, t, T)   the command itself ramps over T seconds
    %   loadStep(v, t)          constant load torque (DC motor plants)
    %   noise(sigma, t)         change the measurement noise level
    %   stuck(v, t)             freeze the sensor at v ([] releases)
    %   dropout(p, t)           sensor repeats its last value with prob. p
    %   actLimits(lo, hi, t)    actuator saturation changes
    %   plantGain(k, t)         static gain changes - the plant ages
    %   plantDelay(l, t)        dead time changes - the pipe got longer
    %   mode(m, t)              controller mode: 0 manual, 1 auto, 2 hold
    %   manual(u, t)            manual output level (with mode 0)
    %   disturbPulse(a, t, dur) rectangular disturbance on the measurement
    %   custom(t, fn)           fn(sim, plant, controller, k, dt) at time t
    %
    % PRESETS - see simlab.Scenario.presets for the built-in studies.

    properties (SetAccess = private)
        name = 'scenario';
        tEnd = 10;            % total duration [s]; extended automatically
        nEvents = 0;
    end

    properties (Access = private)
        % Cell array of one struct per event, rather than a struct array.
        % A struct array cannot hold [] in a field (the assignment vanishes),
        % and 'stuck([], t)' genuinely means "release the fault". The cell
        % array keeps every event exactly as written.
        ev = {};
    end

    methods
        function o = Scenario(name, tEnd)
            if nargin >= 1 && ~isempty(name), o.name = name; end
            if nargin >= 2 && ~isempty(tEnd), o.tEnd = tEnd; end
        end

        function o = setDuration(o, tEnd)
            if ~(tEnd > 0)
                error('simlab:Scenario:badDuration', 'tEnd must be > 0');
            end
            o.tEnd = tEnd;
        end

        function o = setpoint(o, v, t)
            o = o.add(t, 'setpoint', v, 0, 0, []);
        end

        function o = setpointRamp(o, v, t, rampTime)
            % A ramped setpoint. Sim interpolates, so the ramp comes from the
            % command rather than from the controller's shaper.
            if nargin < 4, rampTime = 1.0; end
            o = o.add(t, 'spRamp', v, rampTime, 0, []);
        end

        function o = loadStep(o, v, t)
            o = o.add(t, 'loadStep', v, 0, 0, []);
        end

        function o = noise(o, sigma, t)
            o = o.add(t, 'noise', sigma, 0, 0, []);
        end

        function o = stuck(o, v, t)
            o = o.add(t, 'stuck', v, 0, 0, []);
        end

        function o = dropout(o, p, t)
            o = o.add(t, 'dropout', p, 0, 0, []);
        end

        function o = actLimits(o, lo, hi, t)
            o = o.add(t, 'actLimits', lo, hi, 0, []);
        end

        function o = plantGain(o, k, t)
            o = o.add(t, 'plantGain', k, 0, 0, []);
        end

        function o = plantDelay(o, l, t)
            o = o.add(t, 'plantDelay', l, 0, 0, []);
        end

        function o = mode(o, m, t)
            o = o.add(t, 'mode', m, 0, 0, []);
        end

        function o = manual(o, u, t)
            o = o.add(t, 'manual', u, 0, 0, []);
        end

        function o = disturbPulse(o, amp, t, dur)
            % Additive pulse on the measured variable: a gust of wind on a
            % drone, a cold slug entering a pipe.
            if nargin < 4, dur = 1.0; end
            o = o.add(t, 'disturb', amp, dur, 0, []);
            o = o.add(t + dur, 'disturb', 0, 0, 0, []);
        end

        function o = custom(o, t, fn)
            % fn(sim, plant, controller, k, dt) - anything the list above
            % cannot express. Runs before the sample is taken.
            o = o.add(t, 'custom', 0, 0, 0, fn);
        end

        function idx = dueIndex(o, tNow, dt)
            % Indices of the events whose time has arrived. A half-sample
            % tolerance means an event at t = 1.0000001 fires at the sample
            % where t crosses 1.0 instead of one sample late - invisible on a
            % plot, but it makes a test against a hand calculation exact.
            idx = [];
            tol = 0.5 * dt;
            for i = 1:numel(o.ev)
                e = o.ev{i};
                if ~e.done && e.t <= (tNow + tol)
                    idx(end + 1) = i; %#ok<AGROW>
                end
            end
        end

        function e = getEvent(o, i)
            e = o.ev{i};
        end

        function o = markDone(o, i)
            e = o.ev{i};
            e.done = true;
            o.ev{i} = e;
        end

        function s = describe(o)
            % A printable transcript of the script. What the report embeds.
            [~, ord] = sort(o.times());
            lines = cell(o.nEvents + 2, 1);
            lines{1} = sprintf('scenario "%s", %.6g s, %d event(s)', ...
                               o.name, o.tEnd, o.nEvents);
            for i = 1:o.nEvents
                e = o.ev{ord(i)};
                lines{i + 1} = sprintf('  t=%-10.6g %-12s %s', e.t, e.type, ...
                                       simlab.Scenario.fmtArgs(e));
            end
            lines{end} = '';
            s = sprintf('%s\n', lines{:});
        end

        function tv = times(o)
            tv = zeros(1, o.nEvents);
            for i = 1:o.nEvents
                tv(i) = o.ev{i}.t;
            end
        end
    end

    methods (Access = private)
        function o = add(o, t, type, a, b, c, fn)
            if ~(t >= 0)
                error('simlab:Scenario:badTime', 'event time must be >= 0');
            end
            o.nEvents = o.nEvents + 1;
            o.ev{o.nEvents} = struct('t', t, 'type', type, 'a', a, ...
                                     'b', b, 'c', c, 'fn', fn, ...
                                     'done', false);
            % A scenario that ends before its last event is a silent trap: the
            % event never fires and the plot looks like the controller
            % ignored it. Extend instead.
            if t > o.tEnd
                o.tEnd = t;
            end
        end
    end

    methods (Static)
        function s = fmtArgs(e)
            switch e.type
                case {'setpoint', 'loadStep', 'noise', 'plantGain', ...
                      'plantDelay', 'mode', 'manual', 'dropout', 'stuck'}
                    if isempty(e.a)
                        s = '(release)';
                    else
                        s = num2str(e.a, '%.6g');
                    end
                case {'actLimits', 'spRamp'}
                    s = sprintf('%.6g  %.6g', e.a, e.b);
                case 'disturb'
                    s = sprintf('%.6g  (%.6g s)', e.a, e.b);
                otherwise
                    s = '<function>';
            end
        end

        function sc = presets(which, varargin)
            % PRESETS(NAME) - the studies the demos run.
            %
            %   'stepResponse'    one setpoint step, nothing else. The
            %                     baseline every other number is compared to.
            %   'disturbance'     settle, then a load step. Disturbance
            %                     rejection is a different problem from
            %                     setpoint tracking and needs its own tuning.
            %   'windup'          a step the actuator cannot reach: the whole
            %                     point of an anti-windup comparison.
            %   'noise'           settle, then heavy measurement noise.
            %   'sensorFault'     settle, sensor sticks, recovers. Exercises
            %                     PIDX's safety latch and bumpless re-entry.
            %   'agingPlant'      gain and dead time both drift mid-run. The
            %                     honest case, and the one gain scheduling
            %                     exists for.
            %   'setpointProfile' a ramped setpoint, for motion loops.
            %
            % Name/value pairs override the preset:
            % presets('windup', 'tEnd', 30).
            p = struct('sp', 100, 'sp2', 60, 'dist', 0.3, 'noise', 0.02, ...
                       'tEnd', 30, 'tStep', 1, 'tDist', 15, 'tNoise', 15, ...
                       'tFault', 12, 'tFaultEnd', 16);
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i + 1};
            end

            sc = simlab.Scenario(which, p.tEnd);
            switch lower(which)
                case 'stepresponse'
                    sc.setpoint(0, 0);
                    sc.setpoint(p.sp, p.tStep);

                case 'disturbance'
                    sc.setpoint(0, 0);
                    sc.setpoint(p.sp, p.tStep);
                    sc.loadStep(p.dist, p.tDist);

                case 'windup'
                    % Command far beyond what the actuator can hold, then
                    % bring it back. Without anti-windup the return takes
                    % minutes; with it, seconds. The comparison IS the demo.
                    sc.setpoint(0, 0);
                    sc.setpoint(4 * p.sp, p.tStep);
                    sc.setpoint(p.sp, p.tDist);

                case 'noise'
                    sc.setpoint(0, 0);
                    sc.setpoint(p.sp, p.tStep);
                    sc.noise(p.noise, p.tNoise);

                case 'sensorfault'
                    sc.setpoint(0, 0);
                    sc.setpoint(p.sp, p.tStep);
                    sc.stuck(p.sp * 0.5, p.tFault);
                    sc.stuck([], p.tFaultEnd);

                case 'agingplant'
                    sc.setpoint(0, 0);
                    sc.setpoint(p.sp, p.tStep);
                    sc.plantGain(1.6, p.tDist);
                    sc.plantDelay(1.5, p.tDist);
                    sc.setpoint(p.sp2, p.tNoise);

                case 'setpointprofile'
                    sc.setpoint(0, 0);
                    sc.setpointRamp(p.sp, p.tStep, 2.0);
                    sc.setpointRamp(p.sp2, p.tDist, 2.0);

                otherwise
                    error('simlab:Scenario:badPreset', ...
                          'unknown preset "%s"', which);
            end
        end
    end
end
