classdef GainSchedule < handle
    % PIDX.GAINSCHEDULE  Interpolated gain scheduling. Port of pid_gainsched.c.
    %
    % A single set of gains is only optimal near one operating point. A valve
    % is more effective half open than nearly shut; a motor's effective
    % inertia changes with arm extension; a heater's loss coefficient grows
    % with temperature.
    %
    % Two properties together guarantee a bumpless traversal of the table:
    %
    %   1. gains are interpolated continuously, not selected from discrete
    %      regions - a region-select scheme steps the P term the instant a
    %      boundary is crossed;
    %   2. the core stores the integral term in output units, so changing Ki
    %      does not rescale accumulated history.
    %
    % With SCHED_INTERP_SMOOTH the gain curve is also C1-continuous, which
    % matters when the scheduling variable is itself noisy: linear
    % interpolation has a slope discontinuity at every breakpoint that noise
    % will rattle.
    %
    % The table is an N-by-4 matrix, one row per breakpoint: [x kp ki kd],
    % sorted by strictly ascending x.

    properties
        points = [];      % N-by-4: [x kp ki kd]
        count = 0;
        source = 0;
        interp = 0;
        hysteresis = 0.0;
        last_x = 0.0;
        primed = false;
    end

    methods
        function o = GainSchedule(points, source, interp)
            if nargin >= 1 && ~isempty(points)
                if nargin < 2, source = pidx.Const.SCHED_SRC_SETPOINT; end
                if nargin < 3, interp = pidx.Const.SCHED_INTERP_LINEAR; end
                rc = o.init(points, source, interp);
                if rc ~= pidx.Const.OK
                    error('pidx:GainSchedule:initFailed', ...
                          'schedule init failed: %d', rc);
                end
            end
        end

        function rc = init(o, points, source, interp)
            % Validate and install a table.
            K = pidx.Const;
            if nargin < 3, source = K.SCHED_SRC_SETPOINT; end
            if nargin < 4, interp = K.SCHED_INTERP_LINEAR; end

            if isempty(points)
                rc = K.ERR_NULL; return;
            end
            n = size(points, 1);
            if n < 2 || n > K.GAINSCHED_MAX_POINTS || size(points, 2) ~= 4
                rc = K.ERR_INVALID_PARAM; return;
            end
            if source > K.SCHED_SRC_EXTERNAL || interp > K.SCHED_INTERP_HOLD
                rc = K.ERR_INVALID_PARAM; return;
            end

            for i = 1:n
                if ~isfinite(points(i, 1))
                    rc = K.ERR_INVALID_PARAM; return;
                end
                % Strictly ascending: equal breakpoints would divide by zero,
                % and a descending table is always a mistake, not an intent.
                if i > 1 && points(i, 1) <= points(i - 1, 1)
                    rc = K.ERR_INVALID_PARAM; return;
                end
                for c = 2:4
                    if ~isfinite(points(i, c)) || points(i, c) < 0
                        rc = K.ERR_INVALID_GAIN; return;
                    end
                end
            end

            o.points = points;
            o.count = n;
            o.source = source;
            o.interp = interp;
            o.hysteresis = 0.0;
            o.last_x = points(1, 1);
            o.primed = false;
            rc = K.OK;
        end

        function rc = setHysteresis(o, band)
            K = pidx.Const;
            if ~isfinite(band) || band < 0
                rc = K.ERR_INVALID_PARAM; return;
            end
            o.hysteresis = band;
            rc = K.OK;
        end

        function [rc, kp, ki, kd] = evaluate(o, x)
            % Interpolate at x.
            K = pidx.Const;
            kp = 0; ki = 0; kd = 0;

            if isempty(o.points)
                rc = K.ERR_NULL; return;
            end
            if ~isfinite(x)
                rc = K.ERR_INVALID_PARAM; return;
            end

            % Hysteresis: ignore movement smaller than the band so sensor
            % noise around a breakpoint does not dither the gains.
            if o.primed && o.hysteresis > 0
                if abs(x - o.last_x) < o.hysteresis
                    x = o.last_x;
                end
            end
            o.last_x = x;
            o.primed = true;

            p = o.points;

            % Outside the table the gains saturate at the end points.
            % Extrapolating would be worse than useless: it can go negative.
            if x <= p(1, 1)
                kp = p(1, 2); ki = p(1, 3); kd = p(1, 4);
                rc = K.OK; return;
            end
            if x >= p(o.count, 1)
                kp = p(o.count, 2); ki = p(o.count, 3); kd = p(o.count, 4);
                rc = K.OK; return;
            end

            i = 1;
            while i < o.count
                if x >= p(i, 1) && x < p(i + 1, 1)
                    break;
                end
                i = i + 1;
            end

            t = (x - p(i, 1)) / (p(i + 1, 1) - p(i, 1));  % ascending: safe

            if o.interp == K.SCHED_INTERP_HOLD
                t = 0.0;
            elseif o.interp == K.SCHED_INTERP_SMOOTH
                % Smoothstep 3t^2 - 2t^3: derivative zero at both ends, so the
                % gain curve is C1 continuous across breakpoints.
                t = (t * t) * (3.0 - (2.0 * t));
            end

            % a + t*(b - a), matching the C helper's exact spelling: the
            % algebraically equal (1-t)*a + t*b differs in the last ulp.
            kp = p(i, 2) + t * (p(i + 1, 2) - p(i, 2));
            ki = p(i, 3) + t * (p(i + 1, 3) - p(i, 3));
            kd = p(i, 4) + t * (p(i + 1, 4) - p(i, 4));
            rc = K.OK;
        end
    end

    methods (Static)
        function rc = attach(pid, schedule)
            % Attach a schedule to a controller, or pass [] to detach. On
            % detach the gains stay at their last interpolated values.
            K = pidx.Const;
            if isempty(pid)
                rc = K.ERR_NULL; return;
            end
            if ~isempty(schedule) && ...
               (isempty(schedule.points) || schedule.count < 2)
                rc = K.ERR_INVALID_PARAM; return;   % never passed init()
            end
            pid.sched = schedule;
            if ~isempty(schedule)
                pid.features = bitor(pid.features, K.FEAT_GAIN_SCHED);
            else
                pid.features = bitand(pid.features, ...
                                      pidx.PID.notBits(K.FEAT_GAIN_SCHED));
            end
            rc = K.OK;
        end

        function rc = setVar(pid, value)
            % Supply the scheduling variable when the source is EXTERNAL.
            K = pidx.Const;
            if isempty(pid)
                rc = K.ERR_NULL; return;
            end
            if ~isfinite(value)
                rc = K.ERR_INVALID_PARAM; return;
            end
            pid.sched_var_ext = value;
            rc = K.OK;
        end
    end
end
