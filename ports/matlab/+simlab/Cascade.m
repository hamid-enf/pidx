classdef Cascade < handle
    % SIMLAB.CASCADE  Cascade control coordinator. Port of src/pid_cascade.c.
    %
    % THE IDEA
    %   A single loop must be tuned for the slowest dynamics in the plant.
    %   Cascade splits the problem: a fast inner loop closes around the fast,
    %   well-behaved part (motor current, valve flow) while a slow outer loop
    %   commands the inner loop's setpoint.
    %
    %       sp --->[ outer PID ]---> sp_inner --->[ inner PID ]---> actuator
    %                   ^                              ^
    %                   |                              |
    %                y_outer                        y_inner
    %
    % THE ONE RULE THAT MAKES IT WORK
    %   The inner loop must be substantially faster than the outer one - a
    %   factor of 3 to 10 in bandwidth is the standard guidance. validate()
    %   checks the ratio and reports it; it does not silently "fix" it,
    %   because the fix is a design decision, not a computation.
    %   Tune inner-first, always.
    %
    % THE HARD PART: OUTER-LOOP WINDUP
    %   When the inner loop saturates its actuator, the inner measurement can
    %   no longer follow the setpoint the outer loop is asking for. The outer
    %   loop sees persistent error and integrates - forever - even though its
    %   output is doing nothing. Output limits on the outer loop are NOT
    %   sufficient: the outer output can be perfectly inside its own limits
    %   while the inner loop is pinned at the actuator rail.
    %
    %   This class does it the correct way: back-propagation of saturation.
    %   Each cycle, from the innermost loop outwards, a loop that cannot
    %   deliver reports the value it actually achieved, and its parent's
    %   integrator is corrected to that achievable value.
    %
    % INDEXING, SAME AS C
    %   Index 0 is the OUTERMOST loop - the one that receives the user's
    %   setpoint. The last level is the innermost, and its output drives the
    %   physical actuator.
    %
    %   cl = simlab.Cascade({pidPos, pidVel});
    %   cl.configLevel(0, 10, -200, 200);      % outer runs 10x slower
    %   u = cl.update([theta; omega], spPos, dt);
    %
    % MULTI-RATE
    %   Set decimation on each level: the inner loop runs every call, an
    %   outer level with decimation 10 runs every 10th call and holds its
    %   output in between. Each level integrates over its OWN interval
    %   (dt * decimation), so the integral term is not short by the
    %   decimation factor - the mistake a naive implementation makes.

    properties (SetAccess = private)
        loops = {};           % cell array of pidx.PID, outermost first
        count = 0;
        decimation = [];
        spMin = [];
        spMax = [];
        awMode = 1;           % 0 NONE, 1 BACK_CALC, 2 FREEZE
        awGain = 1;
        mode = 1;             % PID_Mode applied to the whole chain
        initialised = false;
        output = 0;
        lastError = 0;
    end

    properties (Access = private)
        tick = [];
        command = [];
    end

    properties (Constant)
        AW_NONE = 0;
        AW_BACK_CALC = 1;
        AW_FREEZE = 2;
        MAX_LOOPS = 4;        % PIDX_CASCADE_MAX_LOOPS
    end

    methods
        function o = Cascade(loops)
            if nargin >= 1 && ~isempty(loops)
                rc = o.init(loops);
                if rc ~= pidx.Const.OK
                    error('simlab:Cascade:init', ...
                          'cascade init failed: %s', ...
                          pidx.Const.statusToString(rc));
                end
            end
        end

        function rc = init(o, loops)
            % Bind an array of already-initialised handles, OUTERMOST first.
            K = pidx.Const;
            o.lastError = K.OK;
            if isempty(loops)
                rc = K.ERR_NULL; return;
            end
            n = numel(loops);
            if n < 2 || n > o.MAX_LOOPS
                rc = K.ERR_INVALID_PARAM; return;
            end
            for i = 1:n
                if isempty(loops{i})
                    rc = K.ERR_NULL; return;
                end
                % A handle that never went through init has no usable dt, so
                % the cascade could not compute per-level periods. Catch it
                % here rather than producing silent nonsense at the first
                % update.
                if loops{i}.getSampleTime() <= 0
                    rc = K.ERR_NOT_INIT; return;
                end
            end

            o.loops = loops(:).';
            o.count = n;
            o.decimation = ones(1, n);
            o.spMin = zeros(1, n);
            o.spMax = zeros(1, n);      % min >= max => clamp disabled
            o.tick = zeros(1, n);
            o.command = zeros(1, n);
            o.awMode = o.AW_BACK_CALC;
            o.mode = loops{1}.getMode();
            o.output = 0;
            o.initialised = true;

            % Derive Kt_c from the outer loop: unwind at roughly the rate
            % that loop winds up, i.e. 1/Ti = Ki/Kp. Falls back to 1 1/s when
            % the outer loop has no integral action yet.
            o.awGain = 1.0;
            [~, kp, ki] = loops{1}.getGains();
            if ki > 0 && kp > 0
                o.awGain = ki / kp;
            end
            rc = K.OK;
        end

        function rc = configLevel(o, index, decimation, sp_min, sp_max)
            % Configure one level. index 0 = outermost.
            % sp_min >= sp_max disables the command clamp.
            K = pidx.Const;
            if ~o.initialised || index < 0 || index >= o.count
                rc = K.ERR_INVALID_PARAM; return;
            end
            if nargin >= 4 && (~isfinite(sp_min) || ~isfinite(sp_max))
                rc = K.ERR_INVALID_LIMIT; return;
            end
            i = index + 1;
            if nargin < 3 || isempty(decimation) || decimation == 0
                o.decimation(i) = 1;
            else
                o.decimation(i) = max(1, round(decimation));
            end
            if nargin >= 5
                o.spMin(i) = sp_min;
                o.spMax(i) = sp_max;
            end
            o.tick(i) = 0;
            rc = K.OK;
        end

        function rc = setAntiWindup(o, mode, aw_gain)
            K = pidx.Const;
            if mode > o.AW_FREEZE, rc = K.ERR_INVALID_PARAM; return; end
            if nargin >= 3 && ~isempty(aw_gain)
                if ~isfinite(aw_gain), rc = K.ERR_INVALID_PARAM; return; end
                o.awMode = mode;
                if aw_gain > 0
                    o.awGain = aw_gain;
                end
            else
                o.awMode = mode;
            end
            rc = K.OK;
        end

        % ==============================================================
        % Execution
        % ==============================================================

        function u = update(o, measurements, setpoint, dt)
            % One cascade cycle.
            %
            %   measurements : one value per level, SAME order as the handles
            %                  were passed to init (index 1 outermost).
            %   setpoint     : target for the outermost loop.
            %   dt           : elapsed time since the previous call.
            %
            % Execution is outermost to innermost: each level computes its
            % output, that output (clamped) becomes the next level's
            % setpoint, and the innermost output is returned. Saturation is
            % then propagated back inwards-to-outwards IN THE SAME CALL, so
            % no correction is ever a cycle late.
            K = pidx.Const;
            if ~o.isValid()
                o.setError(K.ERR_NOT_INIT);
                u = o.output; return;
            end
            if isempty(measurements)
                o.setError(K.ERR_NULL);
                u = o.output; return;
            end
            measurements = measurements(:).';
            if numel(measurements) ~= o.count
                o.setError(K.ERR_INVALID_PARAM);
                u = o.output; return;
            end
            if ~isfinite(dt) || dt <= 0
                o.setError(K.ERR_INVALID_DT);
                u = o.output; return;
            end
            if ~isfinite(setpoint)
                o.setError(K.ERR_NAN_INPUT);
                u = o.output; return;
            end

            % ---------------- forward pass: outer -> inner ----------------
            sp = setpoint;
            ran = false(1, o.count);

            for i = 1:o.count
                dec = o.decimation(i);
                ran(i) = false;
                o.tick(i) = o.tick(i) + 1;

                if o.tick(i) >= dec
                    % This level is due. It integrates over the whole
                    % interval since it last ran, not over the caller's dt -
                    % otherwise a decimated loop would under-integrate by
                    % exactly its decimation factor.
                    level_dt = dt * dec;
                    o.tick(i) = 0;
                    ran(i) = true;

                    o.loops{i}.setSetpointImmediate(sp);
                    o.command(i) = o.loops{i}.updateDt(measurements(i), ...
                                                       level_dt);
                end
                % else: hold command(i) from the previous run - a zero-order
                % hold, which is what the child physically experiences.

                % The command becomes the child's setpoint, clamped to a
                % range that makes physical sense for the child.
                sp = o.command(i);
                if o.spMin(i) < o.spMax(i)
                    sp = min(max(sp, o.spMin(i)), o.spMax(i));
                end
            end

            % Innermost output is the actuator command.
            o.output = o.command(o.count);

            % ---------------- backward pass: inner -> outer ---------------
            % Walk from the innermost parent outwards. For each parent, ask
            % whether its child could actually deliver what was requested; if
            % not, correct the parent so its integrator stops accumulating
            % against a wall.
            %
            % HOLD means "the integrator does not move", and MANUAL means "the
            % integrator is owned by the tracking back-solve". Both write the
            % parent's integrator directly, which bypasses the core's own
            % mode guard, so the mode has to be honoured here as well.
            if o.awMode ~= o.AW_NONE && o.mode == K.MODE_AUTOMATIC
                for i = o.count:-1:2
                    p = i - 1;                     % parent index
                    child = o.loops{i};
                    parent = o.loops{p};
                    requested = o.command(p);      % parent's raw output
                    achievable = requested;

                    if ~ran(p), continue; end
                    if ~isfinite(requested), continue; end

                    child_high = bitand(child.getFlags(), ...
                                        K.FLAG_SATURATED_HIGH) ~= 0;
                    child_low = bitand(child.getFlags(), ...
                                       K.FLAG_SATURATED_LOW) ~= 0;

                    % Was the parent's request clipped by the level clamp?
                    clipped_high = (o.spMin(p) < o.spMax(p)) && ...
                                   (requested > o.spMax(p));
                    clipped_low = (o.spMin(p) < o.spMax(p)) && ...
                                  (requested < o.spMin(p));

                    % Establish what was actually achievable downstream, and
                    % only act when the parent is pushing FURTHER into the
                    % obstruction. Direction matters: a child pinned at its
                    % upper rail must still let its parent integrate
                    % downwards - that is exactly how the pair escapes
                    % saturation. Correcting both directions would turn
                    % anti-windup into a lock-up.
                    if clipped_high
                        achievable = o.spMax(p);
                    elseif clipped_low
                        achievable = o.spMin(p);
                    elseif child_high && requested > measurements(i)
                        achievable = measurements(i);
                    elseif child_low && requested < measurements(i)
                        achievable = measurements(i);
                    else
                        continue;   % child is keeping up: nothing to correct
                    end
                    if ~isfinite(achievable), continue; end

                    parent_dt = dt * o.decimation(p);

                    if o.awMode == o.AW_BACK_CALC
                        %   I_parent += Kt_c*(u_achievable - u_requested)*dt
                        % Identical in form to the core's own
                        % back-calculation, with the child standing in for
                        % the actuator.
                        corr = o.awGain * (achievable - requested) * parent_dt;
                        if isfinite(corr)
                            parent.setIntegrator(parent.getIntegrator() + corr);
                        end
                    else
                        % FREEZE: undo just this cycle's accumulation, and
                        % only when the parent's error would drive it deeper
                        % into the blocked direction.
                        e = parent.getError();
                        digging = ((achievable < requested) && e > 0) || ...
                                  ((achievable > requested) && e < 0);
                        if digging
                            [~, kp, ki] = parent.getGains(); %#ok<ASGLU>
                            step = ki * e * parent_dt;
                            if isfinite(step)
                                parent.setIntegrator( ...
                                    parent.getIntegrator() - step);
                            end
                        end
                    end
                end
            end

            % Surface the first per-loop failure so a cascade user does not
            % have to poll every handle to notice a dead sensor.
            for i = 1:o.count
                e = o.loops{i}.peekLastError();
                if e ~= K.OK
                    o.setError(e);
                    break;
                end
            end

            u = o.output;
        end

        % ==============================================================
        % Mode, reset, inspection
        % ==============================================================

        function rc = setMode(o, mode)
            % Coordinated, bumpless mode change for the whole chain.
            %
            % Going to MANUAL: the innermost loop takes the manual value and
            % each outer loop is back-solved so that its output equals its
            % child's current setpoint. The whole chain therefore holds a
            % consistent state, and the return to AUTOMATIC is bumpless at
            % EVERY level - not just the innermost one. Switching modes level
            % by level with setMode() does not achieve this.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if mode > K.MODE_HOLD, rc = K.ERR_INVALID_MODE; return; end
            rc = K.OK;

            if mode == K.MODE_MANUAL
                % Innermost first, so each parent reads a child that has
                % already been placed in manual with a settled setpoint.
                for k = o.count:-1:1
                    r = o.loops{k}.setMode(K.MODE_MANUAL);
                    if r ~= K.OK, rc = r; end
                    if k > 1
                        % Parent should hold exactly what this child follows.
                        held = o.loops{k}.getSetpoint();
                        r = o.loops{k - 1}.setManualOutput(held);
                        if r ~= K.OK, rc = r; end
                        o.command(k - 1) = held;
                    end
                end
            else
                % Outermost first on the way back: by the time a child
                % switches to AUTOMATIC its parent is already producing a
                % live setpoint.
                for k = 1:o.count
                    r = o.loops{k}.setMode(mode);
                    if r ~= K.OK, rc = r; end
                end
            end

            o.mode = mode;
            o.setError(rc);
        end

        function rc = setManualOutput(o, output)
            % The manual value is an actuator command, so it belongs to the
            % innermost loop. Outer loops keep tracking their children.
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            if ~isfinite(output), rc = K.ERR_INVALID_PARAM; return; end
            o.command(o.count) = output;
            o.output = output;
            rc = o.loops{o.count}.setManualOutput(output);
        end

        function rc = reset(o)
            K = pidx.Const;
            if ~o.initialised, rc = K.ERR_NOT_INIT; return; end
            rc = K.OK;
            for i = 1:o.count
                r = o.loops{i}.reset();
                if r ~= K.OK, rc = r; end
                o.tick(i) = 0;
                o.command(i) = 0;
            end
            o.output = 0;
            o.lastError = K.OK;
        end

        function u = getOutput(o), u = o.output; end

        function sp = getLevelSetpoint(o, index)
            % The setpoint level INDEX is currently being asked to follow.
            if ~o.isValid() || index < 0 || index >= o.count
                sp = 0; return;
            end
            sp = o.loops{index + 1}.getSetpoint();
        end

        function h = getLoop(o, index)
            if ~o.isValid() || index < 0 || index >= o.count
                h = []; return;
            end
            h = o.loops{index + 1};
        end

        function sat = isSaturated(o)
            sat = false;
            if o.isValid()
                for i = 1:o.count
                    if bitand(o.loops{i}.getFlags(), ...
                              pidx.Const.FLAG_SATURATED) ~= 0
                        sat = true;
                        return;
                    end
                end
            end
        end

        function e = getLastError(o)
            e = o.lastError;
            o.lastError = pidx.Const.OK;
        end

        function [rc, min_ratio, worst_index] = validate(o)
            % Check the timescale separation between adjacent levels.
            %
            % 3x is the low end of the accepted design range; below it the
            % loops interact appreciably. The cascade will still run - this
            % is advice, not a veto - but expect the loops to fight.
            K = pidx.Const;
            min_ratio = inf;
            worst_index = 0;
            if ~o.isValid()
                rc = K.ERR_NOT_INIT; return;
            end
            for i = 1:(o.count - 1)
                t_parent = o.levelPeriod(i);
                t_child = o.levelPeriod(i + 1);
                if t_child <= 0, continue; end
                ratio = t_parent / t_child;
                if ratio < min_ratio
                    min_ratio = ratio;
                    worst_index = i - 1;
                end
            end
            if min_ratio >= 3.0
                rc = K.OK;
            else
                rc = K.ERR_INVALID_PARAM;
            end
        end

        function p = levelPeriods(o)
            % The effective sample interval of every level, outermost first:
            % each level's own dt times its decimation. What validate()
            % compares, exposed so a report can print the actual rates
            % instead of making the reader multiply them out.
            p = zeros(1, o.count);
            if o.isValid()
                for i = 1:o.count
                    p(i) = o.levelPeriod(i);
                end
            end
        end

        function dt = innerSampleTime(o)
            % The rate the actuator is driven at - what simlab.Sim steps at.
            dt = 0;
            if o.isValid()
                dt = o.levelPeriod(o.count);
            end
        end

        function st = innerStatus(o)
            st = [];
            if o.isValid()
                st = o.loops{o.count}.getStatus();
            end
        end

        function ok = isValid(o)
            ok = o.initialised && o.count >= 2;
        end
    end

    methods (Access = private)
        function p = levelPeriod(o, i)
            % Effective sample interval of a level: its own dt times its
            % decimation.
            p = o.loops{i}.getSampleTime() * o.decimation(i);
        end

        function setError(o, e)
            % Sticky, first-wins: an error must survive until somebody reads
            % it, and the first failure in a cycle is the informative one.
            if e ~= pidx.Const.OK && o.lastError == pidx.Const.OK
                o.lastError = e;
            end
        end
    end
end
