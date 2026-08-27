classdef Q15Loop < handle
    % SIMLAB.Q15LOOP  A Q15 controller dressed as a floating-point one.
    %
    % simlab.Sim drives a pidx.PID. This wrapper presents the same handful of
    % methods around a simlab.PIDq so the SAME runner, the SAME plant and the
    % SAME scenario can drive either - which is the only way a float-versus
    % fixed comparison means anything. Two runners would differ in their
    % sample order or their event handling, and then the difference in the
    % traces would be the harness rather than the number format.
    %
    % The scaling is applied at the boundary and nowhere else:
    %
    %   setSetpoint(sp_eng)   ->  q.setSetpoint(sy.toQ15(sp_eng))
    %   u_eng = update(y_eng) ->  su.toEng(q.update(sy.toQ15(y_eng)))
    %
    % Everything inside is integer arithmetic. Nothing in here rounds twice.
    %
    % DELIBERATELY NOT HIDDEN
    %   getStatus() reports the Q15 output converted back to engineering
    %   units, so a plot of u is directly comparable with the float loop's.
    %   The RAW Q15 value is available from rawOutput() because the staircase
    %   it makes is exactly what the study is looking for, and smoothing it
    %   away on the way out would hide the effect being measured.

    properties (SetAccess = private)
        q;              % the simlab.PIDq controller
        sy;             % measurement scaling
        su;             % output scaling
        dt;             % sample time [s]
        initialised = false;
    end

    properties (Access = private)
        spEng = 0;
        lastYEng = 0;
        lastUEng = 0;
        lastUQ15 = int16(0);
    end

    methods
        function o = Q15Loop(q, sy, su, spEng, dt)
            if ~q.isValid()
                error('simlab:Q15Loop:notInit', ...
                      'the PIDq controller is not initialised');
            end
            o.q = q;
            o.sy = sy;
            o.su = su;
            o.dt = q.dt_us / 1e6;
            if nargin >= 4 && ~isempty(spEng)
                o.spEng = spEng;
                o.setSetpoint(spEng);
            end
            if nargin >= 5 && ~isempty(dt)
                o.dt = dt;
            end
            o.initialised = true;
        end

        function u = update(o, y_eng)
            o.lastYEng = y_eng;
            o.lastUQ15 = o.q.update(o.sy.toQ15(y_eng));
            o.lastUEng = o.su.toEng(o.lastUQ15);
            u = o.lastUEng;
        end

        function rc = setSetpoint(o, sp_eng)
            rc = o.q.setSetpoint(o.sy.toQ15(sp_eng));
            o.spEng = sp_eng;
        end

        function v = getSetpoint(o)
            v = o.spEng;
        end

        function rc = reset(o)
            rc = o.q.reset();
            o.lastYEng = 0;
            o.lastUEng = 0;
            o.lastUQ15 = int16(0);
        end

        function v = getSampleTime(o)
            v = o.dt;
        end

        function v = getOutput(o)
            v = o.lastUEng;
        end

        function v = rawOutput(o)
            % The Q15 value before it is converted back. The staircase in
            % this signal is the quantisation the study is measuring.
            v = o.lastUQ15;
        end

        function v = getIntegral(o)
            % Integral term, in engineering units, for a report.
            v = o.su.toEng(o.q.getIntegral());
        end

        function v = isSaturated(o)
            v = o.q.isSaturated();
        end

        function s = getStatus(o)
            % The subset of PID_Status that simlab.Sim and simlab.metrics
            % read, filled from the fixed-point state. Fields this controller
            % does not have are reported as zero rather than omitted, so the
            % same plotting code works for both loops and an absent feature
            % cannot be mistaken for a missing one.
            s = pidx.PID.emptyStatus();
            s.setpoint_raw = o.spEng;
            s.setpoint_shaped = o.spEng;
            s.measurement_raw = o.lastYEng;
            s.measurement_filtered = o.lastYEng;
            s.error = o.spEng - o.lastYEng;
            s.p_term = 0;         % not separable in the fixed-point path
            s.i_term = o.su.toEng(o.q.getIntegral());
            s.d_term = 0;
            s.ff_term = 0;
            s.output = o.lastUEng;
            s.output_unsat = o.lastUEng;
            s.dt_used = o.dt;
            s.kp_active = simlab.PIDq.q16ToF(o.q.kp_q16);
            s.flags = 0;
            if o.q.isSaturated()
                s.flags = pidx.Const.FLAG_SATURATED_HIGH;
            end
        end
    end
end
