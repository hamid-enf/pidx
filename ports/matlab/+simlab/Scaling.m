classdef Scaling < handle
    % SIMLAB.SCALING  The map between engineering units and Q15.
    %
    % WHY THIS IS A CLASS AND NOT A MULTIPLIER
    %   The fixed-point controller works in a normalised domain where 1.0 is
    %   32768. Nothing in pid_fixed.c knows what your degrees Celsius or your
    %   PWM counts are, and the caller is expected to normalise - a shift for
    %   an ADC, the inverse for a duty cycle. That shift IS part of the
    %   controller: it sets the resolution, and therefore it decides whether
    %   the integrator can move at all.
    %
    %   So it is written down once, here, with the arithmetic that follows
    %   from it, rather than being a magic constant in three places.
    %
    %       y_q15 = round((y_eng - lo) / (hi - lo) * 32767)
    %
    %   The map is onto [0, 32767] for a unipolar signal, or [-32768, 32767]
    %   when the range straddles zero. Everything outside the declared range
    %   CLAMPS - an ADC reading above full scale must not wrap to a large
    %   negative command.
    %
    % GAINS DO NOT SCALE THE SAME WAY AS SIGNALS
    %   Kp is dimensionless, so it converts straight to Q16.16. Ki has units
    %   of 1/s and Kd of s, and those are unaffected by the signal scaling
    %   only when the input and the output are scaled by the SAME factor.
    %   When they are not - a temperature loop commanding a PWM duty, say -
    %   the gains carry the ratio:
    %
    %       Kp' = Kp * (su / sy)
    %
    %   where su and sy are the output and measurement spans. fromFloatGains()
    %   does this and STATES the ratio it used, because a gain quietly scaled
    %   by 100 is the classic way a fixed-point port ends up tuning a
    %   different loop from the one you designed.
    %
    % EXAMPLE
    %   sy = simlab.Scaling(0, 300);      % 0..300 degC  -> Q15
    %   su = simlab.Scaling(0, 100);      % 0..100 %     -> Q15
    %   yq = sy.toQ15(150);
    %   gq = simlab.Scaling.fromFloatGains(kp, ki, kd, sy, su);

    properties (SetAccess = private)
        lo = 0;
        hi = 1;
        fullScale = 32767;
        name = 'signal';
    end

    methods
        function o = Scaling(lo, hi, name)
            if nargin >= 1 && ~isempty(lo), o.lo = lo; end
            if nargin >= 2 && ~isempty(hi), o.hi = hi; end
            if nargin >= 3 && ~isempty(name), o.name = name; end
            if ~(o.hi > o.lo)
                error('simlab:Scaling:range', 'hi must be greater than lo');
            end
            % A range that straddles zero uses the whole signed domain; a
            % unipolar one only the positive half. Wasting half the codes on a
            % 0..300 degC thermocouple would halve the resolution for nothing.
            if o.lo >= 0
                o.fullScale = 32767;
            else
                o.fullScale = 32768;
            end
        end

        function q = toQ15(o, v)
            % Unipolar ranges map onto [0, 32767]: a 0..100% duty cycle has no
            % negative side, and wasting half the code space on it would halve
            % the resolution. Bipolar ranges map symmetrically onto the full
            % signed domain, so -full scale is exactly -32768.
            if o.lo >= 0
                q = (v - o.lo) / (o.hi - o.lo) * o.fullScale;
            else
                q = v / max(-o.lo, o.hi) * 32768;
            end
            q = simlab.Scaling.roundAway(q);
            if q > 32767,  q = 32767;  end
            if q < -32768, q = -32768; end
            q = int16(q);
        end

        function v = toEng(o, q)
            if o.lo >= 0
                v = double(q) / o.fullScale * (o.hi - o.lo) + o.lo;
            else
                v = double(q) / 32768 * max(-o.lo, o.hi);
            end
        end

        function s = span(o)
            s = o.hi - o.lo;
        end

        function r = resolution(o)
            % One Q15 LSB, in engineering units. The number that decides
            % whether the derivative term is measuring the process or the
            % converter.
            r = (o.hi - o.lo) / o.fullScale;
        end


        function ok = fitsQ15(o, v)
            % True when every value in V is inside the declared range.
            %
            % Asking this BEFORE building a fixed-point loop is cheaper than
            % discovering it as a wrapped actuator command: a measurement
            % above full scale clamps, which is survivable, but a range that
            % was declared too narrow silently compresses the whole signal
            % into part of the code space.
            ok = all(v >= o.lo & v <= o.hi);
        end
    end

    methods (Static)
        function g = fromFloatGains(kp, ki, kd, sy, su)
            % GAINS = FROMFLOATGAINS(KP, KI, KD, SY, SU)
            %
            % Converts a floating-point tuning to Q16.16, accounting for the
            % fact that the measurement and the output may be scaled
            % differently. Returns a struct with the Q16 values AND the
            % floats they came from, so a report can show both and a reader
            % can see the ratio that was applied.
            %
            % Ki and Kd are NOT rescaled by the signal ratio: Ki integrates
            % error over time and its units are output-per-input-per-second,
            % which the span ratio already appears in through Kp. Rescaling
            % them too would apply the ratio twice.
            ratio = su.span() / sy.span();
            g = struct();
            g.kp_float = kp;
            g.ki_float = ki;
            g.kd_float = kd;
            g.ratio = ratio;
            g.kp_q16 = simlab.PIDq.fToQ16(kp * ratio);
            g.ki_q16 = simlab.PIDq.fToQ16(ki * ratio);
            g.kd_q16 = simlab.PIDq.fToQ16(kd * ratio);
        end

        function r = roundAway(x)
            if x < 0
                r = -floor(-x + 0.5);
            else
                r = floor(x + 0.5);
            end
        end

        function s = describe(sy, su)
            % One sentence stating the resolution on both sides. What a
            % fixed-point report puts at the top, because it is the thing
            % that decides whether the rest of it means anything.
            s = sprintf(['measurement %.6g..%.6g (%d LSB/unit) | output ' ...
                         '%.6g..%.6g (%d LSB/unit)'], ...
                sy.lo, sy.hi, 1 / sy.resolution(), ...
                su.lo, su.hi, 1 / su.resolution());
        end
    end
end
