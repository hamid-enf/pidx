function [pos, vel, moving] = profileStep(pos, vel, target, rate_max, ...
                                          accel, decel, dt)
%PIDX.PROFILESTEP  Advance a trapezoidal / rate-limited profile one step.
%
%   [POS,VEL,MOVING] = PIDX.PROFILESTEP(POS,VEL,TARGET,RATE_MAX,ACCEL,DECEL,DT)
%
%   The single implementation shared by the controller's built-in setpoint
%   shaper and the standalone pidx.Shaper, exactly as pids_profile_step() is
%   in C. Keeping one copy is what makes "the built-in shaper and the
%   standalone object are the same algorithm" a fact rather than a hope.
%
%   Deceleration must begin when the remaining distance equals
%
%       d_brake = v^2 / (2 * a_decel)
%
%   which comes from integrating v*dv = a*dx. Starting later guarantees
%   overshoot; starting earlier just wastes time. Because the check runs every
%   sample against the current velocity, the profile is self-correcting and a
%   mid-flight target change needs no replanning.
%
%   ACCEL <= 0 selects the rate-only profile (constant speed, velocity steps
%   at each end). DECEL <= 0 mirrors ACCEL.

    dist = target - pos;
    v = vel;
    moving = true;

    if rate_max <= 0
        pos = target;
        vel = 0;
        moving = false;
        return;
    end
    if dist == 0
        vel = 0;
        moving = false;
        return;
    end

    if accel <= 0
        if dist > 0
            v = rate_max;
        else
            v = -rate_max;
        end
    else
        if decel > 0
            d = decel;
        else
            d = accel;
        end
        brake = (v * v) / (2 * d);

        if dist > 0
            if v < 0
                v = v + d * dt;          % travelling the wrong way
            elseif brake >= dist
                v = v - d * dt;          % time to start braking
            else
                v = v + accel * dt;      % speed up
            end
        else
            if v > 0
                v = v - d * dt;
            elseif brake >= -dist
                v = v + d * dt;
            else
                v = v - accel * dt;
            end
        end
        v = pidx.PID.clamp(v, -rate_max, rate_max);
    end

    step = v * dt;
    if abs(step) >= abs(dist)
        % Landing sample: snap to the target rather than overshoot it.
        pos = target;
        vel = 0;
        moving = false;
    else
        pos = pos + step;
        vel = v;
    end
end
