function cfg = config(varargin)
%PIDX.CONFIG  Build a controller configuration struct with safe defaults.
%
%   CFG = PIDX.CONFIG() returns every field at its documented default, the
%   analogue of PID_ConfigDefault().
%
%   CFG = PIDX.CONFIG('kp',2,'ki',0.5,'kd',0.1,'dt',0.01) also fills the
%   common shorthand fields in one call.
%
%   The nested field paths are identical to the C struct, so documentation
%   translates directly:
%
%       cfg.core.{kp,ki,kd,sample_time,direction,mode,integration}
%       cfg.limits.{use_output_limits,output_min,output_max,
%                   use_integral_limits,integral_min,integral_max,
%                   dt_min,dt_max}
%       cfg.filter.{derivative_mode,tf,n_filter,input_lpf_tau}
%       cfg.integral.{mode,kt,separation_threshold,deadband,enabled}
%           NOTE: cfg.integral.mode is the ANTI-WINDUP strategy, matching
%           the C field of the same name.
%       cfg.weight.{beta,gamma}
%       cfg.feedforward.{enabled,fn,value,gain}
%       cfg.shaper.{sp_rate_max,sp_accel,sp_decel,out_slew_max}
%       cfg.safety.{enabled,meas_min,meas_max,meas_rate_max,
%                   failsafe_output,fault_persist_n,auto_recover}
%
%   Example - the five-line API:
%       c = pidx.config('kp',2,'ki',0.5,'dt',0.01);
%       p = pidx.PID(c);
%       p.setSetpoint(100);
%       u = p.update(measurement);

    K = pidx.Const;

    cfg = struct();
    cfg.abi_version = K.CONFIG_ABI_VERSION;

    cfg.core = struct( ...
        'kp', 0.0, 'ki', 0.0, 'kd', 0.0, ...
        'sample_time', K.DEFAULT_SAMPLE_TIME, ...
        'direction', K.DIRECT, ...
        'mode', K.MODE_AUTOMATIC, ...
        'integration', K.INTEGRATION_BACKWARD_EULER);

    cfg.limits = struct( ...
        'use_output_limits', false, ...
        'output_min', -K.HUGE_F, 'output_max', K.HUGE_F, ...
        'use_integral_limits', false, ...
        'integral_min', -K.HUGE_F, 'integral_max', K.HUGE_F, ...
        'dt_min', 0.0, 'dt_max', 0.0);

    cfg.filter = struct( ...
        'derivative_mode', K.DERIV_ON_MEASUREMENT, ...
        'tf', 0.0, ...
        'n_filter', K.DEFAULT_N_FILTER, ...
        'input_lpf_tau', 0.0);

    cfg.integral = struct( ...
        'mode', K.AW_CLAMP, ...
        'kt', 0.0, ...
        'separation_threshold', 0.0, ...
        'deadband', 0.0, ...
        'enabled', true);

    cfg.weight = struct('beta', 1.0, 'gamma', 0.0);

    cfg.feedforward = struct( ...
        'enabled', false, 'fn', [], 'value', 0.0, 'gain', 1.0);

    cfg.shaper = struct( ...
        'sp_rate_max', 0.0, 'sp_accel', 0.0, 'sp_decel', 0.0, ...
        'out_slew_max', 0.0);

    cfg.safety = struct( ...
        'enabled', false, ...
        'meas_min', 0.0, 'meas_max', 0.0, 'meas_rate_max', 0.0, ...
        'failsafe_output', 0.0, ...
        'fault_persist_n', 3, ...
        'auto_recover', false);

    % -- optional shorthand ---------------------------------------------
    for i = 1:2:numel(varargin)
        name = varargin{i};
        value = varargin{i + 1};
        switch lower(name)
            case 'kp',      cfg.core.kp = value;
            case 'ki',      cfg.core.ki = value;
            case 'kd',      cfg.core.kd = value;
            case 'dt',      cfg.core.sample_time = value;
            case 'outmin'
                cfg.limits.output_min = value;
                cfg.limits.use_output_limits = true;
            case 'outmax'
                cfg.limits.output_max = value;
                cfg.limits.use_output_limits = true;
            otherwise
                error('pidx:config:unknownOption', ...
                      'unknown option "%s"', name);
        end
    end
end
