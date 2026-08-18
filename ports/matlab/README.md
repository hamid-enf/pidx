# PIDX for MATLAB / Octave

A `classdef` port of the PIDX control framework, verified number by number
against the C library (502/502 conformance rows identical).

> Validated under **GNU Octave 9.4.0**. MathWorks MATLAB was not available
> where this was developed, so while the code uses no toolbox functions and
> no Octave-only syntax, "tested" means Octave.

## Install

```matlab
addpath('/path/to/pidx/ports/matlab');
```

The `+pidx` package directory must be a *child* of a directory on the path,
not on the path itself.

## Five lines

```matlab
p = pidx.PID(pidx.config('kp', 2, 'ki', 0.5, 'kd', 0.1, 'dt', 0.01));
p.setSetpoint(100);
u = p.update(measurement);
```

## Scaling up

```matlab
cfg = pidx.config();
cfg.core.kp = 2.0;  cfg.core.ki = 0.5;  cfg.core.kd = 0.1;
cfg.core.sample_time = 0.01;
cfg.limits.use_output_limits = true;
cfg.limits.output_min = 0;  cfg.limits.output_max = 1;

% cfg.integral.mode is the ANTI-WINDUP strategy, matching the C field.
cfg.integral.mode = pidx.Const.AW_BACK_CALCULATION;
cfg.weight.beta = 0.6;
cfg.safety.enabled = true;

p = pidx.PID(cfg);
u = p.updateDt(measurement, dt);
```

`pidx.PID` derives from `handle`, so it behaves like the C pointer it mirrors:
pass it to a function and the callee mutates the same controller.

## Simulating a loop

```matlab
p = pidx.PID(pidx.config('kp', 2, 'ki', 0.5, 'dt', 0.01));
p.setSetpoint(1);
y = 0;  n = 500;  hist = zeros(n, 2);
for k = 1:n
    u = p.update(y);
    y = y + 0.01 * (2*u - y);      % first-order plant
    hist(k, :) = [u, y];
end
plot((1:n)*0.01, hist);  legend('u', 'y');  grid on;
```

## Gain scheduling

```matlab
% [x kp ki kd], strictly ascending in x
tab = [ 0  1.0 0.10 0
        5  2.0 0.50 0
       10  4.0 1.00 0];
s = pidx.GainSchedule(tab, pidx.Const.SCHED_SRC_ABS_ERROR, ...
                      pidx.Const.SCHED_INTERP_SMOOTH);
pidx.GainSchedule.attach(p, s);
```

## Verifying it yourself

```bash
cd .. && make compare
```
