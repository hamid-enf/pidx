# PIDX for Python

A dependency-free Python port of the PIDX control framework. Not a wrapper
around the C library: a behavioural reimplementation, verified number by
number against it (502/502 conformance rows identical).

## Install

Copy the `pidx/` package next to your code, or put this directory on
`PYTHONPATH`. There is nothing to compile and nothing to pip-install.

## Five lines

```python
import pidx

pid = pidx.PID(pidx.quick_config(kp=2.0, ki=0.5, kd=0.1, dt=0.01,
                                 out_min=0.0, out_max=1.0))
pid.set_setpoint(100.0)
u = pid.update(measurement)
```

## Scaling up

```python
cfg = pidx.Config()
cfg.core.kp, cfg.core.ki, cfg.core.kd = 2.0, 0.5, 0.1
cfg.core.sample_time = 0.01
cfg.limits.use_output_limits = True
cfg.limits.output_min, cfg.limits.output_max = 0.0, 1.0

# cfg.integral.mode is the ANTI-WINDUP strategy, matching the C field name.
cfg.integral.mode = pidx.AntiWindup.BACK_CALCULATION

cfg.weight.beta = 0.6          # 2DOF: soften the setpoint kick
cfg.filter.n_filter = 10.0     # Tf = Kd / (N * Kp)
cfg.safety.enabled = True
cfg.safety.meas_min, cfg.safety.meas_max = -40.0, 150.0

pid = pidx.PID(cfg)
u = pid.update_dt(measurement, dt)     # measured dt, for a jittery loop
```

Field paths are identical to the C struct, so `docs/` translates directly.

## What is here

`PID`, `Cascade`, `Shaper`, `GainSchedule`, the filters (`LPF1`, `MovingAvg`,
`Median3`, `RateLimiter`, `deadband`) and the tuning rules (`rule_apply`).

Deliberately absent: the non-blocking auto-tune state machine, Q15/Q31 fixed
point, and the lock-free telemetry ring. See `../../docs/23_ports.md` for why
each was left out rather than stubbed.

## Verifying it yourself

```bash
cd .. && make compare
```
