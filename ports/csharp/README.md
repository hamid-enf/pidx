# PIDX for C#

A C# port of the PIDX control framework, verified number by number against the
C library (502/502 conformance rows identical).

> Built and tested with **Mono 6.12**. The .NET SDK is not in the Debian
> archive where this was developed. The code targets no Mono-specific API and
> should build unchanged with `dotnet build`, but "tested" means Mono.

## Build

```bash
mcs -target:library -out:Pidx.dll PidxTypes.cs PidxMath.cs Pid.cs \
    Shaper.cs GainSchedule.cs TuningRules.cs
```

or drop the `.cs` files straight into your project.

## Five lines

```csharp
using Pidx;

var pid = new Pid(Config.Quick(kp: 2.0, ki: 0.5, kd: 0.1, dt: 0.01,
                               outMin: 0.0, outMax: 1.0));
pid.SetSetpoint(100.0);
double u = pid.Update(measurement);
```

## Scaling up

```csharp
var cfg = new Config();
cfg.Core.Kp = 2.0; cfg.Core.Ki = 0.5; cfg.Core.Kd = 0.1;
cfg.Core.SampleTime = 0.01;
cfg.Limits.UseOutputLimits = true;
cfg.Limits.OutputMin = 0.0;
cfg.Limits.OutputMax = 1.0;

// Integral.Mode is the ANTI-WINDUP strategy, matching the C field name.
cfg.Integral.Mode = AntiWindup.BackCalculation;
cfg.Weight.Beta = 0.6;          // 2DOF: soften the setpoint kick
cfg.Filter.NFilter = 10.0;      // Tf = Kd / (N * Kp)
cfg.Safety.Enabled = true;

var pid = new Pid(cfg);
double u = pid.UpdateDt(measurement, dt);   // measured dt for a jittery loop
```

Field paths mirror the C struct exactly, only the casing is .NET.

## Status and errors

Every mutator returns a `Status`; nothing throws during control. The
constructor is the one exception — it throws on an invalid config, because a
controller that failed to initialise has no useful state to return.

```csharp
if (pid.PeekLastError() != Status.Ok) { /* sticky, survives until read */ }
```

## What is here

`Pid`, `Shaper`, `GainSchedule` and `TuningRules`. Deliberately absent: the
non-blocking auto-tune state machine, Q15/Q31 fixed point, the lock-free
telemetry ring, and cascade. See `../../docs/23_ports.md`.

## Verifying it yourself

```bash
cd .. && make compare
```
