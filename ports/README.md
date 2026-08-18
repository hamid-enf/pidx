# PIDX ports and the conformance harness

Four more implementations of the same controller, plus the machinery that
proves they *are* the same controller.

```bash
make            # build every available port, run it, diff the numbers
make compare    # compare existing CSVs only
make clean
```

Full documentation (Persian): `../docs/23_ports.md` and
`../docs/24_port_comparison.md`. The scenario language: `SPEC_conformance.md`.

---

## The idea

Comparing code by eye does not find bugs. Comparing numbers does.

```
compare/scenarios.txt ──┬──► c_ref/     (C, PIDX_USE_DOUBLE=1) ──► results/ref_c.csv  ◄── oracle
                        ├──► python/    (Python 3)             ──► results/py.csv
                        ├──► matlab/    (Octave 9.4)           ──► results/m.csv
                        └──► csharp/    (Mono 6.12)            ──► results/cs.csv
                                                                        │
                                              compare/compare.py ◄──────┘
```

69 scenarios, 502 rows, 14 columns per row. Acceptance is a relative error of
`1e-12`; in practice all three ports hit **exactly zero**.

The C library is the oracle and is built in **double** precision on purpose:
Python, Octave and C# are all natively double, so a float reference would
report rounding as disagreement and hide the real thing. That decision paid
for itself immediately — it exposed a genuine defect in the C tuning tables.

---

## Layout

| Path | What |
|---|---|
| `SPEC_conformance.md` | The scenario command language and CSV contract |
| `c_ref/conform_c.c` | Reference runner — drives the real PIDX core |
| `python/pidx/` | Python port: `types`, `mathutil`, `filters`, `shaper`, `controller`, `gainsched`, `autotune_rules`, `cascade` |
| `python/conform_py.py` | Python runner |
| `matlab/+pidx/` | MATLAB/Octave port: `Const`, `config`, `PID`, `GainSchedule`, `profileStep`, `ruleApply`, `plantModel`, `ruleRequiredModel` |
| `matlab/conform_m.m` | Octave runner |
| `csharp/` | C# port: `PidxTypes`, `PidxMath`, `Pid`, `Shaper`, `GainSchedule`, `TuningRules` |
| `csharp/ConformCs.cs` | Mono runner |
| `compare/compare.py` | Cell-by-cell comparison, exits non-zero on failure |
| `results/` | Generated CSVs |

The ESP32 port is not here — it is C, so it lives with the other platform
layers in `../platform/esp32/`, and its host test is
`../tests/test_esp32_host.c`.

---

## Five lines, in each language

All four produce byte-identical output.

**C**
```c
PID_Config c; PID_Handle h;
PID_ConfigDefault(&c);
c.core.kp = 2.0f; c.core.ki = 0.5f; c.core.sample_time = 0.01f;
PID_Init(&h, &c);
PID_SetSetpoint(&h, 100.0f);
float u = PID_Update(&h, measurement);
```

**Python**
```python
import pidx
pid = pidx.PID(pidx.quick_config(kp=2.0, ki=0.5, dt=0.01))
pid.set_setpoint(100.0)
u = pid.update(measurement)
```

**MATLAB / Octave**
```matlab
p = pidx.PID(pidx.config('kp', 2, 'ki', 0.5, 'dt', 0.01));
p.setSetpoint(100);
u = p.update(measurement);
```

**C#**
```csharp
var pid = new Pidx.Pid(Pidx.Config.Quick(kp: 2.0, ki: 0.5, dt: 0.01));
pid.SetSetpoint(100.0);
double u = pid.Update(measurement);
```

---

## Requirements, and what happens without them

| Tool | Used for | If missing |
|---|---|---|
| `gcc` | the reference | hard failure — there is nothing to compare against |
| `python3` | Python port **and** `compare.py` | nothing runs |
| `octave-cli` | MATLAB port | printed as `NOT COMPARED: matlab`, others still run |
| `mcs` + `mono` | C# port | printed as `NOT COMPARED: csharp`, others still run |

A missing toolchain is always **named**. A run that quietly checked three
ports because the fourth would not build reads exactly like a full pass, and
that is the failure mode this harness refuses to have.

---

## Adding a scenario

Append to `compare/scenarios.txt`, then `make`. Every runner picks it up with
no code change — that is the point of driving all five implementations from
one file. If a port lacks a command it will fail loudly with
`unknown run cmd`, never silently skip it.
