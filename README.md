# PIDX

**A production-grade, modular PID control framework in C99 for STM32 and
every other target you can point a compiler at.**

A five-line API for the simple case, scaling up to cascade control, gain
scheduling, non-blocking auto-tuning and fixed-point arithmetic — without
making the simple case pay for any of it.

```c
#include "pidx/pid.h"

PID_Handle pid;
PID_InitDefault(&pid);
PID_SetGains(&pid, 2.0f, 0.5f, 0.1f);
PID_SetOutputLimits(&pid, 0.0f, 100.0f);
PID_SetSetpoint(&pid, 60.0f);

for (;;) { actuate(PID_Update(&pid, read_sensor())); }
```

---

## Why another PID library

Most PID libraries ship a feature list. This one ships **measurements** — and
keeps the unflattering ones.

- **No dynamic allocation.** Not one `malloc` in the entire library. All state
  is caller-owned, so you can put a controller in CCMRAM or DTCM.
- **The core knows nothing about your MCU.** No HAL, no CMSIS, no
  `HAL_GetTick()`. You supply the timebase. The optional `platform/` layer is
  strictly separate, which is also why the whole library is testable on a PC.
- **A disabled feature costs zero.** Zero Flash, zero RAM, zero runtime
  branches — 7.8 KB for `MINIMAL` versus 25.4 KB for `FULL`, measured.
- **No stubs, no TODOs.** Every feature is mathematically real or absent. A
  `grep -r TODO src/` comes back empty.
- **No fabricated numbers.** No ARM toolchain was available during
  development, so this repository contains **zero measured Cortex-M figures**.
  Anything Cortex-M is labelled a design target, and `bench/bench_dwt.c` is
  shipped so you can produce the real table on your own board.

---

## Quick start

No build system required — add the sources and go:

```bash
gcc -std=c99 -Iinclude your_app.c src/*.c -lm -o app
```

1. Add `src/*.c` to your project.
2. Put `include/` on the include path.
3. `#include "pidx/pid.h"`.

Run everything:

```bash
make test       # 16 suites, 752 assertions
make examples   # 10 runnable examples
make sim        # 3 simulation studies
make bench      # host benchmark + size report
make gate       # 88 compilations, every profile, -Os and -O2, zero warnings
```

---

## Features

| | |
|---|---|
| **Core** | P / PI / PD / PID, direct & reverse action, backward-Euler or trapezoidal integration |
| **Anti-windup** | clamp, conditional, back-calculation, tracking |
| **Derivative** | on measurement / error / weighted error, always filtered |
| **2-DOF** | setpoint weighting (β, γ) |
| **Feedforward** | static value or user callback |
| **Modes** | manual / automatic / hold, all bumpless |
| **Shaping** | setpoint ramp with trapezoidal velocity profile, output slew limit |
| **Cascade** | N levels, decimation, saturation back-propagation |
| **Gain scheduling** | 6 scheduling sources, 3 interpolation modes, hysteresis |
| **Auto-tuning** | relay & step identification, 9 tuning rules, non-blocking state machine |
| **Fixed point** | standalone Q15/Q31 controller, zero floating-point ops |
| **Diagnostics** | lock-free SPSC telemetry ring, loop-quality metrics |
| **Safety** | sensor range & rate validation, fail-safe output, bumpless recovery |

Four compile-time profiles: `MINIMAL`, `MOTION`, `PROCESS`, `FULL`.

```bash
gcc -DPIDX_PROFILE_MOTION ...
```

---

## Measured results

### Performance (x86-64, `-O2`, minimum of 7 runs, loop overhead subtracted)

| Path | ns/call | FLOPs | rel |
|---|---|---|---|
| `PID_UpdateFast` | ~2.9 – 3.0 | 12 | 1.00× |
| `PID_Update` | ~11.6 – 12.5 | 18 | ~4.1× |
| `PID_UpdateDt` (constant dt) | ~12.1 – 12.3 | 19 | ~4.1× |
| `PID_UpdateDt` (varying dt) | ~16.6 – 16.9 | 28 | ~5.7× |
| `PIDq_Update` (Q15) | ~7.0 – 7.7 | **0** | ~2.5× |
| `PID_Cascade_Update` (2 loops) | ~41.0 – 41.5 | 40 | ~14× |

Ranges, not single values: repeated runs on a shared machine vary by a few
percent, and quoting `12.16` would be false precision. The **ratios** are
what port; the absolute numbers are host-specific. Run `make bench` to get
your own.

### Footprint (x86-64, `-Os`, `size -A`)

| Profile | Flash | Static RAM |
|---|---|---|
| `MINIMAL` | 7,792 B | 0 |
| `MOTION` | 16,633 B | 0 |
| `PROCESS` | 21,001 B | 80 B |
| `FULL` | 25,399 B | 80 B |

`PID_Handle` is 344 B; `PIDq_Handle` is 80 B.

### Feature effectiveness

| Feature | Measured effect |
|---|---|
| Cascade vs single loop | **8.4×** better disturbance rejection |
| Back-calculation anti-windup | recovery 91.5 s → **23.5 s** |
| Feedforward (function) | transient IAE 10.10 → **6.49** |
| Cascade AW freeze | overshoot 35.99 % → **17.65 %** |
| `PID_UpdateFast` | **4.1×** faster than the full path |

---

## Findings we did not expect

These came out of 810+ simulation runs. They are in the docs because they are
true, not because they are flattering.

**Exact-model performance anti-predicts robustness.** Ranking the nine tuning
rules by IAE against a perfect model correlates **ρ = −0.586** with ranking
them by survival when the plant is wrong. Ziegler-Nichols is 1st on a perfect
model and 7th when it isn't; IMC is 9th and 1st respectively. Low IAE is
bought with stability margin.

> With only 9 rules this is p ≈ 0.10 — suggestive, not significant, and it
> never can be. The *sign* is the finding. The per-rule survival rates rest
> on 90 runs each and are the firmer evidence.

**`NO_OVERSHOOT` overshoots by 42 %.** On an exact model, with zero
identification error. The name comes from a 1940s table and is not a
guarantee. Lowering `Kp` makes it *worse*; stretching `Ti` to 4·Pu is what
takes it to 0.0 %.

**The relay test always under-estimates Ku**, by up to −25 % even with zero
noise. Describing-function theory needs the amplitude of the *fundamental*;
every practical implementation measures the *peak*, and a limit cycle carries
harmonics. We kept the standard form: the error is in the safe direction
(gains too low ⇒ sluggish, not unstable), and no relay-tuned run diverged.

**A near-perfect gain estimate can still cost you 50 % IAE.** The step test
identifies `K` to 0.3 % — and produced the worst single penalty in the study,
because its *dead-time* error reaches +82 % and that is what actually hurts.

**Slew-limiting a 1 kHz servo made it worse.** It is a hardware-protection
tool, not a tuning tool.

---

## Repository layout

```
include/pidx/   13 public headers
src/            9 implementation files, zero HAL dependency
platform/       optional STM32, ESP32 and POSIX integration
ports/          Python, MATLAB/Octave and C# ports + conformance harness
examples/       11 compilable examples
tests/          17 suites, 816 assertions
sim/            3 simulation studies + plotting
bench/          host benchmark, DWT harness, size report
docs/           25 files (Persian)
```

---

## Ports

The same controller in four more places. The C library is the oracle: every
port is checked **number by number** against it, not reviewed by eye.

| Port | Path | Verified with | Result |
|---|---|---|---|
| ESP32 | `platform/esp32/` | host stub, 65 assertions | compiled + logic-tested, **not flashed** |
| Python | `ports/python/pidx/` | Python 3, no dependencies | **502/502 rows identical** |
| MATLAB | `ports/matlab/+pidx/` | GNU Octave 9.4 | **502/502 rows identical** |
| C# | `ports/csharp/` | Mono 6.12 | **502/502 rows identical** |

```bash
cd ports && make          # build every port, run it, diff the numbers
```

```
reference: results/ref_c.csv  (502 rows)
PASS  py          502 rows identical
PASS  m           502 rows identical
PASS  cs          502 rows identical
```

Zero tolerance was consumed: the difference is exactly zero, not "below
threshold". A port whose toolchain is missing is **named as skipped**, never
silently passed over.

The comparison found two real bugs — one in a port, and one in the C library
itself (`float`-suffixed literals in the tuning tables silently degraded a
`PIDX_USE_DOUBLE=1` build to float precision). Both are fixed and documented
in `docs/24_port_comparison.md`.

Honest limits: no Xtensa toolchain, no MathWorks MATLAB and no .NET SDK were
available, so the ESP32 layer is reviewed-but-unflashed, the MATLAB port is
validated under Octave, and the C# port under Mono. The non-blocking auto-tune
engine, Q15/Q31 fixed point and the lock-free telemetry ring are deliberately
**not** ported rather than stubbed; see `docs/23_ports.md`.

---

## Documentation

Full documentation is in **Persian** under `docs/` (API names stay English).
Start with `docs/02_quickstart.md`.

| | | |
|---|---|---|
| `00_architecture.md` | `01_intro_philosophy.md` | `02_quickstart.md` |
| `03_theory_discrete_pid.md` | `04_api_basic.md` | `05_api_intermediate.md` |
| `06_api_advanced.md` | `07_api_expert.md` | `08_antiwindup.md` |
| `09_derivative_filter.md` | `10_1dof_2dof.md` | `11_feedforward.md` |
| `12_gain_scheduling.md` | `13_cascade.md` | `14_autotune.md` |
| `15_safety_diagnostics.md` | `16_fixed_point.md` | `17_performance.md` |
| `18_stm32_integration.md` | `19_rtos_isr.md` | `20_troubleshooting.md` |
| `21_api_reference.md` | `22_misra_deviations.md` | `23_ports.md` |
| `24_port_comparison.md` | | |

---

## Standards

C99, no compiler extensions in the core. MISRA-C:2012 **aligned** — not
certified — with every deliberate deviation documented in
`docs/22_misra_deviations.md`.

The warning gate that every commit must pass:

```
-std=c99 -Wall -Wextra -Wconversion -Wdouble-promotion
-Wshadow -Wcast-qual -pedantic -Werror
```

× 4 profiles × {`-Os`, `-O2`}, one translation unit at a time:
**96 compilations, 0 warnings** (core + POSIX + STM32 + ESP32 layers).

---

## What PIDX is not

- Not MIMO, not MPC, not full adaptive control.
- Not safety-certified. No IEC 61508 / ISO 26262 / DO-178C. A hardware
  watchdog and an independent shutdown path remain your responsibility.
- Not a replacement for 20 lines of your own code if 20 lines is all you need.

---

## License

See `LICENSE`.
