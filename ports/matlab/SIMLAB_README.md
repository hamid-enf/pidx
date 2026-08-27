# PIDX simlab — a MATLAB simulation and tuning workbench

Build the loop here, flash the C from here.

`+simlab` sits on top of `+pidx` — the port of the controller that is verified
**502/502 rows identical** to the C library — and adds the parts a tuning
session actually needs: a plant model with a real sensor and actuator chain, a
scenario engine, the auto-tune state machine, a Monte Carlo robustness study,
frequency-domain margins, and an exporter that writes compilable C for
STM32CubeIDE.

```matlab
cd <repo>/ports/matlab
simlab_setup
simlab_demos.demo_quick      % five-minute tour
```

Three ways in, same engine underneath:

| | | |
|---|---|---|
| `simlab_wizard` | guided console workflow | works over ssh, in Octave |
| `simlabApp` | graphical interface, five tabs | classic `uicontrol`, no App Designer |
| `simlab_demos.*` | scripted studies | the reference for automation |

---

## The problem this exists to solve

You tune in MATLAB because iterating there takes seconds. You flash C because
that is what the STM32 runs. Between those two is where gains get mistyped, a
derivative filter gets forgotten, and a loop that measured beautifully in
simulation behaves differently on the board.

The exporter closes that gap mechanically rather than by transcription:

```matlab
out = simlab.exportSTM32(plant, cfg, struct('symbol', 'heaterLoop', ...
      'profile', 'PROCESS', 'result', r, 'sens', s));
```

```
simlab_export/pidx_tuning_heaterLoop.h   every number as a #define, with units,
                                         plus the measured metrics as comments
simlab_export/pidx_tuning_heaterLoop.c   heaterLoop_init() + heaterLoop_tick(y)
```

The generated source is not a paraphrase. `simlab_tests/test_export.m` compiles
it against `include/pidx/`, links it with `src/*.c`, feeds both sides the same
measurement sequence, and compares the outputs. If the export drifts from the
library, that test fails here rather than on the board.

---

## What is in the package

| file | |
|---|---|
| `+simlab/Plant` | process model + actuator chain + sensor chain |
| `+simlab/Scenario` | a timed script of things happening to the loop |
| `+simlab/Sim` | the closed-loop runner; one sample loop for every study |
| `+simlab/AutoTune` | port of `src/pid_autotune.c`: relay + step + 9 rules |
| `+simlab/Cascade` | port of `src/pid_cascade.c`: N levels, saturation back-propagation |
| `+simlab/metrics` | rise, overshoot, settling, IAE/ISE/ITAE, TV, stability |
| `+simlab/sensitivity` | Ms, gain/phase/delay margin, bandwidth — no toolbox |
| `+simlab/monteCarlo` | K, tau, L each perturbed 0.5×..2×; survival share |
| `+simlab/compareRules` | all nine rules on your plant, both rankings |
| `+simlab/plot`, `plotSensitivity`, `plotRules` | the figures |
| `+simlab/exportSTM32` | the C |
| `+simlab/exportReport` | CSV trace + JSON record |

### The plant is three stages, not a transfer function

```
u_cmd -->[ actuator ]--> u_plant -->[ model ]--> y_true -->[ sensor ]--> y_meas
```

Saturation, deadband, rate limit and PWM resolution on the way in; noise, ADC
quantisation, deadband, transport delay, stuck-at and dropout on the way out.
Every stage is off by default, so a bare model is a pure plant and
`yMeas == yTrue` to the last bit.

`yTrue` and `yMeas` are both logged every sample. The distance between the two
lines on the plot **is** your sensor chain — which is the thing a
transfer-function simulation quietly assumes away.

Models: `fopdt` (exact), `linear` (any tf, ZOH-discretised), `dc_motor` (the
motor from `examples_stm32/03_motor_speed`, same constants, same sub-stepping),
`custom` (your own `f(x, u, dt)`).

```matlab
pl = simlab.Plant.presets('heater');     % K=2, tau=45 s, L=12 s, 12-bit ADC
pl = simlab.Plant('fopdt', 'k', 2, 'tau', 45, 'l', 12);
pl = simlab.Plant('custom', 'f', @(x,u,h) [-x(1)/0.5 + 2*u; x(1)], ...
                  'n', 2, 'yIndex', 2, 'x0', [0 0]);
```

### A scenario is a script, not a step

```matlab
sc = simlab.Scenario.presets('sensorFault');
sc = simlab.Scenario('my run', 60);
sc.setpoint(0, 0);
sc.setpoint(150, 1);
sc.loadStep(0.004, 20);       % load lands
sc.noise(0.4, 30);            % sensor gets noisy
sc.stuck(25, 40);             % sensor sticks
sc.stuck([], 50);             % and recovers
sc.actLimits(0, 60, 55);      % someone lowered the limit
r = simlab.Sim(plant, ctrl, sc).run();
```

`metrics()` finds the first setpoint step by itself and measures from there, so
the same function scores a bare step response and a forty-event script. Where a
metric has no window to measure over, it is `NaN` — never a fabricated zero.

---

## Auto-tuning is the C code, ported

`+simlab/AutoTune` is a line-by-line port of `src/pid_autotune.c`. That is
deliberate: the gains you flash come from `pid_autotune.c` on the target, so a
toolbox autotune would give you numbers that correspond to nothing on the
board.

Everything is preserved, including the parts that are unflattering:

- the hysteresis correction `Ku = 4h / (π √(a² − ε²))`;
- the **midpoint** moment arm in the step fit — sampling it at the right
  endpoint instead would leave the first moment short by `te·dt/2` per unit of
  `dy`, a deficit that *grows* with test length, so a longer and more careful
  experiment would produce a worse model;
- the quality score, and the `quality < 50` gate that refuses to return gains;
- the 20-samples-per-period floor that rejects a limit cycle the sample rate
  cannot resolve.

And the two findings the C study documented, which `simlab_tests` asserts
rather than papers over:

> **The relay always under-estimates Ku**, by up to −25 % even with zero noise.
> Describing-function theory wants the amplitude of the *fundamental*; every
> practical implementation measures the *peak*, and a limit cycle carries
> harmonics. The error is in the safe direction — gains come out low, so the
> loop is sluggish rather than unstable.

> **A near-perfect gain estimate can still cost you.** The step test's settling
> criterion (within 0.5 % of the *running* estimate, plus a flat slope) fires
> at roughly two thirds of the way up when the test starts from a non-zero
> bias, so `K` comes out high while `T` and `L` land within a few percent. The
> C library behaves identically; the test pins the number so a future
> "improvement" cannot change it silently.

```matlab
cfg = simlab.AutoTune.configDefault(pidx.Const.IDENT_RELAY);
cfg.output_step = 20;  cfg.hysteresis = 0.4;  cfg.bias = 50;
cfg.output_min = 0;    cfg.output_max = 100;
at = simlab.AutoTune(cfg);
% ... or let Sim run it and apply the result:
r = simlab.Sim(plant, ctrl, scenario, struct('tuner', at)).run();
```

A FREQ rule fed a FOPDT model is **rejected**, not converted: one point on the
Nyquist curve does not determine three model parameters, and no correct
conversion exists. The config check catches the pairing before any experiment
runs, so the diagnosis costs nothing instead of a two-minute tune.

---

## Robustness: the question that decides whether to flash

A step response against a perfect model is a number with no error bars. Your
model came from an identification, and identifications are 10–30 % wrong.

```matlab
mc = simlab.monteCarlo(plant, gains, struct('nRuns', 60, 'spread', 2));
%  93% of 60 plants stable; median IAE 1.204, worst 3.88
```

`compareRules` gives both rankings side by side, because they are close to
opposite — this is the finding from `sim/sim_robust.c` (810 runs, ρ = −0.586):

```matlab
cr = simlab.compareRules(plant, struct('mode', 'robust', 'nRuns', 30));
simlab.plotRules(cr);
```

The recommendation is **not** the lowest IAE. It is the fastest rule that
survived ≥ 90 % of the perturbed plants, and the text says so out loud.

Frequency-domain margins come from `sensitivity`, which needs no toolbox and
states its own assumptions:

```
L(jω) = C(jω) · G(jω) · exp(−jω(L + dt/2))
```

The `dt/2` is the zero-order hold. Omitting it makes every discrete loop look
more robust than it is, and the error grows with `dt` — at `dt = 0.5 s` on a
loop crossing near 1.7 rad/s it is 25 degrees of phase you did not account for.
`simlab_tests/test_sensitivity.m` checks exactly that.

---

## Verifying it

```matlab
cd simlab_tests
test_suite            % or: test_suite('autotune')
```

```
================ simlab test suite ================
C reference: 62 values from tools/matlab_ref
===================================================
test_plant             ok   (18 checks)
test_sim               ok   (33 checks)
...
  214 checks passed, 0 failed
```

The numeric assertions compare against `simlab_tests/c_reference.csv`, produced
by `tools/matlab_ref/matlab_ref.c` running **the same scenarios through the C
library** in double precision. The C library is the oracle for this repository
— it is what the STM32 runs — so a disagreement means the MATLAB is wrong, not
that the tolerance is tight. Regenerate after changing either side:

```bash
cd tools/matlab_ref && make run
```

The closed-loop test pins the sample order too: the plant advances under the
command the controller issued *last* cycle. Getting that wrong silently adds a
sample of dead time to every study, so the test also verifies that the wrong
order gives a *different* answer — otherwise it would pass for the wrong reason.

### Honest limits

- **No MATLAB or Octave interpreter was available where this was written.** The
  C oracle was built and run; the reference numbers in the tests are real. The
  MATLAB itself has not been executed by its author. Run `test_suite` before
  trusting any number, and `tools/matlab_lint.py` catches the structural class
  of error without an interpreter.
- `simlab.sensitivity` describes the **linear** part of the loop. Saturation,
  deadband and quantisation are not in a Bode plot; `plant.analysisCaveats()`
  says which of them your plant has, and the plots and the JSON report carry it.
- `Plant('linear')` needs `c2d` (Control System Toolbox). `fopdt`, `dc_motor`
  and `custom` are hand-integrated and need nothing.
- The GUI uses classic `uicontrol`, so it runs in Octave with a graphics
  toolkit. It is not an App Designer app and does not need one.

---

## Repository additions

```
ports/matlab/+simlab/           the tool (14 files)
ports/matlab/+simlab_tests/     test suite + assertion helpers
ports/matlab/simlab_tests/      test_suite.m + c_reference.csv
ports/matlab/simlab_demos/      demo_quick, demo_thermal, demo_motion, demo_robust
ports/matlab/simlab_setup.m     addpath
ports/matlab/simlab_wizard.m    console workflow
ports/matlab/simlabApp.m        graphical interface
tools/matlab_ref/               C oracle that generates c_reference.csv
tools/matlab_lint.py            structural linter, no interpreter needed
tools/check_export_identifiers.py  every emitted PID_* name exists in the headers
docs/25_matlab_simlab.md        Persian documentation
```
