# PIDX Examples

Ten runnable programs, in order of increasing scope. Every one compiles and
runs under the library's own build gate:

```
-std=c99 -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow \
-Wcast-qual -pedantic -Werror
```

An example that only compiles with the warnings turned off is not a template
anyone should copy, so they are held to the same standard as the library.

## Build and run

```sh
cd examples
make          # build all ten
make run      # build and run all ten in order
make run-ex06 # just one
make clean
```

Everything runs on the host — no hardware, no RTOS, no toolchain beyond a C99
compiler and libm. The plant models in `common/` are what make that possible.

## The examples

| # | Directory | What it teaches |
|---|---|---|
| 01 | `01_minimal` | The five-line API. Nothing else. |
| 02 | `02_temperature_pwm` | Output limits, all four anti-windup strategies compared, derivative filtering against ADC quantisation |
| 03 | `03_motor_speed` | Bidirectional actuator, encoder noise, derivative kick, setpoint weighting `beta`, Coulomb friction |
| 04 | `04_motor_position` | Integrating plant, trajectory shaping, why a position loop needs a *tight* integral clamp |
| 05 | `05_current_control` | 20 kHz loop, `PID_UpdateFast`, `PID_UpdateFast_IsSafe`, proof the two paths agree bit-for-bit |
| 06 | `06_cascade_pos_vel_cur` | Three-level cascade, decimation, current limiting as an inter-level clamp |
| 07 | `07_autotune_relay` | Non-blocking auto-tune, relay vs step identification, tuning rules, safety envelope |
| 08 | `08_freertos_task` | Drift-free fixed-rate loop, measured `dt`, static vs model-callback feedforward, `PID_UpdateEx` |
| 09 | `09_tim_isr` | ISR/background split, lock-free telemetry ring, loop metrics, `PID_GetStatus` |
| 10 | `10_full_featured` | Gain scheduling, modes, sensor safety, standalone filters, 2DOF, runtime gain changes, fixed-point, `PID_Deinit` |

## Shared harness (`common/`)

Not part of the library.

- **`ex_plant.h/.c`** — reference plants: FOPDT with a real transport delay
  (a ring buffer, not a Padé approximation, because a rational approximation
  of dead time hides exactly the instability dead time causes), a two-state DC
  motor with Coulomb friction, and a heater with nonlinear radiative loss.
  Deterministic noise from a fixed-seed LCG, so two runs are byte-identical.
- **`ex_report.h/.c`** — step-response metrics (rise, settling, overshoot,
  IAE/ISE/ITAE, actuator travel) and ASCII plotting, so an example ends in
  numbers you can check rather than a wall of samples.

The plants are `double` on purpose: the model should be more accurate than the
controller under test, so anything you see in the response comes from the
controller and not from the simulation.

## A note on what these examples measure

Several of them print results that are *worse* than you might expect, and say
so. Examples:

- Slew-limiting a fast servo loop makes it **less** stable (example 04) —
  a slew limit is a lag in the feedback path, and lag costs phase margin.
- `NO_OVERSHOOT` gains overshoot more than Ziegler-Nichols (example 07),
  because relay identification underestimated `Ku` by 27% and a rule can only
  be as good as the model it is given.
- An integral deadband can leave the integrator wandering **more** than no
  deadband at all (example 10), because it destroys the cancellation that
  makes zero-mean noise integrate to nothing.

These are kept because they are true, and because they are the cases where
knowing the mechanism actually matters.

Five real defects were found by writing these examples — three in the library
(`PID_AW_CONDITIONAL` was dead code, the fast path could diverge from the full
path, fault recovery bumped the actuator) and two stale doc claims. All are
fixed, with regression tests in `tests/test_antiwindup.c`,
`tests/test_fastpath.c` and `tests/test_bumpless.c`. See §۹.۸ of
`docs/00_architecture.md` for the full account.
