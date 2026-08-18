/**
 * @file    examples/11_esp32_task/main.c
 * @brief   ESP32 / ESP-IDF: a pinned FreeRTOS control task with a measured dt.
 *
 * WHAT THIS SHOWS
 *   1. The PIDX core is untouched by the platform. Everything ESP-specific
 *      lives behind pid_esp32.h; delete platform/esp32 and the controller code
 *      below still compiles against a different timebase.
 *   2. The control task is PINNED to a core. On ESP32/S3 the Wi-Fi and
 *      Bluetooth stacks default to core 0, and a hard-real-time loop sharing a
 *      core with the radio is how deadline jitter gets in.
 *   3. dt is MEASURED, not assumed. PIDe_Rate reports the real interval, so
 *      the integrator and derivative stay correct when a period slips.
 *   4. The loop load is monitored. A loop quietly using 80% of its period is
 *      one plant change away from missing deadlines.
 *
 * WHY NOT vTaskDelayUntil FOR THE LOOP RATE
 *   FreeRTOS delays are quantised to the tick, 10 ms by default on ESP-IDF
 *   (CONFIG_FREERTOS_HZ = 100). A 1 kHz loop cannot be built on it: the
 *   request would silently run at 100 Hz. This example therefore polls
 *   PIDe_RateElapsed() with a short yielding delay, which keeps the schedule
 *   accurate to the microsecond timebase while still letting lower-priority
 *   work run. For loops faster than a few kHz, drive the update from an
 *   esp_timer callback or a hardware timer ISR instead and keep using
 *   PIDe_Rate only to measure dt.
 *
 * BUILD (ESP-IDF component CMakeLists.txt)
 *   idf_component_register(
 *       SRCS "main.c"
 *            "../../src/pid.c" "../../src/pid_filter.c"
 *            "../../src/pid_shaper.c" "../../src/pid_diag.c"
 *            "../../platform/esp32/pid_esp32.c"
 *       INCLUDE_DIRS "." "../../include" "../../platform/esp32" "config"
 *       REQUIRES esp_timer)
 *
 * STATUS: this file is written against the ESP-IDF API but has NOT been
 * flashed - no Xtensa toolchain exists in the workspace where it was
 * developed. The platform layer's arithmetic is covered by
 * tests/test_esp32_host.c. Verify on your board before trusting it with
 * anything that can move.
 */

#include <stdio.h>

#include "pidx/pid.h"
#include "pid_esp32.h"

/* ---------------------------------------------------------------------- */
/* Application configuration                                               */
/* ---------------------------------------------------------------------- */

/** 200 Hz. Fast enough for a thermal or a modest motion loop, slow enough
 *  that a polled FreeRTOS task can hold the schedule comfortably. */
#define LOOP_PERIOD_US      5000U

#define SETPOINT_C          60.0f
#define PWM_MIN             0.0f
#define PWM_MAX             1.0f

/* ---------------------------------------------------------------------- */
/* Plant I/O - replace with your real drivers                              */
/* ---------------------------------------------------------------------- */

/**
 * Read the process value.
 *
 * On real hardware this is an ADC read plus a conversion. Returning NAN on a
 * failed read is deliberate and is the contract PIDX expects: the core will
 * flag PID_ERR_NAN_INPUT, hold the previous output rather than command a
 * jump, and - with safety enabled - latch a fault after fault_persist_n
 * consecutive failures.
 */
static float sensor_read(void)
{
    /* Placeholder: a plausible temperature so the example is runnable. */
    static float t = 20.0f;
    return t;
}

/** Drive the actuator. On ESP32 this is typically an LEDC duty write. */
static void actuator_write(float duty)
{
    (void)duty;
}

/* ---------------------------------------------------------------------- */
/* Control task                                                            */
/* ---------------------------------------------------------------------- */

static PID_Handle g_pid;
static PIDe_Rate  g_rate;
static PIDe_Load  g_load;

static void control_task(void *arg)
{
    (void)arg;

    if (PIDe_RateInit(&g_rate, LOOP_PERIOD_US) != PID_OK) {
        printf("PIDX: rate init failed\n");
        return;
    }
    PIDe_LoadInit(&g_load);

    for (;;) {
        if (PIDe_RateElapsed(&g_rate)) {
            PIDe_LoadEnter(&g_load);

            {
                /* The measured interval, not the nominal one. When a period
                 * slips - a flash write, a Wi-Fi burst - the integrator
                 * accumulates over the time that actually passed and the
                 * derivative divides by it. Feeding a nominal dt through a
                 * jittery loop is the most common silent tuning error on this
                 * chip. */
                const PID_Float dt = PIDe_RateDt(&g_rate);
                const PID_Float y  = (PID_Float)sensor_read();
                const PID_Float u  = PID_UpdateDt(&g_pid, y, dt);

                actuator_write((float)u);
            }

            PIDe_LoadExit(&g_load);

            /* Report once a second: loop duty, worst case, and any overruns.
             * Cheap, and it turns "the loop feels sluggish" into a number. */
            if ((g_rate.iterations % (1000000U / LOOP_PERIOD_US)) == 0U) {
                printf("PIDX: duty %.1f%%  worst %.1f%%  overruns %u  "
                       "u=%.3f\n",
                       (double)(PIDe_LoadFraction(&g_load, LOOP_PERIOD_US)
                                * 100.0f),
                       (double)(PIDe_LoadWorstFraction(&g_load,
                                                       LOOP_PERIOD_US)
                                * 100.0f),
                       (unsigned)g_rate.overruns,
                       (double)PID_GetOutput(&g_pid));
            }
        }

        /* Yield so lower-priority work runs. One tick is the smallest delay
         * FreeRTOS can honour; the schedule itself is kept by PIDe_Rate
         * against the microsecond timebase, so this granularity costs
         * latency, not accuracy. */
        vTaskDelay(1);
    }
}

/* ---------------------------------------------------------------------- */
/* Entry point                                                             */
/* ---------------------------------------------------------------------- */

void app_main(void)
{
    PID_Config cfg;

    /* 1. Timebase. With the esp_timer source there is nothing to configure -
     *    it is already running - but calling this is what makes
     *    PIDe_TimebaseReady() meaningful and keeps the code identical to the
     *    STM32 version. */
    if (PIDe_TimebaseInit(240000000UL) != PID_OK) {
        printf("PIDX: timebase init failed\n");
        return;
    }

    /* 2. Controller. A temperature loop: slow, dominated by dead time, and
     *    with an actuator that cannot cool - so the integrator must be
     *    protected and the derivative filtered. */
    (void)PID_ConfigDefault(&cfg);
    cfg.core.kp = 0.08f;
    cfg.core.ki = 0.002f;
    cfg.core.kd = 1.5f;
    cfg.core.sample_time = (PID_Float)LOOP_PERIOD_US * 1e-6f;

    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = PWM_MIN;
    cfg.limits.output_max = PWM_MAX;

    /* Back-calculation rather than a plain clamp: a heater spends most of a
     * warm-up saturated, and back-calculation unwinds the integrator as the
     * process value approaches instead of after it overshoots. Measured
     * recovery on the equivalent host example: 91.5 s -> 23.5 s. */
    cfg.integral.mode = PID_AW_BACK_CALCULATION;

    /* Derivative on measurement (the default) so a setpoint change does not
     * produce a derivative kick, filtered at N = 10 to keep ADC noise out of
     * the D term. */
    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.n_filter = 10.0f;

    /* A thermal sensor that reads outside this band, or jumps faster than
     * physics allows, is broken. Fail safe to the heater OFF. */
    cfg.safety.enabled = true;
    cfg.safety.meas_min = -40.0f;
    cfg.safety.meas_max = 150.0f;
    cfg.safety.meas_rate_max = 50.0f;      /* degC/s */
    cfg.safety.failsafe_output = 0.0f;
    cfg.safety.fault_persist_n = 5;
    cfg.safety.auto_recover = true;

    if (PID_Init(&g_pid, &cfg) != PID_OK) {
        printf("PIDX: PID_Init failed\n");
        return;
    }
    (void)PID_SetSetpoint(&g_pid, SETPOINT_C);

    /* 3. Pin the task. Core 1 keeps it away from the Wi-Fi/BT stack on core 0.
     *    4096 bytes of stack is a sensible floor for a loop that uses floating
     *    point and printf. */
    if (PIDe_TaskCreate(control_task, "pidx_ctrl", 4096, NULL, 5,
                        PIDX_ESP32_CONTROL_CORE, NULL) != PID_OK) {
        printf("PIDX: control task creation failed\n");
    }
}
