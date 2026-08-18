/**
 * Example 09 - ISR deployment, telemetry and diagnostics
 * ======================================================
 * The bare-metal pattern: the controller runs in a timer interrupt, and a
 * background loop drains telemetry and watches loop health - without a mutex
 * and without ever blocking the ISR.
 *
 * WHAT THIS SHOWS
 *   - the ISR / background split, simulated faithfully on the host
 *   - PID_Telemetry: a lock-free SPSC ring, producer in the ISR, consumer in
 *     the background
 *   - the drop policy, and how a consumer detects loss without shared writes
 *   - PID_LoopMetrics: IAE / ISE / ITAE, saturation duty, oscillation rate -
 *     cheap enough to leave enabled in production
 *   - PID_GetStatus for a full per-cycle snapshot
 *
 * THE CONCURRENCY CONTRACT, WHICH IS NOT OPTIONAL
 *   Exactly one producer and exactly one consumer. The producer writes only
 *   `head` and the slot it owns; the consumer writes only `tail`. Both indices
 *   are 16-bit single stores. Break that - two tasks draining the same ring,
 *   say - and you have a data race that no amount of testing will find
 *   reliably.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_diag.h"
#include "ex_plant.h"
#include "ex_report.h"

#define DT        0.001            /* 1 kHz TIM interrupt */
#define N         6000             /* 6 s */
#define RING_CAP  32U              /* MUST be a power of two */

/* ------------------------------------------------------------------------ */
/* "ISR" context - on a real target these are file-scope statics touched by  */
/* the interrupt handler.                                                    */
/* ------------------------------------------------------------------------ */

static PID_Handle          g_pid;
static PID_Telemetry       g_tel;
static PID_TelemetryRecord g_store[RING_CAP];
static EX_Motor            g_motor;
static volatile uint32_t   g_isr_count;

/**
 * The control ISR. On STM32 this is TIM6_DAC_IRQHandler(), and its whole body
 * would be exactly this.
 *
 * Note what it does NOT do: no printf, no malloc, no blocking, no mutex. The
 * telemetry push is a handful of stores into a slot the ISR owns.
 */
static void control_isr(void)
{
    PID_Float y = (PID_Float)g_motor.w;
    PID_Float u = PID_Update(&g_pid, y);

    ex_motor_step(&g_motor, (double)u, DT);
    g_isr_count++;
}

int main(void)
{
    PID_Config      c;
    PID_LoopMetrics metrics;
    static double   log_w[N];
    uint32_t consumed = 0U;
    uint32_t gaps     = 0U;
    uint32_t dropped_total = 0U;
    uint16_t last_seq = 0U;
    bool     have_seq = false;
    int i;

    printf("Example 09 - TIM ISR + telemetry + diagnostics\n");
    printf("  1 kHz control ISR, %u-slot telemetry ring"
           " (usable %u), background consumer\n",
           (unsigned)RING_CAP, (unsigned)(RING_CAP - 1U));
    printf("  plant: DC motor velocity, load steps at t = 2 s and t = 4 s\n\n");

    ex_motor_init(&g_motor);

    PID_ConfigDefault(&c);
    c.core.kp = 0.08f;
    c.core.ki = 4.0f;
    c.core.kd = 0.0005f;
    c.core.sample_time = (PID_Float)DT;
    c.limits.use_output_limits = true;
    c.limits.output_min = -24.0f;
    c.limits.output_max =  24.0f;
    c.integral.mode = PID_AW_BACK_CALCULATION;
    c.integral.kt   = 20.0f;
    c.filter.tf = 0.003f;
    c.filter.n_filter = 0.0f;
    if (PID_Init(&g_pid, &c) != PID_OK) {
        printf("  init failed\n");
        return 1;
    }

    if (PID_Telemetry_Init(&g_tel, g_store, (uint16_t)RING_CAP) != PID_OK) {
        printf("  telemetry init failed (capacity must be a power of two)\n");
        return 1;
    }
    (void)PID_Telemetry_Attach(&g_pid, &g_tel);
    (void)PID_Metrics_Reset(&metrics);
    PID_SetSetpoint(&g_pid, 100.0f);

    /* ------------------------------------------------------------------ */
    /* Main loop: ISR fires every sample, background drains every 50th.    */
    /* ------------------------------------------------------------------ */
    for (i = 0; i < N; i++) {
        if (i == 2000) { g_motor.load = 0.020; }
        if (i == 4000) { g_motor.load = 0.0;   }

        control_isr();                     /* <-- interrupt context */
        log_w[i] = g_motor.w;

        /* Metrics are updated from the background here to show that they can
         * be; on a real target you would more likely call this in the ISR
         * (it is a dozen multiply-adds) or on a slow timer. */
        (void)PID_Metrics_Update(&metrics, &g_pid);

        /* Background consumer, deliberately slower than the producer: 1 kHz
         * in, 20 Hz out, with a 31-record ring. Records WILL be dropped, and
         * that is the designed behaviour - a control loop must never stall
         * waiting for a logger. */
        if ((i % 50) == 0) {
            PID_TelemetryRecord rec;

            /* Drain the drop counter on the same schedule as the records, so
             * nothing is lost between polls. */
            dropped_total += PID_Telemetry_Dropped(&g_tel);

            while (PID_Telemetry_Read(&g_tel, &rec) == PID_OK) {
                if (have_seq) {
                    uint16_t step = (uint16_t)(rec.seq - last_seq);
                    if (step != 1U) { gaps++; }
                }
                last_seq = rec.seq;
                have_seq = true;
                consumed++;
            }
        }
    }

    /* ------------------------------------------------------------------ */
    printf("  [telemetry]\n");
    {
        /*
         * PID_Telemetry_Dropped() is READ-AND-CLEAR: it returns the count
         * since the last call and resets it. Read it once and keep the value.
         *
         * Calling it twice - once to print, once to check the arithmetic - is
         * exactly the mistake the first draft of this example made, and the
         * accounting came out at 3721 of 6000 with the difference silently
         * consumed by the second call. The read-and-clear semantics are the
         * right design (they let a consumer ask "how many since I last
         * looked?" with no shared state), but they punish a careless caller.
         */
        uint16_t dropped_now = PID_Telemetry_Dropped(&g_tel);
        uint16_t still_queued = PID_Telemetry_Count(&g_tel);
        unsigned long accounted = (unsigned long)consumed
                                + (unsigned long)dropped_total
                                + (unsigned long)dropped_now
                                + (unsigned long)still_queued;

        printf("      ISR cycles        : %u\n", (unsigned)g_isr_count);
        printf("      records consumed  : %u\n", (unsigned)consumed);
        printf("      records dropped   : %lu\n",
               (unsigned long)dropped_total + (unsigned long)dropped_now);
        printf("      still in the ring : %u\n", (unsigned)still_queued);
        printf("      sequence gaps seen: %u\n", (unsigned)gaps);
        printf("      accounted for     : %lu of %u  %s\n",
               accounted, (unsigned)g_isr_count,
               (accounted == (unsigned long)g_isr_count) ? "(exact)"
                                                         : "(MISMATCH)");
    }
    printf("\n      Every cycle is accounted for: consumed + dropped + still\n"
           "      queued equals the number of ISR runs. The producer drops the\n"
           "      NEWEST record when the ring is full, which is what keeps\n"
           "      `tail` owned by the consumer alone - the alternative, having\n"
           "      the producer advance `tail` to overwrite the oldest, gives\n"
           "      that index two writers and lets a pre-empted producer drive\n"
           "      it backwards, re-delivering a record already consumed.\n");
    printf("\n      The %u gaps are how the consumer LEARNS about the drops:\n"
           "      seq jumps by more than one. No shared counter, no lock.\n",
           (unsigned)gaps);

    /* ------------------------------------------------------------------ */
    printf("\n  [loop metrics] - cheap enough to leave on in production\n");
    printf("      IAE                 %12.4f\n", (double)metrics.iae);
    printf("      ISE                 %12.4f\n", (double)metrics.ise);
    printf("      ITAE                %12.4f\n", (double)metrics.itae);
    printf("      mean |error|        %12.4f\n",
           (double)PID_Metrics_MeanAbsError(&metrics));
    printf("      worst |error|       %12.4f\n", (double)metrics.abs_error_max);
    printf("      saturation duty     %11.2f%%\n",
           (double)PID_Metrics_SaturationDuty(&metrics) * 100.0);
    printf("      oscillation rate    %12.4f  crossings/s\n",
           (double)PID_Metrics_OscillationRate(&metrics));
    printf("      actuator travel     %12.2f  V\n",
           (double)metrics.output_travel);
    printf("      samples             %12u\n", (unsigned)metrics.samples);

    printf("\n      These are the numbers to trend over weeks. A loop whose\n"
           "      IAE and oscillation rate creep up month by month is telling\n"
           "      you a valve is sticking or a bearing is going, long before\n"
           "      anyone notices the process drifting.\n");

    /* ------------------------------------------------------------------ */
    printf("\n  [per-cycle snapshot] PID_GetStatus at the final sample:\n");
    {
        PID_Status st;
        if (PID_GetStatus(&g_pid, &st) == PID_OK) {
            printf("      setpoint %.3f  measurement %.3f  error %.5f\n",
                   (double)st.setpoint_shaped, (double)st.measurement_filtered,
                   (double)st.error);
            printf("      P %+.4f   I %+.4f   D %+.4f   FF %+.4f\n",
                   (double)st.p_term, (double)st.i_term,
                   (double)st.d_term, (double)st.ff_term);
            printf("      u_unsat %+.4f -> u %+.4f    dt %.6f s\n",
                   (double)st.output_unsat, (double)st.output,
                   (double)st.dt_used);
            printf("      updates %u, saturated on %u of them (%.2f%%)\n",
                   (unsigned)st.update_count, (unsigned)st.saturation_count,
                   (100.0 * (double)st.saturation_count)
                   / (double)st.update_count);
            printf("      flags 0x%04X  %s%s%s%s\n", (unsigned)st.flags,
                   ((st.flags & PID_FLAG_SATURATED) != 0U) ? "SATURATED " : "",
                   ((st.flags & PID_FLAG_INTEGRAL_ACTIVE) != 0U)
                       ? "I_ACTIVE " : "",
                   ((st.flags & PID_FLAG_INTEGRAL_LIMITED) != 0U)
                       ? "I_LIMITED " : "",
                   ((st.flags & PID_FLAG_FAULT) != 0U) ? "FAULT " : "");
            printf("      P + I + D + FF = %+.4f, and u_unsat = %+.4f\n",
                   (double)(st.p_term + st.i_term + st.d_term + st.ff_term),
                   (double)st.output_unsat);
        }
    }

    printf("\n");
    ex_plot(log_w, N, 72, 12, "speed [rad/s], load on at 2 s, off at 4 s");

    /* ------------------------------------------------------------------ */
    printf("\n  ----------------------------------------------------------\n");
    printf("  On STM32 the ISR body is unchanged:\n\n");
    printf("    void TIM6_DAC_IRQHandler(void) {\n");
    printf("        TIM6->SR = 0;                    /* clear the flag */\n");
    printf("        float y = adc_to_speed(ADC1->DR);\n");
    printf("        float u = PID_Update(&g_pid, y);\n");
    printf("        TIM1->CCR1 = duty_from(u);\n");
    printf("    }\n\n");
    printf("  Rules that actually matter:\n");
    printf("    - g_pid must be a file-scope static, never a stack local.\n");
    printf("    - Clear the timer flag FIRST; a late clear re-enters the ISR.\n");
    printf("    - Never printf from here. Push telemetry and let a background\n");
    printf("      task format it - that is what the ring above is for.\n");
    printf("    - Exactly one consumer for that ring, in one context.\n");
    printf("    - On a core that reorders stores, define PIDX_MEMORY_BARRIER()\n");
    printf("      to __DMB() so the record is visible before the index that\n");
    printf("      publishes it.\n");
    printf("    - Use platform/stm32's PIDs_IsrMonitor to check the ISR fits:\n");
    printf("      it separates period jitter from execution time, which is\n");
    printf("      the distinction most people get wrong when a loop misbehaves.\n");

    return 0;
}
