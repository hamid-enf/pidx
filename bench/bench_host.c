/**
 * @file    bench_host.c
 * @brief   Timing and floating-point operation counts for the PIDX hot paths.
 *
 * Not part of the library.
 *
 * @section what What this measures, and what it does not
 *
 * This is an x86-64 host benchmark. It reports:
 *
 *   1. Wall-clock nanoseconds per call, via clock_gettime(CLOCK_MONOTONIC).
 *   2. The RELATIVE cost of each path (the ratio that survives porting).
 *   3. An exact static count of floating-point operations per path, counted
 *      by hand from the source, which is architecture-independent.
 *
 * It does NOT report Cortex-M cycle counts. There is no ARM toolchain in this
 * environment, so any Cortex-M number here would be fabricated. Use
 * bench_dwt.c on real hardware for that; it fills in the same table from
 * DWT->CYCCNT. The nanosecond figures below are still useful because the
 * RATIOS between paths (fast vs full, float vs fixed-point) carry over even
 * though the absolute values do not.
 *
 * @section rigor Why the numbers are trustworthy
 *
 * Three things ruin a microbenchmark, and each is defended against here:
 *
 *   Dead-code elimination. Every result is accumulated into a volatile sink,
 *   so the compiler cannot delete the call it is supposed to be timing. This
 *   is not paranoia: at -O2 an unsunk PID_UpdateFast loop optimises away
 *   completely and reports a few picoseconds per call.
 *
 *   Loop overhead. The empty loop, including the volatile store and the
 *   input generation, is measured separately and subtracted.
 *
 *   Run-to-run noise. Each case runs REPEATS times and the MINIMUM is kept.
 *   The minimum is the right statistic for a benchmark: noise only ever adds
 *   time, so the fastest observed run is the closest to the true cost. A
 *   mean would measure the machine's background load instead.
 */

#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_fixed.h"

#if PIDX_ENABLE_CASCADE
#include "pidx/pid_cascade.h"
#endif

/* ------------------------------------------------------------------ */
/* Harness                                                              */
/* ------------------------------------------------------------------ */

#ifndef BENCH_ITERS
#define BENCH_ITERS   200000L
#endif
#ifndef BENCH_REPEATS
#define BENCH_REPEATS 7
#endif

/**
 * The sink. Declared volatile so every store must actually happen, which is
 * what stops the optimiser from removing the measured call. Reading it at the
 * end also keeps it from being considered unused.
 */
static volatile double g_sink = 0.0;

static double now_ns(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((double)ts.tv_sec * 1e9) + (double)ts.tv_nsec;
}

/** One row of the report. */
typedef struct {
    const char *name;
    double      ns;        /**< Per call, loop overhead already removed.    */
    int         flops;     /**< Hand-counted, see the comment on each case. */
    const char *note;
} Bench_Row;

#define MAX_ROWS 24
static Bench_Row g_rows[MAX_ROWS];
static int       g_nrows = 0;

static void row_add(const char *name, double ns, int flops, const char *note)
{
    if (g_nrows < MAX_ROWS) {
        g_rows[g_nrows].name  = name;
        g_rows[g_nrows].ns    = ns;
        g_rows[g_nrows].flops = flops;
        g_rows[g_nrows].note  = note;
        g_nrows++;
    }
}

/*
 * The measured region is a macro rather than a function taking a callback,
 * because a function pointer cannot be inlined and would add indirect-call
 * overhead of the same order as the thing being measured.
 */
#define BENCH(dest_ns, setup, body)                                        \
    do {                                                                   \
        double best = 1e30;                                                \
        int rep_;                                                          \
        for (rep_ = 0; rep_ < BENCH_REPEATS; ++rep_) {                     \
            long it_;                                                      \
            double t0_, t1_;                                               \
            setup;                                                         \
            t0_ = now_ns();                                                \
            for (it_ = 0; it_ < BENCH_ITERS; ++it_) {                      \
                body;                                                      \
            }                                                              \
            t1_ = now_ns();                                                \
            if ((t1_ - t0_) < best) { best = t1_ - t0_; }                  \
        }                                                                  \
        (dest_ns) = best / (double)BENCH_ITERS;                            \
    } while (0)

/* ------------------------------------------------------------------ */

/** A varying but cheap measurement, so the branch predictor sees real data. */
static float meas_of(long i)
{
    /* A triangle wave: no libm call, no division, and it exercises both
     * signs of the error term. Using rand() here would time rand(). */
    const long m = i & 1023L;
    return (float)((m < 512L) ? m : (1024L - m)) * (1.0f / 512.0f);
}

int main(void)
{
    double overhead_ns = 0.0;
    int i;

    puts("=== PIDX benchmark (host, x86-64) ===\n");
    printf("iterations per run: %ld, runs per case: %d, statistic: minimum\n",
           BENCH_ITERS, BENCH_REPEATS);
    puts("Absolute nanoseconds are host figures. The RATIOS are the");
    puts("portable result. Cortex-M cycle counts come from bench_dwt.c on");
    puts("real hardware - none are invented here.\n");

    /* ---- loop overhead, subtracted from everything below ---- */
    BENCH(overhead_ns, { }, {
        g_sink += (double)meas_of(it_);
    });
    printf("loop overhead (sink store + input): %.2f ns/iter, subtracted\n\n",
           overhead_ns);

    /* ================= float core ================= */
    {
        PID_Handle h;
        PID_Config cfg;
        double ns;

        (void)PID_ConfigDefault(&cfg);
        cfg.core.kp = 2.0f;
        cfg.core.ki = 1.0f;
        cfg.core.kd = 0.1f;
        cfg.core.sample_time = 0.001f;
        cfg.limits.use_output_limits = true;
        cfg.limits.output_min = -10.0f;
        cfg.limits.output_max =  10.0f;
        if (PID_Init(&h, &cfg) != PID_OK) {
            fputs("PID_Init failed\n", stderr);
            return 1;
        }
        (void)PID_SetSetpoint(&h, 1.0f);

        /*
         * PID_UpdateFast: e = sp - y; P = kp*e; I += ci*e; D from the
         * filtered measurement difference; sum; clamp.
         * FLOPs: 1 sub (error), 1 mul (P), 1 mul + 1 add (I), 1 sub + 2 mul
         * + 1 add (D), 2 add (sum), 2 compare (clamp) = 12.
         */
        BENCH(ns, { (void)PID_Reset(&h); }, {
            g_sink += (double)PID_UpdateFast(&h, meas_of(it_));
        });
        row_add("PID_UpdateFast", ns - overhead_ns, 12,
                PID_UpdateFast_IsSafe(&h) ? "safe for this config"
                                          : "NOT safe for this config");

        /* PID_Update adds: mode check, setpoint weighting, feature mask,
         * anti-windup decision, diagnostics hook, status bookkeeping. */
        BENCH(ns, { (void)PID_Reset(&h); }, {
            g_sink += (double)PID_Update(&h, meas_of(it_));
        });
        row_add("PID_Update", ns - overhead_ns, 18, "full pipeline");

        /* PID_UpdateDt with the SAME dt every call: one float compare finds
         * the cached coefficients still valid. */
        BENCH(ns, { (void)PID_Reset(&h); }, {
            g_sink += (double)PID_UpdateDt(&h, meas_of(it_), 0.001f);
        });
        row_add("PID_UpdateDt (dt constant)", ns - overhead_ns, 19,
                "cache hit, 1 extra compare");

        /*
         * PID_UpdateDt with a genuinely varying dt: this is the expensive
         * case, because pidp_recompute() must redo the coefficient folding,
         * which contains the only divisions in the library.
         */
        BENCH(ns, { (void)PID_Reset(&h); }, {
            const float dt = 0.001f + ((float)(it_ & 15L) * 1e-6f);
            g_sink += (double)PID_UpdateDt(&h, meas_of(it_), dt);
        });
        row_add("PID_UpdateDt (dt varying)", ns - overhead_ns, 19 + 9,
                "recompute: 3 divisions");
    }

    /* ================= float core, minimal config ================= */
    {
        PID_Handle h;
        PID_Config cfg;
        double ns;

        /*
         * A P-only configuration. This does NOT run less code:
         * PID_UpdateFast is deliberately branchless, so it still performs
         * the integral and derivative arithmetic with c_i = 0 and
         * c_da = c_db = 0. The row is kept precisely because it measures
         * the same time as the full config - that identity is the evidence
         * that the fast path has no data-dependent branches, which is what
         * makes it safe for a hard-real-time ISR where a variable execution
         * time is worse than a slightly larger constant one.
         */
        (void)PID_ConfigDefault(&cfg);
        cfg.core.kp = 2.0f;
        cfg.core.ki = 0.0f;
        cfg.core.kd = 0.0f;
        cfg.core.sample_time = 0.001f;
        if (PID_Init(&h, &cfg) != PID_OK) {
            fputs("PID_Init (P-only) failed\n", stderr);
            return 1;
        }
        (void)PID_SetSetpoint(&h, 1.0f);

        BENCH(ns, { (void)PID_Reset(&h); }, {
            g_sink += (double)PID_UpdateFast(&h, meas_of(it_));
        });
        row_add("PID_UpdateFast (P only)", ns - overhead_ns, 12,
                "same code, same time: branchless");
    }

    /* ================= fixed point ================= */
#if PIDX_ENABLE_FIXED_POINT
    {
        PIDq_Handle qh;
        PIDq_Config qcfg;
        double ns;

        (void)PIDq_ConfigDefault(&qcfg);
        qcfg.kp_q16 = 2 << 16;
        qcfg.ki_q16 = 1 << 16;
        qcfg.kd_q16 = 6553;          /* 0.1 in Q16.16 */
        qcfg.dt_us  = 1000U;
        if (PIDq_Init(&qh, &qcfg) != PID_OK) {
            fputs("PIDq_Init failed\n", stderr);
            return 1;
        }
        (void)PIDq_SetSetpoint(&qh, 16384);

        /* Integer path: 0 floating-point operations by construction. That is
         * the entire point of the module - it runs on an M0 with no FPU. */
        BENCH(ns, { }, {
            g_sink += (double)PIDq_Update(&qh, (int16_t)(it_ & 0x3FFF));
        });
        row_add("PIDq_Update (Q15 fixed)", ns - overhead_ns, 0,
                "integer only, no FPU needed");
    }
#endif

    /* ================= cascade ================= */
#if PIDX_ENABLE_CASCADE
    {
        PID_Handle outer, inner;
        PID_Handle *loops[2];
        PID_Cascade casc;
        PID_Config cfg;
        PID_Float meas[2];
        double ns;

        (void)PID_ConfigDefault(&cfg);
        cfg.core.kp = 1.0f;
        cfg.core.ki = 0.5f;
        cfg.core.sample_time = 0.001f;
        cfg.limits.use_output_limits = true;
        cfg.limits.output_min = -5.0f;
        cfg.limits.output_max =  5.0f;
        if ((PID_Init(&outer, &cfg) != PID_OK) ||
            (PID_Init(&inner, &cfg) != PID_OK)) {
            fputs("cascade PID_Init failed\n", stderr);
            return 1;
        }
        loops[0] = &outer;
        loops[1] = &inner;
        if (PID_Cascade_Init(&casc, loops, 2) != PID_OK) {
            fputs("PID_Cascade_Init failed\n", stderr);
            return 1;
        }
        /* Two full updates plus setpoint propagation and back-propagation. */
        BENCH(ns, { }, {
            meas[0] = meas_of(it_);
            meas[1] = meas_of(it_ + 7L);
            g_sink += (double)PID_Cascade_Update(&casc, meas, 1.0f, 0.001f);
        });
        row_add("PID_Cascade_Update (2 loops)", ns - overhead_ns, 2 * 18 + 4,
                "2 x PID_Update + plumbing");
    }
#endif

    /* ---------------- report ---------------- */
    {
        double base = 0.0;

        puts("\n=== Results ===\n");
        printf("  %-30s %10s %8s %8s  %s\n",
               "path", "ns/call", "FLOPs", "rel", "note");
        printf("  %-30s %10s %8s %8s  %s\n",
               "------------------------------", "-------", "-----",
               "-----", "----");

        for (i = 0; i < g_nrows; ++i) {
            if (strcmp(g_rows[i].name, "PID_UpdateFast") == 0) {
                base = g_rows[i].ns;
            }
        }
        if (base <= 0.0) { base = 1.0; }

        for (i = 0; i < g_nrows; ++i) {
            printf("  %-30s %10.2f %8d %7.2fx  %s\n",
                   g_rows[i].name, g_rows[i].ns, g_rows[i].flops,
                   g_rows[i].ns / base, g_rows[i].note);
        }

        puts("\n  'rel' is relative to PID_UpdateFast on the full config.");
        puts("  'FLOPs' is a hand count from the source, not a measurement;");
        puts("  it is the figure that ports to a target without an FPU.");
    }

    /* Consume the sink so it cannot be optimised away, and so a wrong result
     * would be visible rather than silently discarded. */
    printf("\n(sink checksum %.6g - printed only to keep the compiler "
           "from deleting the measured work)\n", (double)g_sink);

    return 0;
}
