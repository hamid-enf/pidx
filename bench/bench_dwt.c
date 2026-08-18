/**
 * @file    bench_dwt.c
 * @brief   Cycle-accurate PIDX benchmark for Cortex-M, using DWT->CYCCNT.
 *
 * Not part of the library. Not built by the host Makefile - it only compiles
 * for an ARM target.
 *
 * @section why Why this file exists as source rather than as a table
 *
 * The documentation quotes Cortex-M cycle counts as DESIGN TARGETS, not
 * measurements, because no ARM toolchain was available when PIDX was
 * written. Rather than invent numbers, the measurement harness ships instead:
 * run this on your own board and it prints the real table for your part, your
 * clock, your compiler and your flash wait states.
 *
 * Those four things matter more than people expect. The same binary can differ by
 * 2x between an M4F running from ITCM and an M7 running from external QSPI
 * flash with the cache off. A single published number would be wrong for
 * almost every reader.
 *
 * @section use How to use it
 *
 * 1. Add this file to your firmware project (it needs no build system of
 *    ours) and make sure PIDX's include/ is on the include path.
 * 2. Provide bench_dwt_putc() or redirect printf to your UART/RTT/semihost.
 * 3. Call bench_dwt_run() once, after the clock tree is configured.
 * 4. Copy the printed table into docs/17_performance.md.
 *
 * @section req Requirements and limitations
 *
 * DWT->CYCCNT exists on Cortex-M3/M4/M7/M33 but NOT on Cortex-M0/M0+, which
 * have no DWT cycle counter. On M0 use a hardware timer at the core clock
 * instead and replace bench_cycles_now(). This is flagged at compile time
 * rather than silently producing zeros.
 *
 * On some parts the debug block must be unlocked before CYCCNT will count,
 * and on others CYCCNT is held in reset until a debugger attaches. The code
 * below performs the unlock sequence and then VERIFIES the counter actually
 * advances, failing loudly instead of reporting a suspiciously fast zero.
 */

#if !defined(__ARM_ARCH)
#error "bench_dwt.c targets Cortex-M. Use bench_host.c on a PC."
#endif

#if defined(__ARM_ARCH_6M__)
#error "Cortex-M0/M0+ has no DWT cycle counter - use a hardware timer."
#endif

#include <stdint.h>
#include <stdio.h>

#include "pidx/pid.h"
#include "pidx/pid_fixed.h"

/* ------------------------------------------------------------------ */
/* CoreDebug / DWT registers                                            */
/* ------------------------------------------------------------------ */

/*
 * Declared directly rather than via CMSIS so this file has no dependency on
 * a device header. The addresses are architectural and identical on every
 * Cortex-M that has a DWT.
 */
#define BENCH_DEMCR        (*(volatile uint32_t *)0xE000EDFCUL)
#define BENCH_DWT_CTRL     (*(volatile uint32_t *)0xE0001000UL)
#define BENCH_DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004UL)
#define BENCH_DWT_LAR      (*(volatile uint32_t *)0xE0001FB0UL)

#define BENCH_DEMCR_TRCENA     (1UL << 24)
#define BENCH_DWT_CTRL_CYCCNT  (1UL << 0)
#define BENCH_LAR_UNLOCK       0xC5ACCE55UL

/** @return true if the cycle counter is running. */
static int bench_cycles_init(void)
{
    volatile uint32_t a, b;

    BENCH_DEMCR |= BENCH_DEMCR_TRCENA;
    /* Cortex-M7 and some M33 parts require the lock access register to be
     * unlocked first. Writing it is harmless where it does not exist. */
    BENCH_DWT_LAR = BENCH_LAR_UNLOCK;
    BENCH_DWT_CYCCNT = 0U;
    BENCH_DWT_CTRL  |= BENCH_DWT_CTRL_CYCCNT;

    /* Verify rather than assume: if the counter is stuck the whole table
     * would read 0 cycles, which looks like a spectacular result. */
    a = BENCH_DWT_CYCCNT;
    __asm volatile ("nop\n nop\n nop\n nop");
    b = BENCH_DWT_CYCCNT;
    return (b != a);
}

static inline uint32_t bench_cycles_now(void)
{
    return BENCH_DWT_CYCCNT;
}

/* ------------------------------------------------------------------ */
/* Harness                                                              */
/* ------------------------------------------------------------------ */

#ifndef BENCH_DWT_ITERS
#define BENCH_DWT_ITERS   256U
#endif
#ifndef BENCH_DWT_REPEATS
#define BENCH_DWT_REPEATS 5U
#endif

/* Volatile sink: without it the optimiser deletes the call being measured
 * and the harness reports 2 cycles for a full PID update. */
static volatile float g_sink;

/**
 * Measured empty-loop cost, subtracted from every case. On a target with
 * flash wait states this is not negligible.
 */
static uint32_t g_overhead;

static float bench_meas(uint32_t i)
{
    const uint32_t m = i & 255U;
    return (float)((m < 128U) ? m : (256U - m)) * (1.0f / 128.0f);
}

/** Minimum cycles per call over BENCH_DWT_REPEATS runs. */
#define BENCH_DWT(dest, setup, body)                                       \
    do {                                                                   \
        uint32_t best_ = 0xFFFFFFFFUL;                                     \
        uint32_t rep_;                                                     \
        for (rep_ = 0U; rep_ < BENCH_DWT_REPEATS; ++rep_) {                \
            uint32_t it_, t0_, t1_, d_;                                    \
            setup;                                                         \
            t0_ = bench_cycles_now();                                      \
            for (it_ = 0U; it_ < BENCH_DWT_ITERS; ++it_) {                 \
                body;                                                      \
            }                                                              \
            t1_ = bench_cycles_now();                                      \
            d_ = t1_ - t0_;   /* wraps correctly in uint32 arithmetic */   \
            if (d_ < best_) { best_ = d_; }                                \
        }                                                                  \
        (dest) = best_ / BENCH_DWT_ITERS;                                  \
    } while (0)

static void bench_row(const char *name, uint32_t cycles, uint32_t target)
{
    const char *verdict;
    /* Subtract loop overhead; guard against underflow on a very cheap path. */
    const uint32_t net = (cycles > g_overhead) ? (cycles - g_overhead) : 0U;

    if (target == 0U)      { verdict = "";        }
    else if (net <= target) { verdict = "PASS";   }
    else                    { verdict = "OVER";   }

    printf("  %-30s %8lu %8lu  %s\n", name,
           (unsigned long)net, (unsigned long)target, verdict);
}

/* ------------------------------------------------------------------ */

/**
 * Run the whole table. Call once, after clocks are configured.
 * @param core_hz Core clock in Hz, used only to convert cycles to ns.
 */
void bench_dwt_run(uint32_t core_hz);

void bench_dwt_run(uint32_t core_hz)
{
    uint32_t c;

    if (!bench_cycles_init()) {
        printf("DWT CYCCNT is not counting on this part.\n"
               "Some devices hold it in reset until a debugger attaches,\n"
               "and Cortex-M0/M0+ has no DWT at all. No numbers printed -\n"
               "a zero here would be a fabricated result.\n");
        return;
    }

    printf("=== PIDX benchmark (Cortex-M, DWT CYCCNT) ===\n\n");
    printf("core clock: %lu Hz, iterations: %u, runs: %u, statistic: min\n\n",
           (unsigned long)core_hz, (unsigned)BENCH_DWT_ITERS,
           (unsigned)BENCH_DWT_REPEATS);

    /* Loop overhead first; everything else is reported net of it. */
    BENCH_DWT(g_overhead, { }, {
        g_sink += bench_meas(it_);
    });
    printf("loop overhead: %lu cycles/iter (subtracted below)\n\n",
           (unsigned long)g_overhead);

    printf("  %-30s %8s %8s  %s\n", "path", "cycles", "target", "verdict");
    printf("  %-30s %8s %8s  %s\n", "------------------------------",
           "------", "------", "-------");

    {
        PID_Handle h;
        PID_Config cfg;

        (void)PID_ConfigDefault(&cfg);
        cfg.core.kp = 2.0f;
        cfg.core.ki = 1.0f;
        cfg.core.kd = 0.1f;
        cfg.core.sample_time = 0.001f;
        cfg.limits.use_output_limits = true;
        cfg.limits.output_min = -10.0f;
        cfg.limits.output_max =  10.0f;
        if (PID_Init(&h, &cfg) != PID_OK) {
            printf("  PID_Init failed - no results\n");
            return;
        }
        (void)PID_SetSetpoint(&h, 1.0f);

        /* Targets from docs section 10.2. They are DESIGN TARGETS; this
         * harness is what turns them into measurements. */
        BENCH_DWT(c, { (void)PID_Reset(&h); }, {
            g_sink += PID_UpdateFast(&h, bench_meas(it_));
        });
        bench_row("PID_UpdateFast", c, 100U);

        BENCH_DWT(c, { (void)PID_Reset(&h); }, {
            g_sink += PID_Update(&h, bench_meas(it_));
        });
        bench_row("PID_Update (typical)", c, 160U);

        BENCH_DWT(c, { (void)PID_Reset(&h); }, {
            g_sink += PID_UpdateDt(&h, bench_meas(it_), 0.001f);
        });
        bench_row("PID_UpdateDt (dt constant)", c, 168U);

        BENCH_DWT(c, { (void)PID_Reset(&h); }, {
            const float dt = 0.001f + ((float)(it_ & 15U) * 1e-6f);
            g_sink += PID_UpdateDt(&h, bench_meas(it_), dt);
        });
        bench_row("PID_UpdateDt (dt varying)", c, 250U);
    }

#if PIDX_ENABLE_FIXED_POINT
    {
        PIDq_Handle qh;
        PIDq_Config qcfg;

        (void)PIDq_ConfigDefault(&qcfg);
        qcfg.kp_q16 = 2 << 16;
        qcfg.ki_q16 = 1 << 16;
        qcfg.kd_q16 = 6553;
        qcfg.dt_us  = 1000U;
        if (PIDq_Init(&qh, &qcfg) == PID_OK) {
            (void)PIDq_SetSetpoint(&qh, 16384);
            BENCH_DWT(c, { }, {
                g_sink += (float)PIDq_Update(&qh, (int16_t)(it_ & 0x3FFFU));
            });
            /* No target: on an FPU-less M0 this is the ONLY viable path, and
             * on an M4F it is usually slower than float. Measure, do not
             * assume. */
            bench_row("PIDq_Update (Q15)", c, 0U);
        }
    }
#endif

    if (core_hz > 0U) {
        printf("\n  1 cycle = %.2f ns at %lu Hz\n",
               1e9 / (double)core_hz, (unsigned long)core_hz);
    }
    printf("\n  Paste this table into docs/17_performance.md, replacing the\n"
           "  design-target column with your measured values, and state the\n"
           "  part number, clock, compiler version and -O level.\n");
    printf("\n(sink %.3f - printed so the work cannot be optimised away)\n",
           (double)g_sink);
}
