/* PHASE 15 - STM32 platform layer, host syntax + logic test.
 *
 * SCOPE, HONESTLY STATED
 *   No ARM toolchain and no hardware exist in this workspace, so this test
 *   cannot prove that the layer works on silicon. What it does prove:
 *     - the layer compiles clean under the full PIDX warning gate;
 *     - the timebase register sequence is the one intended (PSC/ARR/EGR/CR1);
 *     - the 16-bit wrap arithmetic in PIDs_DeltaUs()/PIDs_NowUs() is correct
 *       across a wrap, which is the single easiest thing to get wrong;
 *     - the rate driver does not drift and counts overruns without bursting;
 *     - the ISR monitor separates jitter from execution time;
 *     - the critical section saves and restores rather than force-enabling.
 *   What it cannot prove: real bus timing, shadow-register behaviour, actual
 *   DWT availability on a given part, or any cycle count.
 *
 * The counter is driven by the test through the callback timebase, so time can
 * be advanced deterministically - a wrap that takes 65 ms on hardware happens
 * instantly here.
 */
#include <stdio.h>
#include <math.h>

#include "pid_stm32.h"
#include "stm32_stub.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

/* ---- virtual microsecond counter driving the callback timebase ---------- */
static uint32_t vclock = 0U;
static uint32_t vmask  = 0xFFFFU;

static uint32_t vclock_read(void) { return vclock & vmask; }
static void     vadvance(uint32_t us) { vclock = (vclock + us) & vmask; }

/* Install the callback timebase and make the virtual counter wrap at exactly
 * the width the library was told about. Letting the two disagree was the first
 * bug this test found in itself. */
static PID_StatusCode vinit(uint32_t mask, uint32_t start)
{
    vmask  = mask;
    vclock = start & mask;
    return PIDs_TimebaseInitCallback(vclock_read, mask);
}

int main(void)
{
    /* ---- 1. TIM timebase bring-up sequence ------------------------------ */
    {
        CK(PIDs_TimebaseReady() == false, "no timebase before init");
        CK(PIDs_CounterMask() == 0U, "counter mask is 0 before init");

        CK(PIDs_TimebaseInitTim(NULL, 84000000UL) == PID_ERR_NULL,
           "TIM init rejects NULL");
        /* 7.3728 MHz cannot be divided down to exactly 1 MHz by an integer
         * prescaler; accepting it would scale every dt by 1.09. */
        CK(PIDs_TimebaseInitTim(TIM2, 7372800UL) == PID_ERR_INVALID_PARAM,
           "TIM init rejects a clock that is not a multiple of 1 MHz");
        CK(PIDs_TimebaseInitTim(TIM2, 500000UL) == PID_ERR_INVALID_PARAM,
           "TIM init rejects a clock below 1 MHz");

        CK(PIDs_TimebaseInitTim(TIM2, 84000000UL) == PID_OK, "TIM init at 84 MHz");
        CK(TIM2->PSC == 83UL, "prescaler gives exactly 1 MHz (84 MHz / 84)");
        CK((TIM2->CR1 & 1UL) == 1UL, "counter enabled (CEN)");
        CK(TIM2->DIER == 0UL, "no interrupt or DMA enabled - read-only use");
        CK(TIM2->SR == 0UL, "update flag cleared after the UG event");
        CK(TIM2->ARR == 0xFFFFFFFFUL, "32-bit width detected on a 32-bit ARR");
        CK(PIDs_TimebaseReady() == true, "timebase reports ready");
        CK(PIDs_CounterMask() == 0xFFFFFFFFUL, "mask matches detected width");

        TIM2->CNT = 12345UL;
        CK(PIDs_NowUs32() == 12345UL, "NowUs32 reads the timer counter");
    }

    /* ---- 2. callback timebase, argument checks -------------------------- */
    {
        CK(PIDs_TimebaseInitCallback(NULL, 0xFFFFU) == PID_ERR_NULL,
           "callback init rejects NULL");
        CK(PIDs_TimebaseInitCallback(vclock_read, 0x0003FFFEUL)
               == PID_ERR_INVALID_PARAM,
           "callback init rejects a mask that is not 2^n - 1");
        CK(PIDs_TimebaseInitCallback(vclock_read, 0xFFUL)
               == PID_ERR_INVALID_PARAM,
           "callback init rejects an absurdly narrow counter");
        CK(PIDs_TimebaseInitCallback(vclock_read, 0xFFFFUL) == PID_OK,
           "callback init accepts a 16-bit counter");
        CK(PIDs_CounterMask() == 0xFFFFUL, "16-bit mask installed");
    }

    /* ---- 3. wrap-safe delta on a 16-bit counter ------------------------- */
    {
        CK(PIDs_DeltaUs(1000U, 1500U) == 500U, "delta without a wrap");
        CK(PIDs_DeltaUs(65000U, 500U) == 1036U,
           "delta across a 16-bit wrap (65536 - 65000 + 500)");
        CK(PIDs_DeltaUs(0U, 65535U) == 65535U, "delta over the full range");
        CK(PIDs_DeltaUs(700U, 700U) == 0U, "zero delta");

        /* The bug this guards: plain (later - earlier) on a masked counter
         * leaves the borrow in the upper 16 bits. */
        CK((uint32_t)(500U - 65000U) != PIDs_DeltaUs(65000U, 500U),
           "naive subtraction really is wrong here (guard is doing work)");
    }

    /* ---- 4. 64-bit monotonic extension across many wraps ---------------- */
    {
        uint64_t t0, t1;
        int      monotonic = 1;
        uint64_t prev;

        (void)vinit(0xFFFFUL, 0U);
        t0 = PIDs_NowUs();
        prev = t0;

        /* 300 steps of 20 ms = 6 s of virtual time over ~91 wraps of a 65.5 ms
         * counter. Each step is well under half a wrap, which is the stated
         * precondition. */
        for (int i = 0; i < 300; i++) {
            vadvance(20000U);
            uint64_t t = PIDs_NowUs();
            if (t <= prev) { monotonic = 0; }
            prev = t;
        }
        t1 = PIDs_NowUs();

        printf("  64-bit extension over 300 x 20 ms on a 16-bit counter:"
               " %llu us elapsed\n", (unsigned long long)(t1 - t0));
        CK(monotonic == 1, "extended time never goes backwards across wraps");
        CK((t1 - t0) == 6000000ULL,
           "extended time is exact: 300 x 20 ms = 6 000 000 us");
    }

    /* ---- 5. critical section saves and restores ------------------------- */
    {
        uint32_t st;

        pidx_stub_primask = 0U;
        st = PIDs_EnterCritical();
        CK(pidx_stub_primask == 1U, "interrupts masked inside the section");
        CK(st == 0U, "previous state captured");
        PIDs_ExitCritical(st);
        CK(pidx_stub_primask == 0U, "interrupts restored on exit");

        /* Nested: entering while already masked must leave the CPU masked. */
        pidx_stub_primask = 1U;
        st = PIDs_EnterCritical();
        PIDs_ExitCritical(st);
        CK(pidx_stub_primask == 1U,
           "a section taken inside a masked region does not re-enable IRQs");
        pidx_stub_primask = 0U;
    }

    /* ---- 6. rate driver: no drift, exact average ------------------------ */
    {
        PIDs_Rate r;
        const uint32_t period = 1000U;   /* 1 kHz */
        const uint32_t body   = 300U;    /* loop body cost                   */
        const uint32_t poll   = 50U;     /* main-loop polling granularity    */
        uint32_t fires = 0U;
        uint32_t t_first = 0U, t_last = 0U;
        int first = 1;

        (void)vinit(0xFFFFFFFFUL, 0U);

        CK(PIDs_RateInit(NULL, period) == PID_ERR_NULL, "RateInit rejects NULL");
        CK(PIDs_RateInit(&r, 0U) == PID_ERR_INVALID_PARAM,
           "RateInit rejects zero period");
        CK(PIDs_RateInit(&r, period) == PID_OK, "RateInit at 1 kHz");

        /* 1 s of virtual time, polled every 50 us. The loop is bounded by the
         * virtual clock itself, not by a count of polls: the body advances the
         * clock too, so counting polls would overrun the second. */
        while (vclock < 1000000U) {
            if (PIDs_RateElapsed(&r)) {
                if (first) { t_first = vclock; first = 0; }
                t_last = vclock;
                fires++;
                vadvance(body);           /* the loop body runs */
            }
            vadvance(poll);
        }

        printf("  rate driver: %u releases in ~1 s, overruns=%u,"
               " worst lateness=%u us\n",
               (unsigned)fires, (unsigned)r.overruns,
               (unsigned)r.worst_late_us);
        if (fires > 1U) {
            printf("    mean interval %.1f us (nominal %u)\n",
                   (double)(t_last - t_first) / (double)(fires - 1U),
                   (unsigned)period);
        }
        CK((fires >= 985U) && (fires <= 1000U),
           "about 1000 releases in 1 s despite a 300 us body");
        CK(r.overruns == 0U, "a 300 us body inside a 1000 us period is not late");
        CK(r.worst_late_us <= (poll + body),
           "lateness bounded by the polling granularity plus the body");
        CK(r.iterations == fires, "iteration counter matches the releases");

        /* The point of the absolute deadline: the mean interval stays at the
         * nominal period instead of period + body + poll. */
        {
            double mean = (double)(t_last - t_first) / (double)(fires - 1U);
            CK(fabs(mean - 1000.0) < 10.0,
               "mean interval stays at the nominal period - no drift");
            CK(mean < 1000.0 + (double)body,
               "and is nowhere near period + body, which is what re-basing"
               " on now would give");
        }
    }

    /* ---- 6b. drift with a polling granularity that does NOT divide the
     *          period. This is the case that exposes re-basing: with poll=37
     *          and body=311, "last_release = now" loses up to 36 us of phase
     *          per release and the loop ends up minutes-per-hour slow. ------ */
    {
        PIDs_Rate r;
        const uint32_t period = 1000U;
        const uint32_t body   = 311U;
        const uint32_t poll   = 37U;
        uint32_t fires = 0U;
        uint32_t t_first = 0U, t_last = 0U;
        int first = 1;

        (void)vinit(0xFFFFFFFFUL, 0U);
        (void)PIDs_RateInit(&r, period);

        while (vclock < 2000000U) {          /* 2 s of virtual time */
            if (PIDs_RateElapsed(&r)) {
                if (first) { t_first = vclock; first = 0; }
                t_last = vclock;
                fires++;
                vadvance(body);
            }
            vadvance(poll);
        }

        {
            double mean = (double)(t_last - t_first) / (double)(fires - 1U);
            double rate = 1e6 / mean;
            printf("  awkward polling (poll=%u, body=%u): %u releases,"
                   " mean %.2f us -> %.2f Hz\n",
                   (unsigned)poll, (unsigned)body, (unsigned)fires, mean, rate);
            /* An absolute deadline keeps the AVERAGE exact even though each
             * individual release is quantised by the polling granularity. */
            CK(fabs(mean - 1000.0) < 1.0,
               "mean period stays within 1 us of nominal under awkward polling");
            CK(fabs(rate - 1000.0) < 1.0, "achieved rate within 1 Hz of 1 kHz");
            CK((fires >= 1995U) && (fires <= 2001U),
               "about 2000 releases in 2 s - no accumulated phase loss");
            CK(r.overruns == 0U, "no spurious overruns from quantisation");
            CK(r.worst_late_us < (poll + body),
               "individual lateness bounded by poll granularity plus body");
        }
    }

    /* ---- 7. rate driver: overrun accounting, no catch-up burst ---------- */
    {
        PIDs_Rate r;
        const uint32_t period = 1000U;
        uint32_t fires = 0U;
        int      burst = 0;
        uint32_t prev_fire = 0U;
        int      first = 1;

        (void)vinit(0xFFFFFFFFUL, 0U);
        (void)PIDs_RateInit(&r, period);

        /* A body that takes 3.5 periods: every release is an overrun. */
        for (int i = 0; i < 40; i++) {
            while (!PIDs_RateElapsed(&r)) { vadvance(100U); }
            if (!first) {
                uint32_t gap = PIDs_DeltaUs(prev_fire, vclock);
                if (gap < (period / 2U)) { burst = 1; }
            }
            prev_fire = vclock;
            first = 0;
            fires++;
            vadvance(3500U);
        }

        printf("  overrun test: %u releases, overruns=%u, virtual time %u us\n",
               (unsigned)fires, (unsigned)r.overruns, (unsigned)vclock);
        CK(r.overruns >= 38U, "a 3.5x period body is reported as an overrun");
        CK(burst == 0, "no catch-up burst: releases never bunch up");
        CK(vclock >= 40U * 3500U, "no attempt to make up the lost time");
    }

    /* ---- 8. rate driver rejects a period too close to the wrap ---------- */
    {
        PIDs_Rate r;
        (void)vinit(0xFFFFUL, 0U);
        CK(PIDs_RateInit(&r, 40000U) == PID_ERR_INVALID_PARAM,
           "40 ms period rejected on a 65.5 ms counter (needs < half a wrap)");
        CK(PIDs_RateInit(&r, 10000U) == PID_OK,
           "10 ms period accepted on the same counter");
    }

    /* ---- 9. ISR monitor: jitter and execution time are separate --------- */
    {
        PIDs_IsrMonitor m;
        const uint32_t nominal = 1000U;

        (void)vinit(0xFFFFFFFFUL, 0U);

        CK(PIDs_IsrMonitorInit(NULL, nominal) == PID_ERR_NULL,
           "monitor init rejects NULL");
        CK(PIDs_IsrMonitorInit(&m, 0U) == PID_ERR_INVALID_PARAM,
           "monitor init rejects zero period");
        CK(PIDs_IsrMonitorInit(&m, nominal) == PID_OK, "monitor init");
        CK(PIDs_IsrLoadPercent(&m) == (PID_Float)0,
           "load is 0 before any entry");

        /* 100 perfectly periodic entries with a 200 us body. */
        for (int i = 0; i < 100; i++) {
            PIDs_IsrEnter(&m);
            vadvance(200U);
            PIDs_IsrExit(&m);
            vadvance(800U);
        }
        printf("  ISR monitor, clean run: exec=%u us max, jitter=%u us max,"
               " load=%.1f%%\n", (unsigned)m.max_exec_us,
               (unsigned)m.max_jitter_us, (double)PIDs_IsrLoadPercent(&m));
        CK(m.entries == 100U, "every entry counted");
        CK(m.last_exec_us == 200U, "execution time measured exactly");
        CK(m.max_exec_us == 200U, "worst execution time is the body length");
        CK(m.max_jitter_us == 0U, "a perfectly periodic ISR shows zero jitter");
        CK(fabs((double)PIDs_IsrLoadPercent(&m) - 20.0) < 0.5,
           "200 us of work per 1000 us period is a 20% load");

        /* Now delay one entry by 350 us: jitter must rise while the execution
         * time stays put. Confusing these two is the classic misdiagnosis. */
        PIDs_IsrEnter(&m);
        vadvance(200U);
        PIDs_IsrExit(&m);
        vadvance(800U + 350U);
        PIDs_IsrEnter(&m);
        vadvance(200U);
        PIDs_IsrExit(&m);

        printf("  after a 350 us late entry: jitter=%u us, exec still %u us\n",
               (unsigned)m.max_jitter_us, (unsigned)m.max_exec_us);
        CK(m.max_jitter_us == 350U, "late entry shows up as jitter, exactly");
        CK(m.max_exec_us == 200U, "and does NOT inflate the execution time");
        CK(m.last_period_us == 1350U, "measured period reflects the delay");
    }

    /* ---- 10. ISR monitor across a counter wrap -------------------------- */
    {
        PIDs_IsrMonitor m;

        (void)vinit(0xFFFFUL, 0xFFFFU - 300U);   /* about to wrap */
        (void)PIDs_IsrMonitorInit(&m, 1000U);

        PIDs_IsrEnter(&m);
        vadvance(200U);                          /* wraps mid-measurement */
        PIDs_IsrExit(&m);

        CK(m.last_exec_us == 200U,
           "execution time correct when the counter wraps inside the ISR");
    }

    /* ---- 11. DWT profiler ----------------------------------------------- */
    {
        PIDs_CycleStat s;

        pidx_stub_cyc_step = 1U;
        CK(PIDs_CycleInit() == PID_OK, "DWT cycle counter starts");
        CK((pidx_stub_coredebug_inst.DEMCR
              & CoreDebug_DEMCR_TRCENA_Msk) != 0U, "TRCENA set");
        CK((pidx_stub_dwt_inst.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U,
           "CYCCNTENA set");

        PIDs_CycleReset(&s);
        CK(s.count == 0U, "cycle stat resets");
        CK(PIDs_CycleMean(&s) == (PID_Float)0, "mean of an empty stat is 0");

        for (int i = 0; i < 10; i++) {
            PIDs_CycleStart(&s);
            PIDs_CycleStop(&s);
        }
        CK(s.count == 10U, "cycle stat counted every interval");
        CK(s.min <= s.max, "min <= max");
        printf("  DWT stub: mean %.1f cycles over %u samples"
               " (stub increments by 1 per access)\n",
               (double)PIDs_CycleMean(&s), (unsigned)s.count);

        /* 168 cycles at 168 MHz is exactly 1 us. */
        CK(fabs((double)PIDs_CyclesToUs(168U) - 1.0) < 1e-6,
           "cycles-to-us conversion uses the configured core clock");
        CK(fabs((double)PIDs_CyclesToUs(16800U) - 100.0) < 1e-4,
           "conversion scales linearly");

        /* NULL tolerance: a profiler must never fault the code it measures. */
        PIDs_CycleReset(NULL);
        PIDs_CycleStart(NULL);
        PIDs_CycleStop(NULL);
        CK(PIDs_CycleMean(NULL) == (PID_Float)0, "NULL cycle stat is inert");
        CK(PIDs_IsrLoadPercent(NULL) == (PID_Float)0, "NULL monitor is inert");
        CK(PIDs_RateElapsed(NULL) == false, "NULL rate driver is inert");
        PIDs_IsrEnter(NULL);
        PIDs_IsrExit(NULL);
        pass++;   /* reaching here means the NULL ISR calls did not crash */
    }

    /* ---- 12. DWT refuses to start when the counter is frozen ------------ */
    {
        pidx_stub_cyc_step        = 0U;      /* debug unit locked down */
        pidx_stub_dwt_inst.CYCCNT = 0U;
        CK(PIDs_CycleInit() == PID_ERR_UNSUPPORTED,
           "a frozen cycle counter is reported, not silently accepted");
        CK(PIDs_TimebaseInitDwt(168000000UL) == PID_ERR_UNSUPPORTED,
           "DWT timebase refuses to install on a frozen counter");
        pidx_stub_cyc_step = 1U;

        CK(PIDs_TimebaseInitDwt(1000UL) == PID_ERR_INVALID_PARAM,
           "DWT timebase rejects an implausible core clock");
        CK(PIDs_TimebaseInitDwt(168000000UL) == PID_OK,
           "DWT timebase installs on a running counter");
        CK(PIDs_CounterMask() == 0xFFFFFFFFUL, "DWT timebase is 32-bit");
    }

    /* ---- 13. driving a controller from the platform timebase ------------ */
    {
        PID_Handle h;
        PID_Config c;
        PIDs_Rate  r;
        PID_StatusCode last = PID_OK;
        double y = 0.0, u = 0.0;
        uint32_t fires = 0U;

        (void)vinit(0xFFFFFFFFUL, 0U);

        PID_ConfigDefault(&c);
        c.core.kp = 2.0f;
        c.core.ki = 30.0f;
        c.core.kd = 0.01f;
        c.core.sample_time = 0.001f;
        c.limits.use_output_limits = true;
        c.limits.output_min = -10.0f;
        c.limits.output_max =  10.0f;
        CK(PID_Init(&h, &c) == PID_OK, "controller init");
        PID_SetSetpoint(&h, 1.0f);
        (void)PIDs_RateInit(&r, 1000U);

        for (int i = 0; i < 400; i++) {
            while (!PIDs_RateElapsed(&r)) { vadvance(100U); }
            {
                PID_Float dt = (PID_Float)((PID_Float)r.last_dt_us * 1e-6f);
                u = (double)PID_UpdateDt(&h, (PID_Float)y, dt);
                y += ((double)dt / 0.05) * (u - y);
            }
            fires++;
            vadvance(250U);   /* controller execution time */
        }

        (void)PID_GetLastError(&h, &last);
        printf("  closed loop on the platform timebase: y=%.4f u=%.4f"
               " releases=%u overruns=%u\n", y, u, (unsigned)fires,
               (unsigned)r.overruns);
        CK(last == PID_OK, "no controller error using the platform dt");
        CK(fabs(y - 1.0) < 0.02, "plant converged on the setpoint");
        CK(r.overruns == 0U, "250 us of work in a 1 ms period is not an overrun");
    }

    printf("\n  test_stm32_host: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}
