/* PHASE 15 - POSIX platform layer.
 *
 * Checks the timebase itself (monotonicity, resolution), the absolute-deadline
 * scheduler (rate accuracy, overrun accounting, no burst catch-up) and the
 * benchmarking timer. Timing assertions are deliberately loose: this runs on a
 * general-purpose kernel with no realtime priority, so the test asserts
 * "clearly better than the naive loop" rather than a hard microsecond bound.
 */
#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pid_posix.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

int main(void)
{
    /* ---- 1. timebase ---------------------------------------------------- */
    {
        uint64_t a, b;
        double   s0, s1;
        int      nondecreasing = 1;
        int      moved = 0;

        a = PIDp_NowUs();
        CK(a != 0U, "PIDp_NowUs returns a usable timestamp");

        for (int i = 0; i < 2000; i++) {
            b = PIDp_NowUs();
            if (b < a) { nondecreasing = 0; }
            if (b > a) { moved = 1; }
            a = b;
        }
        CK(nondecreasing == 1, "monotonic: time never goes backwards");
        CK(moved == 1, "clock actually advances (not a stuck stub)");

        s0 = PIDp_Now();
        PIDp_SleepUs(50000U);
        s1 = PIDp_Now();
        printf("  sleep(50 ms) measured as %.3f ms\n", (s1 - s0) * 1000.0);
        CK((s1 - s0) >= 0.049, "sleep is not short");
        CK((s1 - s0) <  0.200, "sleep is not wildly long");
    }

    /* ---- 2. resolution -------------------------------------------------- */
    {
        uint64_t t0 = PIDp_NowUs();
        uint64_t t1 = t0;
        uint32_t spins = 0U;

        while ((t1 == t0) && (spins < 10000000U)) { t1 = PIDp_NowUs(); spins++; }
        printf("  smallest observed tick: %llu us\n",
               (unsigned long long)(t1 - t0));
        CK((t1 > t0) && ((t1 - t0) <= 100U),
           "timebase resolution is at least 100 us");
    }

    /* ---- 3. fixed-rate loop: argument checking -------------------------- */
    {
        PIDp_Loop lp;
        CK(PIDp_LoopInit(NULL, 1000U) == PID_ERR_NULL, "LoopInit rejects NULL");
        CK(PIDp_LoopInit(&lp, 0U) == PID_ERR_INVALID_PARAM,
           "LoopInit rejects zero period");
        CK(PIDp_LoopInit(&lp, 1000U) == PID_OK, "LoopInit accepts 1 kHz");
        CK(PIDp_LoopMeanRate(&lp) == 0.0, "mean rate is 0 before the first wait");
    }

    /* ---- 4. rate accuracy, and the drift the scheduler exists to kill ---- */
    {
        const uint64_t period_us = 2000U;   /* 500 Hz            */
        const int      n         = 250;     /* 0.5 s of running  */
        const uint64_t body_us   = 700U;    /* simulated work    */

        PIDp_Loop lp;
        uint64_t  t0, t1;
        double    dt_sum = 0.0, dt_min = 1e9, dt_max = 0.0;
        double    rate_sched, rate_naive;

        (void)PIDp_LoopInit(&lp, period_us);
        t0 = PIDp_NowUs();
        for (int i = 0; i < n; i++) {
            double dt = PIDp_LoopWait(&lp);
            if (i > 0) {
                dt_sum += dt;
                if (dt < dt_min) { dt_min = dt; }
                if (dt > dt_max) { dt_max = dt; }
            }
            PIDp_SleepUs(body_us);
        }
        t1 = PIDp_NowUs();
        rate_sched = (double)(n - 1) * 1e6 / (double)(t1 - t0);

        printf("  deadline loop: %d iters, target %.1f Hz, achieved %.2f Hz\n",
               n, 1e6 / (double)period_us, rate_sched);
        printf("    dt mean %.0f us  min %.0f us  max %.0f us  overruns %u"
               "  worst lateness %llu us\n",
               (dt_sum / (double)(n - 1)) * 1e6, dt_min * 1e6, dt_max * 1e6,
               (unsigned)lp.overruns, (unsigned long long)lp.worst_lateness);

        CK(fabs(rate_sched - 500.0) < 25.0,
           "achieved rate within 5% of 500 Hz");
        CK(fabs(PIDp_LoopMeanRate(&lp) - rate_sched) < 15.0,
           "PIDp_LoopMeanRate agrees with externally measured rate");
        CK(lp.iterations == (uint32_t)n, "every iteration counted once");

        /* Same workload with the naive sleep(period - nothing) loop. Every
         * iteration pays body time + wake latency on top of the period, so the
         * error accumulates instead of cancelling. */
        t0 = PIDp_NowUs();
        for (int i = 0; i < n; i++) {
            PIDp_SleepUs(body_us);
            PIDp_SleepUs(period_us);
        }
        t1 = PIDp_NowUs();
        rate_naive = (double)(n - 1) * 1e6 / (double)(t1 - t0);
        printf("  naive sleep(period) loop achieved %.2f Hz  (%.1f%% slow)\n",
               rate_naive, (1.0 - rate_naive / 500.0) * 100.0);

        CK(rate_sched > rate_naive,
           "absolute-deadline schedule beats sleep(period)");
        CK(fabs(rate_sched - 500.0) < fabs(rate_naive - 500.0),
           "and is closer to the requested rate");
    }

    /* ---- 5. overrun accounting, no burst catch-up ----------------------- */
    {
        const uint64_t period_us = 1000U;
        const int      n         = 40;
        PIDp_Loop lp;
        uint64_t  t0, t1;
        int       short_dt = 0;

        (void)PIDp_LoopInit(&lp, period_us);
        t0 = PIDp_NowUs();
        for (int i = 0; i < n; i++) {
            double dt = PIDp_LoopWait(&lp);
            /* A catch-up burst would show up here as a dt far below period. */
            if ((i > 0) && (dt < 0.0005)) { short_dt++; }
            PIDp_SleepUs(3000U);            /* 3x the period: always late */
        }
        t1 = PIDp_NowUs();

        printf("  overrun test: %d iters, overruns=%u, elapsed %.1f ms\n",
               n, (unsigned)lp.overruns, (double)(t1 - t0) / 1000.0);
        CK(lp.overruns >= (uint32_t)(n - 2),
           "an overrunning body is reported on essentially every iteration");
        CK(short_dt == 0,
           "no catch-up burst: dt never collapses below half a period");
        CK((t1 - t0) >= (uint64_t)n * 3000U,
           "loop did not try to make up lost time by running faster");
    }

    /* ---- 6. driving a real controller from the loop --------------------- */
    {
        PID_Handle h;
        PID_Config c;
        PIDp_Loop  lp;
        const uint64_t period_us = 1000U;   /* 1 kHz */
        const int      n         = 600;
        double y = 0.0;                     /* first-order plant, tau = 50 ms */
        double u = 0.0;
        double worst_dt_err = 0.0;

        PID_ConfigDefault(&c);
        c.core.kp = 2.0f;
        c.core.ki = 30.0f;
        c.core.kd = 0.01f;
        c.core.sample_time = (PID_Float)((double)period_us * 1e-6);
        c.limits.use_output_limits = true;
        c.limits.output_min = -10.0f;
        c.limits.output_max =  10.0f;
        CK(PID_Init(&h, &c) == PID_OK, "controller init");
        PID_SetSetpoint(&h, 1.0f);

        (void)PIDp_LoopInit(&lp, period_us);
        for (int i = 0; i < n; i++) {
            double dt = PIDp_LoopWait(&lp);
            double err;

            /* Guard the plant model, not the controller: PID_UpdateDt is fed
             * the measured dt exactly as a real application would. */
            u = (double)PID_UpdateDt(&h, (PID_Float)y, (PID_Float)dt);
            y += (dt / 0.05) * (u - y);

            err = fabs(dt - 0.001) / 0.001;
            if ((i > 0) && (err > worst_dt_err)) { worst_dt_err = err; }
        }
        printf("  1 kHz closed loop: y=%.4f  u=%.4f  worst dt error %.1f%%"
               "  overruns %u\n", y, u, worst_dt_err * 100.0,
               (unsigned)lp.overruns);
        {
            PID_StatusCode last = PID_OK;
            (void)PID_GetLastError(&h, &last);
            CK(last == PID_OK,
               "no controller error while driven by the measured dt");
        }
        CK(fabs(y - 1.0) < 0.05, "plant converged on the setpoint");
    }

    /* ---- 7. benchmarking timer ------------------------------------------ */
    {
        PIDp_Timer t;

        PIDp_TimerReset(&t);
        CK(t.count == 0U, "timer resets to empty");
        CK(PIDp_TimerMeanUs(&t) == 0.0, "mean of an empty timer is 0");

        for (int i = 0; i < 5; i++) {
            PIDp_TimerStart(&t);
            PIDp_SleepUs(2000U);
            PIDp_TimerStop(&t);
        }
        printf("  timer over 5 x 2 ms sleeps: mean %.0f us  min %llu  max %llu\n",
               PIDp_TimerMeanUs(&t), (unsigned long long)t.min_us,
               (unsigned long long)t.max_us);
        CK(t.count == 5U, "timer counted every interval");
        CK(PIDp_TimerMeanUs(&t) >= 1900.0, "mean is at least the sleep time");
        CK(PIDp_TimerMeanUs(&t) <  20000.0, "mean is not absurd");
        CK(t.min_us <= t.max_us, "min <= max");

        PIDp_TimerReset(&t);
        CK(t.count == 0U, "second reset clears the accumulator");

        /* NULL tolerance: the timer must not fault a benchmark harness. */
        PIDp_TimerReset(NULL);
        PIDp_TimerStart(NULL);
        PIDp_TimerStop(NULL);
        CK(PIDp_TimerMeanUs(NULL) == 0.0, "NULL timer is inert");
        CK(PIDp_LoopWait(NULL) == 0.0, "NULL loop wait is inert");
        CK(PIDp_LoopMeanRate(NULL) == 0.0, "NULL loop rate is inert");
    }

    printf("\n  test_posix: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}
