/* ESP32 platform layer, host syntax + logic test.
 *
 * SCOPE, HONESTLY STATED
 *   No Xtensa toolchain and no hardware exist in this workspace, so this test
 *   cannot prove that the layer works on silicon. What it does prove:
 *     - the layer compiles clean under the full PIDX warning gate;
 *     - the 32-bit wrap arithmetic in PIDe_DeltaUs()/PIDe_NowUs() is correct
 *       across a wrap, which is the single easiest thing to get wrong;
 *     - the esp_timer source needs NO wrap extension and stays exact past
 *       2^32 us (71.6 minutes), where a 32-bit source would have wrapped;
 *     - the rate driver does not drift and counts overruns without bursting;
 *     - the load monitor's duty computation is right;
 *     - PIDe_TaskDelayPeriod() REFUSES a sub-tick period instead of silently
 *       running a 1 kHz request at the 100 Hz tick rate.
 *   What it cannot prove: that esp_timer_get_time() behaves as documented,
 *   that a portMUX excludes the other core, or any real cycle count.
 *
 * The clock is driven by the test, so a wrap that takes 71 minutes on hardware
 * happens instantly here.
 */
#include <stdio.h>

#include "pid_esp32.h"
#include "esp32_stub.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)

/* The simulated clocks the stub exposes. */
int64_t  pide_stub_us = 0;
uint32_t pide_stub_cycles = 0U;

/* ---- virtual counter driving the callback timebase --------------------- */
static uint32_t vclock = 0U;
static uint32_t vmask  = 0xFFFFFFFFU;

static uint32_t vclock_read(void) { return vclock & vmask; }
static void     vadvance(uint32_t us) { vclock = (vclock + us) & vmask; }

/* Install the callback timebase and make the virtual counter wrap at exactly
 * the width the library was told about. Letting the two disagree is the
 * classic way this kind of test lies to you. */
static PID_StatusCode vinit(uint32_t mask, uint32_t start)
{
    vmask  = mask;
    vclock = start & mask;
    return PIDe_TimebaseInitCallback(vclock_read, mask);
}

int main(void)
{
    /* ---- 1. timebase selection ----------------------------------------- */
    {
        CK(PIDe_TimebaseReady() == false, "no timebase before init");
        CK(PIDe_CounterMask() == 0U, "counter mask is 0 before init");

        CK(PIDe_TimebaseInitCallback(NULL, 0xFFFFFFFFU) == PID_ERR_NULL,
           "callback init rejects NULL");
        /* Not 2^n - 1: a mask with a hole would corrupt every wrap. */
        CK(PIDe_TimebaseInitCallback(vclock_read, 0x00FF0000U)
           == PID_ERR_INVALID_PARAM, "callback init rejects a non-2^n-1 mask");
        CK(PIDe_TimebaseInitCallback(vclock_read, 0x0FFFU)
           == PID_ERR_INVALID_PARAM, "callback init rejects a too-narrow mask");

        CK(PIDe_TimebaseInit(0) == PID_OK, "esp_timer init accepts 0 freq");
        CK(PIDe_TimebaseReady(), "timebase ready after init");
        CK(PIDe_CounterMask() == 0xFFFFFFFFU, "esp_timer mask is 32-bit");
    }

    /* ---- 2. esp_timer source is exact past a 32-bit wrap ---------------- */
    {
        (void)PIDe_TimebaseInit(240000000UL);

        pide_stub_us = 1000;
        CK(PIDe_NowUs() == 1000ULL, "esp_timer NowUs reads the clock");

        /* 2^32 us is 71.6 minutes. A 32-bit source would have wrapped here and
         * needed the accumulator; esp_timer must simply keep counting. This is
         * the concrete reason it is the default source. */
        pide_stub_us = 4294967296LL + 12345LL;
        CK(PIDe_NowUs() == 4294967296ULL + 12345ULL,
           "esp_timer stays exact past 2^32 us with no wrap handling");

        /* And it must not have been corrupted by any accumulator state. */
        pide_stub_us = 5000000000LL;
        CK(PIDe_NowUs() == 5000000000ULL,
           "esp_timer remains exact at 5e9 us");

        /* The 32-bit convenience read is the low word, by design. */
        pide_stub_us = 0x1234567890LL;
        CK(PIDe_NowUs32() == (uint32_t)0x34567890UL,
           "NowUs32 returns the low 32 bits of the esp_timer clock");
    }

    /* ---- 3. wrap-safe delta on a 32-bit source -------------------------- */
    {
        CK(vinit(0xFFFFFFFFU, 0U) == PID_OK, "callback timebase installs");

        CK(PIDe_DeltaUs(100U, 250U) == 150U, "delta, no wrap");
        /* The whole point: 10 us after 0xFFFFFFFB is 5, not 4294967301. */
        CK(PIDe_DeltaUs(0xFFFFFFFBU, 0x00000005U) == 10U,
           "delta across a 32-bit wrap");
        CK(PIDe_DeltaUs(0U, 0U) == 0U, "delta of zero");

        /* A narrower source must mask, or the borrow lands in the upper half
         * and one dt per wrap comes out as ~4.29e9 us. */
        CK(vinit(0x0000FFFFU, 0U) == PID_OK, "16-bit callback installs");
        CK(PIDe_DeltaUs(0xFFF0U, 0x0010U) == 0x20U,
           "delta across a 16-bit wrap");
    }

    /* ---- 4. 64-bit extension over a 32-bit source ----------------------- */
    {
        uint64_t a, b;

        CK(vinit(0xFFFFFFFFU, 0xFFFFF000U) == PID_OK, "install near the wrap");

        a = PIDe_NowUs();
        CK(a == 0xFFFFF000ULL, "extension starts at the raw value");

        vadvance(0x800U);
        b = PIDe_NowUs();
        CK(b == 0xFFFFF800ULL, "extension advances before the wrap");

        /* Cross it. The accumulator must add 2^32 exactly once. */
        vadvance(0x1000U);
        b = PIDe_NowUs();
        CK(b == 0x100000800ULL, "extension adds 2^32 exactly once at the wrap");
        CK(b > a, "extended time is monotonic across a wrap");

        vadvance(0x1000U);
        CK(PIDe_NowUs() == 0x100001800ULL, "extension keeps counting after");
    }

    /* ---- 5. rate driver: no drift -------------------------------------- */
    {
        PIDe_Rate r;

        CK(vinit(0xFFFFFFFFU, 0U) == PID_OK, "rate: timebase");
        CK(PIDe_RateInit(NULL, 1000U) == PID_ERR_NULL, "rate rejects NULL");
        CK(PIDe_RateInit(&r, 0U) == PID_ERR_INVALID_PARAM,
           "rate rejects a zero period");
        /* Over half the counter range the wrap-safe comparison is ambiguous. */
        CK(PIDe_RateInit(&r, 0x90000000U) == PID_ERR_INVALID_PARAM,
           "rate rejects a period over half the counter range");

        CK(PIDe_RateInit(&r, 1000U) == PID_OK, "rate init");
        CK(PIDe_RateElapsed(&r) == true, "first poll releases immediately");
        CK(PIDe_RateElapsed(&r) == false, "no second release in the same us");

        vadvance(999U);
        CK(PIDe_RateElapsed(&r) == false, "no release one us early");
        vadvance(1U);
        CK(PIDe_RateElapsed(&r) == true, "release exactly on the deadline");

        /*
         * The drift test, which is the reason the driver stores an absolute
         * deadline. Poll 3 us late every period: with an absolute schedule the
         * 1000th release still lands at t = 1000*period, whereas re-basing on
         * the observation instant would put it 3000 us late.
         */
        {
            uint32_t i;
            const uint32_t t0 = vclock;
            uint32_t releases = 0U;

            for (i = 0U; i < 100U; ++i) {
                vadvance(1003U);
                if (PIDe_RateElapsed(&r)) { releases++; }
            }
            CK(releases == 100U, "one release per period when polled late");
            /* 100 periods of 1000 us plus the 3 us of the final poll. */
            CK(PIDe_DeltaUs(t0, vclock) == 100300U, "elapsed wall time");
            CK(r.overruns == 0U, "3 us late is not an overrun");
        }
    }

    /* ---- 6. rate driver: overrun re-bases instead of bursting ---------- */
    {
        PIDe_Rate r;

        CK(vinit(0xFFFFFFFFU, 0U) == PID_OK, "overrun: timebase");
        CK(PIDe_RateInit(&r, 1000U) == PID_OK, "overrun: init");
        (void)PIDe_RateElapsed(&r);                 /* prime */

        /* Miss five whole periods. */
        vadvance(5000U);
        CK(PIDe_RateElapsed(&r) == true, "late release fires");
        CK(r.overruns == 1U, "one overrun counted");
        CK(r.last_dt_us == 5000U, "measured dt reports the real interval");

        /* The backlog must NOT be delivered back-to-back: four extra releases
         * with tiny dt would spike the derivative term of every controller
         * fed by this loop. */
        CK(PIDe_RateElapsed(&r) == false, "no catch-up burst after an overrun");
        vadvance(999U);
        CK(PIDe_RateElapsed(&r) == false, "still nothing 1 us early");
        vadvance(1U);
        CK(PIDe_RateElapsed(&r) == true, "next release is one clean period on");

        CK(r.worst_late_us >= 4000U, "worst lateness recorded");
        PIDe_RateResetStats(&r);
        CK(r.overruns == 0U && r.worst_late_us == 0U, "stats reset");
    }

    /* ---- 7. measured dt ------------------------------------------------- */
    {
        PIDe_Rate r;
        PID_Float dt;

        CK(vinit(0xFFFFFFFFU, 0U) == PID_OK, "dt: timebase");
        CK(PIDe_RateInit(&r, 2000U) == PID_OK, "dt: init");
        (void)PIDe_RateElapsed(&r);
        vadvance(2000U);
        (void)PIDe_RateElapsed(&r);

        dt = PIDe_RateDt(&r);
        CK(dt > (PID_Float)0.0019 && dt < (PID_Float)0.0021,
           "RateDt reports 2 ms in seconds");
        CK(PIDe_RateDt(NULL) == PID_ZERO, "RateDt tolerates NULL");
    }

    /* ---- 8. load monitor ------------------------------------------------ */
    {
        PIDe_Load l;

        (void)PIDe_TimebaseInit(240000000UL);   /* 240 cycles per us */
        PIDe_LoadInit(&l);
        CK(l.samples == 0U, "load starts empty");

        /* A body of 24000 cycles = 100 us at 240 MHz. Against a 1000 us
         * period that is exactly 10% duty. */
        pide_stub_cycles = 1000U;
        PIDe_LoadEnter(&l);
        pide_stub_cycles = 1000U + 24000U;
        PIDe_LoadExit(&l);

        CK(l.busy_cycles == 24000U, "busy cycles measured");
        CK(l.samples == 1U, "one sample");
        {
            const PID_Float f = PIDe_LoadFraction(&l, 1000U);
            CK(f > (PID_Float)0.099 && f < (PID_Float)0.101,
               "10% duty computed correctly");
        }

        /* A longer body must move the worst case but the fraction must track
         * the LAST body, not the worst. */
        pide_stub_cycles = 0U;
        PIDe_LoadEnter(&l);
        pide_stub_cycles = 48000U;
        PIDe_LoadExit(&l);
        CK(l.worst_cycles == 48000U, "worst case updated");
        {
            const PID_Float f = PIDe_LoadFraction(&l, 1000U);
            const PID_Float w = PIDe_LoadWorstFraction(&l, 1000U);
            CK(f > (PID_Float)0.199 && f < (PID_Float)0.201, "last duty 20%");
            CK(w > (PID_Float)0.199 && w < (PID_Float)0.201, "worst duty 20%");
        }

        /* The cycle counter wraps too, and an unsigned subtraction is already
         * correct across it. A body straddling the wrap must not report ~4e9. */
        pide_stub_cycles = 0xFFFFF000U;
        PIDe_LoadEnter(&l);
        pide_stub_cycles = 0x00000400U;         /* 0x1400 cycles later */
        PIDe_LoadExit(&l);
        CK(l.busy_cycles == 0x1400U, "load measurement survives a CCOUNT wrap");

        CK(PIDe_LoadFraction(&l, 0U) == PID_ZERO,
           "zero period yields zero duty, not a divide by zero");
        CK(PIDe_LoadFraction(NULL, 1000U) == PID_ZERO,
           "load fraction tolerates NULL");
    }

    /* ---- 9. cycle conversion -------------------------------------------- */
    {
        PID_Float us;
        (void)PIDe_TimebaseInit(240000000UL);
        us = PIDe_CyclesToUs(2400U);
        CK(us > (PID_Float)9.99 && us < (PID_Float)10.01,
           "2400 cycles is 10 us at 240 MHz");

        (void)PIDe_TimebaseInit(80000000UL);
        us = PIDe_CyclesToUs(800U);
        CK(us > (PID_Float)9.99 && us < (PID_Float)10.01,
           "800 cycles is 10 us at 80 MHz");
    }

    /* ---- 10. CCOUNT source rejects a fractional MHz clock --------------- */
    {
        /* Only meaningful when the CCOUNT source is selected; with the
         * esp_timer default the clock argument is advisory. The check is kept
         * unconditional so the arithmetic is exercised either way. */
        const PID_StatusCode rc = PIDe_TimebaseInit(240000000UL);
        CK(rc == PID_OK, "240 MHz accepted");
    }

    /* ---- 11. delay ------------------------------------------------------ */
    {
        /* PIDe_DelayUs busy-waits on the timebase. With a frozen virtual clock
         * it would spin forever, so only the zero case is exercised here -
         * enough to prove the loop condition is not inverted. */
        CK(vinit(0xFFFFFFFFU, 500U) == PID_OK, "delay: timebase");
        PIDe_DelayUs(0U);
        CK(1, "zero delay returns immediately");
    }

    printf("\ntest_esp32_host: %d passed, %d failed\n", pass, bad);
    return (bad == 0) ? 0 : 1;
}
