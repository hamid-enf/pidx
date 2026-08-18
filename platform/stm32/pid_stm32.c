/**
 * @file    pid_stm32.c
 * @brief   STM32 / Cortex-M integration layer. See pid_stm32.h.
 *
 * Register-level, no HAL. The only peripherals touched are the timer named in
 * pid_stm32_conf.h and, optionally, the DWT unit.
 */
#include "pid_stm32.h"

/* ======================================================================== */
/* Internal state                                                            */
/* ======================================================================== */

typedef enum {
    PIDS_SRC_NONE = 0,
    PIDS_SRC_TIM,
    PIDS_SRC_DWT,
    PIDS_SRC_CB
} pids_source_t;

static struct {
    pids_source_t source;
    TIM_TypeDef  *tim;
    uint32_t    (*cb)(void);
    uint32_t      mask;        /* counter width mask: 0xFFFF or 0xFFFFFFFF   */
    uint32_t      cyc_per_us;  /* for the DWT timebase                       */
    uint32_t      last_raw;    /* last raw sample, for wrap extension        */
    uint64_t      high;        /* accumulated wraps, in counter units        */
} pids_tb = {
    PIDS_SRC_NONE, NULL, NULL, 0xFFFFFFFFUL, 1UL, 0UL, 0ULL
};

/* ======================================================================== */
/* Critical section                                                          */
/* ======================================================================== */

uint32_t PIDs_EnterCritical(void)
{
#if (PIDX_STM32_CRITICAL_BASEPRI > 0)
    uint32_t prev = __get_BASEPRI();
    __set_BASEPRI((uint32_t)PIDX_STM32_CRITICAL_BASEPRI);
    return prev;
#else
    uint32_t prev = __get_PRIMASK();
    __disable_irq();
    return prev;
#endif
}

void PIDs_ExitCritical(uint32_t state)
{
#if (PIDX_STM32_CRITICAL_BASEPRI > 0)
    __set_BASEPRI(state);
#else
    /* Restore rather than blindly enabling: a section entered while already
     * masked must leave the CPU masked on the way out. */
    __set_PRIMASK(state);
#endif
}

/* ======================================================================== */
/* Timebase                                                                  */
/* ======================================================================== */

PID_StatusCode PIDs_TimebaseInitTim(TIM_TypeDef *tim, uint32_t timer_clk_hz)
{
    uint32_t psc;

    if (tim == NULL) {
        return PID_ERR_NULL;
    }
    /* An integer prescaler must land on exactly 1 MHz; anything else makes
     * every measured dt wrong by a fixed ratio, which looks like a badly tuned
     * controller rather than a clock bug. */
    if ((timer_clk_hz < 1000000U) || ((timer_clk_hz % 1000000U) != 0U)) {
        return PID_ERR_INVALID_PARAM;
    }
    psc = (timer_clk_hz / 1000000U) - 1U;
    if (psc > 0xFFFFU) {
        return PID_ERR_INVALID_PARAM;
    }

    tim->CR1 = 0UL;                 /* stop and clear direction/clock-div    */
    tim->PSC = psc;

    /* Detect the counter width instead of trusting a table of which timers
     * are 32-bit on which family: write all ones to ARR and see what sticks. */
    tim->ARR = 0xFFFFFFFFUL;
    pids_tb.mask = (tim->ARR == 0xFFFFFFFFUL) ? 0xFFFFFFFFUL : 0x0000FFFFUL;
    tim->ARR = pids_tb.mask;

    tim->CNT = 0UL;
    tim->EGR = 1UL;                 /* UG: latch PSC/ARR into the shadow regs*/
    tim->SR  = 0UL;                 /* UG raised the update flag; clear it   */
    tim->DIER = 0UL;                /* no interrupt, no DMA - read-only use  */
    tim->CR1 = 1UL;                 /* CEN                                   */

    pids_tb.tim      = tim;
    pids_tb.cb       = NULL;
    pids_tb.last_raw = 0UL;
    pids_tb.high     = 0ULL;
    pids_tb.source   = PIDS_SRC_TIM;
    return PID_OK;
}

#if (PIDX_STM32_HAS_DWT)

/** Turn the DWT cycle counter on. Shared by the timebase and the profiler. */
static PID_StatusCode pids_dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if (PIDX_STM32_DWT_HAS_LAR)
    /* Cortex-M7 and some M33 parts require unlocking before DWT writes. */
    DWT->LAR = 0xC5ACCE55UL;
#endif
    DWT->CYCCNT = 0UL;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* Prove it is actually counting. On a locked-down debug unit the enable
     * bit reads back as written but CYCCNT stays frozen, and a profiler that
     * always reports zero cycles is worse than one that reports failure. */
    if (DWT->CYCCNT == 0UL) {
        volatile uint32_t spin = 0UL;
        while (spin < 16UL) { spin++; }
        if (DWT->CYCCNT == 0UL) {
            return PID_ERR_UNSUPPORTED;
        }
    }
    return PID_OK;
}

PID_StatusCode PIDs_TimebaseInitDwt(uint32_t core_clk_hz)
{
    PID_StatusCode st;

    if (core_clk_hz < 1000000U) {
        return PID_ERR_INVALID_PARAM;
    }
    st = pids_dwt_enable();
    if (st != PID_OK) {
        return st;
    }

    pids_tb.cyc_per_us = core_clk_hz / 1000000U;
    pids_tb.mask       = 0xFFFFFFFFUL;
    pids_tb.tim        = NULL;
    pids_tb.cb         = NULL;
    pids_tb.last_raw   = 0UL;
    pids_tb.high       = 0ULL;
    pids_tb.source     = PIDS_SRC_DWT;
    return PID_OK;
}

#endif /* PIDX_STM32_HAS_DWT */

PID_StatusCode PIDs_TimebaseInitCallback(uint32_t (*fn)(void),
                                         uint32_t counter_mask)
{
    if (fn == NULL) {
        return PID_ERR_NULL;
    }
    /* Must be an all-ones mask, i.e. 2^n - 1, and wide enough to be usable. */
    if ((counter_mask < 0xFFFFUL) ||
        ((((uint64_t)counter_mask + 1ULL) & (uint64_t)counter_mask) != 0ULL)) {
        return PID_ERR_INVALID_PARAM;
    }
    pids_tb.cb       = fn;
    pids_tb.tim      = NULL;
    pids_tb.mask     = counter_mask;
    pids_tb.last_raw = 0UL;
    pids_tb.high     = 0ULL;
    pids_tb.source   = PIDS_SRC_CB;
    return PID_OK;
}

bool PIDs_TimebaseReady(void)
{
    return (pids_tb.source != PIDS_SRC_NONE);
}

uint32_t PIDs_CounterMask(void)
{
    return (pids_tb.source == PIDS_SRC_NONE) ? 0UL : pids_tb.mask;
}

uint32_t PIDs_NowUs32(void)
{
    uint32_t raw;

    switch (pids_tb.source) {
    case PIDS_SRC_TIM:
        raw = pids_tb.tim->CNT & pids_tb.mask;
        break;
#if (PIDX_STM32_HAS_DWT)
    case PIDS_SRC_DWT:
        /* Cycles -> microseconds. The division is by a run-time value, so it
         * is a real divide; this is why the DWT source is offered as an option
         * rather than the default. Profiling uses PIDs_Cycles() and never
         * pays for it. */
        raw = DWT->CYCCNT / pids_tb.cyc_per_us;
        break;
#endif
    case PIDS_SRC_CB:
        raw = pids_tb.cb() & pids_tb.mask;
        break;
    default:
        raw = 0UL;
        break;
    }
    return raw;
}

uint32_t PIDs_DeltaUs(uint32_t earlier, uint32_t later)
{
    /* Unsigned wrap-around subtraction, then mask to the hardware width. On a
     * 16-bit timer plain (later - earlier) would leave the borrow in the upper
     * half and produce a delta of ~4.29e9 us instead of a few hundred. */
    return (later - earlier) & pids_tb.mask;
}

uint64_t PIDs_NowUs(void)
{
    uint32_t raw;
    uint64_t now;
    uint32_t st;

    /* Reading and updating the wrap accumulator is a read-modify-write on
     * shared state, so it needs a critical section when the function is called
     * from both thread and ISR context. The section is a handful of cycles and
     * the control path itself never needs this function. */
    st  = PIDs_EnterCritical();
    raw = PIDs_NowUs32();
    if (raw < pids_tb.last_raw) {
        pids_tb.high += (uint64_t)pids_tb.mask + 1ULL;
    }
    pids_tb.last_raw = raw;
    now = pids_tb.high + (uint64_t)raw;
    PIDs_ExitCritical(st);

    return now;
}

PID_Float PIDs_Now(void)
{
    return (PID_Float)((PID_Float)PIDs_NowUs() * (PID_Float)1e-6);
}

void PIDs_DelayUs(uint32_t us)
{
    uint32_t t0 = PIDs_NowUs32();
    while (PIDs_DeltaUs(t0, PIDs_NowUs32()) < us) {
        /* busy wait */
    }
}

/* ======================================================================== */
/* Fixed-rate driver                                                         */
/* ======================================================================== */

PID_StatusCode PIDs_RateInit(PIDs_Rate *r, uint32_t period_us)
{
    if (r == NULL) {
        return PID_ERR_NULL;
    }
    if (period_us == 0UL) {
        return PID_ERR_INVALID_PARAM;
    }
    /* The deadline comparison uses signed wrap-around arithmetic, which is
     * only unambiguous while the interval is under half the counter range:
     * 32 ms on a 16-bit timer, ~35 minutes on a 32-bit one. */
    if (period_us > (pids_tb.mask / 2UL)) {
        return PID_ERR_INVALID_PARAM;
    }

    r->period_us     = period_us;
    r->next_deadline = 0UL;
    r->last_release  = 0UL;
    r->last_dt_us    = period_us;
    r->iterations    = 0UL;
    r->overruns      = 0UL;
    r->worst_late_us = 0UL;
    r->primed        = false;
    return PID_OK;
}

bool PIDs_RateElapsed(PIDs_Rate *r)
{
    uint32_t now;
    uint32_t late;

    if (r == NULL) {
        return false;
    }

    now = PIDs_NowUs32();

    if (!r->primed) {
        r->last_release  = now;
        r->next_deadline = (now + r->period_us) & pids_tb.mask;
        r->last_dt_us    = r->period_us;
        r->primed        = true;
        r->iterations    = 1U;
        return true;
    }

    /* Wrap-safe "has the deadline passed?".
     *
     * (now - deadline) masked to the counter width is a small number just
     * after the deadline and a huge one just before it, so comparing against
     * half the range tells the two apart. This is the unsigned equivalent of
     * the signed (int32_t)(now - deadline) >= 0 idiom, generalised to a
     * 16-bit counter.
     *
     * The comparison is against next_deadline - an absolute point on the
     * schedule - and NOT against last_release + period. Re-basing on the
     * moment the poll happened to notice would fold the loop body time and the
     * polling granularity into every period, and the loop would slowly run
     * slower than configured while the controller kept using the nominal dt.
     */
    late = PIDs_DeltaUs(r->next_deadline, now);
    if (late > (pids_tb.mask / 2U)) {
        return false;                       /* deadline still in the future */
    }

    r->last_dt_us   = PIDs_DeltaUs(r->last_release, now);
    r->last_release = now;

    if (late > r->worst_late_us) {
        r->worst_late_us = late;
    }

    /* A full period late means the previous iteration did not finish in time.
     * Count it and re-base, rather than firing the backlog back-to-back: a
     * catch-up burst feeds the controller a run of tiny dt values and spikes
     * the derivative term. */
    if (late >= r->period_us) {
        r->overruns++;
        r->next_deadline = (now + r->period_us) & pids_tb.mask;
    } else {
        r->next_deadline = (r->next_deadline + r->period_us) & pids_tb.mask;
    }

    r->iterations++;
    return true;
}

/* ======================================================================== */
/* Cycle profiler                                                            */
/* ======================================================================== */

#if (PIDX_STM32_HAS_DWT)

PID_StatusCode PIDs_CycleInit(void)
{
    return pids_dwt_enable();
}

uint32_t PIDs_Cycles(void)
{
    return DWT->CYCCNT;
}

void PIDs_CycleReset(PIDs_CycleStat *s)
{
    if (s == NULL) {
        return;
    }
    s->t0    = 0UL;
    s->total = 0ULL;
    s->min   = 0xFFFFFFFFUL;
    s->max   = 0UL;
    s->count = 0UL;
}

void PIDs_CycleStart(PIDs_CycleStat *s)
{
    if (s != NULL) {
        s->t0 = DWT->CYCCNT;
    }
}

void PIDs_CycleStop(PIDs_CycleStat *s)
{
    uint32_t d;

    if (s == NULL) {
        return;
    }
    d = DWT->CYCCNT - s->t0;   /* unsigned wrap-around is well defined */
    s->total += (uint64_t)d;
    if (d < s->min) { s->min = d; }
    if (d > s->max) { s->max = d; }
    s->count++;
}

PID_Float PIDs_CycleMean(const PIDs_CycleStat *s)
{
    if ((s == NULL) || (s->count == 0UL)) {
        return (PID_Float)0;
    }
    return (PID_Float)((PID_Float)s->total / (PID_Float)s->count);
}

PID_Float PIDs_CyclesToUs(uint32_t cycles)
{
    return (PID_Float)((PID_Float)cycles
                       / ((PID_Float)PIDX_STM32_CORE_CLK_HZ * (PID_Float)1e-6));
}

#endif /* PIDX_STM32_HAS_DWT */

/* ======================================================================== */
/* ISR monitor                                                               */
/* ======================================================================== */

PID_StatusCode PIDs_IsrMonitorInit(PIDs_IsrMonitor *m, uint32_t nominal_us)
{
    if (m == NULL) {
        return PID_ERR_NULL;
    }
    if (nominal_us == 0UL) {
        return PID_ERR_INVALID_PARAM;
    }
    m->nominal_us     = nominal_us;
    m->t_enter        = 0UL;
    m->last_period_us = 0UL;
    m->last_exec_us   = 0UL;
    m->max_exec_us    = 0UL;
    m->max_jitter_us  = 0UL;
    m->exec_total_us  = 0ULL;
    m->entries        = 0UL;
    m->primed         = false;
    return PID_OK;
}

void PIDs_IsrEnter(PIDs_IsrMonitor *m)
{
    uint32_t now;

    if (m == NULL) {
        return;
    }
    now = PIDs_NowUs32();

    if (m->primed) {
        uint32_t period = PIDs_DeltaUs(m->t_enter, now);
        uint32_t jitter = (period > m->nominal_us)
                        ? (period - m->nominal_us)
                        : (m->nominal_us - period);

        m->last_period_us = period;
        if (jitter > m->max_jitter_us) {
            m->max_jitter_us = jitter;
        }
    } else {
        m->primed = true;
    }

    m->t_enter = now;
    m->entries++;
}

void PIDs_IsrExit(PIDs_IsrMonitor *m)
{
    uint32_t exec;

    if ((m == NULL) || (!m->primed)) {
        return;
    }
    exec = PIDs_DeltaUs(m->t_enter, PIDs_NowUs32());

    m->last_exec_us  = exec;
    m->exec_total_us += (uint64_t)exec;
    if (exec > m->max_exec_us) {
        m->max_exec_us = exec;
    }
}

PID_Float PIDs_IsrLoadPercent(const PIDs_IsrMonitor *m)
{
    PID_Float mean;

    if ((m == NULL) || (m->entries == 0UL) || (m->nominal_us == 0UL)) {
        return (PID_Float)0;
    }
    mean = (PID_Float)((PID_Float)m->exec_total_us / (PID_Float)m->entries);
    return (PID_Float)((mean * (PID_Float)100) / (PID_Float)m->nominal_us);
}
