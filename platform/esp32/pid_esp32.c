/**
 * @file    pid_esp32.c
 * @brief   ESP32 / ESP-IDF integration layer. See pid_esp32.h.
 *
 * Depends only on esp_timer.h and, for the optional helpers, the FreeRTOS
 * headers ESP-IDF already ships. No HAL wrapper, no component registration
 * beyond adding this file to your CMakeLists SRCS.
 */
#include "pid_esp32.h"

/* ======================================================================== */
/* Internal state                                                            */
/* ======================================================================== */

typedef enum {
    PIDE_SRC_NONE = 0,
    PIDE_SRC_ESP_TIMER,
    PIDE_SRC_CCOUNT,
    PIDE_SRC_CB
} pide_source_t;

static struct {
    pide_source_t source;
    uint32_t    (*cb)(void);
    uint32_t      mask;        /* counter width mask                        */
    uint32_t      cyc_per_us;  /* for the CCOUNT source                     */
    uint32_t      last_raw;    /* last raw sample, for wrap extension       */
    uint64_t      high;        /* accumulated wraps, in counter units       */
} pide_tb = {
    PIDE_SRC_NONE, NULL, 0xFFFFFFFFUL, 240UL, 0UL, 0ULL
};

/* ======================================================================== */
/* Timebase                                                                  */
/* ======================================================================== */

PID_StatusCode PIDe_TimebaseInit(uint32_t cpu_freq_hz)
{
#if (PIDX_ESP32_TIMEBASE == PIDX_ESP32_TB_CCOUNT)
    /* CCOUNT counts CPU cycles, so an integer cycles-per-microsecond is
     * required for the conversion to stay exact. Every supported ESP32 clock
     * (80/160/240 MHz) satisfies this; a fractional value would make every
     * measured dt wrong by a fixed ratio, which looks like a badly tuned
     * controller rather than a clock bug. */
    if ((cpu_freq_hz < 1000000U) || ((cpu_freq_hz % 1000000U) != 0U)) {
        return PID_ERR_INVALID_PARAM;
    }
    pide_tb.cyc_per_us = cpu_freq_hz / 1000000U;
    pide_tb.mask       = 0xFFFFFFFFUL;
    pide_tb.source     = PIDE_SRC_CCOUNT;
#else
    /* esp_timer is already running before app_main(), so there is nothing to
     * start. Recording the source is what makes PIDe_TimebaseReady() honest
     * and keeps application code identical across platforms. */
    PIDX_UNUSED(cpu_freq_hz);
    pide_tb.cyc_per_us = (cpu_freq_hz >= 1000000U)
                         ? (cpu_freq_hz / 1000000U) : 240UL;
    pide_tb.mask       = 0xFFFFFFFFUL;
    pide_tb.source     = PIDE_SRC_ESP_TIMER;
#endif
    pide_tb.cb       = NULL;
    pide_tb.last_raw = 0UL;
    pide_tb.high     = 0ULL;
    return PID_OK;
}

PID_StatusCode PIDe_TimebaseInitCallback(uint32_t (*fn)(void),
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
    pide_tb.cb       = fn;
    pide_tb.mask     = counter_mask;
    pide_tb.last_raw = 0UL;
    pide_tb.high     = 0ULL;
    pide_tb.source   = PIDE_SRC_CB;
    return PID_OK;
}

bool PIDe_TimebaseReady(void)
{
    return (pide_tb.source != PIDE_SRC_NONE);
}

uint32_t PIDe_CounterMask(void)
{
    return (pide_tb.source == PIDE_SRC_NONE) ? 0UL : pide_tb.mask;
}

uint32_t PIDe_Cycles(void)
{
    return pide_ccount();
}

PID_Float PIDe_CyclesToUs(uint32_t cycles)
{
    const uint32_t cpu = (pide_tb.cyc_per_us == 0U) ? 1U : pide_tb.cyc_per_us;
    return (PID_Float)((PID_Float)cycles / (PID_Float)cpu);
}

uint32_t PIDe_NowUs32(void)
{
    uint32_t raw;

    switch (pide_tb.source) {
    case PIDE_SRC_ESP_TIMER:
        /* Truncating the 64-bit value is deliberate: this is the cheap read,
         * and PIDe_DeltaUs() is wrap-safe. Callers that need the full range
         * use PIDe_NowUs(). */
        raw = (uint32_t)((uint64_t)pide_esp_timer_us() & 0xFFFFFFFFULL);
        break;
    case PIDE_SRC_CCOUNT:
        /* Cycles -> microseconds. The divisor is a run-time value, so this is
         * a real divide; it is why CCOUNT is offered as an option rather than
         * the default. Profiling uses PIDe_Cycles() and never pays for it. */
        raw = pide_ccount() / pide_tb.cyc_per_us;
        break;
    case PIDE_SRC_CB:
        raw = pide_tb.cb() & pide_tb.mask;
        break;
    default:
        raw = 0UL;
        break;
    }
    return raw;
}

uint32_t PIDe_DeltaUs(uint32_t earlier, uint32_t later)
{
    /* Unsigned wrap-around subtraction, then mask to the source width. */
    return (later - earlier) & pide_tb.mask;
}

uint64_t PIDe_NowUs(void)
{
    uint32_t raw;
    uint64_t now;

    /* The esp_timer source is already a monotonic 64-bit microsecond count -
     * no wrap accumulator, and therefore no shared state and no critical
     * section. This is the whole reason it is the default. */
    if (pide_tb.source == PIDE_SRC_ESP_TIMER) {
        return (uint64_t)pide_esp_timer_us();
    }

    /* The 32-bit sources need the wrap extension, which is a read-modify-write
     * on shared state. It must be atomic against both the other core and an
     * ISR, so it is done inside a portMUX critical section. The section is a
     * handful of instructions and the control path itself never needs this
     * function - PIDe_NowUs32() + PIDe_DeltaUs() is the hot-path pair. */
    PIDE_ENTER_CRITICAL();
    raw = PIDe_NowUs32();
    if (raw < pide_tb.last_raw) {
        pide_tb.high += (uint64_t)pide_tb.mask + 1ULL;
    }
    pide_tb.last_raw = raw;
    now = pide_tb.high + (uint64_t)raw;
    PIDE_EXIT_CRITICAL();

    return now;
}

PID_Float PIDe_Now(void)
{
    return (PID_Float)((PID_Float)PIDe_NowUs() * (PID_Float)1e-6);
}

void PIDe_DelayUs(uint32_t us)
{
    const uint32_t t0 = PIDe_NowUs32();
    while (PIDe_DeltaUs(t0, PIDe_NowUs32()) < us) {
        /* busy wait */
    }
}

/* ======================================================================== */
/* Fixed-rate driver                                                         */
/* ======================================================================== */

PID_StatusCode PIDe_RateInit(PIDe_Rate *r, uint32_t period_us)
{
    if (r == NULL) {
        return PID_ERR_NULL;
    }
    if (period_us == 0UL) {
        return PID_ERR_INVALID_PARAM;
    }
    /* The deadline comparison uses wrap-around arithmetic, which is only
     * unambiguous while the interval is under half the counter range:
     * ~35 minutes on a 32-bit microsecond counter. */
    if (period_us > (pide_tb.mask / 2UL)) {
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

bool PIDe_RateElapsed(PIDe_Rate *r)
{
    uint32_t now;
    uint32_t late;

    if (r == NULL) {
        return false;
    }

    now = PIDe_NowUs32();

    if (!r->primed) {
        r->last_release  = now;
        r->next_deadline = (now + r->period_us) & pide_tb.mask;
        r->last_dt_us    = r->period_us;
        r->primed        = true;
        r->iterations    = 1U;
        return true;
    }

    /*
     * Wrap-safe "has the deadline passed?".
     *
     * (now - deadline) masked to the counter width is a small number just
     * after the deadline and a huge one just before it, so comparing against
     * half the range tells the two apart.
     *
     * The comparison is against next_deadline - an absolute point on the
     * schedule - and NOT against last_release + period. Re-basing on the
     * moment the poll happened to notice would fold the loop body time and the
     * polling granularity into every period, and the loop would slowly run
     * slower than configured while the controller kept using the nominal dt.
     */
    late = PIDe_DeltaUs(r->next_deadline, now);
    if (late > (pide_tb.mask / 2U)) {
        return false;                       /* deadline still in the future */
    }

    r->last_dt_us   = PIDe_DeltaUs(r->last_release, now);
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
        r->next_deadline = (now + r->period_us) & pide_tb.mask;
    } else {
        r->next_deadline = (r->next_deadline + r->period_us) & pide_tb.mask;
    }

    r->iterations++;
    return true;
}

PID_Float PIDe_RateDt(const PIDe_Rate *r)
{
    if (r == NULL) {
        return PID_ZERO;
    }
    return (PID_Float)((PID_Float)r->last_dt_us * (PID_Float)1e-6);
}

void PIDe_RateResetStats(PIDe_Rate *r)
{
    if (r != NULL) {
        r->iterations    = 0UL;
        r->overruns      = 0UL;
        r->worst_late_us = 0UL;
    }
}

/* ======================================================================== */
/* Load monitor                                                              */
/* ======================================================================== */

void PIDe_LoadInit(PIDe_Load *l)
{
    if (l != NULL) {
        l->t_enter      = 0UL;
        l->busy_cycles  = 0UL;
        l->worst_cycles = 0UL;
        l->total_cycles = 0ULL;
        l->samples      = 0UL;
    }
}

void PIDe_LoadEnter(PIDe_Load *l)
{
    if (l != NULL) {
        l->t_enter = pide_ccount();
    }
}

void PIDe_LoadExit(PIDe_Load *l)
{
    if (l != NULL) {
        /* Unsigned subtraction is wrap-correct for a full-width 32-bit
         * counter, so no mask is needed here. */
        const uint32_t d = pide_ccount() - l->t_enter;
        l->busy_cycles = d;
        if (d > l->worst_cycles) {
            l->worst_cycles = d;
        }
        l->total_cycles += (uint64_t)d;
        l->samples++;
    }
}

/** Shared duty computation: cycles against a period expressed in us. */
static PID_Float pide_duty(uint32_t cycles, uint32_t period_us)
{
    PID_Float budget;

    if (period_us == 0U) {
        return PID_ZERO;
    }
    budget = (PID_Float)period_us * (PID_Float)pide_tb.cyc_per_us;
    if (budget <= PID_ZERO) {
        return PID_ZERO;
    }
    return (PID_Float)cycles / budget;
}

PID_Float PIDe_LoadFraction(const PIDe_Load *l, uint32_t period_us)
{
    return (l == NULL) ? PID_ZERO : pide_duty(l->busy_cycles, period_us);
}

PID_Float PIDe_LoadWorstFraction(const PIDe_Load *l, uint32_t period_us)
{
    return (l == NULL) ? PID_ZERO : pide_duty(l->worst_cycles, period_us);
}

/* ======================================================================== */
/* FreeRTOS helpers                                                          */
/* ======================================================================== */

#if PIDX_ESP32_USE_FREERTOS

PID_StatusCode PIDe_TaskCreate(void (*fn)(void *), const char *name,
                               uint32_t stack, void *arg,
                               uint32_t priority, int core_id,
                               void **out_handle)
{
    TaskHandle_t h = NULL;
    BaseType_t rc;

    if ((fn == NULL) || (name == NULL)) {
        return PID_ERR_NULL;
    }
    if (stack == 0U) {
        return PID_ERR_INVALID_PARAM;
    }

    if (core_id == PIDX_ESP32_CORE_ANY) {
        rc = xTaskCreate(fn, name, stack, arg, (UBaseType_t)priority, &h);
    } else {
        if ((core_id < 0) || (core_id > 1)) {
            return PID_ERR_INVALID_PARAM;
        }
        rc = xTaskCreatePinnedToCore(fn, name, stack, arg,
                                     (UBaseType_t)priority, &h,
                                     (BaseType_t)core_id);
    }

    if (rc != pdPASS) {
        /* On ESP-IDF the only realistic failure is the heap being unable to
         * supply the stack. Reporting BUSY rather than a generic error keeps
         * the distinction between "bad argument" and "no memory". */
        return PID_ERR_BUSY;
    }
    if (out_handle != NULL) {
        *out_handle = (void *)h;
    }
    return PID_OK;
}

PID_StatusCode PIDe_TaskDelayPeriod(uint32_t *last_wake, uint32_t period_us)
{
    TickType_t ticks;
    TickType_t lw;

    if (last_wake == NULL) {
        return PID_ERR_NULL;
    }
    if (period_us == 0U) {
        return PID_ERR_INVALID_PARAM;
    }

    ticks = (TickType_t)pdMS_TO_TICKS(period_us / 1000U);
    if (ticks == 0U) {
        /* The requested period is shorter than one FreeRTOS tick. Delaying by
         * zero ticks would spin, and rounding up to one tick would run the
         * loop at the tick rate while the caller believed otherwise - a 1 kHz
         * request quietly becoming 100 Hz at the default CONFIG_FREERTOS_HZ.
         * Refusing is the only honest answer; drive fast loops from a timer
         * ISR and use PIDe_Rate instead. */
        return PID_ERR_INVALID_PARAM;
    }

    lw = (TickType_t)(*last_wake);
    vTaskDelayUntil(&lw, ticks);
    *last_wake = (uint32_t)lw;
    return PID_OK;
}

#endif /* PIDX_ESP32_USE_FREERTOS */
