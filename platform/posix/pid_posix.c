/**
 * @file    pid_posix.c
 * @brief   POSIX timebase implementation. See pid_posix.h.
 */
#if !defined(_POSIX_C_SOURCE) || (_POSIX_C_SOURCE < 199309L)
#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include <time.h>
#include <errno.h>
#include <stddef.h>

#include "pid_posix.h"

#define PIDP_USEC_PER_SEC   1000000ULL
#define PIDP_NSEC_PER_USEC  1000ULL

/* ======================================================================== */
/* Timebase                                                                  */
/* ======================================================================== */

uint64_t PIDp_NowUs(void)
{
    struct timespec ts;

    /* CLOCK_MONOTONIC, not CLOCK_REALTIME: an NTP step on the realtime clock
     * would otherwise show up as a negative or enormous dt. */
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * PIDP_USEC_PER_SEC)
         + ((uint64_t)ts.tv_nsec / PIDP_NSEC_PER_USEC);
}

double PIDp_Now(void)
{
    return (double)PIDp_NowUs() / (double)PIDP_USEC_PER_SEC;
}

void PIDp_SleepUs(uint64_t us)
{
    struct timespec req;
    struct timespec rem;
    int rc;

    req.tv_sec  = (time_t)(us / PIDP_USEC_PER_SEC);
    req.tv_nsec = (long)((us % PIDP_USEC_PER_SEC) * PIDP_NSEC_PER_USEC);

    /* nanosleep returns early when a signal arrives. Resuming with the
     * remainder is what makes the sleep actually last the requested time;
     * ignoring EINTR is a classic source of a control loop that silently
     * runs fast whenever a profiler or debugger is attached. */
    do {
        rc = nanosleep(&req, &rem);
        req = rem;
    } while ((rc != 0) && (errno == EINTR));
}

/* ======================================================================== */
/* Fixed-rate loop                                                           */
/* ======================================================================== */

PID_StatusCode PIDp_LoopInit(PIDp_Loop *lp, uint64_t period_us)
{
    if (lp == NULL) {
        return PID_ERR_NULL;
    }
    if (period_us == 0U) {
        return PID_ERR_INVALID_PARAM;
    }

    lp->period_us     = period_us;
    lp->next_deadline = 0U;
    lp->last_release  = 0U;
    lp->started_us    = 0U;
    lp->iterations    = 0U;
    lp->overruns      = 0U;
    lp->worst_lateness= 0U;
    lp->primed        = false;
    return PID_OK;
}

double PIDp_LoopWait(PIDp_Loop *lp)
{
    uint64_t now;
    uint64_t prev_release;
    double   elapsed;

    if (lp == NULL) {
        return 0.0;
    }

    now = PIDp_NowUs();

    if (!lp->primed) {
        /* First call: establish the schedule origin and return immediately.
         * Returning the nominal period here (rather than 0) keeps the first
         * PID_UpdateDt() from seeing a zero dt. */
        lp->started_us    = now;
        lp->next_deadline = now + lp->period_us;
        lp->last_release  = now;
        lp->primed        = true;
        lp->iterations    = 1U;
        return (double)lp->period_us / (double)PIDP_USEC_PER_SEC;
    }

    prev_release = lp->last_release;

    if (now < lp->next_deadline) {
        PIDp_SleepUs(lp->next_deadline - now);
        now = PIDp_NowUs();
        /* The OS may wake us late; record by how much so the caller can see
         * whether the requested rate is actually achievable on this machine. */
        if (now > lp->next_deadline) {
            uint64_t late = now - lp->next_deadline;
            if (late > lp->worst_lateness) {
                lp->worst_lateness = late;
            }
        }
        lp->next_deadline += lp->period_us;
    } else {
        /* Deadline already missed. Do NOT fire the backlog: a burst of
         * catch-up iterations feeds the controller a series of tiny dt values
         * and makes the derivative term spike. Re-base instead and count it. */
        lp->overruns++;
        lp->next_deadline = now + lp->period_us;
    }

    elapsed = (double)(now - prev_release) / (double)PIDP_USEC_PER_SEC;
    lp->last_release = now;
    lp->iterations++;
    return elapsed;
}

double PIDp_LoopMeanRate(const PIDp_Loop *lp)
{
    uint64_t span;

    if ((lp == NULL) || (!lp->primed) || (lp->iterations < 2U)) {
        return 0.0;
    }
    span = PIDp_NowUs() - lp->started_us;
    if (span == 0U) {
        return 0.0;
    }
    return ((double)(lp->iterations - 1U) * (double)PIDP_USEC_PER_SEC)
           / (double)span;
}

/* ======================================================================== */
/* Timer                                                                     */
/* ======================================================================== */

void PIDp_TimerReset(PIDp_Timer *t)
{
    if (t == NULL) {
        return;
    }
    t->t0_us    = 0U;
    t->total_us = 0U;
    t->min_us   = UINT64_MAX;
    t->max_us   = 0U;
    t->count    = 0U;
}

void PIDp_TimerStart(PIDp_Timer *t)
{
    if (t != NULL) {
        t->t0_us = PIDp_NowUs();
    }
}

void PIDp_TimerStop(PIDp_Timer *t)
{
    uint64_t d;

    if ((t == NULL) || (t->t0_us == 0U)) {
        return;
    }
    d = PIDp_NowUs() - t->t0_us;
    t->total_us += d;
    if (d < t->min_us) { t->min_us = d; }
    if (d > t->max_us) { t->max_us = d; }
    t->count++;
}

double PIDp_TimerMeanUs(const PIDp_Timer *t)
{
    if ((t == NULL) || (t->count == 0U)) {
        return 0.0;
    }
    return (double)t->total_us / (double)t->count;
}
