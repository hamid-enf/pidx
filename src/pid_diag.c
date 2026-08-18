/**
 * @file    pid_diag.c
 * @brief   Telemetry ring buffer and loop-quality metrics.
 */

#include "pidx/pid_diag.h"

#if PIDX_ENABLE_TELEMETRY

PID_StatusCode PID_Telemetry_Init(PID_Telemetry *t,
                                  PID_TelemetryRecord *storage,
                                  uint16_t capacity)
{
    if ((t == NULL) || (storage == NULL)) {
        return PID_ERR_NULL;
    }
    /* Power-of-two capacity turns the wrap into a mask; a modulo on a 16-bit
     * index would otherwise cost a division in the producer, i.e. in an ISR. */
    if ((capacity < 2U) || ((capacity & (uint16_t)(capacity - 1U)) != 0U)) {
        return PID_ERR_INVALID_PARAM;
    }

    t->storage = storage;
    t->capacity_mask = (uint16_t)(capacity - 1U);
    t->head = 0U;
    t->tail = 0U;
    t->seq = 0U;
    t->dropped = 0U;
    return PID_OK;
}

PID_StatusCode PID_Telemetry_Attach(PID_Handle *h, PID_Telemetry *t)
{
    if (h == NULL) {
        return PID_ERR_NULL;
    }
    if ((t != NULL) && (t->storage == NULL)) {
        return PID_ERR_INVALID_PARAM;   /* not through PID_Telemetry_Init() */
    }
    h->telemetry = t;
    if (t != NULL) {
        h->features |= (PID_FEAT_TELEMETRY | PID_FEAT_DIAGNOSTICS);
    } else {
        h->features &= ~(uint32_t)PID_FEAT_TELEMETRY;
    }
    return PID_OK;
}

void pidd_telemetry_push(PID_Telemetry *t, const PID_Status *s)
{
    uint16_t head;
    uint16_t next;
    PID_TelemetryRecord *rec;

    if ((t == NULL) || (t->storage == NULL) || (s == NULL)) {
        return;
    }

    head = t->head;
    next = (uint16_t)((head + 1U) & t->capacity_mask);

    if (next == t->tail) {
        /* Full: drop the NEWEST record (this one) and leave the ring alone.
         *
         * The obvious alternative - overwrite the oldest by advancing tail -
         * breaks the single-producer/single-consumer contract, because tail
         * would then have two writers. That is not merely a lost record: if
         * the producer is pre-empted between reading tail and writing it, it
         * resumes with a stale value and drives tail BACKWARD, so a record
         * the consumer already returned is delivered a second time.
         *
         * Dropping the newest keeps tail consumer-owned, so the buffer stays
         * lock-free with no critical section. The loss is still visible: seq
         * is incremented for dropped records too, so a gap in the sequence
         * numbers tells the consumer exactly how many samples it missed. */
        t->dropped++;
        t->seq++;
        return;
    }

    rec = &t->storage[head];
    rec->setpoint    = s->setpoint_shaped;
    rec->measurement = s->measurement_filtered;
    rec->p_term      = s->p_term;
    rec->i_term      = s->i_term;
    rec->d_term      = s->d_term;
    rec->ff_term     = s->ff_term;
    rec->output      = s->output;
    rec->flags       = s->flags;
    rec->seq         = t->seq;
    t->seq++;

    /* Publish only after the record is fully written, so a consumer that sees
     * the new head always sees complete data. */
    PIDX_MEMORY_BARRIER();
    t->head = next;
}

PID_StatusCode PID_Telemetry_Read(PID_Telemetry *t, PID_TelemetryRecord *out)
{
    uint16_t tail;

    if ((t == NULL) || (out == NULL)) {
        return PID_ERR_NULL;
    }
    if (t->storage == NULL) {
        return PID_ERR_NOT_INIT;
    }

    tail = t->tail;
    if (tail == t->head) {
        return PID_ERR_BUSY;            /* empty */
    }

    *out = t->storage[tail];
    PIDX_MEMORY_BARRIER();
    t->tail = (uint16_t)((tail + 1U) & t->capacity_mask);
    return PID_OK;
}

uint16_t PID_Telemetry_Count(const PID_Telemetry *t)
{
    uint16_t n = 0U;
    if ((t != NULL) && (t->storage != NULL)) {
        n = (uint16_t)((t->head - t->tail) & t->capacity_mask);
    }
    return n;
}

uint16_t PID_Telemetry_Dropped(PID_Telemetry *t)
{
    uint16_t d = 0U;
    if (t != NULL) {
        d = t->dropped;
        t->dropped = 0U;
    }
    return d;
}

PID_StatusCode PID_Telemetry_Flush(PID_Telemetry *t)
{
    if (t == NULL) {
        return PID_ERR_NULL;
    }
    t->tail = t->head;
    return PID_OK;
}

#endif /* PIDX_ENABLE_TELEMETRY */

/* ======================================================================== */
/* Loop metrics                                                              */
/* ======================================================================== */

#if PIDX_ENABLE_DIAGNOSTICS

PID_StatusCode PID_Metrics_Reset(PID_LoopMetrics *m)
{
    uint8_t *p;
    size_t i;

    if (m == NULL) {
        return PID_ERR_NULL;
    }
    p = (uint8_t *)m;
    for (i = 0U; i < sizeof(PID_LoopMetrics); ++i) {
        p[i] = 0U;
    }
    return PID_OK;
}

PID_StatusCode PID_Metrics_Update(PID_LoopMetrics *m, const PID_Handle *h)
{
    PID_Float e;
    PID_Float ae;
    PID_Float dt;
    PID_Float u;

    if ((m == NULL) || (h == NULL)) {
        return PID_ERR_NULL;
    }

    e  = h->status.error;
    dt = h->status.dt_used;
    u  = h->status.output;

    if (!pidm_isfinite(e) || !pidm_isfinite(dt) || (dt <= PID_ZERO)) {
        return PID_ERR_INVALID_PARAM;
    }

    ae = pidm_abs(e);

    /* Rectangle-rule integrals against the real dt of this sample, so jitter
     * does not bias the comparison between two tuning attempts. */
    m->iae  += ae * dt;
    m->ise  += (e * e) * dt;
    m->itae += m->total_time * ae * dt;   /* weight by elapsed time */
    m->total_time += dt;

    if (ae > m->abs_error_max) {
        m->abs_error_max = ae;
    }
    if ((h->status.flags & PID_FLAG_SATURATED) != 0U) {
        m->sat_time += dt;
    }

    if (m->primed) {
        m->output_travel += pidm_abs(u - m->last_output);
        /* Strict sign change only: passing exactly through zero must not be
         * double-counted on the next sample. */
        if (((m->last_error > PID_ZERO) && (e < PID_ZERO)) ||
            ((m->last_error < PID_ZERO) && (e > PID_ZERO))) {
            m->sign_changes++;
        }
    } else {
        m->primed = true;
    }

    m->last_output = u;
    m->last_error = e;
    m->samples++;
    return PID_OK;
}

PID_Float PID_Metrics_MeanAbsError(const PID_LoopMetrics *m)
{
    PID_Float r = PID_ZERO;
    if ((m != NULL) && (m->total_time > PID_ZERO)) {
        r = m->iae / m->total_time;
    }
    return r;
}

PID_Float PID_Metrics_SaturationDuty(const PID_LoopMetrics *m)
{
    PID_Float r = PID_ZERO;
    if ((m != NULL) && (m->total_time > PID_ZERO)) {
        r = m->sat_time / m->total_time;
    }
    return r;
}

PID_Float PID_Metrics_OscillationRate(const PID_LoopMetrics *m)
{
    PID_Float r = PID_ZERO;
    if ((m != NULL) && (m->total_time > PID_ZERO)) {
        r = (PID_Float)m->sign_changes / m->total_time;
    }
    return r;
}

#endif /* PIDX_ENABLE_DIAGNOSTICS */
