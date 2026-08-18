/* PHASE 13 - diagnostics: telemetry ring + loop metrics.
 *
 * The metric assertions are checked against closed-form values, not against
 * whatever the code happens to emit.
 */
#include <stdio.h>
#include <math.h>
#include "pidx/pid.h"
#include "pidx/pid_diag.h"

static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)
#define NEAR(a,b,t) (fabs((double)(a)-(double)(b)) <= (t))

int main(void)
{
    const double dt = 0.01;

    /* ---- 1. ring buffer: FIFO order, exact drop accounting -------------- */
    {
        PID_Handle h; PID_Config c;
        PID_TelemetryRecord store[8];
        PID_Telemetry tel;
        PID_TelemetryRecord rec;
        int got = 0;

        PID_ConfigDefault(&c);
        c.core.kp = 1.0f;
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&h, &c);
        CK(PID_Telemetry_Init(&tel, store, 8U) == PID_OK, "telemetry init");
        CK(PID_Telemetry_Attach(&h, &tel) == PID_OK, "telemetry attach");
        PID_SetSetpoint(&h, 1.0f);

        for (int i = 0; i < 20; i++) { (void)PID_Update(&h, 0.0f); }

        uint16_t cnt  = PID_Telemetry_Count(&tel);
        uint16_t drop = PID_Telemetry_Dropped(&tel);
        printf("  20 updates into an 8-slot ring: readable=%u dropped=%u\n",
               (unsigned)cnt, (unsigned)drop);
        /* A ring of capacity 8 can only hold capacity-1 = 7 unread records. */
        CK(cnt == 7U, "7 records readable (capacity-1)");
        CK((unsigned)cnt + (unsigned)drop == 20U,
           "readable + dropped must account for every update");

        /* Policy is drop-NEWEST: tail stays consumer-owned, so what survives
         * is the OLDEST run, seq 0..6, contiguous. The 13 lost samples show up
         * as a gap between the last seq read and the producer's seq counter,
         * which is how the consumer detects loss without shared writes. */
        uint16_t prev_seq = 0; int first = 1, contiguous = 1;
        while (PID_Telemetry_Read(&tel, &rec) == PID_OK) {
            if (first) { CK(rec.seq == 0U, "first surviving record is seq 0"); }
            else if ((uint16_t)(rec.seq - prev_seq) != 1U) { contiguous = 0; }
            prev_seq = rec.seq; first = 0; got++;
        }
        CK(got == 7, "read exactly the reported count");
        CK(contiguous == 1, "surviving records are contiguous");
        CK(prev_seq == 6U, "last surviving record is seq 6 (newest were dropped)");
        CK(PID_Telemetry_Count(&tel) == 0U, "ring empty after draining");
        CK(PID_Telemetry_Read(&tel, &rec) != PID_OK, "read on empty ring fails");
    }

    /* ---- 2. telemetry content matches the handle state ------------------ */
    {
        PID_Handle h; PID_Config c;
        PID_TelemetryRecord store[16];
        PID_Telemetry tel;
        PID_TelemetryRecord rec;

        PID_ConfigDefault(&c);
        c.core.kp = 2.0f; c.core.ki = 1.0f; c.core.kd = 0.0f;
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&h, &c);
        PID_Telemetry_Init(&tel, store, 16U);
        PID_Telemetry_Attach(&h, &tel);
        PID_SetSetpoint(&h, 1.0f);

        PID_Float u = PID_Update(&h, 0.25f);
        CK(PID_Telemetry_Read(&tel, &rec) == PID_OK, "one record available");
        /* e = 1 - 0.25 = 0.75 ; P = 2*0.75 = 1.5 ; I = 1*0.01*0.75 = 0.0075 */
        CK(NEAR(rec.setpoint, 1.0, 1e-6), "setpoint recorded");
        CK(NEAR(rec.measurement, 0.25, 1e-6), "measurement recorded");
        CK(NEAR(rec.p_term, 1.5, 1e-5), "P term = Kp*e = 1.5");
        CK(NEAR(rec.i_term, 0.0075, 1e-6), "I term = Ki*dt*e = 0.0075");
        CK(NEAR(rec.output, u, 1e-6), "recorded output equals returned output");
        printf("  record: sp=%.3f y=%.3f P=%.4f I=%.6f u=%.4f\n",
               (double)rec.setpoint, (double)rec.measurement,
               (double)rec.p_term, (double)rec.i_term, (double)rec.output);
    }

    /* ---- 3. metrics against a closed-form error signal ------------------ */
    {
        /* Drive the controller in MANUAL so the "plant" cannot interfere:
         * the error is then exactly what we impose. With a constant error of
         * 0.5 held for N samples of dt:
         *    IAE  = 0.5 * N*dt
         *    ISE  = 0.25 * N*dt
         *    ITAE = sum over k of (k*dt) * 0.5 * dt   [k = 0..N-1]
         * The last one uses the elapsed time BEFORE the sample, matching the
         * rectangle rule the implementation documents. */
        PID_Handle h; PID_Config c;
        PID_LoopMetrics m;
        const int N = 1000;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f; c.core.ki = 0.0f; c.core.kd = 0.0f;
        c.core.sample_time = (PID_Float)dt;
        c.core.mode = PID_MODE_MANUAL;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);
        PID_Metrics_Reset(&m);

        double exp_itae = 0.0;
        for (int k = 0; k < N; k++) {
            (void)PID_Update(&h, 0.5f);       /* error is always 0.5 */
            PID_Metrics_Update(&m, &h);
            exp_itae += ((double)k * dt) * 0.5 * dt;
        }
        double exp_iae = 0.5 * (double)N * dt;
        double exp_ise = 0.25 * (double)N * dt;

        printf("  IAE  %.6f (expect %.6f)\n", (double)m.iae,  exp_iae);
        printf("  ISE  %.6f (expect %.6f)\n", (double)m.ise,  exp_ise);
        printf("  ITAE %.6f (expect %.6f)\n", (double)m.itae, exp_itae);
        CK(NEAR(m.iae,  exp_iae,  1e-3), "IAE matches closed form");
        CK(NEAR(m.ise,  exp_ise,  1e-3), "ISE matches closed form");
        CK(NEAR(m.itae, exp_itae, 5e-2), "ITAE matches closed form");
        CK(NEAR(PID_Metrics_MeanAbsError(&m), 0.5, 1e-4), "mean |e| = 0.5");
        CK(NEAR(m.abs_error_max, 0.5, 1e-6), "peak |e| = 0.5");
        CK(m.samples == (uint32_t)N, "sample count");
        CK(NEAR(PID_Metrics_SaturationDuty(&m), 0.0, 1e-9),
           "no saturation without limits");
    }

    /* ---- 4. saturation duty is a real fraction -------------------------- */
    {
        PID_Handle h; PID_Config c;
        PID_LoopMetrics m;
        PID_ConfigDefault(&c);
        c.core.kp = 100.0f; c.core.ki = 0.0f;
        c.core.sample_time = (PID_Float)dt;
        c.limits.use_output_limits = true;
        c.limits.output_min = -1.0f; c.limits.output_max = 1.0f;
        PID_Init(&h, &c);
        PID_Metrics_Reset(&m);
        /* 100 saturated samples then 100 unsaturated: duty must be 0.5 */
        PID_SetSetpoint(&h, 10.0f);
        for (int i = 0; i < 100; i++) { (void)PID_Update(&h, 0.0f); PID_Metrics_Update(&m,&h); }
        PID_SetSetpoint(&h, 0.0f);
        for (int i = 0; i < 100; i++) { (void)PID_Update(&h, 0.0f); PID_Metrics_Update(&m,&h); }
        double duty = (double)PID_Metrics_SaturationDuty(&m);
        printf("  saturation duty = %.4f (expect 0.50)\n", duty);
        CK(NEAR(duty, 0.5, 1e-6), "duty is exactly half");
    }

    /* ---- 5. oscillation rate counts sign changes per second ------------- */
    {
        PID_Handle h; PID_Config c;
        PID_LoopMetrics m;
        PID_ConfigDefault(&c);
        c.core.kp = 1.0f; c.core.ki = 0.0f;
        c.core.sample_time = (PID_Float)dt;
        c.core.mode = PID_MODE_MANUAL;
        PID_Init(&h, &c);
        PID_Metrics_Reset(&m);
        PID_SetSetpoint(&h, 0.0f);
        /* Alternate the measurement so the error flips sign every sample.
         * 200 samples -> 199 sign changes over 2.0 s. */
        for (int i = 0; i < 200; i++) {
            (void)PID_Update(&h, (i & 1) ? 1.0f : -1.0f);
            PID_Metrics_Update(&m, &h);
        }
        double rate = (double)PID_Metrics_OscillationRate(&m);
        printf("  sign changes=%u over %.2fs -> %.2f/s\n",
               (unsigned)m.sign_changes, (double)m.total_time, rate);
        CK(m.sign_changes == 199U, "199 sign changes in 200 alternating samples");
        CK(NEAR(rate, 199.0 / 2.0, 1.0), "oscillation rate ~99.5/s");
    }

    /* ---- 6. null-safety of the diagnostics API -------------------------- */
    {
        PID_TelemetryRecord rec;
        PID_LoopMetrics m;
        CK(PID_Telemetry_Init(NULL, NULL, 8U) == PID_ERR_NULL, "Init(NULL)");
        CK(PID_Telemetry_Read(NULL, &rec) == PID_ERR_NULL, "Read(NULL)");
        CK(PID_Metrics_Reset(NULL) == PID_ERR_NULL, "Metrics_Reset(NULL)");
        CK(PID_Metrics_Update(&m, NULL) == PID_ERR_NULL, "Metrics_Update(h=NULL)");
        /* Capacity must be a power of two - the mask arithmetic requires it. */
        PID_Telemetry tel; PID_TelemetryRecord store[8];
        CK(PID_Telemetry_Init(&tel, store, 7U) != PID_OK,
           "non-power-of-two capacity rejected");
    }

    printf("\ndiagnostics: %d passed, %d failed\n", pass, bad);
    return bad != 0;
}
