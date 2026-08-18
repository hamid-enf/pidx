/* Core regression smoke test - reproduces the reference numbers recorded when
 * the core was first verified, so later phases cannot silently change them.
 * Plant: first order, K = 1, tau = 1, dt = 0.05. */
#include <stdio.h>
#include <math.h>
#include "pidx/pid.h"

static double plant(double y, double u, double dt)
{
    return y + dt * ((1.0 * u - y) / 1.0);
}

int main(void)
{
    const double dt = 0.05;
    int bad = 0;

    /* --- 1. unit step, default gains ------------------------------------ */
    {
        PID_Handle h; PID_Config c;
        double y = 0.0;
        PID_ConfigDefault(&c);
        c.core.kp = 2.0f; c.core.ki = 1.0f; c.core.kd = 0.1f;
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);
        for (int i = 0; i < 2000; i++) {
            y = plant(y, (double)PID_Update(&h, (PID_Float)y), dt);
        }
        printf("step   : y=%.6f u=%.4f I=%.4f\n",
               y, (double)PID_GetOutput(&h), (double)PID_GetIntegrator(&h));
        if (fabs(y - 1.0) > 1e-3) { bad++; puts("  FAIL: setpoint not reached"); }
    }

    /* --- 2. output limits + windup recovery ----------------------------- */
    {
        PID_Handle h; PID_Config c;
        double y = 0.0;
        PID_ConfigDefault(&c);
        c.core.kp = 2.0f; c.core.ki = 1.0f; c.core.kd = 0.1f;
        c.core.sample_time = (PID_Float)dt;
        c.limits.use_output_limits = true;
        c.limits.output_min = -10.0f; c.limits.output_max = 10.0f;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 100.0f);
        for (int i = 0; i < 4000; i++) {
            y = plant(y, (double)PID_Update(&h, (PID_Float)y), dt);
        }
        printf("sat    : y=%.4f I=%.4f\n", y, (double)PID_GetIntegrator(&h));
        if (fabs(y - 10.0) > 1e-2) { bad++; puts("  FAIL: not at limit"); }
        PID_SetSetpoint(&h, 5.0f);
        for (int i = 0; i < 4000; i++) {
            y = plant(y, (double)PID_Update(&h, (PID_Float)y), dt);
        }
        printf("recover: y=%.6f\n", y);
        if (fabs(y - 5.0) > 1e-2) { bad++; puts("  FAIL: no windup recovery"); }
    }

    /* --- 3. NaN is rejected and latched --------------------------------- */
    {
        PID_Handle h; PID_Config c;
        double y = 0.0;
        PID_ConfigDefault(&c);
        c.core.kp = 2.0f; c.core.ki = 1.0f;
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&h, &c);
        PID_SetSetpoint(&h, 1.0f);
        for (int i = 0; i < 200; i++) {
            y = plant(y, (double)PID_Update(&h, (PID_Float)y), dt);
        }
        PID_Float held = PID_GetOutput(&h);
        (void)PID_Update(&h, (PID_Float)(0.0 / 0.0));
        PID_StatusCode code = PID_OK;
        (void)PID_GetLastError(&h, &code);
        printf("NaN    : output held=%.4f -> %.4f, sticky code=%d\n",
               (double)held, (double)PID_GetOutput(&h), (int)code);
        if (PID_GetOutput(&h) != held) { bad++; puts("  FAIL: output moved on NaN"); }
        if (code != PID_ERR_NAN_INPUT) { bad++; puts("  FAIL: NaN not recorded"); }
    }

    /* --- 4. UpdateFast must agree bit-for-bit with Update ---------------- */
    {
        PID_Handle a, b; PID_Config c;
        double ya = 0.0, yb = 0.0;
        int mism = 0;
        PID_ConfigDefault(&c);
        c.core.kp = 2.0f; c.core.ki = 1.0f; c.core.kd = 0.1f;
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&a, &c); PID_Init(&b, &c);
        PID_SetSetpoint(&a, 1.0f); PID_SetSetpoint(&b, 1.0f);
        for (int i = 0; i < 500; i++) {
            PID_Float ua = PID_Update(&a, (PID_Float)ya);
            PID_Float ub = PID_UpdateFast(&b, (PID_Float)yb);
            if (ua != ub) { mism++; }
            ya = plant(ya, (double)ua, dt);
            yb = plant(yb, (double)ub, dt);
        }
        printf("fast   : %d mismatches over 500 steps\n", mism);
        if (mism != 0) { bad++; puts("  FAIL: UpdateFast diverges"); }
    }

    /* --- 5. manual output is readable before the next update ------------- */
    {
        PID_Handle h; PID_Config c;
        PID_ConfigDefault(&c);
        c.core.sample_time = (PID_Float)dt;
        PID_Init(&h, &c);
        PID_SetMode(&h, PID_MODE_MANUAL);
        PID_SetManualOutput(&h, 0.5f);
        printf("manual : GetManualOutput=%.4f GetOutput=%.4f\n",
               (double)PID_GetManualOutput(&h), (double)PID_GetOutput(&h));
        if (PID_GetManualOutput(&h) != 0.5f) {
            bad++; puts("  FAIL: manual output not readable");
        }
    }

    printf("\n%s\n", bad ? "*** REGRESSIONS ***" : "core smoke: all checks passed");
    return bad != 0;
}
