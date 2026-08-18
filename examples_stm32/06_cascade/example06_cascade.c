/**
 * Example 6 - Cascade 3-level: Position <- Velocity <- Current
 */
#include <stdio.h>
#include <math.h>
#include "pidx/pid.h"
#include "pidx/pid_cascade.h"

#define DT          (1.0f / 20000.0f)
#define V_MAX       24.0f
#define I_MAX       8.0f

#define L_H     0.5e-3f
#define R_OHM   1.0f
#define KE      0.05f
#define KT      0.05f
#define J       1.0e-4f
#define B       2.0e-3f

static PID_Handle  pos_loop, vel_loop, cur_loop;
static PID_Handle *loops[3];
static PID_Cascade cascade;
static float motor_th, motor_w, motor_i;

static void motor_init(void) {
    motor_th = motor_w = motor_i = 0.0f;
}

static void motor_step(float v, float dt, float load) {
    float tau_elec = L_H / R_OHM;
    int n = (int)(dt / (0.1f * tau_elec)) + 1;
    float dt_sub = dt / (float)n;
    for (int i = 0; i < n; i++) {
        float di_dt = (v - R_OHM * motor_i - KE * motor_w) / L_H;
        motor_i += di_dt * dt_sub;
        float coulomb = (motor_w > 0.01f) ? 0.003f :
                        (motor_w < -0.01f) ? -0.003f : 0.0f;
        float dw_dt = (KT * motor_i - B * motor_w - load - coulomb) / J;
        motor_w += dw_dt * dt_sub;
        motor_th += motor_w * dt_sub;
    }
}

void example06_init(void) {
    PID_Config c;
    motor_init();

    // Current loop (inner, 20kHz)
    PID_ConfigDefault(&c);
    c.core.kp = 2.0f * 3.14159265f * 2000.0f * L_H;
    c.core.ki = 2.0f * 3.14159265f * 2000.0f * R_OHM;
    c.core.sample_time = DT;
    c.limits.use_output_limits = true;
    c.limits.output_min = -V_MAX;
    c.limits.output_max =  V_MAX;
    c.integral.mode = PID_AW_BACK_CALCULATION;
    c.integral.kt   = 2000.0f;
    PID_Init(&cur_loop, &c);

    // Velocity loop (middle, 2kHz)
    {
        float wv = 2.0f * 3.14159265f * 200.0f;
        PID_ConfigDefault(&c);
        c.core.kp = wv * J / KT;
        c.core.ki = (wv * B / KT) + (0.1f * wv * wv * J / KT);
        c.core.sample_time = DT * 10.0f;
        c.limits.use_output_limits = true;
        c.limits.output_min = -I_MAX;
        c.limits.output_max =  I_MAX;
        c.integral.mode = PID_AW_BACK_CALCULATION;
        c.integral.kt   = wv;
        PID_Init(&vel_loop, &c);
    }

    // Position loop (outer, 500Hz)
    {
        float wp = 2.0f * 3.14159265f * 40.0f;
        PID_ConfigDefault(&c);
        c.core.kp = wp;
        c.core.ki = 20.0f;
        c.core.kd = 0.0f;
        c.core.sample_time = DT * 40.0f;
        c.limits.use_output_limits = true;
        c.limits.output_min = -50.0f;
        c.limits.output_max =  50.0f;
        c.limits.use_integral_limits = true;
        c.limits.integral_min = -5.0f;
        c.limits.integral_max =  5.0f;
        c.integral.mode = PID_AW_BACK_CALCULATION;
        c.integral.kt   = wp;
        PID_Init(&pos_loop, &c);
    }

    // Cascade
    loops[0] = &pos_loop;
    loops[1] = &vel_loop;
    loops[2] = &cur_loop;
    PID_Cascade_Init(&cascade, loops, 3);
    PID_Cascade_ConfigLevel(&cascade, 0, 40, -50.0f, 50.0f);
    PID_Cascade_ConfigLevel(&cascade, 1, 10, -I_MAX, I_MAX);
    PID_Cascade_ConfigLevel(&cascade, 2,  1, 0.0f, 0.0f);
    PID_Cascade_SetAntiWindup(&cascade, PID_CASCADE_AW_BACK_CALC, 10.0f);

    printf("Example 6 - Cascade 3-level initialized\r\n");
    printf("Position(500) <- Velocity(2k) <- Current(20k)\r\n");
    printf("Current limit: %dA\r\n", (int)I_MAX);
}

static int cascade_tick = 0;

void example06_tick(void) {
    static float load_torque = 0.0f;
    cascade_tick++;

    if (cascade_tick == 10000) {
        load_torque = 0.025f;
        printf(">>> Load step 25mNm at t=0.5s <<<\r\n");
    }

    float meas[3] = { motor_th, motor_w, motor_i };
    float v = PID_Cascade_Update(&cascade, meas, 3.0f, DT);
    motor_step(v, DT, load_torque);

    /* Report more frequently: every 1000 ticks = 50ms */
    if (cascade_tick % 1000 == 0 || cascade_tick == 1) {
        printf("%5.3f | %8.4f | %8.1f | %8.2f\r\n",
               (float)cascade_tick * DT, motor_th, motor_w, v);
    }

    if (cascade_tick >= 30000) {
        printf("------------------------------------\r\n");
        printf("Done. Final position: %.4f rad (target: 3.0)\r\n", motor_th);
        printf("Error: %.4f rad\r\n", 3.0f - motor_th);
        printf("====================================\r\n\r\n");
    }
}


#ifdef TEST_ON_PC
int main(void)
{
    example06_init();
    for (int i = 0; i < 60000; i++) example06_tick();
    return 0;
}
#endif
