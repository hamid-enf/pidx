/**
 * ====================================================================
 * مثال 8 — کنترلر PID در تسک FreeRTOS + Feedforward
 * ====================================================================
 *
 * كاربرد:
 *   پروژه های FreeRTOS مثل STM32 + FreeRTOS (CubeMX)
 *   - کنترلر در یك تسك مجزا با priority بالا اجرا می شود
 *   - vTaskDelayUntil برای زمانبندی دقیق (بدون دریفت)
 *   - Feedforward برای تخمین خروجی پایه
 *
 * مفاهیم جدید:
 *   1. vTaskDelayUntil: deadline مطلق — دریفت جمع نمی شود
 *   2. Feedforward: u_ff = fn(setpoint, measurement)
 *      PID فقط خطای باقی مانده را اصلاح می کند
 *   3. Bumpless mode switching: MANUAL/AUTO بدون پرش
 *
 * سناریو:
 *   موتور DC به 100 rad/s فرمان می گیرد
 *   بدون FF: PID باید همه ولتاژ را از انتگرال تامین کند (کند)
 *   با FF: PID فقط خطا را اصلاح می کند (سریع)
 *
 * پیاده سازی STM32:
 *   این فایل را به پروژه اضافه کن و در main.c:
 *     example08_init(1);   // 1 = با Feedforward
 *     MX_FREERTOS_Init();  // تسک را راه اندازی کن
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

/* FreeRTOS (در پروژه واقعی از CubeMX فعال می شود) */
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#include "pidx/pid.h"

/* ===== پیکربندی ===== */
#define CTRL_DT             0.01f       /* 10ms = 100Hz */
#define CTRL_KP             0.30f
#define CTRL_KI             4.0f
#define CTRL_KD             0.002f
#define CTRL_TF             0.005f
#define CTRL_SETPOINT       100.0f      /* rad/s */

/* ===== موتور DC ===== */
#define MOTOR_R     1.0f
#define MOTOR_L     0.0005f
#define MOTOR_KE    0.05f
#define MOTOR_KT    0.05f
#define MOTOR_J     0.0001f
#define MOTOR_B     0.002f
#define MOTOR_COULOMB 0.003f
#define V_MAX       24.0f

/* ===== وضعیت ===== */
static struct {
    PID_Handle pid;

    float motor_i, motor_w, motor_theta;
    float output;
    uint32_t tick;
    float ff_value;

} ctx;

/* ================================================================
 * مدل موتور با زیرگام
 * ================================================================ */
static void motor_step(float v, float dt)
{
    float tau_elec = MOTOR_L / MOTOR_R;
    int n = (int)(dt / (0.1f * tau_elec)) + 1;
    float dt_sub = dt / (float)n;

    for (int i = 0; i < n; i++) {
        float di_dt = (v - MOTOR_R * ctx.motor_i - MOTOR_KE * ctx.motor_w) / MOTOR_L;
        ctx.motor_i += di_dt * dt_sub;

        float coulomb = (ctx.motor_w > 0.01f) ? MOTOR_COULOMB :
                        (ctx.motor_w < -0.01f) ? -MOTOR_COULOMB : 0.0f;
        float dw_dt = (MOTOR_KT * ctx.motor_i - MOTOR_B * ctx.motor_w - coulomb) / MOTOR_J;
        ctx.motor_w += dw_dt * dt_sub;
        ctx.motor_theta += ctx.motor_w * dt_sub;
    }
}

static float read_sensor(void)
{
    /* انکودر: سرعت از مشتق موقعیت */
    static float last_theta = 0.0f;
    float speed = (ctx.motor_theta - last_theta) / CTRL_DT;
    last_theta = ctx.motor_theta;
    return speed;
}

/* ================================================================
 * Feedforward Function
 *
 * تخمین ولتاژ لازم برای نگه داشتن سرعت مشخص:
 *   V_ff = R*I_ss + Ke*w
 *        = R*(B*w + T_coulomb)/Kt + Ke*w
 *        = (R*B/Kt + Ke)*w + R*T_coulomb/Kt
 *
 * با پارامترهای موتور:
 *   V_ff = 0.09*w + 0.06
 *   در w=100: V_ff = 9.06V
 * ================================================================ */
static float motor_feedforward(float setpoint, float measurement, void *ctx_ptr)
{
    (void)ctx_ptr;
    (void)measurement;

    /* مدل معکوس موتور در حالت پایدار */
    float v_ff = (MOTOR_R * MOTOR_B / MOTOR_KT + MOTOR_KE) * setpoint
               + (MOTOR_R * MOTOR_COULOMB / MOTOR_KT);
    return v_ff;
}

/* ================================================================
 * راه اندازی
 *
 * use_ff = 1: با Feedforward
 * use_ff = 0: بدون Feedforward
 * ================================================================ */
void example08_init(int use_ff)
{
    PID_Config cfg;

    ctx.motor_i = 0.0f;
    ctx.motor_w = 0.0f;
    ctx.motor_theta = 0.0f;
    ctx.tick = 0;
    ctx.ff_value = 0.0f;

    PID_ConfigDefault(&cfg);
    cfg.core.kp = CTRL_KP;
    cfg.core.ki = CTRL_KI;
    cfg.core.kd = CTRL_KD;
    cfg.core.sample_time = CTRL_DT;

    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = -V_MAX;
    cfg.limits.output_max =  V_MAX;

    cfg.integral.mode = PID_AW_BACK_CALCULATION;
    cfg.integral.kt   = 10.0f;

    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf = CTRL_TF;

    /* ★ Feedforward ★ */
    cfg.feedforward.enabled = (use_ff != 0);
    cfg.feedforward.fn = use_ff ? motor_feedforward : NULL;
    cfg.feedforward.gain = 1.0f;

    PID_Init(&ctx.pid, &cfg);
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

    printf("\r\n========================================\r\n");
    printf("مثال 8 — FreeRTOS + Feedforward %s\r\n",
           use_ff ? "(فعال)" : "(غیرفعال)");
    printf("========================================\r\n");
    printf("%4s | %8s | %8s | %8s | %8s\r\n",
           "زمان", "هدف", "سرعت", "خروجی", "FF");
    printf("----------------------------------------\r\n");
}

/* ================================================================
 * تیک کنترلر — از تسک FreeRTOS صدا زده می شود
 *
 * در main.c بعد از MX_FREERTOS_Init():
 *
 *   void ControlTask(void *pvParams) {
 *       TickType_t last = xTaskGetTickCount();
 *       for (;;) {
 *           example08_tick();
 *           vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
 *       }
 *   }
 * ================================================================ */
void example08_tick(void)
{
    ctx.tick++;

    /* 1. به روز رسانی موتور */
    motor_step(ctx.output, CTRL_DT);

    /* 2. سنسور */
    float meas = read_sensor();

    /* 3. PID — اگر FF فعال باشد:
     *    u_total = u_PID + ff_fn(sp, meas)
     *    PID فقط خطای باقی مانده را اصلاح می کند */
    float control = PID_Update(&ctx.pid, meas);
    ctx.output = control;

    /* 4. نمایش FF */
    #if PIDX_ENABLE_DIAGNOSTICS
    PID_Status st;
    PID_GetStatus(&ctx.pid, &st);
    ctx.ff_value = st.ff_term;
    #endif

    /* 5. گزارش */
    if (ctx.tick % 100 == 0) {
        printf("%4.1f | %8.1f | %8.1f | %8.2f | %8.2f\r\n",
               (float)ctx.tick * CTRL_DT,
               CTRL_SETPOINT, meas, control, ctx.ff_value);
    }

    /* 6. Bumpless mode switch — هر 2 ثانیه */
    if (ctx.tick % 200 == 0 && ctx.tick > 0) {
        static int toggle = 0;
        toggle = !toggle;
        if (toggle) {
            PID_SetMode(&ctx.pid, PID_MODE_MANUAL);
            PID_SetManualOutput(&ctx.pid, 0.0f);
            printf(">>> MANUAL <<<\r\n");
        } else {
            PID_SetMode(&ctx.pid, PID_MODE_AUTOMATIC);
            printf(">>> AUTOMATIC (bumpless) <<<\r\n");
        }
    }

    /* پایان */
    if (ctx.tick >= 500) {
        printf("----------------------------------------\r\n");
        printf("OK. final speed: %.1f rad/s\r\n", ctx.motor_w);
        printf("========================================\r\n\r\n");
    }
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    /* تست بدون FF */
    example08_init(0);
    for (int i = 0; i < 500; i++) example08_tick();

    /* تست با FF */
    example08_init(1);
    for (int i = 0; i < 500; i++) example08_tick();

    return 0;
}
#endif
