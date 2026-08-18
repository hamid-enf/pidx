/**
 * ====================================================================
 * مثال 9 — TIM ISR + Diagnostics + Telemetry
 * ====================================================================
 *
 * كاربرد:
 *   پیاده سازی واقعی روی STM32:
 *   - TIM3 ISR → PID → PWM (سریع، غیرمسدود)
 *   - main loop → UART → PC (کند، مسدود)
 *   - Telemetry: ثبت داده از ISR بدون مسدود کردن
 *
 * مفاهیم جدید:
 *   1. PID_GetStatus: دیاگنوستیک کامل هر سیکل
 *   2. Telemetry SPSC ring: ISR می نویسد، main loop می خواند
 *   3. PID_FLAG_*: اشباع، خطا، انتگرالگیر
 *
 * سناریو:
 *   پلنت مرتبه اول به 50 هدف می رسد
 *   در t=0.8s سنسور خراب می شود (صفر می دهد)
 *   در t=1.0s سنسور برمی گردد
 *   Telemetry و Status در main loop خوانده می شود
 *
 * پیاده سازی STM32:
 *   TIM3 با 1kHz → example09_isr_tick()
 *   while(1) → example09_main_loop() هر 1 ثانیه
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#if PIDX_ENABLE_DIAGNOSTICS
#include "pidx/pid_diag.h"
#endif

/* ===== پیکربندی ===== */
#define CTRL_DT             0.001f       /* 1kHz */
#define CTRL_KP             1.0f
#define CTRL_KI             0.5f
#define CTRL_KD             0.05f
#define CTRL_TF             0.01f
#define CTRL_SETPOINT       50.0f

#define ACTUATOR_MIN        -100.0f
#define ACTUATOR_MAX         100.0f

/* ===== پلنت FOPDT بدون باگ ===== */
#define PLANT_BIAS          10.0f
#define PLANT_TAU           0.2f
#define PLANT_GAIN          1.0f

/* ===== Telemetry ===== */
#define TELEM_CAPACITY      64
static PID_TelemetryRecord telem_storage[TELEM_CAPACITY];
static PID_Telemetry       telem;

/* ===== وضعیت ===== */
static struct {
    PID_Handle pid;

    float plant_y;      /* خروجی واقعی پلنت */
    float measurement;  /* خروجی سنسور */
    float output;       /* خروجی کنترلر */

    uint32_t tick;
    int sensor_faulted;

} ctx;

/* ================================================================
 * پلنت مرتبه اول — steady state: y = K*u + bias
 * ================================================================ */
static float plant_update(float u, float dt)
{
    float a = expf(-dt / PLANT_TAU);
    ctx.plant_y = a * ctx.plant_y + (1.0f - a) * (PLANT_GAIN * u + PLANT_BIAS);
    return ctx.plant_y;
}

/* ================================================================
 * راه اندازی
 * ================================================================ */
void example09_init(void)
{
    PID_Config cfg;

    ctx.plant_y = PLANT_BIAS;
    ctx.tick = 0;
    ctx.sensor_faulted = 0;

    PID_ConfigDefault(&cfg);
    cfg.core.kp = CTRL_KP;
    cfg.core.ki = CTRL_KI;
    cfg.core.kd = CTRL_KD;
    cfg.core.sample_time = CTRL_DT;

    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = ACTUATOR_MIN;
    cfg.limits.output_max = ACTUATOR_MAX;

    cfg.integral.mode = PID_AW_CLAMP;

    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf = CTRL_TF;

    PID_Init(&ctx.pid, &cfg);
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);

    /* ★ مهم: راه‌اندازی اولیه مشتق ★
     *
     * بعد از PID_Init، حافظه مشتق (d_prev_in) صفر است.
     * اگر مستقیم PID_Update با اندازه‌گیری=10 بدهیم،
     * مشتق یک پرش 0→10 می‌بیند و یک ضربه بزرگ -45 ولتی می‌زند!
     *
     * راه‌حل: چند نمونه با اندازه‌گیری فعلی بده تا
     * فیلتر مشتق به مقدار واقعی برسد.
     */
    PID_SetSetpoint(&ctx.pid, PLANT_BIAS);   /* اول با مقدار فعلی */
    for (int i = 0; i < 50; i++)
        PID_Update(&ctx.pid, PLANT_BIAS);    /* 50 تیک prime */
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT);/* بعد هدف واقعی */

    /* ★ Telemetry ★
     *
     * بافر حلقوی SPSC (Single-Producer Single-Consumer):
     *   - ISR (تولیدکننده) هر تیک یک رکورد می نویسد
     *   - main loop (مصرف‌کننده) هر چند وقت می خواند
     *   - lock-free: بدون نیاز به critical section
     *
     * ساختار: PID_TelemetryRecord (32 بایت)
     * ظرفیت: 64 رکورد — باید توان 2 باشد
     */
    /* 1. Init بافر: ذخیره‌سازی + ظرفیت */
    PID_Telemetry_Init(&telem, telem_storage, TELEM_CAPACITY);

    /* 2. Attach به کنترلر: هر PID_Update خودکار push می‌کند */
    PID_Telemetry_Attach(&ctx.pid, &telem);

    printf("\r\n============================================\r\n");
    printf("مثال 9 — TIM ISR + Telemetry\r\n");
    printf("نرخ: %d Hz, ظرفیت: %d رکورد\r\n",
           (int)(1.0f / CTRL_DT), TELEM_CAPACITY);
    printf("============================================\r\n");
}

/* ================================================================
 * ISR تیک — در TIM3 ISR با 1kHz صدا زده می شود
 *
 * ⚠️ این تابع در ISR اجرا می شود:
 *    - هیچ چیز مسدودکننده ننویسید
 *    - printf فقط برای شبیه‌سازی است (روی STM32 حذف کنید)
 * ================================================================ */
void example09_isr_tick(void)
{
    ctx.tick++;

    /* ---- 1. شبیه‌سازی خطای سنسور ----
     *   در t=0.8s: سنسور یخ می‌زند روی 0
     *   در t=1.0s: سنسور برمی‌گردد */
    float sensor_value;
    if (ctx.tick >= 800 && ctx.tick < 1000) {
        sensor_value = 0.0f;        /* سنسور خراب */
        ctx.sensor_faulted = 1;
    } else {
        sensor_value = ctx.plant_y; /* سنسور سالم */
        ctx.sensor_faulted = 0;
    }

    /* ---- 2. پلنت با خروجی قبلی ---- */
    plant_update(ctx.output, CTRL_DT);

    /* ---- 3. کنترل ---- */
    ctx.measurement = sensor_value;
    ctx.output = PID_Update(&ctx.pid, sensor_value);

    /* ---- 4. Telemetry: خودکار توسط PID_Update انجام می‌شود
     *   (چون PID_Telemetry_Attach کرده‌ایم) */
}

/* ================================================================
 * Main Loop — در while(1) هر 1 ثانیه صدا زده می شود
 * ================================================================ */
void example09_main_loop(void)
{
    /* ---- 1. خواندن Telemetry ---- */
    #if PIDX_ENABLE_DIAGNOSTICS && PIDX_ENABLE_TELEMETRY
    uint16_t n = PID_Telemetry_Count(&telem);
    if (n > 0) {
        /* همه رکوردها را بخوان و آخرین 5 تای آن را نشان بده
         * (بافر ring FIFO است — قدیمی‌ترین اول خوانده می‌شود) */
        PID_TelemetryRecord rec;
        PID_TelemetryRecord last5[5];
        uint16_t last5_n = 0;
        uint16_t total = 0;

        while (PID_Telemetry_Read(&telem, &rec) == PID_OK) {
            /* آخرین 5 را نگه می‌داریم */
            if (last5_n < 5) {
                last5[last5_n++] = rec;
            } else {
                for (int i = 0; i < 4; i++) last5[i] = last5[i+1];
                last5[4] = rec;
            }
            total++;
        }

        printf("\r\n--- Telemetry (آخرین %u از %u رکورد) ---\r\n",
               (unsigned)last5_n, (unsigned)total);
        for (uint16_t i = 0; i < last5_n; i++) {
            printf("  [%u] SP=%.1f PV=%.1f P=%.2f I=%.2f D=%.2f OUT=%.2f\r\n",
                   (unsigned)last5[i].seq,
                   last5[i].setpoint,
                   last5[i].measurement,
                   last5[i].p_term,
                   last5[i].i_term,
                   last5[i].d_term,
                   last5[i].output);
        }
    }
    #endif

    /* ---- 2. خواندن Status ---- */
    #if PIDX_ENABLE_DIAGNOSTICS
    PID_Status st;
    PID_GetStatus(&ctx.pid, &st);
    printf("Status: update#=%lu sat#=%lu | flags=0x%04x | err=%s\r\n",
           (unsigned long)st.update_count,
           (unsigned long)st.saturation_count,
           (unsigned)st.flags,
           PID_StatusToString(st.last_error));
    #endif

    /* ---- 3. Flagها ---- */
    uint16_t flags = PID_GetFlags(&ctx.pid);
    if (flags & PID_FLAG_FAULT)
        printf("  !! FAULT فعال — خروجی fail-safe\r\n");
    if (flags & PID_FLAG_SATURATED_HIGH)
        printf("  !! اشباع بالا\r\n");
    if (flags & PID_FLAG_SATURATED_LOW)
        printf("  !! اشباع پایین\r\n");
    if (flags & PID_FLAG_SENSOR_INVALID)
        printf("  !! سنسور نامعتبر\r\n");
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    example09_init();

    /* شبیه‌سازی 1.5 ثانیه */
    for (int i = 0; i < 1500; i++) {
        example09_isr_tick();

        /* main loop هر 0.5 ثانیه */
        if (i % 500 == 0 && i > 0) {
            printf("\r\n--- t=%.3fs (main loop) ---\r\n",
                   (float)i * CTRL_DT);
            example09_main_loop();
        }
    }
    printf("\r\nOK. مثال 9 تمام شد.\r\n");
    return 0;
}
#endif
