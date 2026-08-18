/**
 * ====================================================================
 * مثال 7 — Auto-Tune (تنظیم خودکار PID)
 * ====================================================================
 *
 * كاربرد:
 *   - وقتی نمی دونی gains چقدر باید باشه
 *   - وقتی plant در طول زمان تغییر می کنه (فرآیند پیر می شه)
 *
 * دو روش شناسایی:
 *   1. RELAY  (Astrom-Hagglund): كنترلر خودش نوسان ایجاد می كنه
 *      - سریع، بدون جابجایی از setpoint
 *      - خروجی: Ku (بهره نهایی) و Pu (دوره نهایی)
 *
 *   2. STEP   (پله باز): plant رو یه پله می ده
 *      - دقیق تر، مدل FOPDT (K, T, L) می ده
 *      - نیاز به جابجایی plant داره
 *
 * ██ رفع مشکلات نسخه قبلی شما ██
 *   ❌ plant_settle() کامنت شده بود → پلنت از y=0 شروع می شد
 *      ✅ حالا ۶۰۰ ثانیه پلنت settle می شه
 *   ❌ SENSOR_NOISE = 2.0 روی سیگنال ~10 (SNR=5:1) خیلی پر نویز
 *      ✅ حالا SENSOR_NOISE = 0.1 (SNR~100:1)
 *   ❌ hysteresis = 0.30 برای STEP test استفاده شده بود (بی معنی)
 *      ✅ حالا فقط برای RELAY استفاده می شه
 *   ❌ output_step = 10.0 با K=1 → تغییر فقط 10 واحد
 *      ✅ حالا output_step = 20.0 برای SNR بهتر
 *
 * ██ پیاده سازی STM32 ██
 *   main.c:
 *     example07_init();            // یکبار
 *     و در TIM ISR با 10Hz (100ms):
 *     example07_tick();
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

#include "pidx/pid.h"
#include "pidx/pid_autotune.h"

/* ===== پیکربندی ===== */
#define CTRL_DT             0.1f        /* 10Hz — برای فرآیند حرارتی */
#define CTRL_SETPOINT       60.0f       /* دمای هدف */

/* ===== پلنت واقعی (FOPDT با تاخیر) =====
 *   G(s) = K * exp(-L*s) / (1 + T*s)
 *
 * K = 2.0 °C per unit duty
 * T = 8.0 s  (ثابت زمانی)
 * L = 1.0 s  (تاخیر حمل)
 */
#define PLANT_K     80.0f    /* °C per unit duty — پلنت حرارتی واقعی */
#define PLANT_T     40.0f    /* s ثابت زمانی */
#define PLANT_L     8.0f     /* s تاخیر حمل */
#define PLANT_BIAS  20.0f    /* دمای محیط */
#define SENSOR_NOISE_SIGMA 0.1f  /* نویز کم - مهم برای شناسایی! */

/* ===== محرک ===== */
#define ACTUATOR_MIN   0.0f
#define ACTUATOR_MAX   1.0f

/* ===== وضعیت ===== */
static PID_Handle  pid;
static PID_AutoTune tuner;
static float plant_y;           /* دمای واقعی پلنت (با تاخیر) */
static float y_nodelay;         /* دمای بدون تاخیر (حالت فیلتر) */
static float sensor_y;          /* دمای سنسور */

/* بافر تاخیر حمل */
#define DELAY_BUF 128
static float delay_buf[DELAY_BUF];
static int   delay_idx;

/* ================================================================
 * مولد نویز
 * ================================================================ */
static float gen_noise(float sigma)
{
    static unsigned int seed = 31337;
    float sum = 0.0f;
    for (int i = 0; i < 12; i++) {
        seed = seed * 1664525 + 1013904223;
        sum += (float)(int)seed * (1.0f / 2147483648.0f);
    }
    return sum * (sigma / 6.0f);
}

/* ================================================================
 * پلنت FOPDT با تاخیر حمل واقعی (رینگ بافر)
 * ================================================================ */
static void plant_init(void)
{
    plant_y = PLANT_BIAS;
    y_nodelay = PLANT_BIAS;
    for (int i = 0; i < DELAY_BUF; i++)
        delay_buf[i] = PLANT_BIAS;
    delay_idx = 0;
}

static float plant_step(float u, float dt)
{
    (void)dt;

    /* 1. فیلتر مرتبه اول بدون تاخیر — steady state: y = K*u + bias
     *
     * مهم: اینجا از y_nodelay (حالت قبلی فیلتر) استفاده می شود،
     * نه از plant_y (که تاخیر دارد)!
     * اگر از plant_y استفاده کنیم، تاخیر 80 نمونه ای داخل فیلتر
     * می افتد و ثابت زمانی موثر 80 برابر می شود — باگ! */
    float a = expf(-CTRL_DT / PLANT_T);
    y_nodelay = a * y_nodelay + (1.0f - a) * (PLANT_K * u + PLANT_BIAS);

    /* 2. تاخیر حمل L ثانیه (رینگ بافر) */
    delay_buf[delay_idx] = y_nodelay;
    delay_idx = (delay_idx + 1) % DELAY_BUF;
    plant_y = delay_buf[delay_idx];

    return plant_y;
}

/* ================================================================
 * سنسور
 * ================================================================ */
static float read_sensor(void)
{
    sensor_y = plant_y + gen_noise(SENSOR_NOISE_SIGMA);
    return sensor_y;
}

/* ================================================================
 * پلنت را به حالت پایدار می برد
 *
 * u_hold = خروجی که دما را روی setpoint نگه می دارد
 * ================================================================ */
static void plant_settle(float u_hold, int seconds)
{
    plant_init();
    int n = (int)(seconds / CTRL_DT);
    for (int i = 0; i < n; i++) {
        plant_step(u_hold, CTRL_DT);
    }
    printf("  Plant settled at %.1f C with u=%.3f\n",
           plant_y, u_hold);
}

/* ================================================================
 * callback پیشرفت
 * ================================================================ */
static void on_progress(uint8_t percent, PID_TuneState state, void *ctx)
{
    static int last_pct = 0;
    (void)ctx;
    if (percent / 10 != last_pct / 10) {
        printf("  %3u%%  [%s]\n", (unsigned)percent,
               PID_TuneStateToString(state));
        last_pct = percent;
    }
}

/* ================================================================
 * راه اندازی
 *
 * ident_method = PID_IDENT_RELAY  یا  PID_IDENT_STEP
 * ================================================================ */
void example07_init(PID_IdentMethod ident_method)
{
    PID_Config cfg;
    PID_AutoTuneConfig tc;
    PID_StatusCode rc;

    /* ---- 1. پلنت را به حالت پایدار ببر (مهم!) ---- */
    /* خروجی لازم برای نگه داشتن 60 درجه:
     *   u = (60 - 20) / K = 40 / 80 = 0.50 */
    float u_hold = (CTRL_SETPOINT - PLANT_BIAS) / PLANT_K;
    plant_settle(u_hold, 600);   /* 600s = 75 * T */

    /* ---- 2. PID را با gains نرم اولیه کن ---- */
    PID_ConfigDefault(&cfg);
    cfg.core.kp = 0.1f;               /* نرم - بعدا AutoTune بهتر می کند */
    cfg.core.ki = 0.005f;
    cfg.core.kd = 0.0f;
    cfg.core.sample_time = CTRL_DT;
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = ACTUATOR_MIN;
    cfg.limits.output_max = ACTUATOR_MAX;
    PID_Init(&pid, &cfg);
    PID_SetSetpoint(&pid, CTRL_SETPOINT);

    /* ---- 3. پیکربندی AutoTune ---- */
    PID_AutoTune_ConfigDefault(&tc, ident_method);

    if (ident_method == PID_IDENT_RELAY) {
        /* روش RELAY:
         *   خروجی بین u_hold ± h سوییچ می کند
         *   hysteresis = 3*نویز برای جلوگیری از chattering */
        tc.rule        = PID_RULE_TYREUS_LUYBEN;  /* مقاوم، کم overshoot */
        tc.output_step = 0.10f;        /* relay amplitude ±10% duty */
        tc.hysteresis  = 0.30f;        /* 3*نویز سنسور */
        tc.bias        = u_hold;       /* مرکز relay */
        tc.auto_bias   = false;        /* bias را خودمان دادیم */
        printf("  Method: RELAY (Astrom-Hagglund)\n");
    } else {
        /* روش STEP:
         *   خروجی یک پله 20% می خورد و پاسخ ثبت می شود */
        tc.rule        = PID_RULE_AMIGO_STEP;  /* بهترین معامله کلی */
        tc.output_step = 0.20f;        /* step size = 20% duty */
        tc.hysteresis  = 0.0f;         /* برای STEP معنی ندارد */
        tc.bias        = u_hold;
        tc.auto_bias   = false;
        printf("  Method: STEP (area/moment FOPDT fit)\n");
    }

    tc.structure   = PID_STRUCT_PID;
    tc.output_min  = ACTUATOR_MIN;
    tc.output_max  = ACTUATOR_MAX;
    tc.meas_min    = 0.0f;
    tc.meas_max    = 200.0f;
    tc.timeout_s   = 600.0f;
    tc.on_progress = on_progress;

    rc = PID_AutoTune_Init(&tuner, &tc);
    if (rc != PID_OK) {
        printf("  AutoTune_Init FAILED: %s\n", PID_StatusToString(rc));
        return;
    }

    rc = PID_AutoTune_Start(&tuner, &pid, CTRL_SETPOINT);
    if (rc != PID_OK) {
        printf("  AutoTune_Start FAILED: %s\n", PID_StatusToString(rc));
        return;
    }

    printf("  AutoTune started around setpoint %.0f C\n",
           CTRL_SETPOINT);
    printf("====================================\n");
}

/* خروجی fallback وقتی تیونر شکست خورده (اعلام جلوتر) */
static float u_hold_fallback(void)
{
    return (CTRL_SETPOINT - PLANT_BIAS) / PLANT_K;
}

/* ================================================================
 * تیک (در TIM ISR با 10Hz)
 * ================================================================ */
void example07_tick(void)
{
    static int tick = 0;
    tick++;

    /* 1. سنسور (پلنت با خروجی تیک قبلی به روز شده) */
    float meas = read_sensor();

    /* 3. کنترلر:
     *    اگر تیونر در حال اجراست، تیونر خروجی می دهد
     *    اگر تمام شده، PID با gains جدید */
    float control;
    if (PID_AutoTune_IsRunning(&tuner)) {
        control = PID_AutoTune_Update(&tuner, meas, CTRL_DT);

        /* تیونر تمام شد؟ gains را اعمال کن */
        if (PID_AutoTune_IsComplete(&tuner)) {
            PID_AutoTuneResult res;
            PID_AutoTune_GetResult(&tuner, &res);
            printf("\n====================================\n");
            printf("  AUTO-TUNE COMPLETE!\n");
            printf("  Model: ");
            if (res.model.kind == PID_MODEL_FREQ)
                printf("Ku=%.4f Pu=%.2fs", res.model.ku, res.model.pu);
            else
                printf("K=%.4f T=%.2fs L=%.2fs",
                       res.model.k, res.model.t, res.model.l);
            printf(" quality=%u/100\n", (unsigned)res.model.quality);
            printf("  Gains: Kp=%.4f Ki=%.4f Kd=%.4f\n",
                   res.gains.kp, res.gains.ki, res.gains.kd);

            PID_AutoTune_Apply(&tuner, &pid);
            float kp, ki, kd;
            PID_GetGains(&pid, &kp, &ki, &kd);
            printf("  Applied: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
            printf("====================================\n\n");
        }
    } else if (PID_AutoTune_IsComplete(&tuner)) {
        control = PID_Update(&pid, meas);
    } else {
        /* شکست خورد - گزارش بده */
        PID_StatusCode err = PID_AutoTune_GetError(&tuner);
        printf("  AUTOTUNE FAILED: %s (state %s)\n",
               PID_StatusToString(err),
               PID_TuneStateToString(PID_AutoTune_GetState(&tuner)));
        control = u_hold_fallback();
    }

    /* 4. پلنت را با خروجی کنترلر به روز کن */
    plant_step(control, CTRL_DT);

    /* 5. گزارش */
    if (tick % 50 == 0) {  /* هر 5 ثانیه */
        printf("t=%4ds SP=60 PV=%6.2f u=%6.3f | tuner=%s\n",
               tick / 10, meas, control,
               PID_AutoTune_IsRunning(&tuner) ? "RUN" :
               PID_AutoTune_IsComplete(&tuner) ? "DONE" : "IDLE");
    }
}



/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    /* تست RELAY */
    example07_init(PID_IDENT_RELAY);
    for (int i = 0; i < 10000 && PID_AutoTune_IsRunning(&tuner); i++)
        example07_tick();

    /* تست STEP */
    example07_init(PID_IDENT_STEP);
    for (int i = 0; i < 10000 && PID_AutoTune_IsRunning(&tuner); i++)
        example07_tick();

    return 0;
}
#endif
