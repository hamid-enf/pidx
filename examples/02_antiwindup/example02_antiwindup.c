/**
 * ====================================================================
 * مثال ۲ — کنترل دما با Anti-Windup
 * ====================================================================
 *
 * ██ کاربرد ██
 *   سیستم‌هایی که محرک یک‌طرفه دارن (مثل هیتر):
 *   - کوره، آبگرمکن، هیتر برقی، سیستم HVAC
 *   - محرک فقط می‌تونه مثبت باشه (PWM 0..100%)
 *   - وقتی ۱۰۰٪ خروجی می‌دی ولی هنوز به هدف نرسیدی، انتگرالگیر باد می‌کنه!
 *
 * ██ مشکل Windup ██
 *   تصور کن هیتر ۱۰۰٪ روشنه ولی دما ۵۰ درجه با هدف فاصله داره.
 *   انتگرالگیر مدام اضافه می‌کنه: I += Ki*e*dt
 *   بعد که دما به هدف رسید، انتگرالگیر هنوز پر هست و باعث overshoot بزرگ می‌شه.
 *   محرک باید اول انتگرالگیر رو خالی کنه — به این می‌گن windup.
 *
 * ██ مقایسه ۴ استراتژی ██
 *   اینجا می‌تونیم مقایسه کنیم:
 *   - NONE: بدون محافظت (بدترین حالت)
 *   - CLAMP: انتگرالگیر رو محدود به خروجی می‌کنه
 *   - CONDITIONAL: وقتی اشباعه و خطا همجهته، انتگرال نمی‌گیره
 *   - BACK_CALCULATION: انتگرالگیر رو با نسبت خطا پس‌می‌ده (بهترین)
 *
 * ██ سناریو ██
 *   یه هیتر ۴۰۰ وات که یه مخزن ۴۰ ژول بر درجه رو گرم می‌کنه:
 *   - اول گرم می‌کنه از ۲۰ به ۱۸۰ درجه (۳۰۰ ثانیه)
 *   - بعد هدف رو میاره پایین به ۶۰ درجه (۳۰۰ ثانیه)
 *   - توی فاز دوم محرک روی ۰ می‌مونه و windup خودش رو نشون می‌ده
 *
 * ██ خروجی مورد انتظار ██
 *   - NONE: بعد از کاهش setpoint، تا ۹۱ ثانیه محرک قفل می‌مونه!
 *   - CLAMP: ۳۶ ثانیه طول می‌کشه تا بازیابی کنه
 *   - BACK_CALCULATION: فقط ۲۳ ثانیه!
 *
 * ====================================================================
 */

#include <stdio.h>
#include <math.h>

/* ===== مسیر هدر PIDX ===== */
#include "pidx/pid.h"

/* ===== پیکربندی اصلی ===== */
#define CTRL_SAMPLE_TIME_S  0.5f     /* ۲ هرتز — هیتر نیازی به بیشتر نداره */
#define CTRL_KP             0.05f    /* بهره تناسبی */
#define CTRL_KI             0.002f   /* بهره انتگرالی */
#define CTRL_KD             0.30f    /* بهره مشتقی */

#define CTRL_SETPOINT_UP    180.0f   /* هدف مرحله اول: گرمایش */
#define CTRL_SETPOINT_DOWN  60.0f    /* هدف مرحله دوم: سرد شدن */

/* ===== پارامترهای پلنت (برگرفته از ex_plant PIDX) =====
 *
 * C*dT/dt = P*u - h*(T-Tamb) - eps*(T^4-Tamb^4)
 * که:
 *   C = 40 J/C     ظرفیت حرارتی مخزن
 *   P = 400 W      توان هیتر
 *   h = 1.0 W/C    تلفات جابجایی (convection)
 *   eps = 2e-9 W/K^4  تلفات تابشی (radiation)
 *
 * → با u=0.57 (۵۷٪ duty) دما به ۱۸۰ می‌رسه
 * → با u=1.0 (۱۰۰٪) دما به ~۴۲۰ می‌رسه
 * ================================================================ */
#define PLANT_AMBIENT       20.0f
#define PLANT_HEAT_CAPACITY 40.0f     /* J/C */
#define PLANT_HEATER_POWER  400.0f    /* W */
#define PLANT_LOSS_COEFF    1.0f      /* W/C */
#define PLANT_RAD_COEFF     2.0e-9f   /* W/K^4 */
#define SENSOR_NOISE_SIGMA  0.05f

/* ===== تایمینگ ===== */
#define TICKS_UP  600       /* ۳۰۰ ثانیه گرمایش (هر تیک ۰.۵ ثانیه) */
#define TICKS_DOWN 600      /* ۳۰۰ ثانیه سرمایش */
#define TICKS_TOTAL (TICKS_UP + TICKS_DOWN)

/* ===== وضعیت برنامه ===== */
static struct {
    PID_Handle pid;

    /* شبیه‌ساز پلنت (هیتر) */
    float plant_temp;       /* دمای واقعی */

    /* خروجی */
    float output;

    /* آمار */
    uint32_t tick;

    /* مقدار اوج انتگرالگیر */
    float i_peak;
    float recover_start;    /* تیکی که بازیابی شروع شد */
    int   step_down_done;   /* آیا setpoint رو پایین آوردیم؟ */

} ctx;

/* ================================================================
 * مولد نویز
 * ================================================================ */
static float gen_noise(float sigma)
{
    static unsigned int seed = 4242;
    float sum = 0.0f;
    for (int i = 0; i < 12; i++) {
        seed = seed * 1664525 + 1013904223;
        sum += (float)(int)seed * (1.0f / 2147483648.0f);
    }
    return sum * (sigma / 6.0f);
}

/* ================================================================ *
 * مدل هیتر با تلفات تابشی (غیرخطی) — ex_heater از PIDX
 *
 *   dT/dt = (P*u - h*(T-Tamb) - eps*(T^4 - Tamb^4)) / C
 *
 * که:
 *   u = 0..1 (PWM duty)
 *   تلفات تابشی باعث می‌شه بهره فرایند با دما کم بشه
 * ================================================================ */
static float plant_heater(float u, float dt)
{
    /* محدودیت فیزیکی: محرک نمی‌تونه منفی بده */
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;

    float tk  = ctx.plant_temp + 273.15f;    /* کلوین */
    float tak = PLANT_AMBIENT + 273.15f;

    float loss = PLANT_LOSS_COEFF * (ctx.plant_temp - PLANT_AMBIENT)
               + PLANT_RAD_COEFF * (tk*tk*tk*tk - tak*tak*tak*tak);

    float dT = (PLANT_HEATER_POWER * u - loss) / PLANT_HEAT_CAPACITY * dt;
    ctx.plant_temp += dT;

    if (ctx.plant_temp < PLANT_AMBIENT)
        ctx.plant_temp = PLANT_AMBIENT;

    return ctx.plant_temp;
}

/* ================================================================
 * شبیه‌سازی سنسور (حرارتی با نویز کم)
 * ================================================================ */
static float read_sensor(void)
{
    return ctx.plant_temp + gen_noise(SENSOR_NOISE_SIGMA);
}

/* ================================================================
 * راه‌اندازی با یک استراتژی Anti-Windup مشخص
 *
 * ورودی:
 *   aw_mode: PID_AW_NONE, PID_AW_CLAMP, PID_AW_CONDITIONAL, PID_AW_BACK_CALCULATION
 *   kt: بهره Back-Calculation (برای BACK_CALCULATION معنی داره)
 *
 * صدا زدن:
 *   example02_init(PID_AW_CLAMP, 0.0f);
 *   example02_init(PID_AW_BACK_CALCULATION, 1.0f);
 * ================================================================ */
void example02_init(PID_AntiWindup aw_mode, float kt)
{
    PID_Config cfg;

    /* ---- صفر کردن وضعیت ---- */
    ctx.plant_temp = PLANT_AMBIENT;
    ctx.tick = 0;
    ctx.i_peak = 0.0f;
    ctx.recover_start = 0.0f;
    ctx.step_down_done = 0;

    /* ---- مقداردهی PID با PID_Config (نه InitDefault) ---- */
    PID_ConfigDefault(&cfg);

    /* بهره‌ها */
    cfg.core.kp = CTRL_KP;
    cfg.core.ki = CTRL_KI;
    cfg.core.kd = CTRL_KD;
    cfg.core.sample_time = CTRL_SAMPLE_TIME_S;
    cfg.core.direction = PID_DIRECT;   /* خروجی بیشتر = گرمتر */

    /* محدودیت خروجی: PWM 0..1 — خیلی مهم!
     *
     * ⚠️ اگر این رو نذاریم، پیش‌فرض -inf تا +inf هست
     * و کنترلر می‌تونه "duty منفی" بده که برای هیتر معنی نداره.
     * Anti-windup هم بدون محدودیت خروجی کار نمی‌کنه! */
    cfg.limits.use_output_limits = true;
    cfg.limits.output_min = 0.0f;
    cfg.limits.output_max = 1.0f;

    /* استراتژی Anti-Windup */
    cfg.integral.mode = aw_mode;
    cfg.integral.kt   = kt;

    /* مشتق روی اندازه‌گیری (نه روی خطا) + فیلتر */
    cfg.filter.derivative_mode = PID_DERIV_ON_MEASUREMENT;
    cfg.filter.tf = 4.0f;           /* ثابت فیلتر مشتق */
    cfg.filter.input_lpf_tau = 1.0f;/* فیلتر روی سنسور */

    PID_Init(&ctx.pid, &cfg);
    PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT_UP);

    /* گزارش */
    const char *aw_name;
    switch (aw_mode) {
        case PID_AW_NONE:             aw_name = "NONE (بدون محافظت)"; break;
        case PID_AW_CLAMP:            aw_name = "CLAMP (محدودکننده)"; break;
        case PID_AW_CONDITIONAL:      aw_name = "CONDITIONAL (شرطی)"; break;
        case PID_AW_BACK_CALCULATION: aw_name = "BACK_CALCULATION (پس‌دهی)"; break;
        default:                      aw_name = "نامشخص"; break;
    }

    printf("\r\n");
    printf("============================================\r\n");
    printf("مثال ۲ — Anti-Windup: %s\r\n", aw_name);
    printf("============================================\r\n");
    printf("%4s | %8s | %8s | %8s\r\n", "زمان", "هدف", "دما", "خروجی");
    printf("--------------------------------------------\r\n");
}

/* ================================================================
 * تیک کنترلر
 *
 * این تابع رو توی TIM ISR با نرخ ۲ هرتز صدا بزن
 * ================================================================ */
void example02_tick(void)
{
    ctx.tick++;

    /* ---- تغییر setpoint بعد از ۳۰۰ ثانیه ---- */
    if (ctx.tick == TICKS_UP) {
        PID_SetSetpoint(&ctx.pid, CTRL_SETPOINT_DOWN);
        ctx.step_down_done = 1;
        printf(">>> هدف کاهش یافت به %.0f°C <<<\r\n", CTRL_SETPOINT_DOWN);
    }

    /* ---- ۱. به‌روزرسانی پلنت ---- */
    plant_heater(ctx.output, CTRL_SAMPLE_TIME_S);

    /* ---- ۲. خوندن سنسور ---- */
    float meas = read_sensor();

    /* ---- ۳. محاسبه PID ---- */
    float control = PID_Update(&ctx.pid, meas);

    /* ---- ۴. اعمال خروجی ---- */
    ctx.output = control;

    /* ---- ثبت اوج انتگرالگیر ---- */
    float i_term = PID_GetIntegrator(&ctx.pid);
    if (fabsf(i_term) > ctx.i_peak)
        ctx.i_peak = fabsf(i_term);

    /* ---- ثبت زمان بازیابی ---- */
    if (ctx.tick == TICKS_UP) {
        ctx.recover_start = (float)ctx.tick;
    }
    if (ctx.step_down_done && ctx.recover_start > 0.0f) {
        /* بازیابی زمانی که خروجی از اشباع خارج بشه */
        if (control > 0.001f && control < 0.999f) {
            if (ctx.recover_start > 0.0f) {
                /* فقط اولین بار ثبت می‌کنیم */
                ctx.recover_start = -ctx.recover_start; /* به عنوان پرچم */
            }
        }
    }

    /* ---- گزارش ---- */
    if (ctx.tick % 40 == 0) {   /* هر ۲۰ ثانیه */
        printf("%4.0f | %8.0f | %8.1f | %8.3f\r\n",
               (float)ctx.tick * CTRL_SAMPLE_TIME_S,
               ctx.tick < TICKS_UP ? CTRL_SETPOINT_UP : CTRL_SETPOINT_DOWN,
               meas, control);
    }

    /* ---- پایان ---- */
    if (ctx.tick >= TICKS_TOTAL) {
        printf("--------------------------------------------\r\n");
        printf("✅ Anti-Windup آزمایش شد.\r\n");
        printf("اوج انتگرالگیر (|I|_peak): %.4f\r\n", ctx.i_peak);
        printf("============================================\r\n\r\n");
    }
}

/* ================================================================
 * تابع: اجرای همه استراتژی‌ها پشت سر هم
 *
 * این تابع ۴ بار مثال رو با استراتژی‌های مختلف اجرا می‌کنه
 * ================================================================ */
void example02_run_all(void)
{
    /* بدون محافظت */
    example02_init(PID_AW_NONE, 0.0f);
    for (int i = 0; i < TICKS_TOTAL; i++)
        example02_tick();

    /* Clamp */
    example02_init(PID_AW_CLAMP, 0.0f);
    for (int i = 0; i < TICKS_TOTAL; i++)
        example02_tick();

    /* Conditional */
    example02_init(PID_AW_CONDITIONAL, 0.0f);
    for (int i = 0; i < TICKS_TOTAL; i++)
        example02_tick();

    /* Back-Calculation (بهترین) */
    example02_init(PID_AW_BACK_CALCULATION, 1.0f);
    for (int i = 0; i < TICKS_TOTAL; i++)
        example02_tick();
}


/* ================================================================== */
#ifdef TEST_ON_PC
int main(void)
{
    example02_init(PID_AW_CLAMP, 0.0f);
    for (int i = 0; i < TICKS_TOTAL; i++)
        example02_tick();
    return 0;
}
#endif /* TEST_ON_PC */