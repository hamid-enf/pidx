# PIDX — Portable PID Control Framework
## سند معماری (PHASE 1–3) — نسخه ۰.۹ پیش از پیاده‌سازی

> **وضعیت:** این سند خروجی بند ۸۹ درخواست شماست: معماری، Feature Matrix، ساختار فایل، Public API،
> Data Structures، Dependency Graph، Execution Flow، Auto-Tune Architecture، Memory/Performance
> Strategy، Test Strategy و در انتها **Critical Design Review** روی خود Specification.
> هیچ کدی هنوز نوشته نشده است. پس از تأیید (یا اعمال اصلاحات) وارد PHASE 4 می‌شویم.

---

## ۰. خلاصه اجرایی و هویت پروژه

| مورد | مقدار |
|---|---|
| نام کتابخانه | **PIDX** (Portable PID eXtended) |
| Public prefix | `PID_` (توابع)، `PID_` (typeها)، `PIDX_` (ماکروهای پیکربندی) |
| Internal prefix | `pidp_` (private/static)، `pidi_` (inline internal) |
| زبان | **C99** (freestanding-friendly) |
| وابستگی خارجی | فقط `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`, `<float.h>` و از `math.h` تنها `fabsf`, `sqrtf` (آن هم قابل جایگزینی) |
| حافظه پویا | **ممنوع مطلق** — بدون `malloc/calloc/realloc/free` |
| وابستگی HAL در Core | **صفر** |
| مدل PID | Parallel (Independent) + Setpoint Weighting (2DOF) |
| Discretization | Integral: Backward-Euler (پیش‌فرض) / Trapezoidal (اختیاری) — Derivative: Filtered Backward-Difference |

**جمله هدف:** کاربر باید بتواند با ۵ خط شروع کند و بدون تغییر آن ۵ خط، تا 2DOF + Feedforward +
Back-Calculation + Gain Scheduling + Cascade + Auto-Tune + Diagnostics رشد کند.

---

## ۱. اصول معماری (Design Principles)

این هفت اصل، هر تصمیم بعدی در این سند را توجیه می‌کنند:

1. **Core Purity** — هسته (`pid.c`) به هیچ ماژول اختیاری، به هیچ HAL و به هیچ Global State وابسته نیست.
   جهت وابستگی همیشه یک‌طرفه است: `Optional → Core`، هرگز برعکس.
2. **Zero-Cost Abstraction** — قابلیت خاموش (compile-time) نباید حتی یک بایت RAM/Flash یا یک شاخه
   Runtime هزینه داشته باشد. قابلیت روشن ولی غیرفعال (runtime) باید حداکثر یک bit-test هزینه داشته باشد.
3. **Precompute over Divide** — هیچ تقسیم و هیچ فراخوانی `math.h` در مسیر داغ `PID_Update()` نیست.
   همه ضرایب هنگام تغییر gain/dt یک‌بار محاسبه می‌شوند (`pidp_recompute()`).
4. **State in Handle** — تمام وضعیت داخل `PID_Handle` است. بدون متغیر global، بدون `static` قابل تغییر.
   نتیجه: reentrancy نسبت به handleهای مجزا به‌صورت ذاتی برقرار است.
5. **Bumpless by Default** — هر تغییر runtime (gain، mode، limit، schedule) نباید پرش خروجی بسازد،
   مگر کاربر صریحاً `PID_SetGainsHard()` را انتخاب کند.
6. **Fail Loud, Run Safe** — خطا هرگز بی‌صدا نیست (sticky error + status flags)، ولی کنترلر در
   حالت خطا خروجی fail-safe می‌دهد و قفل نمی‌شود.
7. **No Fake Features** — هر قابلیتی که به‌صورت ریاضی درست پیاده نشود، اصلاً وارد کتابخانه نمی‌شود.
   (بخش ۱۲ این سند فهرست صریح «آنچه پیاده *نمی‌شود*» را دارد.)

---

## ۲. معماری لایه‌ای

```
┌──────────────────────────────────────────────────────────────────────┐
│                          APPLICATION                                  │
│        (Timer ISR / FreeRTOS Task / Superloop / PC Simulation)        │
└───────────────────────────────┬──────────────────────────────────────┘
                                │  فقط از طریق Public API
┌───────────────────────────────▼──────────────────────────────────────┐
│  LAYER 4 — OPTIONAL / EXPERT MODULES   (هرکدام مستقل، هرکدام #if)     │
│  ┌───────────┐ ┌────────────┐ ┌───────────┐ ┌────────┐ ┌──────────┐  │
│  │ Auto-Tune │ │  Cascade   │ │GainSched  │ │ Diag/  │ │  Fixed   │  │
│  │ (ident +  │ │ (outer→    │ │ (interp.  │ │Telemetry│ │  Point   │  │
│  │  rules)   │ │  inner+AW) │ │  table)   │ │        │ │ (Q15/Q31)│  │
│  └─────┬─────┘ └─────┬──────┘ └─────┬─────┘ └───┬────┘ └────┬─────┘  │
└────────┼─────────────┼──────────────┼───────────┼───────────┼────────┘
         │             │              │           │           │ (مستقل از Core)
┌────────▼─────────────▼──────────────▼───────────▼───────────┼────────┐
│  LAYER 3 — PID PUBLIC API  (pid.h)                          │        │
│  Init/Config · Setters/Getters · Update · Mode · Status     │        │
└───────────────────────────────┬─────────────────────────────┼────────┘
┌───────────────────────────────▼─────────────────────────────┼────────┐
│  LAYER 2 — PROCESSING BLOCKS                                │        │
│  ┌──────────────┐   ┌───────────────┐   ┌────────────────┐  │        │
│  │  Setpoint    │   │   PID CORE    │   │    Output      │  │        │
│  │  Processing  │──▶│  P · I · D    │──▶│  Processing    │  │        │
│  │  (ramp/SP-LPF│   │  AW · 2DOF    │   │ (limit/slew/   │  │        │
│  │   weighting) │   │  FF · Mode    │   │  ramp/failsafe)│  │        │
│  └──────────────┘   └───────┬───────┘   └────────────────┘  │        │
│                             │  ▲                             │        │
│                    ┌────────▼──┴────────┐                    │        │
│                    │ Filters (D-LPF,    │                    │        │
│                    │ input LPF, MA*)    │                    │        │
│                    └────────────────────┘                    │        │
└───────────────────────────────┬─────────────────────────────┴────────┘
┌───────────────────────────────▼──────────────────────────────────────┐
│  LAYER 1 — FOUNDATION                                                 │
│  pid_types.h (enums/structs) · pid_conf.h (compile-time) ·            │
│  pid_math.h (inline: clamp/isfinite/sat — بدون libm در مسیر داغ)      │
└───────────────────────────────┬──────────────────────────────────────┘
                                │  (اختیاری، فقط برای راحتی کاربر)
┌───────────────────────────────▼──────────────────────────────────────┐
│  LAYER 0 — PLATFORM INTEGRATION (اختیاری، خارج از Core)               │
│  platform/stm32: timebase از TIM · DWT cycle counter · CMSIS-DSP hook │
│  platform/posix: timebase از clock_gettime (برای شبیه‌سازی/تست)       │
└───────────────────────────────────────────────────────────────────────┘
```

**قانون طلایی:** فلش‌ها فقط به سمت پایین. `pid.c` هرگز `pid_autotune.h` را include نمی‌کند.
Auto-Tune است که `pid.h` را include می‌کند و از بیرون روی handle کار می‌کند.

---

## ۳. Feature Matrix

سطوح: **B** = Basic، **I** = Intermediate، **A** = Advanced، **X** = Expert.
«Opt» = ماژول اختیاری compile-time. «RAM» = افزایش تقریبی اندازه handle (بایت، float32).

| # | Feature | B | I | A | X | Opt (macro) | RAM | مسیر داغ |
|---|---|:-:|:-:|:-:|:-:|---|--:|---|
| 1 | P / PI / PD / PID | ✓ | ✓ | ✓ | ✓ | Core | 0 | همیشه |
| 2 | Reset / Init / Default config | ✓ | ✓ | ✓ | ✓ | Core | 0 | — |
| 3 | Fixed-dt `PID_Update` | ✓ | ✓ | ✓ | ✓ | Core | 0 | همیشه |
| 4 | Variable-dt `PID_UpdateDt` | | ✓ | ✓ | ✓ | Core | 4 | فقط در فراخوانی |
| 5 | Output limits (hard) | | ✓ | ✓ | ✓ | Core | 8 | ۲ مقایسه |
| 6 | Integral limits | | ✓ | ✓ | ✓ | Core | 8 | ۲ مقایسه |
| 7 | Anti-Windup: Clamp | | ✓ | ✓ | ✓ | Core | 0 | ۲ مقایسه |
| 8 | Anti-Windup: Conditional | | ✓ | ✓ | ✓ | Core | 0 | ۱ شاخه |
| 9 | Anti-Windup: Back-Calculation | | ✓ | ✓ | ✓ | Core | 8 | ۱ ضرب+جمع |
| 10 | Anti-Windup: Tracking (ext. reset) | | | ✓ | ✓ | `AW_TRACKING` | 4 | ۱ شاخه |
| 11 | Derivative on Measurement / Error | | ✓ | ✓ | ✓ | Core | 0 | انتخاب مبدأ |
| 12 | Derivative LPF (1st order, N یا Tf) | | ✓ | ✓ | ✓ | Core | 8 | ۲ ضرب |
| 13 | Controller direction (Direct/Reverse) | | ✓ | ✓ | ✓ | Core | 0 | ۱ ضرب علامت |
| 14 | Manual / Auto / Hold + Bumpless | | ✓ | ✓ | ✓ | Core | 4 | ۱ switch |
| 15 | Setpoint ramp (rate + accel/decel) | | ✓ | ✓ | ✓ | `SP_SHAPER` | 20 | اگر فعال |
| 16 | Output ramp / slew-rate limit | | ✓ | ✓ | ✓ | `OUT_SHAPER` | 12 | اگر فعال |
| 17 | Setpoint weighting β (1DOF tuned) | | | ✓ | ✓ | Core | 4 | ۱ ضرب |
| 18 | 2DOF (β + γ) | | | ✓ | ✓ | Core | 4 | ۱ ضرب |
| 19 | Feedforward (مقدار یا callback) | | | ✓ | ✓ | `FEEDFORWARD` | 8 | ۱ جمع |
| 20 | Integral separation (deadband/threshold) | | | ✓ | ✓ | Core | 4 | ۱ مقایسه |
| 21 | Integral enable/disable runtime | | | ✓ | ✓ | Core | 0 | bit-test |
| 22 | External reset / integrator preset | | | ✓ | ✓ | Core | 0 | — |
| 23 | Runtime bumpless gain change | | | ✓ | ✓ | Core | 0 | — |
| 24 | Input (measurement) LPF | | | ✓ | ✓ | `INPUT_FILTER` | 8 | ۲ ضرب |
| 25 | Sensor validation (range/rate/NaN) | | | ✓ | ✓ | `SAFETY` | 16 | ۳ مقایسه |
| 26 | Fail-safe output + fault latch | | | ✓ | ✓ | `SAFETY` | 8 | ۱ شاخه |
| 27 | Gain Scheduling (interpolated) | | | | ✓ | `GAIN_SCHED` | 4+table | ۱ جست‌وجو |
| 28 | Cascade helper (2..N loops) | | | | ✓ | `CASCADE` | struct جدا | — |
| 29 | Auto-Tune: Relay (Åström–Hägglund) | | | | ✓ | `AUTOTUNE` | ~120 جدا | خارج از مسیر |
| 30 | Auto-Tune: Step/FOPDT ident | | | | ✓ | `AUTOTUNE_STEP` | مشترک | خارج از مسیر |
| 31 | Tuning rules: ZN, TL, Pessen, No/Some-OS | | | | ✓ | `AUTOTUNE` | 0 | — |
| 32 | Tuning rules: Cohen-Coon, AMIGO, IMC/λ | | | | ✓ | `AUTOTUNE_STEP` | 0 | — |
| 33 | Custom tuning rule (function pointer) | | | | ✓ | `AUTOTUNE` | 8 | — |
| 34 | Diagnostics `PID_Status` | | | ✓ | ✓ | `DIAG` | 40 | ذخیره فیلدها |
| 35 | Telemetry ring buffer (SPSC) | | | | ✓ | `TELEMETRY` | N×32 | ۱ کپی |
| 36 | Runtime profiling hook (cycles) | | | | ✓ | `PROFILING` | 16 | ۲ فراخوان hook |
| 37 | Fixed-Point Q15/Q31 controller | | | | ✓ | `FIXED_POINT` | ~56 (جدا) | مسیر مستقل |
| 38 | CMSIS-DSP acceleration hooks | | | | ✓ | `CMSIS_DSP` | 0 | — |

> **نکته انطباق با §69 (No Overengineering):** «Moving Average» عمداً از هسته حذف شده و فقط به‌عنوان
> ابزار مستقل در `pid_filter.h` می‌ماند، با هشدار صریح که در مسیر Derivative تأخیر فاز ایجاد می‌کند و
> برای PID توصیه نمی‌شود. دلیل کامل در بخش ۱۲.۷.

---

## ۴. ساختار فایل‌ها

```
pidx/
├── include/pidx/
│   ├── pid.h              # API عمومی هسته (تنها فایلی که کاربر Basic نیاز دارد)
│   ├── pid_types.h        # enum ها، struct های config، PID_Handle
│   ├── pid_conf.h         # پیکربندی compile-time (قابل override با PIDX_USER_CONF)
│   ├── pid_status.h       # فقط PID_StatusCode — برگ گراف، بدون هیچ وابستگی
│   ├── pid_math.h         # inline: clamp, isfinite, deadband, lerp  (بدون libm در مسیر داغ)
│   ├── pid_filter.h       # فیلترهای مستقل (LPF1, MovingAvg, Median3, RateLimiter, Deadband)
│   ├── pid_shaper.h       # پروفایل trapezoidal/rate + PID_Shaper مستقل
│   ├── pid_gainsched.h    # جدول Gain Scheduling
│   ├── pid_cascade.h      # چارچوب Cascade
│   ├── pid_autotune.h     # ماشین حالت Auto-Tune + قوانین tuning
│   ├── pid_diag.h         # Status / Telemetry / Profiling
│   ├── pid_fixed.h        # کنترلر Fixed-Point مستقل (Q15/Q31)
│   └── pid_version.h      # نسخه معنایی + ماکروی بررسی سازگاری
│
├── src/
│   ├── pid.c              # هسته: update, config, mode, AW, derivative  (~۷۰۰ خط)
│   ├── pid_shaper.c
│   ├── pid_filter.c
│   ├── pid_gainsched.c
│   ├── pid_cascade.c
│   ├── pid_autotune.c     # ماشین حالت + شناسایی (relay/step)
│   ├── pid_autotune_rules.c  # جدول قوانین tuning (داده‌محور)
│   ├── pid_diag.c
│   └── pid_fixed.c
│
├── platform/
│   ├── stm32/
│   │   ├── pid_stm32.h/.c     # timebase از TIM، DWT cycle counter، الگوی ISR
│   │   └── pid_stm32_conf_template.h
│   ├── esp32/
│   │   ├── pid_esp32.h/.c     # timebase از esp_timer/CCOUNT، rate driver،
│   │   │                      #   مانیتور بار، کمک‌کننده‌های FreeRTOS (pinned)
│   │   └── pid_esp32_conf_template.h
│   └── posix/
│       └── pid_posix.h/.c     # timebase برای PC/شبیه‌سازی
│
├── examples/
│   ├── 01_minimal/                 # ۵ خطی
│   ├── 02_temperature_pwm/         # ADC→NTC→PID→PWM heater (STM32)
│   ├── 03_motor_speed/             # Encoder→speed→PID→PWM
│   ├── 04_motor_position/          # Position loop + setpoint ramp
│   ├── 05_current_control/         # حلقه جریان سریع (10–20 kHz) + fast path
│   ├── 06_cascade_pos_vel_cur/     # آبشاری سه‌لایه
│   ├── 07_autotune_relay/          # Auto-tune روی سیستم واقعی
│   ├── 08_freertos_task/           # اجرا در Task با vTaskDelayUntil
│   ├── 09_tim_isr/                 # اجرا در TIM6 ISR
│   └── 10_full_featured/           # همه قابلیت‌ها یکجا (مرجع پیکربندی)
│
├── tests/
│   ├── pid_test.h                  # هارنس تست سبک (بدون وابستگی خارجی)
│   ├── test_core.c  test_limits.c  test_antiwindup.c  test_derivative.c
│   ├── test_modes.c test_shaper.c  test_2dof.c        test_feedforward.c
│   ├── test_gainsched.c test_cascade.c test_autotune.c
│   ├── test_safety.c test_numerics.c test_fixed.c
│   └── plants/                     # مدل‌های مرجع (FOPDT+delay، 2nd order، integrator)
│
├── sim/
│   ├── sim_step.c        # پاسخ پله + معیارها (rise/settle/OS/IAE)
│   ├── sim_autotune.c    # قبل/بعد از tuning
│   ├── plot.py           # رسم نمودار از CSV
│   └── results/
│
├── bench/
│   ├── bench_host.c      # اندازه‌گیری روی PC + شمارش عملیات
│   ├── bench_dwt.c       # هارنس DWT برای Cortex-M (روی سخت‌افزار کاربر)
│   └── size_report.sh    # گزارش Flash/RAM با nm/size
│
├── docs/  (فارسی)
│   ├── 00_architecture.md   ← همین سند
│   ├── 01_intro_philosophy.md   02_quickstart.md      03_theory_discrete_pid.md
│   ├── 04_api_basic.md          05_api_intermediate.md 06_api_advanced.md
│   ├── 07_api_expert.md         08_antiwindup.md       09_derivative_filter.md
│   ├── 10_1dof_2dof.md          11_feedforward.md      12_gain_scheduling.md
│   ├── 13_cascade.md            14_autotune.md         15_safety_diagnostics.md
│   ├── 16_fixed_point.md        17_performance.md      18_stm32_integration.md
│   ├── 19_rtos_isr.md           20_troubleshooting.md  21_api_reference.md
│   ├── 22_misra_deviations.md   23_ports.md            24_port_comparison.md
│
├── ports/                # پورت‌های زبان‌های دیگر + هارنس مقایسه عددی
│   ├── SPEC_conformance.md  # قرارداد سناریو که هر پنج پیاده‌سازی اجرا می‌کنند
│   ├── c_ref/            # runner مرجع C (بیلد double)
│   ├── python/pidx/      # پورت Python (بدون وابستگی)
│   ├── matlab/+pidx/     # پورت MATLAB/Octave (classdef)
│   ├── csharp/           # پورت C# (ساخته‌شده با Mono)
│   ├── compare/          # scenarios.txt + compare.py
│   └── results/          # CSVهای تولیدشده
│
├── Makefile              # ساخت host: tests, sim, bench  (بدون cmake اجباری)
├── CMakeLists.txt        # اختیاری، برای STM32CubeIDE/CMake users
├── README.md             # انگلیسی (GitHub-ready)
└── LICENSE               # placeholder
```

**پیشنهاد بهبود نسبت به ساختار §50 شما:** اضافه‌شدن `platform/` (جداسازی صریح HAL)، `sim/` و
`bench/` به‌عنوان دایرکتوری مستقل (نه زیرمجموعه tests، چون چرخه اجرا و هدفشان متفاوت است) و
`include/pidx/` به‌جای `include/` مسطح (جلوگیری از تصادم نام `pid.h` با پروژه‌های دیگر؛
کاربر `#include "pidx/pid.h"` می‌نویسد).

---

## ۵. Data Structures (PHASE 3)

### ۵.۱ نوع پایه و enumها

```c
/* ---- pid_types.h ---- */

typedef float PID_Float;              /* قابل تغییر به double در pid_conf.h */

typedef enum {                        /* کد خطا/وضعیت — همه APIهای config برمی‌گردانند */
    PID_OK = 0,
    PID_ERR_NULL,                     /* اشاره‌گر NULL */
    PID_ERR_NOT_INIT,                 /* handle مقداردهی اولیه نشده */
    PID_ERR_INVALID_CONFIG,
    PID_ERR_INVALID_GAIN,             /* NaN/Inf یا منفی در جایی که مجاز نیست */
    PID_ERR_INVALID_LIMIT,            /* min >= max */
    PID_ERR_INVALID_DT,               /* dt <= 0 یا خارج از [dt_min, dt_max] */
    PID_ERR_INVALID_MODE,
    PID_ERR_NAN_INPUT,                /* measurement/setpoint = NaN */
    PID_ERR_INF_INPUT,
    PID_ERR_SENSOR_RANGE,             /* خارج از بازه مجاز سنسور */
    PID_ERR_SENSOR_RATE,              /* جهش غیرفیزیکی اندازه‌گیری */
    PID_ERR_UNSUPPORTED,              /* قابلیت در build فعلی compile نشده */
    PID_ERR_BUSY,                     /* مثلاً تغییر gain حین auto-tune */
    PID_ERR_TUNE_TIMEOUT,
    PID_ERR_TUNE_UNSTABLE,            /* نوسان واگرا یا خارج از حد ایمنی */
    PID_ERR_TUNE_NO_OSCILLATION,      /* دامنه ناکافی */
    PID_ERR_TUNE_MODEL_MISMATCH,      /* قانون tuning با مدل شناسایی‌شده سازگار نیست */
    PID_ERR_TUNE_ABORTED,
    PID_ERR_TUNE_VALIDATION           /* gain حاصل نامعتبر */
} PID_StatusCode;

typedef enum { PID_DIRECT = 0, PID_REVERSE = 1 } PID_Direction;

typedef enum {
    PID_MODE_MANUAL = 0,   /* خروجی = مقدار دستی؛ integrator با back-solve هم‌گام می‌ماند */
    PID_MODE_AUTOMATIC,    /* حلقه بسته */
    PID_MODE_HOLD          /* محاسبه ادامه دارد، integrator منجمد (برای freeze/tracking) */
} PID_Mode;

typedef enum {
    PID_AW_NONE = 0,
    PID_AW_CLAMP,              /* محدودسازی مستقیم state انتگرالگیر */
    PID_AW_CONDITIONAL,        /* توقف انتگرال‌گیری وقتی اشباع و خطا هم‌جهت است */
    PID_AW_BACK_CALCULATION,   /* i += Kt*(u_sat - u_raw)*dt */
    PID_AW_TRACKING            /* i دنبال سیگنال خارجی (external reset feedback) */
} PID_AntiWindup;

typedef enum {
    PID_DERIV_ON_MEASUREMENT = 0,  /* پیش‌فرض — بدون derivative kick */
    PID_DERIV_ON_ERROR,
    PID_DERIV_ON_WEIGHTED_ERROR    /* d/dt(γ·r − y) — حالت 2DOF کامل */
} PID_DerivativeMode;

typedef enum {
    PID_INTEGRATION_BACKWARD_EULER = 0,   /* پیش‌فرض: پایدار، ساده، سازگار با back-calculation */
    PID_INTEGRATION_TRAPEZOIDAL           /* Tustin: دقت بالاتر، نیم‌نمونه فاز بهتر */
} PID_IntegrationMethod;
```

### ۵.۲ ساختارهای Configuration (تودرتو، طبق §۹)

```c
typedef struct {                 /* حداقلی که هر کنترلر لازم دارد */
    PID_Float kp, ki, kd;        /* Parallel form: Ki [1/s]، Kd [s] */
    PID_Float sample_time;       /* dt ثانیه — باید > 0 */
    PID_Direction direction;
    PID_Mode mode;
    PID_IntegrationMethod integration;
} PID_CoreConfig;

typedef struct {
    PID_Float output_min, output_max;
    PID_Float integral_min, integral_max;   /* اگر NAN → از output limits مشتق می‌شود */
    bool      use_integral_limits;
    PID_Float dt_min, dt_max;               /* اعتبارسنجی dt متغیر */
} PID_LimitConfig;

typedef struct {
    PID_DerivativeMode derivative_mode;
    PID_Float tf;                /* ثابت زمانی فیلتر مشتق [s]؛ 0 = بدون فیلتر */
    PID_Float n_filter;          /* جایگزین: Tf = Kd/(N*Kp)، N معمول 5..20؛ 0 = استفاده از tf */
    PID_Float input_lpf_tau;     /* فیلتر ورودی (اختیاری)؛ 0 = خاموش */
} PID_FilterConfig;

typedef struct {
    PID_AntiWindup mode;
    PID_Float kt;                /* بهره back-calculation = 1/Tt؛ 0 → خودکار */
    PID_Float integral_sep_threshold;  /* Integral Separation: |e| بزرگ‌تر → I غیرفعال؛ 0=خاموش */
    PID_Float integral_deadband;       /* |e| کوچک‌تر → I متوقف (ضد lim-cycle)؛ 0=خاموش */
} PID_IntegralConfig;

typedef struct {                 /* 1DOF/2DOF */
    PID_Float beta;              /* وزن setpoint در ترم P (پیش‌فرض 1) */
    PID_Float gamma;             /* وزن setpoint در ترم D (پیش‌فرض 0) */
} PID_WeightConfig;

typedef struct {
    bool  enable;
    PID_Float (*fn)(PID_Float setpoint, PID_Float measurement, void *ctx); /* NULL = مقدار ثابت */
    void *ctx;
    PID_Float gain;              /* ضریب مقیاس روی FF */
} PID_FeedforwardConfig;

typedef struct {
    PID_Float sp_rate_max;       /* واحد/ثانیه؛ 0 = بدون محدودیت */
    PID_Float sp_accel;          /* واحد/ثانیه²؛ 0 = پروفایل مستطیلی (فقط rate) */
    PID_Float sp_decel;
    PID_Float out_slew_max;      /* واحد/ثانیه روی خروجی؛ 0 = خاموش */
} PID_ShaperConfig;

typedef struct {
    bool      enable;
    PID_Float meas_min, meas_max;
    PID_Float meas_rate_max;     /* واحد/ثانیه — تشخیص جهش سنسور */
    PID_Float failsafe_output;
    uint16_t  fault_persist_n;   /* چند نمونه خطا تا latch شدن fault */
    bool      auto_recover;      /* بازگشت خودکار پس از رفع خطا (bumpless) */
} PID_SafetyConfig;

typedef struct {                 /* ساختار چتری — کاربر Basic هرگز نمی‌بیندش */
    PID_CoreConfig        core;
    PID_LimitConfig       limits;
    PID_FilterConfig      filter;
    PID_IntegralConfig    integral;
    PID_WeightConfig      weight;
    PID_FeedforwardConfig feedforward;   /* #if ENABLE_FEEDFORWARD */
    PID_ShaperConfig      shaper;        /* #if ENABLE_SHAPER */
    PID_SafetyConfig      safety;        /* #if ENABLE_SAFETY */
} PID_Config;
```

### ۵.۳ ورودی/خروجی پیشرفته

```c
typedef struct {
    PID_Float measurement;
    PID_Float setpoint;          /* NAN → از setpoint داخلی استفاده کن */
    PID_Float feedforward;       /* NAN → از callback/مقدار داخلی */
    PID_Float dt;                /* <=0 → از sample_time پیکربندی */
    PID_Float tracking;          /* برای PID_AW_TRACKING؛ NAN → بی‌اثر */
    PID_Float schedule_var;      /* متغیر gain scheduling؛ NAN → خودکار */
} PID_Input;

typedef struct {                 /* #if PIDX_ENABLE_DIAGNOSTICS */
    PID_Float setpoint_raw, setpoint_shaped;
    PID_Float measurement_raw, measurement_filtered;
    PID_Float error;
    PID_Float p_term, i_term, d_term, ff_term;
    PID_Float output_unsat, output;
    PID_Float dt_used;
    PID_Float kp_active, ki_active, kd_active;   /* پس از gain scheduling */
    uint32_t  update_count;
    uint32_t  saturation_count;
    uint16_t  flags;             /* PID_FLAG_* */
    PID_StatusCode last_error;
} PID_Status;

/* بیت‌های وضعیت */
#define PID_FLAG_SATURATED_HIGH   (1u << 0)
#define PID_FLAG_SATURATED_LOW    (1u << 1)
#define PID_FLAG_INTEGRAL_ACTIVE  (1u << 2)
#define PID_FLAG_INTEGRAL_LIMITED (1u << 3)
#define PID_FLAG_FAULT            (1u << 4)
#define PID_FLAG_MANUAL           (1u << 5)
#define PID_FLAG_TUNING           (1u << 6)
#define PID_FLAG_DT_VIOLATION     (1u << 7)
#define PID_FLAG_SENSOR_INVALID   (1u << 8)
#define PID_FLAG_SP_RAMPING       (1u << 9)
```

### ۵.۴ `PID_Handle` — چیدمان و منطق

ساختار **مسطح** است (نه تودرتوی config) تا فیلدهای مسیر داغ در یک یا دو cache-line/کنار هم بنشینند
و آدرس‌دهی offset کوچک باشد (روی Cortex-M دستور `VLDR` با offset تا 1020 بایت — ما خیلی کمتریم).

```c
typedef struct PID_Handle {
    /* ---------- HOT: خوانده‌شده در هر Update (اول ساختار) ---------- */
    PID_Float integrator;     /* بر حسب واحد خروجی (نه ∫e) — دلیل: §۶.۳ */
    PID_Float d_state;        /* ترم مشتق فیلترشده (واحد خروجی) */
    PID_Float d_prev_in;      /* ورودی قبلی مشتق (y یا e یا γr−y) */
    PID_Float setpoint;       /* setpoint مؤثر (پس از shaper) */
    PID_Float output;         /* آخرین خروجی نهایی */

    PID_Float kp, ki, kd;     /* بهره‌های فعال (پس از gain scheduling) */
    PID_Float beta, gamma;

    PID_Float c_i;            /* = ki*dt            (Backward-Euler) */
    PID_Float c_d_a;          /* = tf/(tf+dt)       (قطب فیلتر مشتق) */
    PID_Float c_d_b;          /* = kd/(tf+dt)       (بهره فیلتر مشتق) */
    PID_Float c_aw;           /* = kt*dt            (back-calculation) */

    PID_Float out_min, out_max;
    PID_Float i_min, i_max;

    uint32_t  features;       /* بیت‌مسک runtime enable */
    uint16_t  flags;          /* PID_FLAG_* */
    uint8_t   mode;           /* PID_Mode */
    uint8_t   aw_mode;        /* PID_AntiWindup */
    uint8_t   d_mode;         /* PID_DerivativeMode */
    uint8_t   dir_sign;       /* +1 / -1 به‌صورت int8 برای ضرب سریع */
    uint8_t   integ_method;
    uint8_t   init_magic;     /* تشخیص handle مقداردهی‌نشده */

    /* ---------- WARM: پیکربندی، کمتر خوانده می‌شود ---------- */
    PID_Float dt_nominal, dt_min, dt_max, dt_last;
    PID_Float tf, kt, n_filter;
    PID_Float i_sep_threshold, i_deadband;
    PID_Float setpoint_target;    /* هدف قبل از shaper */
    PID_Float manual_output;
    PID_Float ff_value;

#if PIDX_ENABLE_SHAPER
    PID_Float sp_rate_max, sp_accel, sp_decel, sp_velocity, out_slew_max;
#endif
#if PIDX_ENABLE_INPUT_FILTER
    PID_Float in_lpf_a, in_lpf_state;
#endif
#if PIDX_ENABLE_FEEDFORWARD
    PID_Float (*ff_fn)(PID_Float, PID_Float, void*);
    void     *ff_ctx;
    PID_Float ff_gain;
#endif
#if PIDX_ENABLE_SAFETY
    PID_Float meas_min, meas_max, meas_rate_max, failsafe_output, meas_prev;
    uint16_t  fault_count, fault_persist_n;
    bool      auto_recover;
#endif
#if PIDX_ENABLE_GAIN_SCHED
    const struct PID_GainSchedule *sched;
    uint8_t   sched_source;       /* setpoint/measurement/error/|error|/external */
    uint8_t   sched_index_cache;  /* شتاب جست‌وجو (پیوستگی زمانی) */
    PID_Float sched_var_ext;
#endif
#if PIDX_ENABLE_DIAGNOSTICS
    PID_Status status;
#endif
#if PIDX_ENABLE_TELEMETRY
    struct PID_Telemetry *telemetry;   /* بافر متعلق به کاربر — بدون malloc */
#endif
    PID_StatusCode last_error;
} PID_Handle;
```

**تخمین اندازه (float32، ARM32، بدون padding اضافی):**

| پیکربندی | sizeof(PID_Handle) |
|---|---:|
| Minimal (`PIDX_PROFILE_MINIMAL`: بدون shaper/safety/diag/FF) | **≈ 132 B** |
| Typical (FF + shaper + safety) | ≈ 208 B |
| Full (به‌علاوه diag + gain-sched + telemetry ptr) | ≈ 344 B |
| `PID_AutoTune` (ساختار جدا) | ≈ 140 B |
| `PID_Cascade` (تا ۴ حلقه، اندازهٔ ثابت) | 136 B اندازه‌گیری‌شده روی x86-64 (فقط اشاره‌گر + پیکربندی هر سطح) |

---

## ۶. PID Core — ریاضیات و Execution Flow (PHASE 4–8 preview)

### ۶.۱ فرم پیوسته (2DOF، مشتق روی اندازه‌گیری، فیلترشده)

$$
u(t)=\underbrace{K_p\big(\beta r - y\big)}_{P}
+\underbrace{K_i\!\int_0^t\!\big(r-y\big)d\tau}_{I}
+\underbrace{\frac{K_d\,s}{1+sT_f}\big(\gamma r - y\big)}_{D}
+\underbrace{u_{ff}}_{FF}
$$

- $\beta=1,\gamma=0$ → PI-D کلاسیک (پیش‌فرض امن، بدون derivative kick)
- $\beta=1,\gamma=1$ → PID کلاسیک روی خطا
- $0<\beta<1$ → کاهش overshoot بدون تغییر پاسخ اغتشاش ← **این جوهرهٔ 2DOF است**
- انتگرال **همیشه** روی خطای کامل $(r-y)$ است؛ در غیر این صورت خطای ماندگار صفر نمی‌شود.

### ۶.۲ گسسته‌سازی و توجیه انتخاب (§۷۵)

**انتگرال — Backward Euler (پیش‌فرض):**
$$I_k = I_{k-1} + K_i\,\Delta t\, e_k$$
انتخاب شد چون: (۱) پایدار به ازای هر $\Delta t>0$؛ (۲) با Back-Calculation ترکیب تمیزی دارد
(جمله اصلاحی همان گام به‌روزرسانی است)؛ (۳) یک ضرب و یک جمع.
Forward Euler عمداً استفاده نمی‌شود چون قطب گسسته را بیرون دایره واحد می‌برد وقتی $K_i\Delta t$ بزرگ شود.
**Trapezoidal** به‌صورت گزینه: $I_k = I_{k-1} + \frac{K_i\Delta t}{2}(e_k+e_{k-1})$ — دقت بالاتر
(نیم‌نمونه تأخیر کمتر) به قیمت یک state اضافه؛ برای حلقه‌های آهسته (دما) ارزش زیادی ندارد، برای
حلقه جریان با $\Delta t$ بزرگ نسبت به دینامیک ارزشمند است.

**مشتق — Filtered Backward Difference (Åström–Hägglund):**
$$D_k = \frac{T_f}{T_f+\Delta t}D_{k-1} - \frac{K_d}{T_f+\Delta t}\big(x_k - x_{k-1}\big),
\quad x = y \;\text{یا}\; \gamma r - y$$
این فرم **به ازای هر $T_f \ge 0$ و هر $\Delta t>0$ بدون قید پایدار است** (قطب در
$T_f/(T_f+\Delta t) \in [0,1)$) — برخلاف پیاده‌سازی رایج «مشتق خام سپس LPF جداگانه» که در
$T_f < \Delta t/2$ به نوسان می‌افتد. با $T_f=0$ دقیقاً به مشتق خام $-K_d\Delta x/\Delta t$ فرومی‌پاشد.
اگر کاربر $N$ بدهد: $T_f = K_d/(N\,K_p)$ (با محافظت در برابر $K_p\to 0$؛ در آن حالت $T_f$ صریح لازم است).

**تقسیم‌ها:** هر سه ضریب $c_i, c_{da}, c_{db}$ فقط در `pidp_recompute()` محاسبه می‌شوند
(هنگام تغییر gain/dt/tf). مسیر داغ **صفر تقسیم** دارد.

### ۶.۳ تصمیم کلیدی: واحد integrator

`integrator` مقدار **ترم I بر حسب واحد خروجی** را نگه می‌دارد (یعنی $K_i\!\int\!e$)، نه $\int\!e$.
پیامدها:

| اثر | نتیجه |
|---|---|
| تغییر runtime «Ki» | خروجی **پرش نمی‌کند** (تاریخچه بازمقیاس نمی‌شود) ← مطلوب §۳۲ |
| Gain Scheduling | عبور از مرز ناحیه بدون discontinuity ← مطلوب §۲۳ |
| Anti-windup clamp/back-calc | حدود انتگرال هم‌واحد با حدود خروجی → تنظیم شهودی |
| عیب | «$\int e$ خام» مستقیماً در دسترس نیست (اگر Ki=0 اطلاعات تاریخی حفظ نمی‌شود) |

برای کاربری که رفتار کلاسیک (بازمقیاس) را می‌خواهد:
`PID_SetGainsRescaleIntegral()` ارائه می‌شود که $I \leftarrow I\cdot K_i^{new}/K_i^{old}$ را اعمال می‌کند
(با محافظت در برابر $K_i^{old}=0$). **رفتار پیش‌فرض = bumpless.**

### ۶.۴ Execution Flow دقیق `PID_Update()`

```
PID_Update(h, y)  ──▶  PID_UpdateEx(h, &input)          [inline wrapper]
                                │
     ┌──────────────────────────▼───────────────────────────┐
     │ 0. GUARD  (#if SAFETY یا build DEBUG)                 │
     │    h==NULL? → 0 و last_error=NULL                     │
     │    init_magic معتبر؟  isfinite(y)?                    │
     │    y در [meas_min,meas_max]؟  |Δy|/dt ≤ rate_max؟     │
     │    → در صورت خطا: fault_count++ ؛ اگر latch شد:       │
     │      خروجی=failsafe، integrator منجمد، FLAG_FAULT     │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 1. dt  (فقط در UpdateDt)                              │
     │    dt<=0 → ERR_INVALID_DT، از dt_nominal استفاده کن   │
     │    dt≠dt_last → pidp_recompute_dt()  (تقسیم اینجا)    │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 2. INPUT FILTER (#if، runtime bit)                    │
     │    y_f = a*y_f + (1-a)*y                              │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 3. GAIN SCHEDULING (#if، runtime bit)                 │
     │    var = {sp|y|e|abs(e)|ext} → درون‌یابی خطی جدول     │
     │    اگر gainها تغییر کرد → recompute ضرایب             │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 4. SETPOINT SHAPER (#if، runtime bit)                 │
     │    پروفایل ذوزنقه‌ای سرعت: sp → sp_target             │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 5. MODE                                               │
     │  MANUAL → u = manual_output؛ سپس BACK-SOLVE:          │
     │           I = u_clamped − P − D − FF                  │
     │           (تضمین Bumpless در لحظه سوئیچ به AUTO)      │
     │           d_state و d_prev_in هم tracked می‌مانند     │
     │  HOLD   → مثل AUTO ولی بدون به‌روزرسانی I             │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 6. ERROR  e = dir_sign * (sp − y_f)                   │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 7. P  = kp * dir_sign * (beta*sp − y_f)               │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 8. D  (x طبق d_mode؛ همیشه فرم فیلترشده پایدار)       │
     │    D = c_d_a*D − c_d_b*(x − x_prev) ؛ x_prev = x      │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 9. FF (#if) : ff = ff_gain * (fn? fn(sp,y,ctx): val)  │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 10. INTEGRAL UPDATE (شرطی)                            │
     │  گیت‌ها به ترتیب ارزان→گران:                          │
     │   • bit I_ENABLE                                      │
     │   • separation:   |e| > sep_thr  → skip               │
     │   • deadband:     |e| < db       → skip               │
     │   • conditional:  saturated && sign(e)==sign(sat)→skip│
     │  I += c_i * e            (Backward Euler)             │
     │  clamp → [i_min, i_max]  (اگر AW_CLAMP)               │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 11. SUM  u_raw = P + I + D + FF                       │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 12. OUTPUT LIMIT  u = clamp(u_raw)                    │
     │     پرچم‌های SATURATED_HIGH/LOW                       │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 13. BACK-CALCULATION (اگر AW_BACK_CALCULATION)        │
     │     I += c_aw * (u − u_raw)      ← اصلاح پس از اشباع  │
     │     (AW_TRACKING: I += c_aw*(u_track − u_raw))        │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 14. OUTPUT SHAPER (#if): slew-rate limit روی u        │
     └──────────────────────────┬───────────────────────────┘
     ┌──────────────────────────▼───────────────────────────┐
     │ 15. DIAG/TELEMETRY (#if): پرکردن status، push بافر    │
     └──────────────────────────┬───────────────────────────┘
                                ▼   return u
```

**ترتیب ۱۲→۱۳ عمدی است:** back-calculation باید *بعد از* اشباع و *در همان نمونه* اعمال شود
تا انتگرال‌گیر بلافاصله «بداند» عملگر جا ندارد. اعمال آن در نمونه بعد، یک نمونه تأخیر و
overshoot اضافی می‌سازد.

### ۶.۵ Fast Path

```c
PID_Float PID_UpdateFast(PID_Handle *h, PID_Float y);
```
قرارداد صریح: فقط **P + I(Backward-Euler) + D(filtered, on measurement) + clamp + AW_CLAMP**.
Shaper، safety، gain-sched، FF، telemetry و mode-switch را **نادیده می‌گیرد** (نه اینکه چک کند).
مصرف: ۹ ضرب/جمع، ۴ مقایسه، صفر تقسیم، صفر شاخه غیرقابل پیش‌بینی جز clamp.
هدف: حلقه جریان ۲۰ kHz روی Cortex-M4F در **< 100 cycles**.
مستندسازی: اگر کاربر featureای فعال کرده که fast path نادیده می‌گیرد، `PID_UpdateFast` در build
دیباگ یک assert می‌دهد (در release بی‌هزینه).

---

## ۷. Public API (PHASE 2)

قرارداد کلی: توابع `Set*`/`Get*`/`Init*` کد خطا برمی‌گردانند؛ `Update*` مقدار خروجی برمی‌گرداند و
خطا را در `last_error` + flags می‌گذارد (تا مسیر داغ تمیز بماند). نسخه `PID_UpdateEx()` هر دو را می‌دهد.

### Level 1 — Basic (۶ تابع، تمام چیزی که مبتدی لازم دارد)
```c
PID_StatusCode PID_InitDefault (PID_Handle *h);                 /* dt=0.01، بدون limit */
PID_StatusCode PID_SetGains    (PID_Handle *h, PID_Float kp, PID_Float ki, PID_Float kd);
PID_StatusCode PID_SetSetpoint (PID_Handle *h, PID_Float sp);
PID_Float      PID_Update      (PID_Handle *h, PID_Float measurement);
PID_StatusCode PID_Reset       (PID_Handle *h);
PID_Float      PID_GetOutput   (const PID_Handle *h);
```

### Level 2 — Intermediate
```c
PID_StatusCode PID_ConfigDefault    (PID_Config *cfg);
PID_StatusCode PID_Init             (PID_Handle *h, const PID_Config *cfg);
PID_StatusCode PID_Deinit           (PID_Handle *h);   /* بدون free — فقط invalidate امن */
PID_StatusCode PID_SetSampleTime    (PID_Handle *h, PID_Float dt);
PID_Float      PID_UpdateDt         (PID_Handle *h, PID_Float y, PID_Float dt);
PID_StatusCode PID_SetOutputLimits  (PID_Handle *h, PID_Float min, PID_Float max);
PID_StatusCode PID_SetIntegralLimits(PID_Handle *h, PID_Float min, PID_Float max);
PID_StatusCode PID_SetAntiWindup    (PID_Handle *h, PID_AntiWindup m, PID_Float kt);
PID_StatusCode PID_SetDerivativeMode(PID_Handle *h, PID_DerivativeMode m);
PID_StatusCode PID_SetDerivativeFilter(PID_Handle *h, PID_Float tf);      /* ثانیه */
PID_StatusCode PID_SetDerivativeFilterN(PID_Handle *h, PID_Float n);      /* Tf=Kd/(N·Kp) */
PID_StatusCode PID_SetDirection     (PID_Handle *h, PID_Direction d);
PID_StatusCode PID_SetMode          (PID_Handle *h, PID_Mode m);          /* bumpless */
PID_StatusCode PID_SetManualOutput  (PID_Handle *h, PID_Float u);
PID_StatusCode PID_SetSetpointRamp  (PID_Handle *h, PID_Float rate, PID_Float acc, PID_Float dec);
PID_StatusCode PID_SetOutputSlewRate(PID_Handle *h, PID_Float slew);
```

### Level 3 — Advanced
```c
PID_Float      PID_UpdateEx        (PID_Handle *h, const PID_Input *in, PID_StatusCode *err);
PID_StatusCode PID_SetKp/Ki/Kd     (PID_Handle *h, PID_Float v);          /* bumpless */
PID_StatusCode PID_SetGainsRescaleIntegral(PID_Handle*, PID_Float, PID_Float, PID_Float);
PID_StatusCode PID_SetWeights      (PID_Handle *h, PID_Float beta, PID_Float gamma); /* 2DOF */
PID_StatusCode PID_SetFeedforward  (PID_Handle *h, PID_Float ff);
PID_StatusCode PID_SetFeedforwardFn(PID_Handle *h, PID_FeedforwardFn fn, void *ctx, PID_Float g);
PID_StatusCode PID_SetIntegralSeparation(PID_Handle *h, PID_Float threshold);
PID_StatusCode PID_SetIntegralDeadband  (PID_Handle *h, PID_Float db);
PID_StatusCode PID_EnableIntegral  (PID_Handle *h, bool en);
PID_StatusCode PID_SetIntegrator   (PID_Handle *h, PID_Float value);      /* external reset */
PID_StatusCode PID_SetTrackingInput(PID_Handle *h, PID_Float u_track);
PID_StatusCode PID_SetSafety       (PID_Handle *h, const PID_SafetyConfig *sc);
PID_StatusCode PID_SetFaultOutput  (PID_Handle *h, PID_Float u);
PID_StatusCode PID_ClearFault      (PID_Handle *h);
PID_StatusCode PID_EnableFeature   (PID_Handle *h, uint32_t mask, bool en); /* runtime toggle */
PID_StatusCode PID_GetStatus       (const PID_Handle *h, PID_Status *out);
PID_StatusCode PID_GetLastError    (const PID_Handle *h, PID_StatusCode *e);
PID_Float      PID_GetError        (const PID_Handle *h);
```

### Level 4 — Expert (هر گروه در هدر خودش)
```c
/* pid_gainsched.h */
PID_StatusCode PID_GainSched_Init  (PID_GainSchedule *s, const PID_GainPoint *pts, uint8_t n,
                                    PID_SchedSource src, PID_SchedInterp interp);
PID_StatusCode PID_GainSched_Attach(PID_Handle *h, const PID_GainSchedule *s);
PID_StatusCode PID_GainSched_SetVar(PID_Handle *h, PID_Float v);   /* منبع external */

/* pid_cascade.h  (امضای نهایی؛ dt صریح است تا هستهٔ بدون HAL حفظ شود) */
PID_StatusCode PID_Cascade_Init         (PID_Cascade *c, PID_Handle **loops, uint8_t n);
PID_StatusCode PID_Cascade_ConfigLevel  (PID_Cascade *c, uint8_t idx, uint16_t decimation,
                                         PID_Float sp_min, PID_Float sp_max);
PID_StatusCode PID_Cascade_SetAntiWindup(PID_Cascade *c, PID_CascadeAW mode, PID_Float aw_gain);
PID_Float      PID_Cascade_Update       (PID_Cascade *c, const PID_Float *measurements,
                                         PID_Float sp, PID_Float dt);
PID_StatusCode PID_Cascade_SetMode      (PID_Cascade *c, PID_Mode m);   /* هماهنگ، bumpless */
PID_StatusCode PID_Cascade_SetManualOutput(PID_Cascade *c, PID_Float u);
PID_StatusCode PID_Cascade_Validate     (const PID_Cascade *c, PID_Float *worst_ratio, uint8_t *worst_idx);
bool           PID_Cascade_IsSaturated  (const PID_Cascade *c);

/* pid_autotune.h */
PID_StatusCode PID_AutoTune_Init     (PID_AutoTune *t, const PID_AutoTuneConfig *cfg);
PID_StatusCode PID_AutoTune_ConfigDefault(PID_AutoTuneConfig *cfg, PID_TuneMethod m);
PID_StatusCode PID_AutoTune_Start    (PID_AutoTune *t, PID_Handle *h, PID_Float sp);
PID_Float      PID_AutoTune_Update   (PID_AutoTune *t, PID_Float measurement, PID_Float dt);
PID_StatusCode PID_AutoTune_Abort    (PID_AutoTune *t);
bool           PID_AutoTune_IsComplete(const PID_AutoTune *t);
PID_TuneState  PID_AutoTune_GetState (const PID_AutoTune *t);
PID_StatusCode PID_AutoTune_GetResult(const PID_AutoTune *t, PID_AutoTuneResult *r);
PID_StatusCode PID_AutoTune_Apply    (PID_AutoTune *t, PID_Handle *h); /* با validation+bumpless */
PID_StatusCode PID_AutoTune_RegisterRule(PID_AutoTune *t, PID_TuneRuleFn fn, void *ctx);

/* pid_diag.h */
PID_StatusCode PID_Telemetry_Attach  (PID_Handle *h, PID_Telemetry *tb,
                                      PID_TelemetryRecord *storage, uint16_t capacity);
uint16_t       PID_Telemetry_Read    (PID_Telemetry *tb, PID_TelemetryRecord *dst, uint16_t max);
PID_StatusCode PID_Profile_Attach    (PID_Handle *h, PID_CycleFn now, PID_Profile *p);

/* pid_fixed.h  — کنترلر مستقل، نه لایه روی float */
PID_StatusCode PIDq_Init   (PIDq_Handle *h, const PIDq_Config *cfg);
int16_t        PIDq_Update (PIDq_Handle *h, int16_t measurement_q15);
```

**پایداری API (§۶۷/۶۸):** Level 1 هرگز تغییر نمی‌کند. رشد آینده فقط از سه راه:
(الف) تابع جدید، (ب) فیلد جدید در انتهای structهای config با پیش‌فرض صفر-معنادار،
(ج) عضو جدید در انتهای enumها. `PID_Config` نسخه‌دار می‌شود (`cfg.abi_version`) تا
`PID_Init` بتواند configهای قدیمی را تشخیص دهد.

---

## ۸. Dependency Graph

```
            pid_conf.h        pid_status.h   (هر دو بدون وابستگی)
                    ╲            ╱   ╲
                 pid_math.h ────╱     ╲
                    ╱   ╲              ╲
          pid_filter.h  pid_shaper.h    │
                    ╲       ╱           │
                  pid_types.h ──────────┘
                        │      ╲
                      pid.h   pid_version.h
                        │             │
                    ┌───┴─────────────┘
                    └────▶ pid.c ◀──────┐
                              ▲                    │
        ┌──────────┬──────────┼──────────┬─────────┴──────┐
        │          │          │          │                │
 pid_autotune.h  pid_       pid_      pid_diag.h    pid_fixed.h
        │       cascade.h  gainsched.h    │          (مستقل کامل:
 pid_autotune_      │          │          │           فقط pid_conf.h
   rules.c          │          │          │           + stdint)
        └───────────┴──────────┴──────────┘
                    همه یک‌طرفه به pid.h
```

قواعد الزام‌آور (در CI بررسی می‌شوند با یک اسکریپت grep ساده):
- `src/pid.c` مجاز به include کردن این‌هاست و بس: `pid.h`, `pid_types.h`, `pid_conf.h`,
  `pid_status.h`, `pid_math.h`, `pid_shaper.h`, `pid_filter.h`, `pid_gainsched.h`, `pid_diag.h`.
  (سه تای آخر فقط داخل `#if` مربوطه‌شان.)
- `pid_filter.h` و `pid_shaper.h` **بالادست** `pid_types.h` هستند، نه پایین‌دست آن. علت: هندل
  کنترلر یک `PID_LPF1` را به‌صورت value درون خودش نگه می‌دارد (فیلتر ورودی)، پس تعریف آن باید
  پیش از `PID_Handle` دیده شود. برای شکستن حلقهٔ include، `PID_StatusCode` به یک برگ مستقل
  (`pid_status.h`) منتقل شد تا ماژول‌های مستقل بدون کشیدن کل `pid_types.h` بتوانند خطا برگردانند.
- پروفایل حرکتی **یک پیاده‌سازی بیشتر ندارد**: تابع inline‌ی `pids_profile_step()` در
  `pid_shaper.h`. هم شیپر داخلی هستهٔ (`PID_SetSetpointRamp`) و هم `PID_Shaper` مستقل هر دو
  همان را صدا می‌زنند، پس رفتارشان طبق ساخت یکسان است نه طبق قرارداد.
- هیچ فایلی در `src/` مجاز به include کردن `stm32*.h`, `cmsis*`, `stdio.h`, `stdlib.h` نیست.
- `pid_fixed.c` هیچ وابستگی به `pid.h` ندارد (کنترلر کاملاً جدا — دلیل در ۱۲.۵).

---

## ۹. Auto-Tune Architecture (PHASE 12 preview) — مهم‌ترین بخش

### ۹.۱ تفکیک بنیادی: Identification ≠ Tuning Rule

این تفکیک، خطای رایج کتابخانه‌های موجود را حذف می‌کند:

```
┌───────────────────────┐        ┌──────────────────┐        ┌──────────────┐
│  IDENTIFICATION       │        │  PLANT MODEL     │        │ TUNING RULE  │
│  (آزمایش روی سیستم)   │───────▶│  (داده میانی)    │───────▶│ (فرمول)      │
├───────────────────────┤        ├──────────────────┤        ├──────────────┤
│ RELAY (closed-loop)   │        │ FREQ: {Ku, Pu}   │        │ ZN, TL,      │
│  ± h با hysteresis ε  │        │                  │        │ Pessen,      │
│                       │        │                  │        │ No/Some-OS,  │
│                       │        │                  │        │ AMIGO-freq   │
├───────────────────────┤        ├──────────────────┤        ├──────────────┤
│ STEP (open-loop bump) │        │ FOPDT: {K, T, L} │        │ Cohen-Coon,  │
│  پله در حالت MANUAL   │        │                  │        │ AMIGO-step,  │
│                       │        │                  │        │ IMC/λ,       │
│                       │        │                  │        │ ZN-step      │
└───────────────────────┘        └──────────────────┘        └──────────────┘
```

**ادعای صریح (اصلاح Specification):** Cohen-Coon و IMC/λ و AMIGO-step **ریاضیاً به مدل FOPDT
$(K, T, L)$ نیاز دارند** و از داده رله (که فقط یک نقطه فرکانسی $(K_u,P_u)$ می‌دهد) قابل استخراج
دقیق نیستند. اگر کاربر ترکیب ناسازگار بخواهد، API با `PID_ERR_TUNE_MODEL_MISMATCH` برمی‌گردد
و پیشنهاد روش درست را در نتیجه می‌گذارد. **هیچ تبدیل ساختگی $(K_u,P_u)\to(K,T,L)$ انجام نمی‌شود.**

### ۹.۲ Relay Feedback (Åström–Hägglund) — ریاضیات واقعی

خروجی رله با هیسترزیس $\varepsilon$ حول بایاس $u_0$:
$$u = u_0 + h\cdot\operatorname{sign}_\varepsilon(e)$$
پس از رسیدن به چرخه حدی پایدار با دامنه پیک-تا-پیک $2a$ و دوره $P_u$، توصیف تابعی رله می‌دهد:
$$K_u = \frac{4h}{\pi\sqrt{a^2-\varepsilon^2}} \qquad (a>\varepsilon)$$
- $h$: نصف دامنه رله، $a$: دامنه نوسان خروجی (نصف پیک-تا-پیک، میانگین چند سیکل)
- هیسترزیس $\varepsilon$ **الزامی** است در حضور نویز؛ قاعده: $\varepsilon \approx 2\sigma_{noise}$ تا $3\sigma$
- بدون هیسترزیس ($\varepsilon=0$): $K_u = 4h/(\pi a)$

**اعتبارسنجی چرخه حدی (نه صرفاً «چند پیک شمردم»):** پذیرش وقتی برای $m$ سیکل متوالی
(پیش‌فرض ۴، پس از دور انداختن ۲ سیکل اول):
$$\frac{|P_i - \bar P|}{\bar P} < 0.10 \;\;\wedge\;\; \frac{|a_i-\bar a|}{\bar a} < 0.15
\;\;\wedge\;\; \frac{|a^+ - a^-|}{a^+ + a^-} < 0.30$$
شرط سوم عدم‌تقارن را می‌گیرد (نشانه بایاس یا نانخطی) و اگر نقض شود کاربر با
`PID_WARN_ASYMMETRIC` مطلع می‌شود اما نتیجه رد نمی‌شود (فقط علامت‌گذاری).

### ۹.۳ Step / FOPDT Identification

پله $\Delta u$ در حالت MANUAL؛ نیاز به حالت پایای اولیه (تشخیص خودکار: $|\dot y| < $ آستانه
برای $T_{settle}$). سپس:
- $K = \Delta y_\infty/\Delta u$
- **روش سطح/ممان (پیاده‌سازی‌شده)** — دو ممان اول باقیمانده‌ی $e(t)=y_\infty-y(t)$
  روی پاسخ نرمال‌شده:

  $$A_1 = \frac{1}{\Delta y}\int_0^{t_e} e\,dt = L+T, \qquad
    M_1 = \frac{1}{\Delta y}\int_0^{t_e} t\,e\,dt = \frac{L^2}{2}+LT+T^2$$

  که وارون بسته دارد (با sympy تأیید شد):
  $$T=\sqrt{2M_1-A_1^2}, \qquad L=A_1-T$$

> **تصحیح نسبت به پیش‌نویس این سند.** پیش‌نویس، روش دو-نقطه‌ای
> ($T=1.5(t_{63.2}-t_{28.3})$) را پیش‌فرض گرفته بود. در پیاده‌سازی کنار گذاشته شد:
> آن روش مدل را از **دقیقاً دو نمونه** می‌خواند و به تخمین $y_\infty$ نیاز دارد
> در حالی که پاسخ هنوز در حال حرکت است. روش سطح روی کل گذرا میانگین می‌گیرد.
> هزینه‌ی محاسباتی دو ضرب-و-جمع در هر نمونه است — استدلال «برای MCU سبک‌تر» در
> پیش‌نویس نادرست بود.
>
> سه دام عددی که در پیاده‌سازی کشف و رفع شد (هر سه با تست عددی مستند شده‌اند):
> ۱. **نرمال‌سازی**: بدون تقسیم بر $\Delta y$، رادیکال برای هر $\Delta y>1$ منفی
>    می‌شود (برای پلنت نمونه: $2M_1-A_1^2=-1.38$) و شناسایی همیشه شکست می‌خورد.
> ۲. **بازوی ممان در نقطه‌ی میانی**: تفریق تحلیلی از $\int t\,dt=t_e^2/2$ استفاده
>    می‌کند، ولی جمع گسسته با بازوی انتهای بازه برابر $t_e^2/2 + t_e\,dt/2$ است.
>    این کسری دقیقاً برابر $t_e\,dt/2$ **با طولانی‌تر شدن تست رشد می‌کند** — یعنی
>    آزمایش دقیق‌تر، مدل بدتری می‌داد (خطای $T$ از ۶.۷٪ در ۵ ثانیه به ۲۲.۴٪ در
>    ۴۰ ثانیه). با بازوی نقطه‌ی میانی، خطا روی ۰.۵٪ ثابت می‌ماند.
> ۳. **حساسیت به $y_\infty$**: چون $A_1 = t_e - \text{area}/\Delta y$ تفاضل دو عدد
>    هم‌اندازه است، خطای نسبی $\Delta y$ حدود $t_e/A_1$ برابر (≈۸×) تقویت می‌شود.
>    بنابراین $y_\infty$ از **میانگین پنجره‌ی تأییدشده‌ی صاف** گرفته می‌شود، نه از
>    تخمین LPF که گذرا را دنبال می‌کند.

### ۹.۴ جدول قوانین Tuning (داده‌محور، قابل توسعه)

پیاده‌سازی به‌صورت آرایه `static const` از توصیف‌گرها، نه زنجیره `if/else`:

```c
typedef enum { PID_MODEL_FREQ, PID_MODEL_FOPDT } PID_ModelKind;

typedef struct {
    PID_ModelKind kind;
    union {
        struct { PID_Float ku, pu; } freq;
        struct { PID_Float k, t, l; } fopdt;
    } m;
    PID_Float noise_sigma;      /* تخمین نویز حین آزمایش (برای انتخاب Tf) */
    uint8_t   quality;          /* 0..100 — کیفیت برازش/تکرارپذیری */
} PID_PlantModel;

typedef PID_StatusCode (*PID_TuneRuleFn)(const PID_PlantModel *m,
                                         PID_TuneStructure s,   /* P / PI / PID */
                                         PID_Gains *out, void *ctx);
```

| قانون | مدل | Kp | Ti | Td | کاربرد پیشنهادی |
|---|---|---|---|---|---|
| ZN-PID | FREQ | 0.6Ku | Pu/2 | Pu/8 | مرجع تاریخی، تهاجمی (¼-decay) |
| ZN-PI | FREQ | 0.45Ku | Pu/1.2 | — | حلقه‌های نویزی |
| Tyreus–Luyben PID | FREQ | 0.45Ku | 2.2Pu | Pu/6.3 | **پیش‌فرض توصیه‌شده** — مقاوم، کم-overshoot |
| Pessen Integral | FREQ | 0.7Ku | Pu/2.5 | 3Pu/20 | پاسخ سریع |
| Some Overshoot | FREQ | 0.33Ku | Pu/2 | Pu/3 | سرووها |
| No Overshoot | FREQ | 0.2Ku | Pu/2 | Pu/3 | دما/فرآیند |
| AMIGO-freq | FREQ | تابع نسبت κ | — | — | مدرن، متعادل |
| Cohen–Coon | FOPDT | $\frac{1}{K}\frac{T}{L}(1.35+\frac{L}{4T})$ | … | … | $L/T \in [0.1, 1]$ |
| AMIGO-step | FOPDT | $\frac{1}{K}(0.2+0.45\frac{T}{L})$ | … | … | **پیش‌فرض FOPDT** |
| IMC / λ-tuning | FOPDT | $\frac{T+L/2}{K(\lambda+L/2)}$ | $T+L/2$ | $\frac{TL}{2T+L}$ | تنظیم صریح مصالحه سرعت/مقاومت با λ |
| CUSTOM | هر دو | callback کاربر | | | توسعه‌پذیری §۲۵ Method 5 |

سپس تبدیل به فرم موازی: $K_i = K_p/T_i$، $K_d = K_p T_d$، و $T_f = T_d/N$ با $N=10$ پیش‌فرض.

### ۹.۵ ماشین حالت (غیرمسدودکننده)

```
        PID_AutoTune_Start()
                │
                ▼
    ┌──────────────────────┐
    │       IDLE           │◀──────────────────────────┐
    └──────────┬───────────┘                           │
               ▼                                       │
    ┌──────────────────────┐   timeout/نوسان اولیه     │
    │   STABILIZING        │───────────────┐           │
    │ صبر تا |dy/dt|<thr   │               │           │
    └──────────┬───────────┘               │           │
               ▼                           │           │
    ┌──────────────────────┐               │           │
    │ RELAY_WARMUP         │               │           │
    │ ۱–۲ سیکل دور ریخته   │               │           │
    └──────────┬───────────┘               │           │
               ▼                           ▼           │
    ┌──────────────────────┐        ┌─────────────┐    │
    │ RELAY_OSCILLATING    │───────▶│   FAILED    │    │
    │ ثبت پیک/دوره         │  خطا   │ TIMEOUT     │    │
    └──────────┬───────────┘        │ UNSTABLE    │    │
               │ m سیکل معتبر       │ NO_OSC      │    │
               ▼                    │ ABORTED     │    │
    ┌──────────────────────┐        └──────┬──────┘    │
    │ ANALYZING            │               │           │
    │ Ku, Pu, σ_noise      │               │           │
    └──────────┬───────────┘               │           │
               ▼                           │           │
    ┌──────────────────────┐               │           │
    │ COMPUTING (rule fn)  │               │           │
    └──────────┬───────────┘               │           │
               ▼                           │           │
    ┌──────────────────────┐  رد شد        │           │
    │ VALIDATING           │───────────────┤           │
    │ finite? مثبت? بازه?  │               │           │
    └──────────┬───────────┘               │           │
               ▼ قبول                      │           │
    ┌──────────────────────┐               │           │
    │ COMPLETE             │───────────────┴───────────┘
    │ (منتظر Apply کاربر)  │   PID_AutoTune_Init/Abort
    └──────────────────────┘
```
مسیر STEP: `IDLE → STABILIZING → STEP_APPLY → STEP_RECORD → ANALYZING → …` (همان انتهای مشترک).

**غیرمسدودکننده بودن:** هر state فقط چند مقایسه در هر فراخوانی انجام می‌دهد. هیچ `while` انتظاری،
هیچ `delay`. تنها محاسبه سنگین (`sqrtf` و تقسیم‌های فرمول) در گذارِ یک‌باره ANALYZING/COMPUTING
رخ می‌دهد — تخمین < ۲ µs روی M4F، که حتی داخل ISR هم قابل قبول است؛ با این حال گزینه
`defer_compute` وجود دارد تا این گام در main loop انجام شود.

### ۹.۶ لایه ایمنی Auto-Tune

| محافظ | پارامتر | رفتار در نقض |
|---|---|---|
| کران خروجی | `u_min/u_max` | خروجی رله همیشه clamp؛ کاهش h در صورت نیاز |
| کران اندازه‌گیری | `meas_min/meas_max` | ABORT فوری + بازگشت به mode قبلی |
| حداکثر دامنه نوسان | `osc_max` | ABORT (`UNSTABLE`) |
| حداقل دامنه نوسان | `osc_min` | پس از `n` سیکل → `NO_OSCILLATION`؛ گزینه: افزایش خودکار h |
| حداکثر زمان کل | `timeout_s` | `TIMEOUT` |
| حداکثر سیکل | `max_cycles` | `TIMEOUT` |
| نرخ تغییر غیرفیزیکی | `rate_max` | ABORT |
| Watchdog کاربر | `abort_fn(ctx)` | فراخوانی هر نمونه؛ true → ABORT |
| بازگشت امن | همیشه | بازگرداندن mode/gain/خروجی قبلی handle |

Callbackها: `on_start`, `on_progress(percent, state)`, `on_complete(result)`, `on_error(code)` —
همه اختیاری (NULL-able)، از ISR فراخوانی می‌شوند ⇒ مستند: باید کوتاه و بدون blocking باشند.

### ۹.۷ اعتبارسنجی نتیجه (§۳۱)
`isfinite(Kp,Ki,Kd)` · `Kp>0` · `Ki≥0` · `Kd≥0` · `Pu ∈ [20·dt, timeout/4]`
(کف $20\Delta t$ — در ۹.۸ توضیح داده شده؛ در پیش‌نویس $8\Delta t$ بود که ±۲۵٪ خطای کوانتیزاسیون می‌داد) ·
`Ku ∈ [ku_min, ku_max]` · `quality ≥ 50` · و در نهایت **بررسی حد بالای gain نسبت به بازه خروجی**:
$K_p \cdot \text{span}(y_{typ}) \le 10\cdot\text{span}(u)$ به‌عنوان سلامت‌سنجی مقیاس.

### ۹.۸ نتایج اندازه‌گیری‌شده‌ی PHASE 12 (روی میزبان x86-64)

تست‌ها: `tests/test_autotune_accuracy.c` و `tests/test_autotune_safety.c`.
پلنت مرجع: $G(s)=K e^{-Ls}/(1+Ts)$ با انتگرال‌گیری اویلر و بافر تأخیر گسسته.

**شناسایی Step (بدون نویز) — خطای مدل:**

| پلنت | K | T | L | خطای K | خطای T | خطای L |
|---|---|---|---|---|---|---|
| nominal (2, 1, 0.3) | 1.9994 | 0.9844 | 0.3081 | −0.0٪ | −1.6٪ | +2.7٪ |
| slow thermal (1, 10, 1) | 0.9996 | 9.8493 | 1.0948 | −0.0٪ | −1.5٪ | +9.5٪ |
| fast, small L (5, 0.5, 0.05) | 4.9981 | 0.4916 | 0.0561 | −0.0٪ | −1.7٪ | +12.2٪ |
| L/T = 1 (0.5, 2, 2) | 0.4999 | 1.9856 | 2.0074 | −0.0٪ | −0.7٪ | +0.4٪ |
| L/T = 0.05 (1, 1, 0.05) | 0.9996 | 0.9816 | 0.0602 | −0.0٪ | −1.8٪ | +20.4٪ |
| big furnace (3, 20, 4) | 2.9990 | 19.7226 | 4.1722 | −0.0٪ | −1.4٪ | +4.3٪ |

**با نویز اندازه‌گیری ۱٪:** هر ۶ پلنت `rc=0`؛ خطای $T$ در بازه‌ی −۰.۷٪ تا −۴.۲٪
باقی می‌ماند. $L$ برای پلنت‌های با $L$ خیلی کوچک بدتر می‌شود (تا +۴۱٪ در
$L/T=0.05$) — انتظار می‌رود، چون آنجا $L$ خودش از مرتبه‌ی چند نمونه است.

**شناسایی Relay — $K_u$ ذاتاً محافظه‌کارانه است:**

| پلنت | $P_u$ اندازه‌گیری | خطای $P_u$ | خطای $K_u$ | **حد نظری روش DF** |
|---|---|---|---|---|
| nominal | 1.1200 | +3.5٪ | −21.1٪ | −18.9٪ |
| slow thermal | 4.3000 | +11.7٪ | −27.1٪ | −24.9٪ |
| fast, small L | 0.2120 | +10.1٪ | −26.0٪ | −24.9٪ |
| L/T = 1 | 6.0198 | −2.8٪ | −11.7٪ | −11.5٪ |
| L/T = 0.05 | 0.2600 | +32.6٪ | −38.3٪ | −30.8٪ |
| big furnace | 15.5998 | +4.8٪ | −22.3٪ | −20.8٪ |

> **این خطای $K_u$ باگ نیست و قابل رفع در پیاده‌سازی نیست.** ستون آخر با حل
> تحلیلی چرخه‌ی حدی دقیق relay (نه شبیه‌سازی) و سپس اعمال همان فرمول
> describing-function محاسبه شده است. DF فقط هارمونیک اول را نگه می‌دارد و
> هارمونیک‌های بالاتر موج مربعی را دور می‌ریزد، پس ذاتاً $K_u$ را کم‌برآورد
> می‌کند. پیاده‌سازی ما در همه‌ی موارد **۱ تا ۲ درصد** از این حد نظری فاصله
> دارد — یعنی خطای اندازه‌گیری ناچیز و باقی خطا متعلق به خودِ روش است.
> پیامد عملی بی‌خطر است: گین‌های حاصل محافظه‌کارانه‌تر از حالت ایده‌آل‌اند.
> $P_u$ که بار اصلی $T_i$ و $T_d$ را می‌کشد، برای پلنت‌های متعارف دقیق است
> (چند درصد).
>
> **استثنا — پلنت‌های lag-dominated.** ردیف $L/T=0.05$ خطای $P_u$ برابر +۳۲.۶٪
> دارد. حل تحلیلی دقیق چرخه‌ی حدی نشان می‌دهد خودِ چرخه ۰.۲۳۳۴ ثانیه است در
> برابر $P_u=0.1961$ — یعنی **۱۹٪ از این خطا فیزیک آزمایش است، نه اندازه‌گیری**.
> وقتی dead time در برابر ثابت زمانی ناچیز است، relay در فرکانسی به‌مراتب
> پایین‌تر از فرکانس بحرانی نوسان می‌کند. هیچ پیاده‌سازی‌ای نمی‌تواند $P_u$ را از
> این آزمایش بازیابی کند؛ **برای چنین پلنت‌هایی باید از آزمایش Step استفاده
> کرد** (همان پلنت با Step: خطای $T$ فقط −۱.۸٪).

**دروازه‌ی نمونه‌برداری (سخت‌گیرتر شد).** relay فقط روی مرز نمونه سوئیچ می‌کند،
پس هر نیم‌دوره به مضربی از $\Delta t$ کوانتیزه می‌شود و خطای $\pm 2\Delta t$
وارد می‌شود. کف قبلی $P_u \ge 8\Delta t$ یعنی $\pm۲۵٪$ خطای کوانتیزاسیون — بسیار
درشت‌تر از آن که بتوان از رویش تنظیم کرد. به $P_u \ge 20\Delta t$ افزایش یافت
(سهم کوانتیزاسیون ≈۱۰٪، هم‌مرتبه با خطای ذاتی روش).

**اعتبارسنجی حلقه‌بسته (تنها معیاری که واقعاً مهم است):** با AMIGO-step روی
پلنت nominal، گین‌های خودکار $K_p=0.8191$، $K_i=1.1866$، $K_d=0.1154$ اعمال شد؛
حلقه‌بسته: مقدار نهایی **1.00000**، فراجهش **۱۵.۳٪**، بدون خطای ماندگار.

**تست‌های ایمنی — ۱۹/۱۹ پاس:** رد ترکیب ناسازگار rule/ident در زمان `Init`
(کد ۱۸)؛ مدیریت `NULL` و handle صفرشده؛ بازگردانی mode و setpoint پس از
`Abort`؛ رعایت timeout (توقف در ۳.۰۱ ثانیه با بودجه‌ی ۳.۰)؛ سنسور یخ‌زده
منجر به خاتمه می‌شود نه hang.

### ۹.۸.۱ سه باگ واقعی که تست‌های عددی آشکار کردند

۱. **بایاس relay از خروجی بیات خوانده می‌شد.** `auto_bias` مقدار را از
   `PID_GetOutput()` می‌گرفت که خروجی *آخرین `Update()` اجراشده* است. کاربری که
   تازه `SetMode(MANUAL)` + `SetManualOutput(0.5)` صدا زده و هنوز حلقه را
   قدم نزده، صفر تحویل tuner می‌داد و relay حول نقطه‌ی کار غلط نوسان می‌کرد —
   در عمل همه‌ی تست‌ها timeout می‌شدند. رفع: افزودن `PID_GetManualOutput()` به
   API عمومی و استفاده از آن.
۲. **آستانه‌های settle زیر کف نویز بودند.** آستانه‌ی پایدارسازی از `hysteresis`
   مشتق می‌شد؛ با نویز ۱٪ کف نویزِ شیب **۱۰۰ برابر** بالاتر از آستانه بود، پس
   شرط هرگز برقرار نمی‌شد و تنها خروجی ممکن timeout بود. رفع: آستانه‌ها به
   نویز اندازه‌گیری‌شده ($\overline{|y_k-y_{k-1}|}$) کف‌گذاری شدند و شیب روی
   سیگنال فیلترشده سنجیده می‌شود.
۳. **settle می‌توانست پیش از واکنش پلنت اعلام شود.** بلافاصله پس از پله،
   `total` ناچیز است و آستانه‌های نسبی هم ناچیز؛ نویز به‌تنهایی آنها را ارضا
   می‌کرد و tuner در طول dead time «پایدار» اعلام می‌شد و به هیچ مدلی fit
   می‌کرد. رفع: پاسخ باید ابتدا حداقل ۱۰ برابر نویز حرکت کرده باشد.

### ۹.۸.۲ حافظه و اندازه (اندازه‌گیری‌شده، x86-64، `-Os`)

| مورد | اندازه |
|---|---|
| `PID_AutoTune` | ۴۰۰ B |
| `PID_AutoTuneConfig` | ۱۱۲ B |
| `PID_AutoTuneResult` | ۸۸ B |
| `PID_PlantModel` / `PID_Gains` | ۳۲ B / ۲۴ B |
| کد `pid_autotune.c` + `pid_autotune_rules.c` | ۷۴۵۱ B (text ۷۳۲۷ + rodata ۱۲۴) |
| همان دو فایل در پروفایل MINIMAL و MOTION | **۰ B** |

ادعای «قابلیت غیرفعال = صفر Flash / صفر RAM» برای auto-tune تأیید شد.
دروازه‌ی ساخت: **۳۲۰ کامپایل** (۴ پروفایل × ۵ ترکیب پیکربندی × ۲ سطح بهینه‌سازی
× همه‌ی فایل‌های `src/`) با `-Werror -Wconversion -Wdouble-promotion -Wshadow
-Wcast-qual -pedantic` → **صفر هشدار**.

---

## ۹.۵ (PHASE 13) Diagnostics & Safety — نتایج اندازه‌گیری‌شده

این فاز عمدتاً **راستی‌آزمایی** بود، نه پیاده‌سازی جدید: منطق safety از قبل در
`src/pid.c` و متریک‌ها در `src/pid_diag.c` وجود داشت. دو مجموعه تست جدید نوشته
شد و یک **باگ واقعی همزمانی** پیدا و رفع شد.

### ۹.۵.۱ باگ SPSC در ring تلمتری (رفع شد)

نسخه‌ی قبلی `pidd_telemetry_push()` هنگام پر بودن بافر، **قدیمی‌ترین** رکورد را
با جلو بردن `tail` دور می‌ریخت. اما `tail` در تعریف ساختار «فقط توسط مصرف‌کننده
نوشته می‌شود» علامت خورده است؛ این کار قرارداد single-producer/single-consumer
را می‌شکست. کامنت داخل کد ادعا می‌کرد بدترین حالت «یک رکورد دورریز اضافه» است.
این ادعا **غلط بود**. بازپخش دستی درهم‌آمیزی نشان داد:

```
ring پر: head=7 tail=0
producer: پر بودن را می‌بیند (next=0 == tail=0)، اینجا pre-empt می‌شود
consumer: slot 0 را می‌خواند، tail -> 1
consumer: slot 1 را می‌خواند، tail -> 2
producer: ادامه می‌دهد و با مقدار کهنه tail را به 1 برمی‌گرداند
=> tail به عقب رفت؛ رکورد slot 1 که قبلاً تحویل شده بود، دوباره تحویل می‌شود.
```

یعنی نتیجه، دادهٔ **تکراری و نادرست** بود، نه صرفاً از دست رفتن یک نمونه.

سیاست جدید: **drop-newest**. وقتی بافر پر است تولیدکننده رکورد خودش را دور
می‌ریزد و به `head`/`tail` دست نمی‌زند، پس `tail` تک-نویسنده می‌ماند و ring
واقعاً lock-free است بدون critical section. برای اینکه از دست رفتن داده پنهان
نماند، `seq` برای رکوردهای دورریخته هم افزایش می‌یابد؛ مصرف‌کننده با دیدن
**گسست در شماره‌ی ترتیب** دقیقاً می‌فهمد چند نمونه را از دست داده است.

### ۹.۵.۲ تست‌ها

`tests/test_safety.c` → **۲۲/۲۲ pass**؛ شامل: latch شدن fault پس از N نمونه،
خروجی failsafe، بازیابی خودکار با `auto_recover`، گارد جهش نرخ (`code=12`) در
داخل بازه‌ی مجاز، صفر بودن هزینه وقتی safety خاموش است، رد شدن NaN/Inf با
نگه‌داشتن آخرین خروجی سالم، و اعتبارسنجی پیکربندی failsafe.

`tests/test_diag.c` → **۳۱/۳۱ pass**؛ همه‌ی متریک‌ها در برابر **فرم بسته** سنجیده
شدند نه در برابر خروجی خودِ کد:

| متریک | انتظار تحلیلی | اندازه‌گیری |
|---|---|---|
| IAE (خطای ثابت ۰٫۵ به مدت ۱۰ ثانیه) | ۵٫۰۰۰۰۰۰ | ۵٫۰۰۰۰۶۷ |
| ISE | ۲٫۵۰۰۰۰۰ | ۲٫۵۰۰۰۳۳ |
| ITAE | ۲۴٫۹۷۵۰۰۰ | ۲۴٫۹۷۵۱۹۳ |
| Saturation duty (۱۰۰ نمونه اشباع + ۱۰۰ نمونه آزاد) | ۰٫۵۰۰۰ | ۰٫۵۰۰۰ |
| Oscillation rate (۲۰۰ نمونه متناوب) | ۱۹۹ تغییر علامت → ۹۹٫۵/s | ۱۹۹ → ۹۹٫۵/s |

حساب‌داری ring هم دقیق است: ۲۰ به‌روزرسانی روی بافر ۸-خانه‌ای →
readable=۷ (ظرفیت منهای یک) + dropped=۱۳ = ۲۰، و رکوردهای بازمانده پیوسته‌اند
(`seq` صفر تا شش).

### ۹.۵.۳ Flash اندازه‌گیری‌شده به تفکیک پروفایل

اعداد قبلی این سند با پرچم اشتباه (`-DPIDX_PROFILE=...` به‌جای
`-DPIDX_PROFILE_MINIMAL`) گرفته شده بودند و در عمل همه FULL بودند. اعداد زیر
تصحیح‌شده‌اند (x86-64، `-Os`، float، مجموع `text+rodata+data` همه‌ی `src/`):

| پروفایل | text | rodata | data | مجموع Flash | `pid_diag.o` |
|---|---|---|---|---|---|
| MINIMAL | ۷۱۱۶ | ۴۸ | ۰ | **۷۱۶۴ B** | **۰ B** |
| MOTION | ۱۳۶۰۶ | ۶۴ | ۰ | **۱۳۶۷۰ B** | ۴۳۰ B |
| PROCESS | ۱۹۱۴۰ | ۱۷۶ | ۸۰ | **۱۹۳۹۶ B** | ۸۷۵ B |
| FULL | ۲۱۴۰۹ | ۱۸۸ | ۸۰ | **۲۱۶۷۷ B** | ۸۷۵ B |

`pid_diag.o` در MINIMAL دقیقاً صفر بایت تولید می‌کند؛ در MOTION فقط متریک‌ها
(بدون تلمتری) کامپایل می‌شوند. ادعای «قابلیت غیرفعال = صفر Flash» تأیید شد.

> این اعداد x86-64 هستند و **معیار نسبی**‌اند؛ چون `arm-none-eabi-gcc` در این
> محیط نصب نیست، هیچ عدد Cortex-M در این سند اندازه‌گیری واقعی نیست.

دروازه‌ی ساخت پس از تغییرات: **۳۲۰ کامپایل، صفر هشدار**. کل تست‌ها:
`smoke_core`، `test_autotune_safety` (۱۹/۱۹)، `test_autotune_accuracy`،
`test_safety` (۲۲/۲۲)، `test_diag` (۳۱/۳۱) — همه سبز.

---

## ۹.۶ (PHASE 14) Fixed-Point — پیاده‌سازی و راستی‌آزمایی

طبق §۱۲.۵ این یک **کنترلر مستقل** است، نه لایه‌ای روی هستهٔ float:
`include/pidx/pid_fixed.h` + `src/pid_fixed.c`، بدون هیچ `#include` از
`pid.h`/`pid_types.h`/`pid_math.h`. پس یک باینری می‌تواند هم‌زمان حلقهٔ جریان
fixed-point و حلقهٔ دمای float داشته باشد.

### ۹.۶.۱ چرا حالت داخلی Q30 است و نه Q15 (مهم‌ترین تصمیم این فاز)

شکست کلاسیک PID ممیز ثابت، **مرگ رزولوشن انتگرال‌گیر** است. با
`Ki = 0.5 1/s`، `dt = 1 ms` و خطای ۳۰ LSB، افزایش هر نمونه برابر است با:

```
Ki*dt*e = 0.5 × 0.001 × (30/32768) = 4.6e-7  ≈  0.015 LSB از Q15
```

در انباشتگر Q15 این عدد **دقیقاً صفر** گرد می‌شود؛ انتگرال‌گیر هرگز تکان
نمی‌خورد و خطای ماندگار دائمی می‌ماند که هیچ tuning آن را حذف نمی‌کند. راه‌حل:
انتگرال‌گیر و حالت فیلتر ورودی در **Q30** نگهداری می‌شوند (۳۲۷۶۸ برابر ریزتر از
LSB خروجی).

اندازه‌گیری: پس از ۲۰۰۰۰ نمونه با همان شرایط، انتظار تحلیلی `I = 0.009155` و
مقدار به‌دست‌آمده **۰٫۰۰۹۱۵۵** — خطای **۰٫۰۰٪**.

همین مسئله برای فیلتر EMA هم هست: `y += (x-y)>>shift` در Q15 به‌محض اینکه
`|x-y| < 2^shift` شود متوقف می‌گردد و افستی دائمی تا `2^shift` LSB باقی
می‌گذارد. در Q30 اندازه‌گیری شد: خروجی نهایی **۰ LSB** (کاملاً همگرا).

### ۹.۶.۲ معادل‌سازی float ↔ fixed (تعهد §۱۱)

همان plant و همان tuning (`Kp=1.2, Ki=3.0, Kd=0.02, Tf=5ms, dt=1ms`)، ۴۰۰۰ نمونه:

| سنجه | مقدار |
|---|---|
| RMS اختلاف مسیر | **۱٫۵۳e-6** (≈ ۰٫۰۵ LSB) |
| بیشینه انحراف | ۵٫۸۵e-6 |
| مقدار نهایی float / fixed | ۰٫۴۹۹۳۵۰ / ۰٫۴۹۹۳۴۶ |

اختلاف در حد کوانتیزاسیون است، نه اختلاف الگوریتمی. یک LSB برابر ۳٫۰۵e-5 است،
پس کل خطا زیر یک‌بیستم LSB می‌ماند.

### ۹.۶.۳ سایر نتایج اندازه‌گیری‌شده

| آزمون | نتیجه |
|---|---|
| Anti-windup: `I_peak` (none / clamp / back-calc) | ۱٫۰۰۰۰ / ۰٫۳۰۰۰ / ۰٫۵۹۰۰ |
| بازیابی از اشباع | none: **هرگز**، clamp و back-calc: بله |
| Bumpless manual→auto | ۷ LSB = دقیقاً یک گام `Ki·dt·e` (انتظار ۶٫۵۵) |
| Derivative kick در پلهٔ setpoint | صفر (خروجی ۰٫۴۰۰۰ = فقط جملهٔ P) |
| سوگیری گِردکردن (ورودی میانگین‌صفر، ۲۰۰۰ نمونه) | مجموع **۰ LSB** |
| سرریز با بهره‌های افراطی | صفر خروجی منفی، بدون تغییر علامت |
| `PIDq_SetGains` فعال‌کردن Ki از صفر | `I = 0.25000` در برابر انتظار ۰٫۲۵۰۰۰ |
| پرش خروجی هنگام `Kp: 1→3` در `e=0.2` | ۰٫۴۰۰۲ = دقیقاً تغییر جملهٔ P |

`tests/test_fixed.c` → **۴۲/۴۲ pass**.

### ۹.۶.۴ دو باگ واقعی که این فاز آشکار کرد

۱. **`PIDq_SetGains` قابل پیاده‌سازی نبود آن‌طور که اول نوشتم.** تلاش کردم
`dt` و `Tf` را از ضرایب تاشده بازسازی کنم، ولی `ci = Ki·dt` وقتی `Ki=0` باشد
به صفر فرو می‌ریزد و هیچ مقیاسی برای بازسازی باقی نمی‌ماند — یعنی «فعال کردن
انتگرال‌گیر از صفر» اصلاً ممکن نبود. اصلاح: `dt_us` و `tf_us` در handle
نگهداری می‌شوند (۸ بایت) و `SetGains` ضرایب را از نو تا می‌زند. اگر بهرهٔ جدید
جا نشود، ضرایب قبلی **rollback** می‌شوند تا تنظیم نیمه‌اعمال‌شده باقی نماند.

۲. **`PIDq_SelfTest` خودش غلط بود.** برای اثبات اینکه ضرب ۳۲×۳۲ به ۶۴ بیت
گسترش می‌یابد، `65536*65536` را با `2^40` مقایسه کرده بودم در حالی که حاصل
`2^32` است؛ تست همیشه fail می‌داد. مقایسه اصلاح شد.

سه شکست دیگر، **تست‌های بد** بودند نه باگ کتابخانه: انتظار bumpless=۰ (در حالی
که یک گام انتگرال‌گیری طبیعی است) و یک آزمون سرریز که با `Kd=100` و
`Tf+dt=2ms` باعث می‌شد `Init` به‌درستی رد کند، handle نامعتبر بماند و آزمون
**به دلیل غلط** پاس شود. حالا هر دو حالت صریحاً assert می‌شوند.

### ۹.۶.۵ حجم و اثبات «صفر هزینه»

| پروفایل | `pid_fixed.o` text | کل Flash کتابخانه |
|---|---|---|
| MINIMAL | **۰ B** | ۷۱۶۴ B |
| MOTION | ۲۱۰۷ B | ۱۵۷۷۷ B |
| PROCESS | **۰ B** | ۱۹۳۹۶ B |
| FULL | ۲۱۰۷ B | ۲۳۷۸۴ B |

`sizeof(PIDq_Handle)` = **۸۰ B**، `sizeof(PIDq_Config)` = **۴۰ B**.

راستی‌آزمایی «بدون ممیز شناور»: در آبجکت کامپایل‌شده **صفر** دستور
FP اسکالر و **صفر** سمبل حل‌نشدهٔ libm/soft-float وجود دارد.

### ۹.۶.۶ انحراف MISRA و محدودیت‌های صادقانه

**انحراف Rule 10.1:** این ماژول به شیفت راستِ **حسابی** روی اعداد منفی تکیه
می‌کند که C99 §6.5.7 آن را implementation-defined گذاشته است. همهٔ کامپایلرهای
هدف (GCC، Clang، IAR، ARMCC) آن را حسابی تعریف می‌کنند و `PIDq_SelfTest()` در
زمان اجرا بررسی‌اش می‌کند.

**پشتیبانی نمی‌شود** (حذف شده، نه stub): auto-tuning، gain scheduling با
درون‌یابی، setpoint weighting، shaper، feedforward، و back-calculation با `Kt`
دلخواه (فقط توان‌های ۲ تا تقسیم وارد مسیر داغ نشود).

**هشدار Cortex-M0/M0+:** ضرب‌های داخلی `int64_t` هستند. روی M3 به بالا این
همان دستور تک‌سیکلی `SMULL` است، ولی روی M0/M0+ به فراخوانی helper کامپایلر
تبدیل می‌شود. این هزینهٔ مستندشدهٔ درستی روی آن هسته است.

دروازهٔ ساخت در پایان همین فاز: **۳۶۰ کامپایل، صفر هشدار** (این عدد در فازهای ۱۵ و ۱۶ با افزوده‌شدن `platform/` و `examples/common/` به **۵۲۰ کامپایل** رسید — بخش ۹.۸ را ببینید).

---

## ۹.۷ (PHASE 15) لایه یکپارچه‌سازی پلتفرم — POSIX و STM32

این فاز چیزی به هسته اضافه **نمی‌کند**. کل خروجی آن در `platform/` است و
هسته حتی نمی‌داند این دایرکتوری وجود دارد. اگر آن را پاک کنید، کتابخانه
کماکان build می‌شود و هر شش تست هسته سبز می‌مانند.

### ۹.۷.۱ ممیزی «صفر HAL» — تأیید شد

پیش از نوشتن هر خطی، ادعای معماری راستی‌آزمایی شد:

```
grep -rn "HAL_|stm32|cmsis|CMSIS|__DMB|SysTick|DWT" src/ include/
```

نتیجه: هیچ تطابقی در کد. تنها دو تطابق در **کامنت** است
(`pid_conf.h:175` و `pid_diag.h:27`) که به کاربر پیشنهاد می‌دهند در صورت
نیاز `PIDX_MEMORY_BARRIER()` را به `__DMB()` نگاشت کند — یعنی نقطهٔ توسعه،
نه وابستگی. تنها include های سیستمی در `src/*.c` و `include/pidx/*.h`:
`<stdint.h>`، `<stdbool.h>`، `<stddef.h>`.

### ۹.۷.۲ تصمیم بنیادی این فاز: زمان‌بندی با ددلاین مطلق

هر دو لایه یک اشتباه رایج را رد می‌کنند: حلقه‌ای که با
`sleep(period)` یا `if (now - last >= period) { last = now; ... }`
نوشته شود **دریفت** می‌کند، چون زمان اجرای بدنهٔ حلقه و تأخیر بیدارشدن به
هر دوره اضافه می‌شود و هرگز جبران نمی‌شود. کنترلر همچنان `dt` اسمی را
استفاده می‌کند در حالی که نرخ واقعی کمتر است — یعنی `Ki·dt` و `Kd/dt`
هر دو غلط‌اند.

راه‌حل: نگهداری یک **ددلاین مطلق** و افزودن دقیقاً یک `period` به آن در هر
release. خطا انباشته نمی‌شود و فقط کوانتیزه می‌شود.

**اندازه‌گیری روی همین ماشین** (`tests/test_posix.c`، هدف ۵۰۰ هرتز،
بدنهٔ ۷۰۰ میکروثانیه، بدون اولویت realtime):

| روش | نرخ حاصل | خطا |
|---|---|---|
| ددلاین مطلق (`PIDp_LoopWait`) | ۴۹۹.۱ هرتز | ‎−۰.۲٪ |
| `sleep(body); sleep(period)` ساده | ۳۴۱ هرتز | **‎−۳۱.۸٪** |

پنج اجرای متوالی همین اعداد را داد (۴۹۸.۲ تا ۴۹۹.۱ در برابر ۳۳۹.۹ تا ۳۴۳.۸).

### ۹.۷.۳ لایه POSIX — `platform/posix/`

| مؤلفه | نقش |
|---|---|
| `PIDp_NowUs` / `PIDp_Now` | زمان یکنواخت از `CLOCK_MONOTONIC` |
| `PIDp_SleepUs` | خواب با ازسرگیری پس از `EINTR` |
| `PIDp_Loop` + `PIDp_LoopWait` | زمان‌بند ددلاین مطلق، برمی‌گرداند `dt` واقعی |
| `PIDp_Timer` | انباشتگر min/max/mean برای بنچمارک نسبی |

سه تصمیم که ارزش توضیح دارند:

1. **هرگز `gettimeofday`.** یک پرش NTP روی ساعت realtime یا `dt` منفی
   می‌سازد یا `dt` غول‌آسا؛ هر دو ترم مشتق را منفجر می‌کنند.
   `CLOCK_MONOTONIC` این کلاس خطا را حذف می‌کند.
2. **ازسرگیری `nanosleep` با باقی‌ماندهٔ زمان.** نادیده‌گرفتن `EINTR`
   باعث می‌شود حلقهٔ کنترل هر وقت دیباگر یا profiler وصل است بی‌صدا
   سریع‌تر بدود.
3. **در overrun، rebase به‌جای catch-up.** اجرای پشت‌سرهم عقب‌ماندگی،
   رشته‌ای از `dt`های ریز به کنترلر می‌دهد و مشتق را spike می‌کند. در عوض
   overrun شمرده می‌شود و برنامه روی «الان» بازتنظیم می‌شود.

**تست:** `tests/test_posix.c` — ۳۱ ادعا، ۳۱ سبز، در ۵ اجرای متوالی پایدار.
پوشش: یکنواختی زمان روی ۲۰۰۰ نمونه، رزولوشن ۱ میکروثانیه، دقت نرخ، مقایسه
با حلقهٔ ساده، شمارش overrun با بدنهٔ ۳ برابر دوره (۳۹ از ۴۰)، نبود
catch-up burst، اجرای یک کنترلر واقعی ۱ کیلوهرتزی روی مدل مرتبهٔ اول تا
همگرایی، و بی‌اثر بودن ورودی `NULL`.

### ۹.۷.۴ لایه STM32 — `platform/stm32/`

بدون HAL و بدون LL؛ فقط ثبات‌ها. تنها پریفرالی که لمس می‌شود همان تایمری
است که خودتان نام می‌برید، به‌علاوهٔ DWT در صورت انتخاب.

| مؤلفه | نقش |
|---|---|
| `PIDs_TimebaseInitTim` | تایمر general-purpose آزاد روی دقیقاً ۱ مگاهرتز |
| `PIDs_TimebaseInitDwt` | شمارندهٔ سیکل DWT (فقط ARMv7-M) |
| `PIDs_TimebaseInitCallback` | شمارندهٔ خودتان (RTOS، LPTIM…) به‌همراه ماسک عرض |
| `PIDs_DeltaUs` / `PIDs_NowUs` | تفریق wrap-safe و توسعهٔ ۶۴ بیتی |
| `PIDs_Rate` | درایور نرخ غیرمسدودکننده برای super-loop |
| `PIDs_CycleStat` | پروفایلر سیکل DWT |
| `PIDs_IsrMonitor` | جیتر، زمان اجرا و بار CPU یک ISR دوره‌ای |
| `PIDs_EnterCritical` | PRIMASK یا BASEPRI، با save/restore |

نکات طراحی:

- **پرسکیلر باید دقیقاً ۱ مگاهرتز بدهد.** اگر کلاک تایمر مضرب صحیح
  ۱ مگاهرتز نباشد، `PID_ERR_INVALID_PARAM` برمی‌گردد. پذیرفتن تقریب یعنی
  هر `dt` در سیستم با یک ضریب ثابت غلط باشد — که شبیه tuning بد به‌نظر
  می‌رسد نه باگ کلاک. `timer_clk_hz` هم کلاک تایمر APB است نه
  `SystemCoreClock`؛ روی اکثر STM32ها وقتی پرسکیلر APB برابر ۱ نیست، دو
  برابر PCLK است.
- **عرض شمارنده در زمان اجرا تشخیص داده می‌شود** (نوشتن `0xFFFFFFFF` در
  `ARR` و خواندن آنچه می‌ماند)، به‌جای جدولی از اینکه کدام تایمر روی کدام
  خانواده ۳۲ بیتی است.
- **`PIDs_DeltaUs` به‌جای تفریق ساده.** روی تایمر ۱۶ بیتی
  `later - earlier` قرضِ تفریق را در ۱۶ بیت بالا رها می‌کند و اختلاف
  چند صد میکروثانیه‌ای به ‎۴.۲۹e۹ تبدیل می‌شود. تست این را صریحاً به‌عنوان
  «guard دارد کار می‌کند» چک می‌کند.
- **DWT فقط وقتی پذیرفته می‌شود که واقعاً بشمارد.** روی دیباگ‌یونیت قفل‌شده
  بیت enable همان‌طور که نوشته شده خوانده می‌شود ولی `CYCCNT` یخ می‌زند؛
  پروفایلری که همیشه صفر گزارش کند بدتر از پروفایلری است که شکست را
  گزارش کند. پس مقدار بازخوانی می‌شود و در صورت یخ‌زدگی
  `PID_ERR_UNSUPPORTED`.
- **بخش بحرانی save/restore است، نه enable بی‌قید.** ورود به بخش بحرانی
  داخل ناحیه‌ای که از قبل mask شده نباید هنگام خروج پنجرهٔ وقفه باز کند.

### ۹.۷.۵ یک باگ واقعی که این فاز آشکار کرد

نسخهٔ اول `PIDs_RateElapsed` در کامنت ادعای «ددلاین مطلق» می‌کرد ولی در کد
`last_release = now` می‌گذاشت و شرط را با `now - last_release` می‌سنجید —
یعنی دقیقاً همان rebase که قرار بود رد شود. تست اولیه آن را ندید، چون گام
polling مضرب دقیق دوره بود و کوانتیزاسیون صفر می‌شد.

تستی اضافه شد با گام polling ۳۷ میکروثانیه و بدنهٔ ۳۱۱ میکروثانیه (هیچ‌کدام
مقسوم‌علیه دورهٔ ۱۰۰۰ میکروثانیه نیستند). نتیجه روی نسخهٔ معیوب و اصلاح‌شده:

| نسخه | releaseها در ۲ ثانیه | میانگین دوره | نرخ |
|---|---|---|---|
| rebase روی `now` (معیوب) | ۱۹۷۳ | ۱۰۱۴.۰۰ µs | ۹۸۶.۱۹ Hz |
| ددلاین مطلق (اصلاح‌شده) | ۲۰۰۰ | ۱۰۰۰.۰۱ µs | ۹۹۹.۹۹ Hz |

نسخهٔ معیوب عمداً بازساخته شد تا ثابت شود تست جدید واقعاً آن را می‌گیرد
(۳ ادعای شکست‌خورده)، سپس دور انداخته شد.

### ۹.۷.۶ محدودیت صادقانه: این لایه روی سیلیکون اجرا نشده است

در این محیط نه toolchain آرم هست و نه سخت‌افزار. برای اینکه لایهٔ STM32
«کد نوشته‌شده و امتحان‌نشده» نماند، یک **stub سازگار با CMSIS** ساخته شد
(`tests/stm32_stub/`) که همان چیدمان ثبات‌ها، ماکروها و intrinsicها را روی
حافظهٔ معمولی می‌دهد، و شمارنده از خود تست رانده می‌شود تا wrapی که روی
سخت‌افزار ۶۵ میلی‌ثانیه طول می‌کشد فوراً اتفاق بیفتد.

`tests/test_stm32_host.c` — **۸۴ ادعا، ۸۴ سبز**. چه چیزی *اثبات* می‌شود:
توالی راه‌اندازی ثبات‌ها (PSC/ARR/EGR/CR1، بدون فعال‌کردن وقفه)، تشخیص عرض
شمارنده، حساب wrap در ۱۶ بیت، توسعهٔ ۶۴ بیتی روی ۹۱ wrap متوالی با نتیجهٔ
دقیقاً ۶۰۰۰۰۰۰ میکروثانیه، عدم دریفت درایور نرخ، شمارش overrun بدون burst،
رد شدن دوره‌ای که به نصف بازهٔ شمارنده نزدیک است، تفکیک جیتر از زمان اجرا
در ISR monitor (ورود ۳۵۰ میکروثانیه دیرهنگام → جیتر ۳۵۰، زمان اجرا
بدون تغییر ۲۰۰)، بار CPU ۲۰٪ برای بدنهٔ ۲۰۰ در دورهٔ ۱۰۰۰، و رفتار
درست روی شمارندهٔ یخ‌زده.

چه چیزی **اثبات نمی‌شود**: تایمینگ واقعی باس، رفتار shadow register،
در دسترس بودن DWT روی یک قطعهٔ مشخص، و هیچ عدد سیکلی. این کد را
«مرور‌شده ولی فلش‌نشده» بدانید و پیش از استفاده در ماشینی که می‌تواند
آسیب بزند، روی برد خودتان راستی‌آزمایی کنید. مطابق §۱۰.۲ هیچ عدد
Cortex-M جعل نمی‌شود.

### ۹.۷.۷ دروازهٔ ساخت پس از این فاز

`src/*.c` به‌علاوهٔ هر دو فایل پلتفرم، در ۴ پروفایل × ۵ ترکیب پرچم × دو
سطح بهینه‌سازی، با مجموعهٔ کامل هشدارها و `-Werror`:
**۴۴۰ کامپایل، صفر هشدار**.

علاوه بر آن، لایهٔ STM32 در ۵ پیکربندی دیگر هم کامپایل شد تا شکل
Cortex-M0 و مسیرهای جایگزین پوشش داده شوند: `HAS_DWT=0`،
`CRITICAL_BASEPRI=0x50`، `DWT_HAS_LAR=1`، `TIMEBASE=DWT`، و
`HAS_DWT=0 + TIMEBASE=CALLBACK` — هر کدام در `-Os` و `-O2`، همه پاک.

اندازه روی میزبان x86-64 با `-Os` (فقط برای مقیاس نسبی، نه عدد Cortex-M):
`pid_posix.o` = ۷۲۷ بایت text؛ `pid_stm32.o` = ۱۴۵۸ بایت text + ۴۸ بایت
data (همان یک struct وضعیت timebase).

هر شش تست هستهٔ قبلی پس از این فاز دوباره اجرا شدند: smoke سبز،
autotune-safety ۱۹/۱۹، autotune-accuracy همه در تلورانس، diagnostics ۳۱/۳۱،
safety ۲۲/۲۲، fixed-point ۴۲/۴۲.


---

## ۹.۸ (PHASE 16) ده مثال — و پنج باگ واقعی که پیدا کردند

هر ده مثال **کامپایل و اجرا می‌شوند**، زیر همان دروازهٔ ساخت کتابخانه
(`-Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow -Wcast-qual
-pedantic -Werror`). مثالی که فقط با هشدارهای خاموش کامپایل شود الگوی
قابل استفاده نیست.

### ۹.۸.۱ زیرساخت مشترک

`examples/common/` شامل مدل‌های مرجع پلنت (FOPDT با تأخیر انتقال واقعی نه
تقریب Padé، موتور DC با دو حالت الکتریکی/مکانیکی و اصطکاک کولمب، هیتر با
تلفات تابشی غیرخطی) و متریک‌های پاسخ پله + رسم ASCII است. مولد نویز
LCG با دانهٔ ثابت است، پس هر اجرا بایت‌به‌بایت یکسان است — مثال ناپایدار
بی‌فایده است.

### ۹.۸.۲ فهرست مثال‌ها

| # | موضوع | نکتهٔ مرکزی |
|---|---|---|
| ۰۱ | Minimal | API پنج‌خطی، هیچ چیز اضافه |
| ۰۲ | Temperature/PWM | مقایسهٔ چهار استراتژی anti-windup روی یک پلنت |
| ۰۳ | Motor speed | derivative kick، وزن‌دهی β، اصطکاک کولمب |
| ۰۴ | Motor position | شکل‌دهی مسیر، پلنت انتگرال‌گیر |
| ۰۵ | Current 20 kHz | `PID_UpdateFast` و اثبات هم‌ارزی عددی |
| ۰۶ | Cascade ×۳ | position ← velocity ← current |
| ۰۷ | Auto-tune | relay در برابر step، پاکت ایمنی |
| ۰۸ | RTOS task | ددلاین مطلق + feedforward مدل‌محور |
| ۰۹ | TIM ISR | تلمتری lock-free، متریک‌های حلقه |
| ۱۰ | Full-featured | مرجع پیکربندی، همهٔ زیرسیستم‌ها |

### ۹.۸.۳ چند عدد اندازه‌گیری‌شده

**مثال ۰۲ — anti-windup** (بعد از پلهٔ نزولی، هیتر نمی‌تواند سرد کند):

| استراتژی | اوج ‎\|I\| | زمان بازیابی اختیار |
|---|---|---|
| none | ۳.۵۱ | ۹۱.۵ s |
| clamp | ۰.۴۵ | ۳۶.۰ s |
| conditional | ۰.۵۷ | ۲۸.۰ s |
| back-calculation | ۱.۰۰ | **۲۳.۵ s** |

**مثال ۰۶ — cascade در برابر تک‌حلقه:**

| ساختار | اوج ‎\|I\| | افت اغتشاش | بازیابی |
|---|---|---|---|
| cascade ×۳ | ۷.۹۹ A (حد ۸) | ۰.۰۰۰۳۶ rad | ۰.۰۱۴ s |
| تک‌حلقه | **۲۳.۴۴ A** | ۰.۰۱۰۹ rad | ۰.۵۰ s |

**مثال ۰۷ — relay در برابر step** (پلنت واقعی: K=80، T=40 s، L=8 s):

| روش | نتیجه | خطا | زمان |
|---|---|---|---|
| relay | Ku=۰.۰۷۷۶ | **‎−۲۷.۰٪** روی Ku | ۱۸۹ s |
| step | K=۷۹.۱۹ | **‎−۱.۰٪** روی K | ۲۴۵ s |

خطای ‎−۲۷٪ رله دقیقاً در بازهٔ ۱۱–۳۸٪ مستندشدهٔ §۹.۸ (PHASE 12) است — فیزیک
describing-function، نه باگ.

> **اصلاحیهٔ فاز ۱۸ — توضیح قبلی مثال ۰۷ نادرست بود.** متن پیشین
> اورشوت ۲۹٪ قانون `NO_OVERSHOOT` را به همان خطای ‎−۲۷٪ رله نسبت
> می‌داد. این نسبت‌دادن **غلط** بود و خودش هم متناقض بود: چون
> `Kp = 0.20·Ku`، یک `Ku` کمتر `Kp` **کمتر** می‌دهد، نه بیشتر.
> آزمون قطعی: همان قانون با مدل **دقیق** (خطای شناسایی صفر) روی همان
> پلنت باز هم **۳۰.۷٪** اورشوت می‌دهد؛ یعنی خطای رله در عمل اورشوت را
> ~۳ واحد **کم** می‌کند. علت واقعی خود جدول قانون است — جزئیات در
> §۹.۱۰.۲.

### ۹.۸.۴ پنج باگ واقعی که مثال‌ها آشکار کردند

هدف از مثال‌ها فقط نمایش نبود؛ هر کدام یک آزمون مستقل از کتابخانه است.
نتیجه: **سه باگ در کتابخانه، دو مورد سند نادرست**.

**۱. `PID_AW_CONDITIONAL` کد مرده بود.** مرحلهٔ ۱۰ پرچم‌های اشباع را
می‌خواند که مرحلهٔ ۱۲ *بعداً* تنظیم می‌کند، و خط ۳۰۰ آن‌ها را در ابتدای هر
سیکل پاک می‌کرد. پس شرط هرگز درست نمی‌شد و `CONDITIONAL` دقیقاً مثل
`NONE` رفتار می‌کرد (اعداد یکسان تا آخرین رقم). رفع: تصمیم به مرحلهٔ ۱۳
منتقل شد، جایی که `u_raw` معلوم است، و افزایش انتگرال‌گیر در صورت لزوم
**برگردانده** می‌شود نه صرفاً رد. تست `tests/test_antiwindup.c` (۳۵ ادعا)
اکنون صریحاً چک می‌کند که `CONDITIONAL` با `NONE` یکی نباشد.

**۲. `PID_UpdateFast` و `PID_Update` می‌توانستند واگرا شوند.** مسیر کامل
حد انتگرال‌گیر را در زمان اجرا حل می‌کرد (وقتی کاربر حد صریح نداده،
از حد خروجی ارث می‌برد) ولی `PID_UpdateFast` مستقیم روی `h->i_min/i_max`
کلمپ می‌کرد که هنوز ‎±HUGE بود. پیامد جانبی:
`PID_UpdateFast_IsSafe()` یک پیکربندی PI کاملاً معمولی را رد می‌کرد.
رفع: `i_min/i_max` همیشه حد **مؤثر** را نگه می‌دارند.
`tests/test_fastpath.c` (۴۹ ادعا) هم‌ارزی بیت‌به‌بیت را حتی پس از
تغییر حد در زمان اجرا تثبیت می‌کند.

**۳. بازیابی از خطا پرش داشت.** `PID_ClearFault` و مسیر auto-recover،
انتگرال‌گیر را با `P=D=FF=0` حل می‌کردند — ولی این کار در مرحلهٔ ۲ انجام
می‌شد، قبل از آنکه ترم‌های این نمونه اصلاً محاسبه شوند. نتیجه:
`I = u_failsafe` تنظیم می‌شد و بعد `P` واقعی رویش اضافه می‌شد.
اندازه‌گیری‌شده: پرش ۰.۲۰۰ → ۰.۲۹۲. رفع: back-solve به مرحلهٔ ۱۰ موکول
شد. حالا ۰.۲۰۰ → ۰.۲۰۱۸ که مانده‌اش دقیقاً یک گام `Ki·dt·e` است.

**۴. ادعای «bumpless» بدون قید بود.** back-solve مقدار
`I = u - P - D - FF` را به حدود انتگرال‌گیر کلمپ می‌کند. وقتی کلمپ فعال
شود — معمولاً چون اندازه‌گیری از setpoint دور است و `Kp·e` به‌تنهایی از
بازهٔ محرک بیشتر است — انتقال **نمی‌تواند** bumpless باشد. مثال ۱۰ پرش
۰.۴۰ → ۱.۰۰ را نشان داد. این محدودیت ذاتی است (درخواست از نظر حسابی
نشدنی است)، پس رفع درست «گزارش کردن» بود نه پنهان‌کردن:
`PID_FLAG_INTEGRAL_LIMITED` به‌علاوهٔ ثبت در کانال خطای چسبنده — چون
خود پرچم هر سیکل بازسازی می‌شود و لحظهٔ مهم را از دست می‌دهید.
`tests/test_bumpless.c` (۴۵ ادعا) هم تضمین و هم پیش‌شرطش را تثبیت می‌کند.

**۵. کامنت سرصفحهٔ `pid_diag.h` هنوز می‌گفت قدیمی‌ترین رکورد دور انداخته
می‌شود** — بازماندهٔ سند از قبل از رفع PHASE 13. کد **جدیدترین** را دور
می‌اندازد و دلیلش هم قرارداد lock-free است. متن اصلاح شد.

### ۹.۸.۵ چند خطای خودِ مثال‌ها که ارزش ثبت دارند

اینها باگ کتابخانه نبودند ولی همان درس را می‌دهند:

- **پلنت غیرقابل‌دسترس، مقایسه را پوچ می‌کند.** هیتر ۵۰ واتی هرگز به
  ۱۸۰ °C نمی‌رسید، پس هر چهار استراتژی anti-windup اعداد یکسان دادند.
  همچنین در مثال ۰۷ پلنتی با K=2.5 در برابر هدف ۶۰ °C ⇒ همهٔ tuneها
  `PID_ERR_TUNE_TIMEOUT` — کتابخانه درست رفتار کرد.
- **`tf = 0` یعنی «از `n_filter` استفاده کن»، نه «بدون فیلتر».**
  با `N=10, Kd=0.3, Kp=0.05` مقدار مؤثر `Tf = Kd/(N·Kp) = 0.6 s` می‌شود.
  برچسب «unfiltered» دروغ بود.
- **`output_min` را فراموش نکنید.** تنظیم `output_max=1` بدون
  `output_min` مقدار پیش‌فرض ‎−HUGE را باقی می‌گذارد؛ کنترلر duty منفی
  فرمان می‌دهد و پلنت واگرا می‌شود.
- **`PID_Telemetry_Dropped()` خواندن-و-پاک‌کردن است.** دو بار صدا زدنش
  شمارش را می‌بلعد (حسابرسی ۳۷۲۱ از ۶۰۰۰ درآمد).
- **حد slew روی حلقهٔ سریع، پایداری را بدتر می‌کند.** اندازه‌گیری‌شده:
  travel محرک ۲۱۴ بدون حد، ۹۰۰ در ۳۰۰ V/s، ۷۵۵۴ در ۳۰۰۰ V/s. حد slew
  یک تأخیر در مسیر فیدبک است و تأخیر حاشیهٔ فاز می‌خورد. setpoint را
  شکل بدهید (حلقه‌باز، رایگان).
- **deadband انتگرالی می‌تواند بدتر از نبودنش باشد.** نویز صفر-میانگین
  خودش تقریباً انتگرال نمی‌گیرد چون مثبت و منفی خنثی می‌شوند؛ deadband
  این خنثی‌سازی را نابود می‌کند. اندازه‌گیری: بازهٔ انتگرال‌گیر
  ۰.۰۰۲۸۰ (خاموش) → ۰.۰۰۴۵۷ (باند ۰.۵) → ۰.۰۰۰۰۰ (باند ۲.۰).
  این ابزار برای محرک کوانتیزه است، نه برای نویز حسگر.

### ۹.۸.۶ دروازهٔ ساخت پس از این فاز

- کتابخانه + پلتفرم + هارنس: **۵۲۰ کامپایل، صفر هشدار**
- ده مثال در `-Os` و `-O2`: **۲۰ کامپایل، صفر هشدار**
- تست‌ها: smoke ✓ · anti-windup ۳۵ · bumpless ۴۵ · fast-path ۴۹ ·
  autotune-safety ۱۹ · autotune-accuracy ✓ · diagnostics ۳۱ · safety ۲۲ ·
  fixed-point ۴۲ · posix ۳۱ · stm32-host ۸۴ ⇒ **۳۵۸ ادعا، همه سبز**

رشد اندازه از این فاز (بخش `.text`، x86-64، `-Os`): MINIMAL ‎+۵۳ B ·
MOTION ‎+۲۳۵ B · PROCESS ‎+۴۳ B · FULL ‎+۳۱ B — هزینهٔ rollback در
`CONDITIONAL` و back-solve موکول‌شده.


---

## ۹.۹ PHASE 17 — Unit Tests (اجرا شد)

### ۹.۹.۱ وضعیت نهایی

هارنس واقعی (`tests/Makefile`) جایگزین خط‌های دستی `gcc` شد. **۱۶ suite، ۷۵۲ assertion، همه سبز**،
و نتیجه در `-O0` و `-Os` و `-O2` **یکسان** است (هدف `make gate`).

| suite | assertion | موضوع |
|---|---|---|
| `smoke_core` | ✓ | دود اولیه |
| `test_antiwindup` | ۳۵ | چهار استراتژی anti-windup |
| `test_bumpless` | ۴۵ | سوئیچ حالت و تغییر gain |
| `test_fastpath` | ۴۹ | هم‌ارزی بیتی fast/full |
| `test_autotune_safety` | ۱۹ | ایمنی خودتنظیم |
| `test_autotune_accuracy` | ✓ | دقت شناسایی در برابر حل تحلیلی |
| `test_diag` | ۳۱ | تله‌متری و عیب‌یابی |
| `test_safety` | ۲۲ | محدودهٔ سنسور و failsafe |
| `test_fixed` | ۴۲ | Q15/Q31 |
| **`test_filter`** | **۸۲** | LPF1، میانگین متحرک، median-3، rate limiter، deadband |
| **`test_shaper`** | **۶۶** | پروفایل ذوزنقه‌ای/مثلثی، حفظ مسافت، retarget |
| **`test_gainsched`** | **۶۴** | درون‌یابی، hysteresis، پنج منبع، پیوستگی C1 |
| **`test_cascade`** | **۸۹** | decimation، clamp، anti-windup برگشتی، حالت‌ها |
| **`test_core_contract`** | **۹۳** | قرارداد API: NULL، هندل نامعتبر، اتمی بودن setter، جبرگرایی |
| `test_posix` | ۳۱ | لایهٔ POSIX |
| `test_stm32_host` | ۸۴ | لایهٔ STM32 روی stub |

### ۹.۹.۲ چهار نقص واقعیِ کتابخانه که این فاز پیدا کرد

هر چهار مورد **پیش از اصلاح** با یک assertion شکست‌خورده اثبات شد، نه با بازرسی چشمی.

**۱. `PID_UpdateFast(NULL)` → segfault.**
کامنت داخل تابع نوشته بود «بدون بررسی NULL، طبق قرارداد»، اما این قرارداد در هدر عمومی
مستند **نشده بود** و ۵۹ نقطهٔ ورودی دیگر NULL را تحمل می‌کردند. کتابخانه‌ای که در یک نقطه
crash می‌کند و در بقیه نه، یک تله است. هزینهٔ اندازه‌گیری‌شدهٔ guard: **۱۳ بایت** `.text` در `-Os`
و **بدون اثر زمانی** (۶٫۱۱ در برابر ۶٫۱۰ ns/call روی ۲۰ میلیون تکرار — داخل نویز، چون branch
کاملاً predict می‌شود). guard اضافه شد و قرارداد در هدر صریح شد: NULL امن است، اما هندلِ
Init-نشده همچنان بررسی نمی‌شود.

**۲. `PID_GetSampleTime()` روی هندل Init-نشده مقدار زباله برمی‌گرداند.**
تابع فقط NULL را چک می‌کرد و `dt_nominal` را از حافظهٔ اعتبارسنجی‌نشده می‌خواند. این تنها
شاهدی است که `PID_Cascade_Init()` برای تشخیص یک loop خراب در آرایه دارد — یعنی گاردِ
`PID_ERR_NOT_INIT` آن **هرگز نمی‌توانست فعال شود**. حالا با `pidp_valid()` اعتبارسنجی می‌شود.

**۳. `PID_GetGains()` همان مشکل را داشت** و بدتر: `PID_OK` برمی‌گرداند در حالی که محتوای
پشتهٔ فراخوان را در اشاره‌گرهای خروجی می‌ریخت — از یک tuning واقعی قابل تشخیص نیست.
هم‌خانوادهٔ آن `PID_GetStatus()` از ابتدا درست بود؛ حالا هر دو یکسان‌اند.

**۴. anti-windup آبشاری، حالت MANUAL/HOLD را نادیده می‌گرفت.**
مسیر back-propagation انتگرال‌گیرِ والد را مستقیماً با `PID_SetIntegrator()` می‌نوشت و بنابراین
گاردِ مرحلهٔ ۱۰ هستهٔ کنترل را دور می‌زد. نتیجه: آبشاری در HOLD با اشباع فرزند همچنان
«می‌خزید»، و در MANUAL با راه‌حل tracking خودش می‌جنگید. شرط `c->mode == AUTOMATIC` افزوده شد.

هزینهٔ هر چهار اصلاح روی همهٔ پروفایل‌ها یکسان است: **۳۵+ بایت** `.text`
(MINIMAL ۷۲۵۲ · MOTION ۱۶۰۵۷ · PROCESS ۱۹۴۷۴ · FULL ۲۳۸۶۰).

### ۹.۹.۳ نتایج کمّی تازه

**Anti-windup آبشاری** (پلنت دو-حالته، عملگر ±۱، پلهٔ واحد):

| حالت | overshoot | اوج \|I\| بیرونی | نشست |
|---|---|---|---|
| `AW_NONE` | ۳۵٫۹۹٪ | ۱٫۶۲۸ | ۲٫۹۹۸ s |
| `AW_BACK_CALC` | ۲۵٫۹۳٪ | ۱٫۱۵۳ | ۲٫۸۸۸ s |
| `AW_FREEZE` | ۱۷٫۶۵٪ | ۰٫۷۷۳ | ۲٫۷۲۶ s |

روی این پلنت `FREEZE` بهتر از `BACK_CALC` است — خلاف حدس رایج. دلیلش این است که پلنت یک
انتگرال‌گیر مضاعف است و «قطع کامل» انباشت، از «تصحیح متناسب» سریع‌تر بار انتگرال‌گیر را
خالی می‌کند. عدد همان‌طور که اندازه‌گیری شد ثبت می‌شود.

**رد کردن اغتشاش آبشاری در برابر تک‌حلقه** (همان تنظیم بیرونی، همان ریل): انحراف اوج
۰٫۰۰۶۲۸ در برابر ۰٫۰۵۲۸۹ — **۸٫۴ برابر** بهتر.

### ۹.۹.۴ درس‌های تست (افزوده به فهرست موجود)

- **(p)** مقایسهٔ `float` با ثابت اعشاری با تلورانس صفر، *نمایش* را تست می‌کند نه *رفتار* را:
  `0.1f != 0.1`. دو assertion به همین دلیل شکست خوردند و هر دو باگ تست بودند نه کتابخانه.
- **(q)** تست anti-windup روی پلنت **منجمد** بی‌معناست: وقتی فرزند هرگز نمی‌تواند بازیابی کند،
  انتگرال‌گیر والد در هر دو جهت به‌درستی قفل می‌ماند و تست چیزی دربارهٔ «جهت‌مندی» ثابت نمی‌کند.
  پلنت زنده لازم است.
- **(r)** خطای sticky «اولین برنده» باید **پیش از** بررسی بعدی خوانده و پاک شود، وگرنه کد قبلی
  خوانده می‌شود و به نظر می‌رسد خطای جدید هرگز گزارش نشده.
- **(s)** معنای `HOLD` را از مستند بخوانید نه از شهود: در PIDX یعنی «انتگرال‌گیر منجمد»، نه
  «خروجی منجمد». P و D باید همچنان پاسخ دهند.
- **(t)** `-D_POSIX_C_SOURCE` به‌تنهایی glibc را در حالت سخت‌گیر POSIX می‌گذارد و `M_PI` را
  (که نام X/Open است) **پنهان می‌کند**. suiteای که POSIX نمی‌خواهد نباید آن را تعریف کند.
- **(u)** یک suite که فقط NULL-safety را می‌آزماید، ارزشش را در همان اولین اجرا ثابت کرد:
  یک segfault واقعی. آزمودن «قرارداد» جدا از «قابلیت» ارزش دارد.

---

## ۹.۱۰ PHASE 18 — Simulation (در حال اجرا)

### ۹.۱۰.۱ مقایسهٔ قوانین تنظیم روی مدل دقیق

`sim/sim_rules.c` هر ۹ قانون داخلی را روی **مدل دقیق** هر پلنت اعمال
می‌کند. این تفکیک عمدی است: وقتی مدل دقیق باشد، هر تفاوتی که می‌بینید
متعلق به **خود قانون** است، نه به خطای شناسایی. `exact_ku_pu()` معادلهٔ
`atan(ωT)+ωL=π` را با bisection حل می‌کند تا `Ku`/`Pu` مرجع داشته باشیم.

بانک پلنت‌ها ۵ FOPDT با `L/T` از ۰.۱ تا ۱.۰. رتبه‌بندی بر پایهٔ IAE
نرمال‌شده (میانگین روی پلنت‌هایی که قانون از آن‌ها جان سالم برد):

| رتبه | قانون | mean nIAE | واگرا |
|---|---|---|---|
| ۱ | Ziegler-Nichols | ۰.۵۷۲۸ | ۰ |
| ۲ | Cohen-Coon | ۰.۶۰۵۹ | ۰ |
| ۳ | AMIGO-step | ۰.۶۵۹۷ | ۰ |
| ۴ | Pessen-Integral | ۰.۶۶۳۲ | ۰ |
| ۵ | Some-Overshoot | ۰.۷۷۸۰ | ۰ |
| ۶ | No-Overshoot | ۱.۰۲۷۵ | ۰ |
| ۷ | AMIGO-freq | ۱.۱۳۲۷ | ۰ |
| ۸ | IMC-lambda | ۱.۲۵۸۳ | ۰ |
| ۹ | Tyreus-Luyben | ۱.۵۴۷۴ | ۰ |

کمترین IAE **به‌خودی‌خود** انتخاب درست نیست: ZN رتبهٔ اول IAE را دارد
ولی `u_TV` و اورشوت بالایی هم دارد. این جدول باید کنار خروجی
`sim_robust` (عدم‌تطابق عمدی مدل) خوانده شود.

### ۹.۱۰.۲ ⚠️ قانون `NO_OVERSHOOT` واقعاً اورشوت می‌دهد — و توضیح قبلی ما غلط بود

شبیه‌سازی چیزی را آشکار کرد که باید توضیح داده می‌شد: قانونی به نام
«بدون اورشوت» روی پلنت آسان `K=2 T=1 L=0.1` **۴۲٪** اورشوت داد، آن هم
با مدل دقیق.

**اول فرض کردیم باگ است.** سه بار اشتباه از خودِ harness من درآمد
(درس‌های (ب) و (و): `PID_GetGains` داخل آرگومان‌های `printf`، و
`PID_Init` بدون بررسی `rc` که `rc=5` برمی‌گرداند و handle قدیمی را
زنده نگه می‌داشت). پس از اصلاح، نتیجه پابرجا ماند.

**آزمون قطعی** روی پلنت مثال ۰۷ (`K=80 T=40 L=8`):

| مدل داده‌شده به قانون | Kp | اورشوت |
|---|---|---|
| مدل **دقیق** (خطای شناسایی صفر) | ۰.۰۲۱۲۶ | **۳۰.۷٪** |
| مدل رله (‎−۲۷٪ روی Ku) | ۰.۰۱۵۵۲ | ۲۸.۰٪ |

یعنی خطای رله اورشوت را ۲.۷ واحد **کم** می‌کند. توضیح قبلی مثال ۰۷ که
اورشوت را به خطای رله نسبت می‌داد نه‌تنها اشتباه بلکه از نظر منطقی
متناقض بود (`Kp=0.20·Ku` ⇒ `Ku` کمتر ⇒ `Kp` کمتر).

**علت واقعی، اندازه‌گیری‌شده.** واریانت‌های «some/no overshoot» فقط
`Kp` را کم می‌کنند و `Ti=Pu/2`, `Td=Pu/3` را دست‌نخورده نگه می‌دارند.
روی FOPDT همان `Ti` عامل اصلی اورشوت است. با `Ti`, `Td` ثابت:

| `Kp/Ku` | ۰.۶۰ | ۰.۴۵ | ۰.۳۳ | ۰.۲۵ | ۰.۲۰ | ۰.۱۰ |
|---|---|---|---|---|---|---|
| اورشوت | ۴۴.۲٪ | ۳۸.۳٪ | ۳۸.۳٪ | ۴۱.۸٪ | ۴۳.۲٪ | ۴۱.۶٪ |

کم‌کردن `Kp` اورشوت را **بدتر** می‌کند: حلقه integral-dominated است و
کنش تناسبی ضعیف‌تر، انتگرال‌گیر را کمتر مهار می‌کند. اهرم واقعی `Ti`
است (در `Kp=0.20Ku`):

| `Ti/Pu` | ۰.۵ | ۱.۰ | ۲.۰ | ۳.۰ | ۴.۰ |
|---|---|---|---|---|---|
| اورشوت | ۴۳.۲٪ | ۲۲.۷٪ | ۸.۴٪ | ۲.۳٪ | ۰.۰٪ |

**ضرایب ما با جدول منتشرشده مطابق‌اند** (`Ti = 0.33Ku/(0.66Ku/Tu) =
Pu/2`، `Td = 0.11/0.33 = Pu/3`)، پس این محدودیت *خود قانون* است نه
پیاده‌سازی. سه اصلاح انجام شد:

1. `include/pidx/pid_autotune.h` — جدول راهنما دیگر `NO_OVERSHOOT` را
   برای «دما و فرایندهای یک‌طرفه» توصیه نمی‌کند (بدترین توصیهٔ ممکن:
   دقیقاً جایی که اورشوت پذیرفتنی نیست). یادداشت هشدار افزوده شد.
2. `src/pid_autotune_rules.c` — بلوک توضیح قانون، با اعداد.
3. `examples/07_autotune_relay/main.c` — توضیح غلط با مکانیزم واقعی
   جایگزین شد.

اگر اورشوت واقعاً باید نزدیک صفر باشد: `IMC` (با `lambda` بزرگ‌تر) یا
`AMIGO_STEP`، یا `Ti` را دستی کِش بدهید. **نام قانون یک قرارداد نیست.**

---

### ۹.۱۰.۳ مقاومت در برابر خطای مدل — مهم‌ترین یافتهٔ فاز ۱۸

فایل: `sim/sim_robust.c` → `sim/results/robust.csv` (۸۱۰ اجرا).

جدول §۹.۱۰.۱ هر قانون را روی **مدل دقیق** سنجید. اما در عمل مدل هیچ‌وقت
دقیق نیست: بهرهٔ فرایند با نقطهٔ کار عوض می‌شود، ثابت زمانی با دما، و
تأخیر با طول لوله و بار CPU. پرسش واقعی این است:

> کنترلری که روی مدل اسمی تنظیم شده، وقتی فرایند **زیر پایش عوض شود** چه
> می‌کند؟

**روش.** هر ۹ قانون روی مدل اسمی تنظیم می‌شود، سپس بدون اطلاع کنترلر،
پلنت روی **یک محور** مقیاس می‌خورد: `gain` یا `tau` یا `delay`، با ضرایب
`{0.5, 0.7, 1.0, 1.4, 1.7, 2.0}` (ضریب ۱.۰ شاهد است)، روی هر ۵ پلنت.
۹ × ۳ × ۶ × ۵ = ۸۱۰ اجرا.

**«بقا»** یعنی هر چهار شرط: IAE و پیک متناهی، رسیدن به ۹۰٪ پله، نشست در
باند ۲٪ پیش از افق شبیه‌سازی، و اورشوت کمتر از ۶۰٪.

| قانون | نرخ بقا | بدترین IAE |
|---|---|---|
| IMC-lambda | **۱۰۰٪** | ۳۵.۹۰ |
| No-Overshoot | ۹۹٪ | ۳۲.۵۹ |
| AMIGO-step | ۹۸٪ | **۲۱.۷۵** |
| AMIGO-freq | ۹۷٪ | ۳۴.۹۹ |
| Tyreus-Luyben | ۸۶٪ | ۲۵.۱۹ |
| Some-Overshoot | ۷۸٪ | ۲۵.۹۸ |
| Ziegler-Nichols | ۷۱٪ | ۲۱.۶۹ |
| Cohen-Coon | ۶۷٪ | ۲۲.۸۷ |
| Pessen-Integral | ۶۷٪ | ۲۸.۰۲ |

**این ترتیب تقریباً وارونهٔ §۹.۱۰.۱ است.** برنامه هر دو رتبه‌بندی را از
یک اجرا بازمی‌سازد تا کهنه نشود:

| قانون | رتبهٔ مدل دقیق | رتبهٔ مقاومت | جابه‌جایی |
|---|---|---|---|
| Ziegler-Nichols | ۱ | ۷ | **+۶** |
| Cohen-Coon | ۲ | ۸.۵ | **+۶.۵** |
| AMIGO-step | ۳ | ۳ | ۰ |
| Pessen-Integral | ۴ | ۸.۵ | +۴.۵ |
| Some-Overshoot | ۵ | ۶ | +۱ |
| No-Overshoot | ۶ | ۲ | −۴ |
| AMIGO-freq | ۷ | ۴ | −۳ |
| IMC-lambda | ۸ | ۱ | **−۷** |
| Tyreus-Luyben | ۹ | ۵ | −۴ |

همبستگی رتبه‌ای اسپیرمن: **ρ = −۰.۵۸۶**.

یعنی رتبه‌بندی بر پایهٔ کارایی روی مدل دقیق نه‌تنها مقاومت را پیش‌بینی
**نمی‌کند**، بلکه جهت را **اشتباه** نشان می‌دهد. ZN و Cohen-Coon که در
§۹.۱۰.۱ اول و دوم بودند، اینجا هفتم و هشتم‌اند؛ IMC که آخر بود، اول است.
آن IAE پایین با **حاشیهٔ پایداری** خریده شده بود.

> **صداقت آماری:** با ۹ قانون، این همبستگی دوطرفه p ≈ ۰.۱۰ است — یعنی
> **قابل‌توجه ولی نه معنادار در سطح ۵٪**، و هرگز هم معنادار نمی‌شود چون
> بیش از ۹ قانون وجود ندارد. علامت ρ یافته است، نه اندازه‌اش. شاهد
> محکم‌تر، نرخ بقای هر قانون است که روی ۹۰ اجرا بنا شده، و خود سلول‌های
> FAIL که هر کدام یک واگرایی بازتولیدپذیرند.

**چند مشاهدهٔ ریز که ارزش ثبت دارند:**

- محور `tau` با ضریب ۰.۵ بیشترین قربانی را می‌گیرد. غیرشهودی است — پلنت
  *سریع‌تر* می‌شود — ولی چون `L` ثابت می‌ماند، نسبت `L/T` **دو برابر**
  می‌شود و همین سختی را می‌سازد. راستی‌آزمایی شد: ZN روی پلنت easy با
  `tau` نصف‌شده در ثانیهٔ ۲۰ هنوز چرخهٔ حدی پایدار بین ۰.۵۰ و ۱.۲۲ دارد؛
  این واگرایی واقعی است نه ایراد سنجه.
- از ۱۲۵ شکست، ۸۹ مورد اورشوت > ۶۰٪ و ۳۶ مورد ننشستن در افق است؛ هیچ
  موردی سرریز عددی نیست.
- `IMC-lambda` در ستون `delay` تقریباً تخت است (۱۷.۹۱ → ۱۸.۵۰ از ۰.۵× تا
  ۲.۰×). این ویژگی طراحی IMC است، نه تصادف.

**نتیجهٔ عملی برای کاربر PIDX:** قانون را از جدول کارایی انتخاب نکنید. از
جدول مقاومت شروع کنید و بعد بررسی کنید کارایی قابل‌قبول است. اگر مدل‌تان
از یک تست پله‌ای ساده آمده — یعنی احتمالاً ±۳۰٪ خطا دارد — `AMIGO_STEP`
بهترین معامله است: رتبهٔ ۳ در هر دو جدول و **کمترین بدترین‌حالت IAE** در
کل مطالعه.

---

### ۹.۱۰.۴ دقت auto-tune — و اینکه آیا اصلاً مهم است

فایل: `sim/sim_autotune.c` → `sim/results/autotune.csv`.

§۹.۱۰.۱ و §۹.۱۰.۳ به هر قانون مدل **دقیق** دادند تا تفاوت‌ها قابل‌انتساب
باشد. ولی کاربر واقعی مدل دقیق ندارد؛ هرچه `PID_AutoTune_Update()` از یک
آزمایش روی پلنت نویزی درآورده را دارد. سؤال سوم — که تقریباً هیچ‌وقت پرسیده
نمی‌شود — این است: **خطای شناسایی چقدر هزینه دارد؟**

روش **جفتی**: برای هر مدل شناسایی‌شده، یک کنترلر دوم از مدل دقیق و با همان
قانون ساخته می‌شود و هر دو روی همان پلنت اجرا می‌شوند. پس اختلاف IAE فقط
خطای شناسایی است. قانون AMIGO است (نه ZN) چون §۹.۱۰.۳ نشان داد ZN کم‌تحمل‌ترین
قانون نسبت به خطای مدل است و مطالعه‌ای دربارهٔ خطای مدل نباید با آن اجرا شود.

| روش | تکمیل | میانگین خطای بهره | بدترین پارامتر | میانگین جریمه | بدترین جریمه |
|---|---|---|---|---|---|
| relay | ۱۳/۱۵ | ۲۷.۷٪ | Ku/Pu ۳۰.۳٪ | ۱۴.۶٪ | ۳۳.۶٪ |
| step | ۱۵/۱۵ | **۰.۳٪** | K/T/L ۱۸.۹٪ | ۹.۹٪ | ۵۱.۶٪ |

**تلهٔ تفسیری که کم مانده بود در آن بیفتیم:** اگر فقط ستون «خطای بهره» را
گزارش می‌کردیم، تست پله بی‌نقص به نظر می‌رسید (`K` با ۰.۳٪ خطا) در حالی که
بدترین جریمهٔ کل مطالعه (۵۱.۶٪) مال خودش است. علت این است که `K` پارامتر
مقصر نیست: خطای **تأخیر مرده** در تست پله تا **+۸۲٪** می‌رسد. همبستگی جریمه
با |خطای L| برابر ۰.۹۱ در برابر ۰.۸۹ برای |خطای K|. ستون «بدترین پارامتر»
دقیقاً برای جلوگیری از این تفسیر غلط اضافه شد.

**چرا relay همیشه `Ku` را کم می‌زند** (ادامهٔ یافتهٔ مثال ۰۷ با −۲۷٪): تئوری
تابع توصیفی دامنهٔ **مؤلفهٔ اصلی** را می‌خواهد، پیاده‌سازی عملی دامنهٔ **پیک**
را می‌سنجد. چرخهٔ حدی پاسخ به موج مربعی است و هارمونیک دارد، پس
`a_pk > a1` همیشه، و `Ku = 4h/(pi·a)` کوچک می‌شود:

| پلنت | L/T | a_pk/a1 | خطای Ku (پیک) | خطای Ku (اصلی) |
|---|---|---|---|---|
| easy | ۰.۱۰ | ۱.۲۳ | **−۲۴.۹٪** | −۷.۴٪ |
| typical | ۰.۳۰ | ۱.۲۲ | −۱۸.۹٪ | −۰.۹٪ |
| hard | ۱.۰۰ | ۱.۱۶ | −۱۱.۵٪ | +۲.۵٪ |

هرچه تأخیر مرده کمتر، پلنت هارمونیک‌ها را کمتر فیلتر می‌کند و خطا بدتر
می‌شود — همان ترتیبی که جاروب گزارش می‌کند. تصحیح پیک→اصلی بیشتر خطا را
برمی‌دارد، پس علت **هارمونیک** است نه باگ؛ `src/pid_autotune.c` فرمول
استاندارد با تصحیح hysteresis را درست پیاده کرده است.

PIDX عمداً فرم پیک را نگه می‌دارد: روی تارگت به FFT نیاز ندارد و خطایش در
**جهت امن** است (`Ku` کمتر ⇒ بهرهٔ کمتر ⇒ حلقهٔ کُند نه ناپایدار). در کل
جاروب هیچ اجرای relay واگرا نشد.

دو شکست گزارش شد نه پنهان: `slow` با نویز ۰.۰۱ → `no oscillation`،
`furnace` → `validation failed`. اینها شکست‌های واقعی commissioning‌اند.

### ۹.۱۰.۵ نمودارها

`sim/plot.py` پنج شکل می‌سازد (`sim/results/fig*.png`). این اسکریپت هیچ
محاسبهٔ کنترلی در پایتون انجام نمی‌دهد و فقط CSVهای تولیدشده توسط C را
می‌خواند، پس یک نمودار نمی‌تواند با مطالعه‌ای که تصویرش می‌کند مخالف باشد.

`fig4_rank_inversion.png` یافتهٔ §۹.۱۰.۳ را در یک نگاه نشان می‌دهد.

> **یک اشتباه که گرفته شد:** عنوان `fig1` روی «easy L/T=0.1» hardcode شده
> بود در حالی که `sim_rules.c` پلنت **typical** را ترسیم می‌کند. فقط به این
> دلیل لو رفت که محور زمان تا ۲۵ ثانیه می‌رفت و افق پلنت easy بیست ثانیه
> است. حالا عنوان از روی خود داده استخراج می‌شود.

### ۹.۱۰.۶ یک تلهٔ محیطی که به اشتباه شبیه رگرسیون است

هنگام تأیید نهایی فاز ۱۸، `make run` در `tests/` **چهار suite** را ناموفق
نشان داد که قبلاً سبز بودند. کد سالم بود: چهار باینری قدیمی بیت اجرا (`+x`)
را از دست داده بودند و خطا `Permission denied` بود، نه assertion failure.
تایم‌استمپشان هم با بقیه فرق داشت.

این در محیط‌هایی رخ می‌دهد که workspace را snapshot/restore می‌کنند، یا با
استخراج آرشیو، کپی از ویندوز، یا mount شدن با `noexec`.

چون تشخیصش وقت می‌گیرد و دقیقاً شبیه یک رگرسیون واقعی است، هر سه Makefile
(`tests/`, `sim/`, `examples/`) حالا پیش از اجرا `chmod +x` می‌زنند. وقتی
مجوزها درست باشند این کار هزینه‌ای ندارد. راستی‌آزمایی شد: دو باینری عمداً
`chmod -x` شدند و `make run` باز هم **ALL SUITES PASSED** داد.

> **درس:** وقتی چند suite بی‌ربط هم‌زمان و بدون تغییر کد شکست می‌خورند، اول
> `ls -l bin/` را نگاه کنید نه کد را.

## ۱۰. Memory & Performance Strategy

### ۱۰.۱ مدل حافظه
- **Static/Automatic only.** کاربر `PID_Handle` را تعریف می‌کند (global/static توصیه می‌شود اگر
  با ISR به اشتراک گذاشته می‌شود؛ روی stack فقط برای استفاده محلی).
- بافر telemetry متعلق به کاربر است و با اشاره‌گر attach می‌شود ⇒ اندازه‌اش تصمیم کاربر است، نه ما.
- جدول Gain Schedule می‌تواند `const` در Flash باشد (handle فقط اشاره‌گر نگه می‌دارد).
- مصرف stack هدف: `PID_Update` < 64 بایت، `PID_AutoTune_Update` < 96 بایت (بدون بازگشتی، بدون VLA، بدون آرایه بزرگ محلی).

### ۱۰.۲ اهداف کارایی (Cortex-M4F @ 168 MHz، `-O2 -mfpu=fpv4-sp-d16`)

| تابع | هدف (cycles) | یادداشت |
|---|---:|---|
| `PID_UpdateFast` | **< 100** | ۹ FP op، صفر تقسیم |
| `PID_Update` (Typical: limits+AW+D-filter) | < 160 | |
| `PID_Update` (Full: +FF+shaper+safety+diag) | < 320 | |
| `PID_UpdateDt` با dt ثابت (کش‌شده) | +8 | یک مقایسه |
| `PID_UpdateDt` با dt متغیر واقعی | +90 | ۳ تقسیم بازمحاسبه |
| `PID_AutoTune_Update` (حالت عادی) | < 60 | |
| Flash (Core فقط، `-Os`) | < 2.0 KB | هدف |
| Flash (همه ماژول‌ها) | < 9 KB | هدف |

**⚠️ صداقت روش‌شناختی:** در محیط فعلی این Workspace، `arm-none-eabi-gcc` نصب نیست. بنابراین:
- اعداد بالا **هدف طراحی**‌اند، نه اندازه‌گیری.
- Benchmark واقعی که تحویل می‌دهم شامل: (الف) اندازه‌گیری روی x86 با `clock_gettime` و
  شمارش دقیق عملیات ممیز شناور به‌ازای هر مسیر، (ب) هارنس `bench_dwt.c` آماده که کاربر روی
  سخت‌افزار خودش با DWT->CYCCNT اجرا می‌کند و جدول را پر می‌کند، (ج) گزارش اندازه با
  `size`/`nm` روی build میزبان به‌عنوان تقریب نسبی.
- اگر می‌خواهید اعداد واقعی Cortex-M در سند باشد، باید toolchain نصب شود؛ در غیر این صورت
  در مستندات صریحاً «هدف/تخمین» برچسب می‌خورند. **عدد ساختگی گزارش نمی‌شود.**

### ۱۰.۳ تکنیک‌های بهینه‌سازی به‌کاررفته
1. پیش‌محاسبه ضرایب (حذف کامل تقسیم از مسیر داغ).
2. بیت‌مسک `features` + بررسی گروهی: اگر `(features & PID_ADVANCED_MASK) == 0` یک شاخه کل بلوک
   پیشرفته را رد می‌کند (branch predictor-friendly، در عمل همیشه یک نتیجه).
3. `static inline` برای clamp/isfinite در هدر (بدون هزینه فراخوانی).
4. ترتیب فیلدها بر اساس دمای دسترسی (فیلدهای داغ در ۶۴ بایت اول).
5. اجتناب از `double` promotion: همه ثابت‌ها با پسوند `f`؛ warning `-Wdouble-promotion` فعال.
6. اجتناب از `fabsf` در جاهایی که مقایسه علامت کافی است.
7. عدم استفاده از `math.h` در مسیر داغ؛ `sqrtf` فقط در Auto-Tune.

---

## ۱۱. Test Strategy (طرح اولیه؛ نتیجهٔ اجرا در §۹.۹ PHASE 17)

### ۱۱.۱ سطوح تست
| سطح | ابزار | محتوا |
|---|---|---|
| L0 Static | `gcc -std=c99 -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow -Wcast-qual -pedantic -Werror` | صفر warning |
| L1 Unit | هارنس داخلی `pid_test.h` (بدون وابستگی) | ~۹۰ تست، هر feature جدا |
| L2 Property | تست‌های خاصیتی | مثال: «خروجی همیشه در [min,max]»، «I هرگز NaN نمی‌شود» با ورودی تصادفی |
| L3 Integration | plantهای شبیه‌سازی | FOPDT+delay، دو-قطبی، انتگرال‌گیر+تأخیر، عملگر اشباع‌شونده، نویز گاوسی |
| L4 Regression | CSV مرجع + مقایسه با tolerance | جلوگیری از تغییر ناخواسته رفتار عددی |
| L5 Coverage | `gcov` | هدف: > 90% خط برای Core، > 80% کل |

### ۱۱.۲ تست‌های کلیدی (نمونه از فهرست کامل)
- **صحت ریاضی**: P خالص با ورودی پله → دقیقاً `Kp*e`؛ I خالص → رشد خطی با شیب `Ki*e`؛
  D با ورودی رمپ → مقدار پایای `Kd*slope` (تست تحلیلی، نه چشمی).
- **بدون Anti-Windup در برابر با آن**: پله بزرگ روی plant اشباع‌شونده؛ سنجش overshoot و
  زمان بازگشت — باید عدد مشخصی بهتر شود (assert کمی، نه کیفی).
- **Bumpless**: `|u[k] − u[k−1]|` در لحظه سوئیچ MANUAL→AUTO باید `< 1e-5`.
- **Bumpless gain change**: تغییر Ki در حالت پایا → پرش خروجی `< 1e-6`.
- **Derivative kick**: پرش setpoint با `DERIV_ON_ERROR` (باید spike بزرگ باشد) در برابر
  `DERIV_ON_MEASUREMENT` (باید spike صفر باشد) — assert روی نسبت.
- **پایداری عددی**: تزریق NaN/Inf/dt=0/dt منفی/limitهای معکوس/Ki=0/Kp=0/Tf=0/Tf≫dt.
- **معادل‌سازی Fixed-Point**: اجرای همان سناریو روی float و Q15، assert خطای RMS < آستانه مستند.
- **Auto-Tune واقعی**: relay روی FOPDT با پارامترهای معلوم؛ مقایسه $K_u,P_u$ شناسایی‌شده با
  مقدار تحلیلی (از حل $\angle G(j\omega)=-\pi$) — تلورانس ۱۵٪ برای $K_u$، ۱۰٪ برای $P_u$.
  **این تست، تضمین «Auto-Tune واقعی است و fake نیست» را می‌دهد.**
- **قبل/بعد از tuning**: IAE پس از tuning باید از IAE با gainهای اولیه نامناسب کمتر باشد.
- **Cascade**: با اشباع حلقه داخلی، حلقه بیرونی نباید windup کند (assert روی integrator بیرونی).
  **نتیجهٔ اندازه‌گیری‌شده** (پلنت دومرحله‌ای: ریل ±۱ → سرعت τ=0.05 → موقعیت τ=0.5، setpoint دست‌نیافتنی ۲۰ و سپس افت به ۰.۵):
  `AW_NONE` → اوج انتگرال‌گیر بیرونی ۴۹.۳۸ و **هرگز** در بازهٔ ۲٪ ننشست؛
  `AW_BACK_CALC` → اوج ۱.۶۱ و نشست در ۶.۱۵ ثانیه؛ `AW_FREEZE` → اوج ۷.۳۵ و نشست در ۱۰.۶۷ ثانیه.
  یعنی back-calculation اوج انتگرال‌گیر را ۳۰ برابر کوچک می‌کند و تنها حالتی است که بازیابی سریع می‌دهد.

### ۱۱.۳ معیارهای شبیه‌سازی
`sim/` این‌ها را محاسبه و در CSV می‌ریزد: Rise Time (10–90%)، Settling Time (±2%)،
Overshoot %، Steady-State Error، IAE، ITAE، Total Variation of control (معیار فرسایش عملگر).
`plot.py` نمودارها را می‌سازد (matplotlib موجود است) و در `sim/results/` ذخیره می‌کند.
## ۱۲. Critical Design Review — نقد Specification شما

طبق بند ۸۹ خواستید هرچه از نظر کنترلی/Embedded/عددی/معماری اشتباه یا غیرمنطقی است را صریح بگویم.
۱۳ مورد یافتم. برای هرکدام: **مشکل → دلیل → اصلاح پیشنهادی**.

### ۱۲.۱ ❌ Cohen–Coon از داده Relay قابل استخراج نیست (بند ۲۵)
شما ZN، TL و Cohen-Coon را کنار هم زیر Relay Auto-Tune فهرست کرده‌اید. اما رله یک نقطه فرکانسی
$(K_u, P_u)$ می‌دهد، در حالی که Cohen-Coon (و IMC و AMIGO-step) به مدل FOPDT $(K,T,L)$ نیاز دارند.
هر «تبدیل» بین این دو بدون فرض اضافی، **جعلی** است.
**اصلاح:** معماری دو-مرحله‌ای §۹.۱ (Identification جدا از Rule) + افزودن **Step-Test Identification**
به‌عنوان روش شناسایی دوم. ترکیب ناسازگار → `PID_ERR_TUNE_MODEL_MISMATCH`.

### ۱۲.۲ ❌ «Adaptive Tuning» در سطح Expert (بند ۴) — ریسک Fake شدن
کنترل تطبیقی واقعی (MRAC، Self-Tuning Regulator با RLS، پایداری Lyapunov) یک حوزه مستقل است،
نه یک feature جانبی PID. پیاده‌سازی سرسری‌اش دقیقاً همان چیزی است که بند ۸۴ خودتان منع کرده.
**اصلاح:** «Adaptive» از فهرست حذف و با سه چیز *واقعی* جایگزین می‌شود:
(الف) **Gain Scheduling** با درون‌یابی (تطبیق برنامه‌ریزی‌شده)،
(ب) **Supervised Re-Tuning**: API برای اجرای دوره‌ای/دستی مجدد relay test در نقطه‌کار جدید،
(ج) قلاب `PID_TuneRuleFn` تا کاربر الگوریتم تطبیقی خودش را وصل کند.
اگر واقعاً STR/RLS می‌خواهید، باید به‌عنوان پروژه جدا با تست پایداری خودش تعریف شود — بگویید تا اضافه کنم.

### ۱۲.۳ ⚠️ دسترسی مستقیم `pid.setpoint = target` (بند ۱۲)
نوشتن مستقیم روی فیلد، shaper و اعتبارسنجی و پرچم‌ها را دور می‌زند و در آینده هر تغییر چیدمان
struct کد کاربر را می‌شکند (ضد بند ۶۷).
**اصلاح:** `PID_Handle` عمومی می‌ماند (لازم برای تخصیص static)، اما مستند می‌شود که
**نوشتن مستقیم روی فیلدها پشتیبانی نمی‌شود**. برای نیاز سرعت: `PID_SetSetpointImmediate()`
که یک `static inline` بدون اعتبارسنجی است و shaper را دور می‌زند — صریح و آگاهانه.

### ۱۲.۴ ⚠️ «Maximum Temperature» به‌عنوان محافظ Auto-Tune (بند ۲۷)
دما یک متغیر دامنه-خاص است؛ کتابخانه عمومی نباید بداند measurement دما است.
**اصلاح:** تعمیم به `meas_min/meas_max` + `rate_max` + callback `abort_fn` که کاربر می‌تواند
در آن هر شرط دامنه‌ای (دمای دوم، جریان، فشار…) را بررسی کند.

### ۱۲.۵ ⚠️ Fixed-Point به‌عنوان «لایه» روی همان API (بند ۸)
اگر Q15/Q31 را پشت همان `PID_Handle` و همان توابع بگذاریم، یا typedef شرطی می‌شود (که
مانع هم‌زیستی float و fixed در یک پروژه است — سناریوی واقعی: حلقه جریان fixed، حلقه دما float)،
یا هسته پر از `#if` می‌شود.
**اصلاح:** `PIDq_*` یک **کنترلر مستقل و کوچک** با API آینه‌ای (`PIDq_Init/Update/Reset/SetGains`)
در فایل جدا، بدون هیچ وابستگی به هسته float. هر دو می‌توانند هم‌زمان در یک باینری باشند.
**دامنه صادقانه Fixed-Point:** P/I/D + مشتق روی اندازه‌گیری + EMA فیلتر (شیفت‌محور) +
clamp + conditional integration + جهت + manual/auto. **پشتیبانی نمی‌شود:** auto-tune، gain
scheduling با درون‌یابی، back-calculation با Kt دلخواه (فقط توان‌های ۲)، shaper. این محدودیت‌ها
در `docs/16_fixed_point.md` صریح فهرست می‌شوند.

### ۱۲.۶ ⚠️ `PID_DeInit()` (بند ۴۷) بدون حافظه پویا بی‌معناست
**اصلاح:** نگه داشته می‌شود ولی با معنای دقیق: پاک‌کردن state، صفر کردن `init_magic` (تا
`PID_Update` روی handle باطل با `PID_ERR_NOT_INIT` برگردد) و جدا کردن telemetry. مفید برای
تست و برای فرآیندهای ایمنی. نامش `PID_Deinit` (سازگار با سبک نام‌گذاری) می‌شود.

### ۱۲.۷ ⚠️ Moving Average در هسته (بند ۴۶)
MA با پنجره $N$ تأخیر فاز $\frac{(N-1)T_s}{2}$ می‌سازد و در مسیر مشتق حاشیه فاز را می‌خورد؛
برای PID تقریباً همیشه انتخاب بدتری از LPF مرتبه‌اول است، ضمن اینکه بافر $N$ تایی RAM می‌خواهد.
**اصلاح:** MA فقط به‌عنوان ابزار مستقل در `pid_filter.h` (برای پیش‌پردازش سنسور، مثلاً میانگین‌گیری
ADC) با هشدار مستند؛ **در هسته و در مسیر مشتق قرار نمی‌گیرد**.

### ۱۲.۸ ⚠️ Setpoint Ramp با accel/decel = مینی تولیدکننده مسیر (بند ۱۲)
این در واقع یک trajectory generator ذوزنقه‌ای است، نه یک ramp ساده. پیاده‌اش می‌کنم (واقعی، با
منطق فاصله ترمز $v^2/2a$ برای جلوگیری از overshoot در setpoint)، اما مستند می‌کنم که برای
کنترل حرکت جدی (S-curve/jerk-limited، هماهنگی چندمحوره) باید از یک motion planner واقعی استفاده شود.
پیشنهاد: در `PID_ShaperConfig` بماند و از هسته با `#if` جدا شود.

### ۱۲.۹ ⚠️ ادعای «MISRA-C:2012» بدون ابزار بررسی (بند ۶)
بدون PC-lint/Cppcheck-MISRA/Polyspace، ادعای انطباق قابل اثبات نیست و در پروژه صنعتی
ادعای اثبات‌نشده بدتر از نبود ادعاست.
**اصلاح:** عبارت رسمی می‌شود «**MISRA-C:2012 aligned**» با:
(الف) رعایت قواعد پرکاربرد (بدون تخصیص پویا، بدون بازگشتی، یک return در توابع کوتاه، پرانتزگذاری
کامل، cast صریح، `const` correctness، عدم اتکا به ترتیب ارزیابی، بدون `goto`)،
(ب) فایل `docs/22_misra_deviations.md` با فهرست انحرافات آگاهانه و توجیه‌شان
(مثلاً Rule 11.5 در `void*` مربوط به `ctx` کاربر، Rule 17.8 در جاهایی، Directive 4.9 برای
ماکروهای inline-like)،
(ج) `-Wall -Wextra -Wconversion -Werror` به‌عنوان دروازه حداقلی قابل اثبات.

### ۱۲.۱۰ ⚠️ Telemetry داخل ISR (بندهای ۳۳/۳۹)
نوشتن رکورد از ISR و خواندن از main loop، بدون طراحی درست، race است.
**اصلاح:** بافر حلقوی **SPSC lock-free** با اندازه توان-۲، فقط یک تولیدکننده (ISR) و یک
مصرف‌کننده (main)، اندیس‌های `volatile uint16_t`، و در نسخه ARM با `__DMB()` اختیاری (از طریق
ماکروی قابل override `PIDX_MEMORY_BARRIER()` تا هسته به CMSIS وابسته نشود). سیاست overflow:
drop-oldest، با شمارنده `dropped` تا کاربر بداند.

### ۱۲.۱۱ ⚠️ دو سطح فعال‌سازی feature (بندهای ۳۶/۳۷) ریسک ناسازگاری
اگر compile-time خاموش باشد ولی runtime enable صدا زده شود، رفتار باید مشخص باشد.
**اصلاح:** یک بیت‌مسک واحد `features`. ماکروهای compile-time، بیت‌های متناظر را در ماسک
«مجاز» می‌سازند. `PID_EnableFeature()` روی feature کامپایل‌نشده → `PID_ERR_UNSUPPORTED`
(نه شکست خاموش). ماکروی `PID_HAS_FEATURE(x)` برای بررسی زمان کامپایل.

### ۱۲.۱۲ ⚠️ `PID_Update` هم خروجی هم خطا (بندهای ۴۰/۴۷)
نمی‌شود هم float برگرداند هم کد خطا، و اضافه‌کردن پارامتر خروجی به مسیر داغ هزینه دارد.
**اصلاح:** سه‌گانه صریح: `PID_Update()` (float، سریع، خطا در sticky state)،
`PID_UpdateEx()` (کامل، با `PID_StatusCode*` اختیاری)، `PID_GetLastError()` برای بازیابی.
`last_error` **sticky** است تا رویداد نادر در بین نمونه‌ها گم نشود؛ `PID_ClearError()` پاکش می‌کند.

### ۱۲.۱۳ ℹ️ Trapezoidal (Tustin) به‌عنوان پیش‌فرض؟ — نه
Tustin دقت بهتری دارد ولی: state اضافه، حساسیت بیشتر به نویز در ترکیب با clamp، و رفتار
پیچیده‌تر با back-calculation. برای اکثریت قاطع کاربردهای Embedded که $\Delta t \ll$ ثابت زمانی
غالب، تفاوت عملی ناچیز است.
**تصمیم:** Backward-Euler پیش‌فرض، Trapezoidal گزینه‌ای مستند برای حلقه‌های آهسته‌نمونه.

### مواردی از Specification که کاملاً تأیید می‌کنم
مشتق روی اندازه‌گیری به‌عنوان پیش‌فرض، ممنوعیت HAL در Core، ممنوعیت malloc، Auto-Tune
غیرمسدودکننده به‌صورت ماشین حالت، جداسازی fast path، بیت‌مسک compile-time، الزام bumpless
در تغییر gain، و بند ۸۴ (منع Fake Completeness) — این‌ها دقیقاً همان تصمیم‌هایی هستند که یک
کتابخانه صنعتی را از نمونه آموزشی جدا می‌کنند.

---

## ۱۳. برنامه اجرای فازها و تحویل

| فاز | محتوا | خروجی |
|---|---|---|
| ۱–۳ ✅ | Architecture / API / Data Structures | همین سند |
| ۴–۶ ✅ | Core + Limits + Anti-Windup + Derivative/Filter | `pid_types.h`, `pid_conf.h`, `pid_math.h`, `pid.h`, `pid.c`, `pid_filter.*` |
| ۷–۹ ✅ | 1DOF/2DOF + Feedforward + Mode/Bumpless + Shaper | تکمیل `pid.c`, `pid_shaper.*` |
| ۱۰–۱۱ ✅ | Gain Scheduling + Cascade | `pid_gainsched.*`, `pid_cascade.*` |
| ۱۲ ✅ | Auto-Tune (relay + step + rules + safety) | `pid_autotune.*`, `pid_autotune_rules.c` |
| ۱۳–۱۴ ✅ | Diagnostics/Telemetry/Safety + Fixed-Point | `pid_diag.*`, `pid_fixed.*` |
| ۱۵–۱۶ ✅ | STM32/POSIX Integration + ۱۰ مثال | `platform/`, `examples/` |
| ۱۷ ✅ | Unit Tests (۱۶ suite، ۷۵۲ assertion — §۹.۹) | `tests/` + `tests/Makefile` |
| ۱۸ ✅ | Simulation — ۳ مطالعه (قوانین §۹.۱۰.۱، مقاومت/۸۱۰ اجرا §۹.۱۰.۳، دقت auto-tune §۹.۱۰.۴) + ۵ نمودار + README | `sim/` |
| ۱۹ ✅ | Benchmark — `bench_host.c` + `bench_dwt.c` + `size_report.sh` + Makefile | `bench/` |
| ۲۰ ✅ | مستندات فارسی (۲۳ فایل) + README انگلیسی + LICENSE + Makefile ریشه | `docs/`, `README.md` |
| ۲۱ ✅ | Review نهایی — همهٔ اعداد و مسیرها راستی‌آزمایی شد | — |

در پایان هر فاز: کامپایل با `-Werror`، اجرای کل تست‌ها، و گزارش کوتاه «چه ساخته شد / چه ریسکی باقی است».
