# PIDX Examples — یادگیری گام‌به‌گام با مثال‌های عملی

مجموعه‌ای از ۱۰ مثال کامل و مستقل برای یادگیری کتابخانه **PIDX** — از ساده تا پیشرفته.

هر مثال یک فایل `.c` مستقل است که می‌توانید مستقیم در پروژه STM32 خود کپی کنید.
هر فایل شامل: شبیه‌ساز پلنت، راه‌اندازی (`exampleXX_init()`) و تیک (`exampleXX_tick()`).

---

## فهرست مثال‌ها

| # | پوشه | موضوع | سطح | مفاهیم کلیدی |
|:-:|------|-------|:----:|--------------|
| ۱ | `01_minimal` | ساده‌ترین PID | Basic | `PID_InitDefault`, `PID_SetGains`, `PID_Update` |
| ۲ | `02_antiwindup` | کنترل دما | Basic | Anti-Windup (۴ استراتژی), محدودیت خروجی |
| ۳ | `03_motor_speed` | سرعت موتور | Intermediate | Derivative kick, 2DOF (β), اصطکاک کولن |
| ۴ | `04_motor_position` | موقعیت موتور | Intermediate | Setpoint Shaper, Ramp ذوزنقه‌ای |
| ۵ | `05_current_fastpath` | جریان ۲۰kHz | Advanced | `PID_UpdateFast`, تنظیم تحلیلی PI |
| ۶ | `06_cascade` | آبشاری سه‌سطحی | Advanced | Cascade: موقعیت←سرعت←جریان |
| ۷ | `07_autotune` | تنظیم خودکار | Expert | Relay + Step شناسایی, قوانین Tuning |
| ۸ | `08_freertos` | FreeRTOS | Expert | `vTaskDelayUntil`, Feedforward تابعی |
| ۹ | `09_tim_isr` | TIM ISR | Expert | Telemetry SPSC, Diagnostics, Flags |
| ۱۰ | `10_full_featured` | همه قابلیت‌ها | Expert | Gain Sched + FF + Shaper + Safety + 2DOF |

---

## نصب و استفاده

### پیش‌نیازها

- کتابخانه PIDX (فایل‌های `src/*.c` و `include/pidx/`)
- کامپایلر C99 (GCC برای STM32 یا PC)

### در STM32 (CubeIDE / Makefile)

1. فایل مثال را در `Core/Src/` کپی کنید.
2. مسیر include را به پوشه `include/` کتابخانه PIDX تنظیم کنید.
3. فایل‌های `src/*.c` کتابخانه را به پروژه اضافه کنید.
4. در `main.c`:

```c
#include "example01_minimal.c"   /* یا هر مثال دیگر */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    example01_init();              /* یکبار راه‌اندازی */

    /* تایمر را با نرخ مثال (مثلاً 10ms) شروع کنید */
    HAL_TIM_Base_Start_IT(&htim3);
    while (1) { }
}

/* در TIM ISR: */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        example01_tick();          /* هر تیک کنترلر */
    }
}
```

### در PC (تست سریع بدون سخت‌افزار)

هر فایل با `-DTEST_ON_PC` کامپایل می‌شود و خودش یک `main()` دارد:

```bash
gcc -std=c99 -DTEST_ON_PC -I../include \
    ../src/pid.c ../src/pid_filter.c ../src/pid_diag.c \
    ../src/pid_gainsched.c ../src/pid_shaper.c \
    example01_minimal.c -lm -o test_ex01
./test_ex01
```

**نکته:** مثال ۷ (Auto-Tune) به فایل‌های `pid_autotune.c` و `pid_autotune_rules.c` نیز نیاز دارد.
مثال ۶ (Cascade) به `pid_cascade.c` نیاز دارد.

---

## جدول نرخ نمونه‌برداری هر مثال

| مثال | نرخ | تایمر STM32 (توصیه‌شده) |
|:----:|:---:|:------------------------:|
| ۱ | ۱۰۰ Hz | TIM3 با 10ms |
| ۲ | ۲ Hz | TIM3 با 500ms |
| ۳ | ۱ kHz | TIM3 با 1ms |
| ۴ | ۱ kHz | TIM3 با 1ms |
| ۵ | ۲۰ kHz | TIM1 با 50µs (PWM master) |
| ۶ | ۲۰ kHz | TIM1 با 50µs |
| ۷ | ۱۰ Hz | TIM3 با 100ms |
| ۸ | ۱۰۰ Hz | FreeRTOS task (10ms) |
| ۹ | ۱ kHz | TIM3 با 1ms |
| ۱۰ | ۱ kHz | TIM3 با 1ms |

---

## مشکلات رایج Auto-Tune (از تجربه واقعی)

| مشکل | علت | راه‌حل |
|------|------|--------|
| تیونر timeout می‌شود | پلنت به حالت پایدار نرسیده | `plant_settle()` را قبل از شروع صدا بزنید |
| نویز زیاد | SNR پایین برای شناسایی | نویز ≤ ۰.۱ یا `output_step` را زیاد کنید |
| کیفیت < ۵۰ | L/T < 0.05 (بدون تاخیر) | از STEP test با پوشش کافی استفاده کنید |
| مدل FOPDT غلط | تاخیر داخل فیلتر پلنت | فیلتر و تاخیر را جدا نگه دارید (`y_nodelay` vs `plant_y`) |

---

## مجوز

همان مجوز کتابخانه PIDX — MIT License.
