# ۱۸ — یکپارچه‌سازی با STM32

## هستهٔ کتابخانه هیچ چیز از STM32 نمی‌داند

در `src/` نه `stm32f4xx_hal.h` هست و نه `HAL_GetTick()`. لایهٔ پلتفرم
**اختیاری** و جداست: `platform/stm32/`.

می‌توانید کل `platform/` را نادیده بگیرید و خودتان `dt` بدهید.

## ساده‌ترین حالت: ISR تایمر

```c
#include "pidx/pid.h"

static PID_Handle pid;

void control_init(void)
{
    PID_InitDefault(&pid);
    PID_SetGains(&pid, 2.0f, 0.5f, 0.1f);
    PID_SetSampleTime(&pid, 0.001f);        /* ۱ کیلوهرتز */
    PID_SetOutputLimits(&pid, 0.0f, 1000.0f);   /* ARR تایمر PWM */
    PID_SetSetpoint(&pid, 100.0f);
}

void TIM2_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        LL_TIM_ClearFlag_UPDATE(TIM2);

        float y = (float)LL_ADC_REG_ReadConversionData12(ADC1);
        float u = PID_Update(&pid, y);
        LL_TIM_OC_SetCompareCH1(TIM3, (uint32_t)u);
    }
}
```

چون `sample_time` ثابت است و تایمر نرخ را تضمین می‌کند، `PID_Update` کافی
است — نیازی به `PID_UpdateDt` نیست.

## لایهٔ پلتفرم (اختیاری)

اگر می‌خواهید timebase آماده داشته باشید:

```c
#include "pid_stm32.h"

PIDs_TimebaseInitDwt(SystemCoreClock);   /* یا InitTim / InitCallback */

uint32_t t0 = PIDs_NowUs32();
/* ... */
uint32_t dt_us = PIDs_DeltaUs(t0, PIDs_NowUs32());
```

سه منبع زمان:

| تابع | منبع |
|---|---|
| `PIDs_TimebaseInitTim` | یک تایمر عمومی |
| `PIDs_TimebaseInitDwt` | شمارندهٔ سیکل DWT (M3/M4/M7) |
| `PIDs_TimebaseInitCallback` | هر تابع دلخواه شما |

`PIDs_DeltaUs` سرریز شمارنده را درست مدیریت می‌کند — یک اشتباه کلاسیک که
هر ۷۱ دقیقه (روی شمارندهٔ ۳۲ بیتی میکروثانیه‌ای) یک بار خودش را نشان
می‌دهد.

## اندازه‌گیری بار CPU

```c
static PIDs_IsrMonitor mon;
PIDs_IsrMonitorInit(&mon, 1000);   /* دورهٔ اسمی ۱۰۰۰µs */

void TIM2_IRQHandler(void)
{
    PIDs_IsrEnter(&mon);
    /* ... کار کنترلی ... */
    PIDs_IsrExit(&mon);
}

/* در حلقهٔ اصلی: */
float load = PIDs_IsrLoadPercent(&mon);
```

## قرار دادن در CCMRAM

روی F3/F4 حافظهٔ CCM به بأس CPU وصل است و با DMA رقابت نمی‌کند:

```c
__attribute__((section(".ccmram"))) static PID_Handle pid;
```

⚠️ **CCMRAM با DMA قابل‌دسترسی نیست.** بافرهای DMA را آنجا نگذارید.

## نکات ADC

**۱. مقیاس‌بندی را یک بار انجام دهید.** اگر ADC شما ۱۲ بیتی است و ۰..۳.۳ ولت
را می‌خواند:

```c
float volts = (float)adc_raw * (3.3f / 4095.0f);
```

**۲. میانگین‌گیری سخت‌افزاری بهتر از نرم‌افزاری است.** اگر ADC شما oversampling
دارد، از آن استفاده کنید — رایگان است.

**۳. نویز کوانتیزاسیون واقعی است.** یک ADC ۱۲ بیتی روی بازهٔ ۰..۲۰۰ درجه،
رزولوشن ۰.۰۵ درجه دارد. اگر `Kd` بزرگ باشد، همین پله‌ها در مشتق تقویت
می‌شوند.

## نکات PWM

**خروجی کنترلر را مستقیم به `CCR` ندهید مگر مقیاسش درست باشد:**

```c
PID_SetOutputLimits(&pid, 0.0f, (float)LL_TIM_GetAutoReload(TIM3));
```

اینطوری واحد خروجی کنترلر همان واحد `CCR` است و anti-windup دقیقاً می‌داند
اشباع کجاست.

## FPU را روشن کنید

روی F4/F7/H7 اگر FPU فعال نباشد، هر عمل `float` نرم‌افزاری می‌شود:

```
-mfpu=fpv4-sp-d16 -mfloat-abi=hard
```

و در `SystemInit` باید `CPACR` ست شود (CubeMX خودش می‌کند).

**اگر FPU ندارید** (F0/G0/L0)، `PIDq_*` را استفاده کنید (§۱۶).

## `float` یا `double`

`PID_Float` پیش‌فرض `float` است. روی Cortex-M4F که FPU فقط تک‌دقتی است،
`double` یعنی کتابخانهٔ نرم‌افزاری و ۱۰ تا ۵۰ برابر کندتر.

`-Wdouble-promotion -Werror` در build gate ما دقیقاً برای همین است.

## چک‌لیست راه‌اندازی

- [ ] FPU فعال (`-mfloat-abi=hard`)
- [ ] `PID_SetOutputLimits` با واحد واقعی `CCR`
- [ ] تایمر با نرخ ثابت و اولویت وقفهٔ درست
- [ ] `PID_Update` داخل ISR، نه در حلقهٔ اصلی
- [ ] handle در CCMRAM (اختیاری)
- [ ] `PID_UpdateFast_IsSafe()` چک شده اگر از مسیر سریع استفاده می‌کنید
- [ ] watchdog سخت‌افزاری فعال — کتابخانه جایگزینش نیست
