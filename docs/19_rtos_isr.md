# ۱۹ — RTOS و ISR

## قواعد ایمنی نخ (Thread Safety)

**PIDX هیچ قفلی ندارد.** این عمدی است: یک mutex در ISR یا کار نمی‌کند یا
گران است. به‌جایش قرارداد صریح است:

| عملیات | ایمن است؟ |
|---|---|
| `PID_Update` روی handleهای **متفاوت** از نخ‌های متفاوت | ✅ بله |
| `PID_Update` روی **همان** handle از دو نخ | ❌ خیر |
| `PID_SetGains` همزمان با `PID_Update` روی همان handle | ❌ خیر |
| `PID_GetOutput` همزمان با `PID_Update` | ⚠️ مقدار کهنه می‌دهد، خراب نمی‌شود |
| `PID_Telemetry_Read` از نخ دیگر | ✅ بله (SPSC بدون قفل) |

## الگوی توصیه‌شده: کنترل در ISR، پیکربندی در تسک

```c
static PID_Handle pid;
static volatile bool gains_pending;
static volatile float new_kp, new_ki, new_kd;

void TIM2_IRQHandler(void)          /* اولویت بالا */
{
    if (gains_pending) {
        PID_SetGains(&pid, new_kp, new_ki, new_kd);
        gains_pending = false;
    }
    write_pwm(PID_Update(&pid, read_adc()));
}

void config_task(void *arg)         /* اولویت پایین */
{
    new_kp = 3.0f; new_ki = 0.7f; new_kd = 0.05f;
    gains_pending = true;           /* ISR در نمونهٔ بعد اعمال می‌کند */
}
```

**چرا اینطوری؟** چون `PID_SetGains` اتمیک نیست. اگر وسط اجرایش ISR بزند،
کنترلر با ترکیبی از بهره‌های قدیم و جدید کار می‌کند. با این الگو، تغییر
همیشه در مرز نمونه اتفاق می‌افتد.

و چون انتگرال‌گر در واحد خروجی است (§۰۳)، این تغییر **بدون ضربه** است.

## بخش بحرانی

اگر واقعاً لازم شد:

```c
uint32_t s = PIDs_EnterCritical();
PID_SetGains(&pid, kp, ki, kd);
PIDs_ExitCritical(s);
```

`PIDs_EnterCritical` وضعیت قبلی را برمی‌گرداند، پس تودرتو بودن مشکلی
ندارد — برخلاف `__disable_irq()` خام که وقتی از داخل یک بخش بحرانی دیگر
صدا زده شود، هنگام خروج وقفه‌ها را زودتر از موعد باز می‌کند.

## Telemetry از ISR

حلقهٔ تله‌متری **SPSC بدون قفل** است: ISR می‌نویسد، تسک می‌خواند.

```c
/* در ISR: خودکار، اگر Attach کرده باشید */
/* در تسک: */
PID_TelemetryRecord rec;
while (PID_Telemetry_Read(&tel, &rec) == PID_OK) {
    uart_send(&rec, sizeof rec);
}
```

**تصمیم طراحی:** وقتی حلقه پر است، **جدیدترین** رکورد دور ریخته می‌شود.
دلیلش این است که تولیدکننده (ISR) هرگز نباید `tail` را جلو ببرد — آن کار
به یک بخش بحرانی نیاز دارد و کل نکتهٔ «بدون قفل» را از بین می‌برد.

اگر گم‌شدن داده برایتان مهم است، حلقه را بزرگ‌تر کنید یا سریع‌تر بخوانید.
شمارندهٔ `dropped` می‌گوید چقدر از دست رفته.

## FreeRTOS: تسک به‌جای ISR

اگر نرخ کنترل پایین است (< ۱ کیلوهرتز)، یک تسک تمیزتر است:

```c
void control_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        write_pwm(PID_Update(&pid, read_adc()));
    }
}
```

`vTaskDelayUntil` (نه `vTaskDelay`) — چون نرخ ثابت می‌خواهیم نه فاصلهٔ ثابت
بین پایان یک اجرا و شروع بعدی.

**اگر jitter دارید**، `PID_UpdateDt` با `dt` واقعی بهتر است:

```c
uint32_t now = PIDs_NowUs32();
float dt = (float)PIDs_DeltaUs(prev, now) * 1e-6f;
prev = now;
write_pwm(PID_UpdateDt(&pid, read_adc(), dt));
```

ولی یادتان باشد ۳۸٪ گران‌تر است (§۱۷).

## Auto-tune در ISR

ماشین حالت auto-tune **غیرمسدودکننده** است و می‌تواند مستقیم در ISR اجرا
شود:

```c
void TIM2_IRQHandler(void)
{
    float y = read_adc();
    if (PID_AutoTune_IsRunning(&tuner)) {
        write_pwm(PID_AutoTune_Update(&tuner, y, DT));
    } else {
        write_pwm(PID_Update(&pid, y));
    }
}
```

⚠️ اگر `on_progress` یا `on_done` ست کرده‌اید، آن callbackها **از داخل ISR**
صدا زده می‌شوند. کوتاه و غیرمسدودکننده نگهشان دارید — `printf` در ISR یک
اشتباه کلاسیک است.

## اولویت‌ها

- حلقهٔ کنترل: بالاترین اولویت غیربحرانی.
- ارتباطات (UART/USB): پایین‌تر.
- **هرگز** حلقهٔ کنترل را پشت یک ISR طولانی نگذارید.

اگر `PIDs_IsrLoadPercent()` بالای ۵۰٪ می‌دهد، یا نرخ را کم کنید یا از
`PID_UpdateFast` استفاده کنید.
