# PIDX Examples Roadmap — گام‌به‌گام با مثال‌های ملموس

## هدف
یادگیری Progressive PIDX از ساده به پیچیده، با مثال‌های قابل اجرا روی STM32.

## ساختار
هر مثال = یک فایل `main_stm32.c` با سه تابع:
- `pid_exampleXX_init()`  — یکبار موقع راه‌اندازی
- `pid_exampleXX_tick()`  — در TIM ISR با نرخ مشخص
- کامنت‌های کاربردی به فارسی

## گام‌ها

### [گام ۱] Minimal — ۵ خط API
**فایل:** `examples/01_minimal/main_stm32.c`
**نرخ:** ۱۰۰ هرتز
**مفاهیم:** InitDefault, SetGains, SetSetpoint, Update
**کاربرد:** ساده‌ترین کنترلر — دما، سرعت فن، فشار ساده
**وضعیت:** ✨ نوشته شد

### [گام ۲] Temperature/PWM — Anti-Windup
**فایل:** `examples/02_temperature_pwm/main_stm32.c`
**نرخ:** ۲ هرتز
**مفاهیم:** PID_Config, output limits, ۴ استراتژی AW, فیلتر ورودی
**کاربرد:** هیتر، کوره، سیستم HVAC — محرک یک‌طرفه
**وضعیت:** ✨ نوشته شد

### [گام ۳] Motor Speed — 2DOF & Derivative Kick
**نرخ:** ۱ کیلوهرتز
**مفاهیم:** Setpoint weighting β, derivative on measurement vs error
**کاربرد:** موتور DC با انکودر، نوار نقاله

### [گام ۴] Motor Position — Setpoint Shaping
**نرخ:** ۱ کیلوهرتز
**مفاهیم:** SetpointRamp, position loop, integral limits
**کاربرد:** سروو پوزیشن، ربات، valve positioner

### [گام ۵] Current Control — Fast Path
**نرخ:** ۲۰ کیلوهرتز
**مفاهیم:** PID_UpdateFast, analytic tuning
**کاربرد:** درایو موتور، منبع تغذیه، کنترل جریان

### [گام ۶] Cascade — 3-Level
**فایل:** `examples/06_cascade_pos_vel_cur/main_stm32.c`
**نرخ:** ۲۰ کیلوهرتز (درونی)
**مفاهیم:** PID_Cascade, decimation, inter-level clamp, AW back-propagation
**کاربرد:** سروو حرفه‌ای، CNC، ربات صنعتی
**وضعیت:** ✨ نوشته شد

### [گام ۷] Auto-Tune — Relay & Step
**فایل:** `examples/07_autotune_relay/main_stm32.c`
**نرخ:** ۱۰ هرتز
**مفاهیم:** PID_AutoTune, relay vs step, ۹ tuning rule, safety envelope
**کاربرد:** هر جا نمی‌دونید gain چقدر باید باشه
**وضعیت:** ✨ نوشته شد (با رفع مشکلات کد شما)

### [گام ۸] RTOS Task — FreeRTOS
**نرخ:** ۱ کیلوهرتز
**مفاهیم:** absolute deadline, vTaskDelayUntil, feedforward callback
**کاربرد:** پروژه‌های FreeRTOS, سیستم‌های چندوظیفه‌ای

### [گام ۹] TIM ISR — Hardware Timer
**نرخ:** متغیر
**مفاهیم:** lock-free telemetry, cycle counting, ISR safety
**کاربرد:** پیاده‌سازی واقعی روی STM32 تایمر

### [گام ۱۰] Full Featured — All Together
**نرخ:** ۱ کیلوهرتز
**مفاهیم:** همه چیز یکجا: cascade + autotune + diagnostics + telemetry
**کاربرد:** مرجع پیکربندی کامل

## مشکلات رایج Auto-Tune (از تجربه شما)

| مشکل | علت | راهکار |
|------|------|--------|
| plant_settle() فراموش شده | plant از y=0 شروع می‌شه | ۶۰۰ ثانیه settle کنید |
| نویز زیاد | SNR پایین برای شناسایی | SENSOR_NOISE ≤ 0.1 یا output_step را زیاد کنید |
| quality < 50 | L/T < 0.05 (بدون تاخیر) | از STEP test با پوشش کافی استفاده کنید |
| timeout | plant خیلی کُنده | timeout_s را زیاد کنید یا output_step را |
| hysteresis برای STEP test | پارامتر بی‌اثر | فقط برای relay استفاده می‌شه |