# قرارداد سناریوی انطباق (Conformance Scenario Contract)

این فایل زبانِ مشترکی را تعریف می‌کند که **هر پنج پیاده‌سازی** (C مرجع، ESP32، Python،
MATLAB/Octave، C#) آن را می‌خوانند و اجرا می‌کنند. هدف این است که مقایسهٔ بین‌زبانی
یک *مقایسهٔ عددی* باشد، نه یک بازبینی کد.

هر پیاده‌سازی یک «runner» دارد که:

1. فایل سناریو (`ports/compare/scenarios.txt`) را می‌خواند،
2. دستورها را به‌ترتیب روی کنترلر اجرا می‌کند،
3. برای هر گام یک ردیف CSV با دقت کامل چاپ می‌کند.

سپس `ports/compare/compare.py` خروجی‌ها را ردیف‌به‌ردیف با مرجع C مقایسه می‌کند.

---

## ۱. چرا `double` و نه `float`

هستهٔ C به‌طور پیش‌فرض `PID_Float = float` است، ولی `PIDX_USE_DOUBLE=1` را هم
پشتیبانی می‌کند و بدون هیچ اخطاری زیر همان gate کامپایل می‌شود (تأیید شد).

Python، Octave و C# (`double`) همگی به‌صورت بومی IEEE-754 با دقت مضاعف کار می‌کنند.
اگر مرجع را در حالت `float` می‌گرفتیم، هر اختلافی که می‌دیدیم صرفاً «گِرد شدن به
`float`» بود و هیچ باگ واقعی‌ای را آشکار نمی‌کرد. با ساختن مرجع در حالت `double`،
هر اختلاف غیرصفر یک **اختلاف الگوریتمی واقعی** است.

> پیامد: پورت‌ها معادلِ *معناییِ* حالت `double` هسته‌اند. اعداد حالت `float`
> (که فِرم‌ور واقعی روی STM32/ESP32 اجرا می‌کند) با گِرد کردن همان جبر به‌دست می‌آید.

## ۲. قالب فایل سناریو

- خطوط خالی و خطوطی که با `#` شروع می‌شوند نادیده گرفته می‌شوند.
- توکن‌ها با فاصله جدا می‌شوند.
- اعداد با `%.17g` نوشته می‌شوند تا رفت‌وبرگشت متن↔دودویی بدون اتلاف باشد.
- `nan` و `inf` / `-inf` مقادیر معتبرند (برای آزمون مسیرهای خطا).

```
scenario <name>          شروع یک سناریوی جدید؛ handle و config ریست می‌شوند
  <config-cmd> ...       پیش از init
init                     PID_Init را صدا می‌زند؛ کد بازگشتی در ستون rc گزارش می‌شود
  <run-cmd> ...          پس از init
end                      پایان سناریو
```

### دستورهای پیکربندی (پیش از `init`)

| دستور | معادل C |
|---|---|
| `gains <kp> <ki> <kd>` | `cfg.core.{kp,ki,kd}` |
| `dt <s>` | `cfg.core.sample_time` |
| `direction <0\|1>` | `PID_DIRECT` / `PID_REVERSE` |
| `mode <0\|1\|2>` | `MANUAL` / `AUTOMATIC` / `HOLD` |
| `integration <0\|1>` | `BACKWARD_EULER` / `TRAPEZOIDAL` |
| `outlim <min> <max>` | `use_output_limits=true` + حدود |
| `intlim <min> <max>` | `use_integral_limits=true` + حدود |
| `dtlim <min> <max>` | `cfg.limits.dt_min/dt_max` |
| `aw <mode> <kt>` | `cfg.integral.{mode,kt}` |
| `separation <x>` | `cfg.integral.separation_threshold` |
| `deadband <x>` | `cfg.integral.deadband` |
| `ienable <0\|1>` | `cfg.integral.enabled` |
| `dmode <0\|1\|2>` | `ON_MEASUREMENT` / `ON_ERROR` / `ON_WEIGHTED_ERROR` |
| `tf <x>` | `cfg.filter.tf` |
| `nfilter <x>` | `cfg.filter.n_filter` |
| `inlpf <tau>` | `cfg.filter.input_lpf_tau` |
| `weights <beta> <gamma>` | `cfg.weight.{beta,gamma}` |
| `ff <en> <value> <gain>` | `cfg.feedforward.{enabled,value,gain}` |
| `shaper <rate> <acc> <dec> <slew>` | `cfg.shaper.*` |
| `safety <en> <min> <max> <rate> <fs> <n> <recover>` | `cfg.safety.*` |

### دستورهای اجرا (پس از `init`)

| دستور | معادل C | ردیف CSV تولید می‌کند؟ |
|---|---|---|
| `u <meas> <dt>` | `PID_UpdateDt` | بله |
| `un <meas>` | `PID_Update` (dt اسمی) | بله |
| `ufast <meas>` | `PID_UpdateFast` | بله |
| `uex <meas> <dt> <sp> <ff> <track> <schedvar>` | `PID_UpdateEx` (`nan` = بدون تغییر) | بله |
| `sp <x>` | `PID_SetSetpoint` | خیر |
| `spimm <x>` | `PID_SetSetpointImmediate` | خیر |
| `setmode <m>` | `PID_SetMode` | خیر |
| `manual <x>` | `PID_SetManualOutput` | خیر |
| `setgains <kp> <ki> <kd>` | `PID_SetGains` | خیر |
| `rescale <kp> <ki> <kd>` | `PID_SetGainsRescaleIntegral` | خیر |
| `setaw <mode> <kt>` | `PID_SetAntiWindup` | خیر |
| `setoutlim <min> <max>` | `PID_SetOutputLimits` | خیر |
| `clroutlim` | `PID_ClearOutputLimits` | خیر |
| `setintlim <min> <max>` | `PID_SetIntegralLimits` | خیر |
| `setint <x>` | `PID_SetIntegrator` | خیر |
| `track <x>` | `PID_SetTrackingInput` | خیر |
| `setdmode <m>` | `PID_SetDerivativeMode` | خیر |
| `settf <x>` | `PID_SetDerivativeFilter` | خیر |
| `setn <x>` | `PID_SetDerivativeFilterN` | خیر |
| `setdir <d>` | `PID_SetDirection` | خیر |
| `setweights <b> <g>` | `PID_SetWeights` | خیر |
| `setff <x>` | `PID_SetFeedforward` | خیر |
| `setramp <r> <a> <d>` | `PID_SetSetpointRamp` | خیر |
| `setslew <x>` | `PID_SetOutputSlewRate` | خیر |
| `setinlpf <tau>` | `PID_SetInputFilter` | خیر |
| `setsep <x>` | `PID_SetIntegralSeparation` | خیر |
| `setdb <x>` | `PID_SetIntegralDeadband` | خیر |
| `setienable <0\|1>` | `PID_EnableIntegral` | خیر |
| `setdtnom <x>` | `PID_SetSampleTime` | خیر |
| `reset` | `PID_Reset` | خیر |
| `clearfault` | `PID_ClearFault` | خیر |
| `schedpoints <n> <x kp ki kd>×n` | `PID_GainSched_Init` + `Attach` | خیر |
| `schedcfg <source> <interp> <hyst>` | منبع/درون‌یابی/هیسترزیس | خیر |
| `schedvar <x>` | `PID_GainSched_SetVar` | خیر |
| `rule <rule> <struct> <kind> <a> <b> <c> <lambda>` | `PID_TuneRule_Apply` | بله (ردیف `rule`) |

برای `rule`، `kind=1` یعنی FREQ با `(a,b) = (Ku,Pu)` و `kind=2` یعنی
FOPDT با `(a,b,c) = (K,T,L)`.

## ۳. قالب خروجی CSV

سرستون دقیقاً:

```
scenario,k,cmd,rc,output,setpoint,error,p,i,d,ff,unsat,flags,last_error
```

- `k` — شمارندهٔ ردیف در همان سناریو، از ۰.
- `cmd` — دستوری که ردیف را ساخت (`u`, `un`, `ufast`, `uex`, `init`, `rule`).
- `rc` — کد وضعیت عددی همان فراخوانی (برای `init` نتیجهٔ `PID_Init`).
- ستون‌های عددی با `%.17g`.
- `flags` — عدد صحیح ماسک `PID_FLAG_*`.
- `last_error` — کد چسبندهٔ `PID_PeekLastError`.

برای `ufast` ستون‌های تشخیصی از handle خوانده می‌شوند نه از snapshot، چون مسیر
سریع عمداً snapshot را پر نمی‌کند؛ مقادیر `p/d/unsat` در این حالت `nan` گزارش
می‌شوند تا مقایسه‌گر آن‌ها را نادیده بگیرد و فقط `output/i/flags` را بسنجد.

برای ردیف `rule` نگاشت ستون‌ها این است:
`output=kp`, `setpoint=ki`, `error=kd`, `p=ti`, `i=td`, `d=tf`.

## ۴. معیار پذیرش

مقایسه‌گر برای هر سلول عددی خطای نسبی می‌گیرد:

```
rel = |a - b| / max(1, |a|, |b|)
```

- `rel <= 1e-12` → **PASS** (اختلاف صرفاً ترتیب عملیات ممیز شناور)
- در غیر این صورت → **FAIL** و ردیف در گزارش می‌آید.

ستون‌های صحیح (`rc`, `flags`, `last_error`) باید **دقیقاً** برابر باشند.
