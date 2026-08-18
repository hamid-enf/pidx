# PIDX — Unit Tests

۱۶ suite، **۷۵۲ assertion**، همه سبز — و نتیجه در `-O0` و `-Os` و `-O2` یکسان.

## اجرا

```sh
cd tests
make              # ساخت و اجرای همه، با خلاصه در انتها
make build        # فقط ساخت
make test_filter  # ساخت و اجرای یک suite
make gate         # کامپایل همه در -O0 / -Os / -O2 (شکار باگ‌های وابسته به بهینه‌سازی)
make OPT=-O0      # اجرای همه در یک سطح بهینه‌سازی دلخواه
make clean
```

خروجی باینری‌ها در `tests/bin/` است. کد خروج غیرصفر یعنی حداقل یک suite شکست خورده،
پس `make` مستقیماً در CI قابل استفاده است.

## پرچم‌های کامپایل

همان دیوارِ `-Werror` که خود کتابخانه باید از آن رد شود:

```
-std=c99 -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow -Wcast-qual -pedantic -Werror
```

یعنی یک warning در فایل تست، به‌اندازهٔ warning در `src/` کشنده است.

**تنها استثنا** `test_autotune_accuracy` است: آن suite به `M_PI` نیاز دارد (که `-std=c99 -pedantic`
پنهان می‌کند) و کل مدل مرجعش با `double` نوشته شده، پس `-Wdouble-promotion` و `-Wconversion`
روی هر خط از ریاضیاتِ مرجع فعال می‌شدند، نه روی چیز واقعی. `-Wall -Wextra -Werror` همچنان برقرار است.
سورس‌های کتابخانه‌ای که آن suite لینک می‌کند با همان قاعده کامپایل می‌شوند، پس این تخفیف
هرگز به `src/` نمی‌رسد.

> تلهٔ ثبت‌شده: `-D_POSIX_C_SOURCE` به‌تنهایی glibc را در حالت سخت‌گیر POSIX می‌گذارد و
> `M_PI` را (نامی از X/Open) پنهان می‌کند. suiteای که POSIX نمی‌خواهد نباید آن را تعریف کند.

## فهرست suiteها

| فایل | assertion | چه چیزی را تضمین می‌کند |
|---|---|---|
| `smoke_core.c` | ✓ | دود اولیه؛ سریع‌ترین سیگنال خرابی |
| `test_antiwindup.c` | ۳۵ | چهار استراتژی anti-windup، با عدد نه با حس |
| `test_bumpless.c` | ۴۵ | سوئیچ MANUAL/AUTO، تغییر gain، بازیابی از خطا |
| `test_fastpath.c` | ۴۹ | `PID_UpdateFast` وقتی `IsSafe` است **بیت‌به‌بیت** با `PID_Update` یکی است |
| `test_autotune_safety.c` | ۱۹ | خودتنظیم هرگز پلنت را به جای خطرناک نمی‌برد |
| `test_autotune_accuracy.c` | ✓ | $K_u,P_u$ و $(K,T,L)$ در برابر حل تحلیلی، نه در برابر خودش |
| `test_diag.c` | ۳۱ | حلقهٔ تله‌متری lock-free، شمارش دقیق drop |
| `test_safety.c` | ۲۲ | محدودهٔ سنسور، نرخ، failsafe، بازیابی |
| `test_fixed.c` | ۴۲ | Q15/Q31 در برابر مرجع float |
| `test_filter.c` | ۸۲ | LPF1 در برابر فرم بسته، میانگین متحرک، median-3، rate limiter، deadband |
| `test_shaper.c` | ۶۶ | پروفایل ذوزنقه‌ای/مثلثی، حفظ مسافت، retarget بدون پرش سرعت |
| `test_gainsched.c` | ۶۴ | درون‌یابی، اشباع در دو انتها، hysteresis، پیوستگی C1، پنج منبع |
| `test_cascade.c` | ۸۹ | decimation چندنرخی، clamp بین سطوح، anti-windup برگشتی و جهت‌مندی‌اش |
| `test_core_contract.c` | ۹۳ | قرارداد API مستقل از هر feature (زیر را ببینید) |
| `test_posix.c` | ۳۱ | لایهٔ POSIX |
| `test_stm32_host.c` | ۸۴ | لایهٔ STM32 روی stub شبه-CMSIS |

## چرا `test_core_contract` جدا است

بقیهٔ suiteها هرکدام یک **قابلیت** را می‌آزمایند. این یکی **وعده‌ها**ی API را می‌آزماید:
هر نقطهٔ ورودی باید از NULL و از هندلِ Init-نشده جان سالم ببرد؛ یک setterِ ردشده باید
**هیچ چیز** را تغییر ندهد (اعمال جزئی از رد کردن بدتر است)؛ `PID_Update` باید تابعی
جبرگرا از (حالت، ورودی) باشد.

ارزشش در همان اولین اجرا ثابت شد: یک **segfault واقعی** در `PID_UpdateFast(NULL)`.
جزئیات آن و سه نقص دیگری که این فاز پیدا کرد در `docs/00_architecture.md` §۹.۹.۲ (PHASE 17) است.

## سبک خانه

`main()` ساده، بدون فریم‌ورک، بدون وابستگی. هر suite:

```c
static int pass = 0, bad = 0;
#define CK(c,m) do{ if(c){pass++;} else {bad++;printf("  FAIL: %s\n",(m));} }while(0)
```

و در انتها `return (bad == 0) ? 0 : 1;`.

هر فایل با کامنتی شروع می‌شود که می‌گوید **چرا** آن suite وجود دارد و کدام حالت خرابی را
شکار می‌کند — نه اینکه چه می‌کند (کد خودش این را می‌گوید).

قاعدهٔ سخت پروژه: وقتی یک assertion شکست می‌خورد، **اول ثابت کن کدام طرف اشتباه است**.
در این فاز ۱۵ شکست رخ داد؛ ۱۱ تا باگ تست بودند و ۴ تا باگ واقعی کتابخانه. هیچ تلورانسی
برای سبز کردن تست شل نشد.
