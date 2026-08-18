/**
 * @file    esp32_stub.h
 * @brief   Minimal ESP-IDF stand-in so platform/esp32 can be compiled and
 *          logic-tested on the host.
 *
 * This is NOT an emulator. It provides just enough of esp_timer, the cycle
 * counter and the critical-section macros for the layer's own arithmetic - the
 * wrap extension, the rate driver's absolute-deadline scheduling, the load
 * monitor's duty computation - to be exercised with a counter the test drives
 * by hand.
 *
 * What this can prove: that the maths is right, that a 32-bit counter wrap is
 * handled, that an overrun re-bases instead of bursting.
 * What it cannot prove: that esp_timer_get_time() behaves as documented, or
 * that a portMUX actually excludes the other core. Those need silicon, which
 * is why pid_esp32.h labels the layer reviewed-but-unflashed.
 */
#ifndef PIDX_ESP32_STUB_H
#define PIDX_ESP32_STUB_H

#include <stdint.h>

/* The simulated clock the test drives. Microseconds for the esp_timer source,
 * CPU cycles for the CCOUNT source. */
extern int64_t  pide_stub_us;
extern uint32_t pide_stub_cycles;

#define pide_esp_timer_us()   (pide_stub_us)
#define pide_ccount()         (pide_stub_cycles)

/* No concurrency on the host, so the critical section is a no-op. The macros
 * still have to exist and still have to be balanced, which is itself worth
 * compiling. */
#define PIDE_ENTER_CRITICAL()  ((void)0)
#define PIDE_EXIT_CRITICAL()   ((void)0)

#endif /* PIDX_ESP32_STUB_H */
