/**
 * @file    pid_esp32_conf_template.h
 * @brief   Template for the ESP32 platform-layer configuration.
 *
 * HOW TO USE
 *   Copy this file into your project as "pid_esp32_conf.h", edit it, and make
 *   sure its directory is on the include path ahead of platform/esp32. Do not
 *   edit this template in place: it is the reference copy, and a library
 *   update will overwrite it.
 *
 * With ESP-IDF, adding the layer to a component is:
 *
 *     idf_component_register(
 *         SRCS "src/pid.c" "src/pid_filter.c" ...
 *              "platform/esp32/pid_esp32.c"
 *         INCLUDE_DIRS "include" "platform/esp32" "config")
 */
#ifndef PID_ESP32_CONF_H
#define PID_ESP32_CONF_H

/* ======================================================================== */
/* 1. Timebase source - REQUIRED                                             */
/* ======================================================================== */

/**
 * One of:
 *   PIDX_ESP32_TB_ESP_TIMER  esp_timer_get_time(): 64-bit microseconds,
 *                            monotonic, core-agnostic, survives light sleep.
 *                            The right answer unless you have a reason.
 *   PIDX_ESP32_TB_CCOUNT     Xtensa cycle counter: finest resolution and the
 *                            cheapest read, but 32-bit (wraps every ~17.9 s
 *                            at 240 MHz), PER-CORE, and frozen during sleep.
 *   PIDX_ESP32_TB_CALLBACK   A microsecond counter you already maintain;
 *                            register it with PIDe_TimebaseInitCallback().
 */
#define PIDX_ESP32_TIMEBASE      PIDX_ESP32_TB_ESP_TIMER

/* ======================================================================== */
/* 2. FreeRTOS helpers                                                       */
/* ======================================================================== */

/**
 * 1 to build PIDe_TaskCreate() and PIDe_TaskDelayPeriod(), 0 to leave FreeRTOS
 * out entirely (bare-metal, or a unit-test host build).
 */
#ifndef PIDX_ESP32_USE_FREERTOS
#define PIDX_ESP32_USE_FREERTOS  1
#endif

/* ======================================================================== */
/* 3. Headers and primitives                                                 */
/* ======================================================================== */

/*
 * These four hooks are what the host test replaces with a stub. Keeping them
 * as macros here - rather than #including esp_timer.h from pid_esp32.c
 * directly - is what lets the layer be compiled and logic-tested on a PC with
 * no Xtensa toolchain present.
 */

#include <stdint.h>

#if defined(PIDX_ESP32_HOST_STUB)
/* Host build: the stub supplies these. See tests/test_esp32_host.c. */
#include "esp32_stub.h"
#else

#include "esp_timer.h"
#include "esp_cpu.h"

#if PIDX_ESP32_USE_FREERTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

/** 64-bit monotonic microseconds since boot. */
#define pide_esp_timer_us()   esp_timer_get_time()

/**
 * Xtensa CCOUNT. esp_cpu_get_cycle_count() is the portable ESP-IDF spelling
 * and covers the RISC-V parts (C3/C6/H2) as well, where the register is
 * mcycle rather than CCOUNT.
 */
#define pide_ccount()         ((uint32_t)esp_cpu_get_cycle_count())

/**
 * Critical section guarding the 64-bit wrap accumulator.
 *
 * A portMUX spinlock, not just an interrupt disable: on the dual-core parts
 * the accumulator can be touched from both cores, and disabling interrupts on
 * one core does not exclude the other. The section is a handful of
 * instructions.
 */
#define PIDE_ENTER_CRITICAL()  portENTER_CRITICAL(&pide_spinlock)
#define PIDE_EXIT_CRITICAL()   portEXIT_CRITICAL(&pide_spinlock)

static portMUX_TYPE pide_spinlock = portMUX_INITIALIZER_UNLOCKED;

#endif /* PIDX_ESP32_HOST_STUB */

/* ======================================================================== */
/* 4. Suggested core assignment                                              */
/* ======================================================================== */

/**
 * Core to pin the control task to. On ESP32/S3 the Wi-Fi and Bluetooth stacks
 * default to core 0, so putting a hard-real-time loop on core 1 keeps radio
 * activity out of your deadline jitter. Single-core parts (S2, C3) must use 0.
 */
#ifndef PIDX_ESP32_CONTROL_CORE
#define PIDX_ESP32_CONTROL_CORE  1
#endif

#endif /* PID_ESP32_CONF_H */
