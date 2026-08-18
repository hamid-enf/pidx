/**
 * @file    pid_esp32_conf.h
 * @brief   Host-test configuration for the ESP32 platform layer.
 *
 * The real template is platform/esp32/pid_esp32_conf_template.h; this copy
 * selects the callback/esp_timer sources against the stub in esp32_stub.h and
 * switches FreeRTOS off, because no FreeRTOS exists on the host.
 */
#ifndef PID_ESP32_CONF_H
#define PID_ESP32_CONF_H

#include <stdint.h>

#define PIDX_ESP32_TIMEBASE      PIDX_ESP32_TB_ESP_TIMER
#define PIDX_ESP32_USE_FREERTOS  0
#define PIDX_ESP32_HOST_STUB     1

#include "esp32_stub.h"

#endif /* PID_ESP32_CONF_H */
