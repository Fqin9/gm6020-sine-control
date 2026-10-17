#include "KeyControl.h"
#include "ControlTask.h"
#include "ControlConfig.h"
#include "SerialCommand.h"
#include "gpio.h"

volatile KeyControlStatus g_key_status = {};
namespace {
bool candidate = false, stable = false, ready = false;
bool suppress_release = false, long_handled = false;
uint32_t changed_ms = 0, pressed_ms = 0;
constexpr uint32_t kDebounceMs = 30, kShortPressMs = 500, kClearMs = 1000;
}

void KeyControlInit(void) {
    // C board manual: KEY is PA0, active low. Override Cube's unused analog pin.
    GPIO_InitTypeDef pin = {};
    pin.Pin = GPIO_PIN_0;
    pin.Mode = GPIO_MODE_INPUT;
    pin.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &pin);
    candidate = stable = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET;
    ready = false; // Require a stable release after boot; held KEY never starts.
    changed_ms = HAL_GetTick();
}

void KeyControlPoll(void) {
    const uint32_t now = HAL_GetTick();
    const bool pressed = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET;
    if (pressed != candidate) { candidate = pressed; changed_ms = now; }
    if (!ready) {
        if (!candidate && now - changed_ms >= kDebounceMs) { ready = true; stable = false; }
        return;
    }
    if (stable != candidate && now - changed_ms >= kDebounceMs) {
        stable = candidate;
        g_key_status.pressed = stable;
        if (stable) {
            pressed_ms = now;
            long_handled = false;
            suppress_release = g_control_status.output_mode != CONTROL_OUTPUT_STOPPED;
            if (suppress_release) {
                ControlStopOutput();
                g_key_status.stop_count = g_key_status.stop_count + 1U;
                g_key_status.last_result = 2;
            }
        } else if (!suppress_release && !long_handled && now - pressed_ms < kShortPressMs) {
            const bool started = !g_serial_status.link_fault &&
                (CONTROL_KEY_ASSESSMENT ? ControlStartNextAssessment() : (CONTROL_KEY_SPEED_TEST
                    ? ControlStartSpeedTest(CONTROL_KEY_SPEED_TARGET_RAD_S, CONTROL_KEY_SPEED_DURATION_MS)
                    : ControlStartOutputTest(CONTROL_KEY_TEST_RAW, CONTROL_KEY_TEST_MS)));
            if (started) {
                g_key_status.start_count = g_key_status.start_count + 1U;
                g_key_status.last_result = 1;
            } else {
                g_key_status.rejected = g_key_status.rejected + 1U;
                g_key_status.last_result = 4;
            }
        }
    }
    if (stable && !long_handled && now - pressed_ms >= kClearMs) {
        long_handled = true;
        suppress_release = true;
        ControlStopOutput();
        g_key_status.stop_count = g_key_status.stop_count + 1U;
        g_key_status.last_result = SerialCommandClearFaults() ? 3U : 4U;
    }
}
