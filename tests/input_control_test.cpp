#include "SerialCommand.h"
#include "KeyControl.h"
#include "ControlTask.h"
#include "ControlConfig.h"
#include "usart.h"
#include "gpio.h"
#include <cassert>
#include <string>
#include <cstdio>

UART_HandleTypeDef huart6;
GPIO_TypeDef gpio_a;
volatile ControlStatus g_control_status = {};
volatile AssessmentStatus g_assessment_status = {};
namespace {
uint32_t now_ms = 0, mask = 0, starts = 0, stops = 0;
int last_raw = 0;
float last_target = 0;
uint32_t last_duration = 0;
bool key_down = false, tx_pending = false;
uint8_t *rx_target = nullptr, *tx_source = nullptr;
uint16_t tx_size = 0;
std::string response;
void finishTx() {
    if (tx_pending) {
        response.append(reinterpret_cast<char *>(tx_source), tx_size);
        tx_pending = false;
        HAL_UART_TxCpltCallback(&huart6);
    }
}
void pump() { for (int i = 0; i < 8; ++i) { SerialCommandPoll(); finishTx(); } }
void rawInput(const std::string &text) {
    for (unsigned char byte : text) {
        assert(rx_target);
        *rx_target = byte;
        HAL_UART_RxCpltCallback(&huart6);
    }
}
void command(const std::string &text) { response.clear(); rawInput(text); pump(); }
void keyAt(uint32_t time, bool down) {
    now_ms = time;
    key_down = down;
    KeyControlPoll();
}
}
uint32_t HAL_GetTick() { return now_ms; }
uint32_t __get_PRIMASK() { return mask; }
void __disable_irq() { mask = 1; }
void __set_PRIMASK(uint32_t value) { mask = value; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n) {
    assert(h == &huart6 && n == 1); rx_target = p; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n) {
    assert(h == &huart6 && !tx_pending); tx_source = p; tx_size = n; tx_pending = true; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) {
    assert(h == &huart6); rx_target = nullptr; return HAL_OK;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *pin) {
    assert(port == GPIOA && pin->Pin == GPIO_PIN_0 && pin->Mode == GPIO_MODE_INPUT && pin->Pull == GPIO_PULLUP);
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    assert(port == GPIOA && pin == GPIO_PIN_0); return key_down ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
uint32_t ControlStartOutputTest(int16_t raw, uint32_t duration) {
    if (!g_control_status.initialized || !g_control_status.feedback_count || g_control_status.output_fault ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED) return 0;
    ++starts; last_raw = raw; last_duration = duration;
    g_control_status.output_mode = CONTROL_OUTPUT_TEST; return 1;
}
void ControlStopOutput() { ++stops; g_control_status.output_mode = CONTROL_OUTPUT_STOPPED; }
uint32_t ControlStartSpeedTest(float target, uint32_t duration) {
    if (!g_control_status.initialized || !g_control_status.feedback_count || g_control_status.output_fault ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED) return 0;
    ++starts; last_target = target; last_duration = duration;
    g_control_status.output_mode = CONTROL_OUTPUT_SPEED_TEST; return 1;
}
uint32_t ControlClearOutputFault() {
    if (!g_control_status.feedback_count || g_control_status.output_mode != CONTROL_OUTPUT_STOPPED) return 0;
    g_control_status.output_fault = 0; return 1;
}
uint32_t ControlSelectAssessment(uint32_t mode) {
    if (g_control_status.output_mode != CONTROL_OUTPUT_STOPPED) return 0;
    g_assessment_status.next_mode = mode; return 1;
}
uint32_t ControlStartNextAssessment() {
    if (!g_control_status.initialized || !g_control_status.feedback_count || g_control_status.output_fault ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED) return 0;
    ++starts;
    g_control_status.output_mode = static_cast<ControlOutputMode>(g_assessment_status.next_mode);
    return 1;
}

int main() {
    SerialCommandInit(); pump();
    assert(response.find("READY") != std::string::npos && starts == 0);
    command("START 100 200\n"); assert(starts == 0 && response.find("ERR START") != std::string::npos);
    g_control_status.initialized = 1; g_control_status.feedback_count = 1;
    command("START 100 "); assert(starts == 0); // fragmented command waits for newline
    command("200\r\n"); assert(starts == 1 && last_raw == 100 && last_duration == 200);
    command("START 200 500\n"); assert(starts == 1 && last_duration == 200); // no extension
    command("STOP\n"); assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
    assert(response.find("OK STOP") != std::string::npos);
    command("START -100 200\n"); assert(starts == 2 && last_raw == -100);
    command("STOP\r");
    const auto accepted = starts;
    for (const char *bad : {"START\n", "START 100\n", "START 100 200junk\n", "START 0 200\n",
                           "START 1001 200\n", "START -1001 200\n", "START 100 1001\n", "START 100 -1\n",
                           "START 100 0\n", "START 9999999999999999999999 200\n", "START 1+200\n"}) {
        command(bad); assert(starts == accepted && response.find("ERR") != std::string::npos);
    }
    command(std::string(60, 'X') + "START 100 200\n"); assert(starts == accepted);
    command(std::string("START 100") + char(0) + " 200\n"); assert(starts == accepted);
    command("START 100 "); now_ms += 501; pump(); command("200\n"); assert(starts == accepted);
    command("START 100 200\n"); assert(starts == accepted + 1);
    // Corrupt/overflowed RX stops motion and requires explicit CLEAR after a new delimiter.
    rawInput(std::string(160, 'A')); pump();
    assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED && g_serial_status.link_fault == 1);
    command("\nSTART 100 200\n"); assert(starts == accepted + 1);
    command("CLEAR\n"); assert(g_serial_status.link_fault == 0);
    command("START 100 200\n"); assert(starts == accepted + 2);
    HAL_UART_ErrorCallback(&huart6); pump();
    assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED && g_serial_status.link_fault == 1);
    command("\nCLEAR\n"); assert(g_serial_status.link_fault == 0);
    command("STATUS\n"); assert(response.find("mode=0") != std::string::npos);
    command("MODE POSITION\n"); assert(g_assessment_status.next_mode == CONTROL_OUTPUT_POSITION_SINE);
    command("RUN\n"); assert(g_control_status.output_mode == CONTROL_OUTPUT_POSITION_SINE);
    command("MODE SPEED\n"); assert(response.find("ERR MODE") != std::string::npos);
    assert(g_assessment_status.next_mode == CONTROL_OUTPUT_POSITION_SINE);
    command("STOP\nMODE SPEED\n"); assert(g_assessment_status.next_mode == CONTROL_OUTPUT_SPEED_SINE);
    command("PLOT 1\n"); assert(response.find("OK PLOT") != std::string::npos);
    g_assessment_status.running = 1;
    g_assessment_status.mode = CONTROL_OUTPUT_SPEED_SINE;
    g_assessment_status.target_speed_rad_s = 1.25f;
    g_assessment_status.actual_speed_rad_s = 1.0f;
    now_ms += 20; response.clear(); pump();
    assert(response.find("plot:1.25000,1.00000,0.25000,") != std::string::npos);
    g_assessment_status.running = 0;
    command("PLOT 0\n");
    // Saturated response queue cannot delay STOP.
    command("STATUS\nSTATUS\nSTATUS\nSTATUS\nSTOP\n");
    assert(g_serial_status.tx_dropped > 0 && g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);

    now_ms = 10000; key_down = true; KeyControlInit();
    const auto before_key = starts;
    keyAt(12000, true); assert(starts == before_key); // boot held
    keyAt(12001, false); keyAt(12032, false);
    keyAt(12100, true); keyAt(12110, false); keyAt(12141, false);
    assert(starts == before_key); // contact bounce
    keyAt(12200, true); keyAt(12231, true);
    keyAt(12300, false); keyAt(12331, false);
    assert(starts == before_key + 1);
    if (CONTROL_KEY_ASSESSMENT) {
        assert(g_control_status.output_mode == g_assessment_status.next_mode);
    } else if (CONTROL_KEY_SPEED_TEST) {
        assert(last_target == CONTROL_KEY_SPEED_TARGET_RAD_S && last_duration == CONTROL_KEY_SPEED_DURATION_MS);
        assert(g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST);
    } else {
        assert(last_raw == CONTROL_KEY_TEST_RAW && last_duration == CONTROL_KEY_TEST_MS);
    }
    assert(g_key_status.start_count == 1);
    keyAt(12400, true); keyAt(12431, true);
    assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
    keyAt(12500, false); keyAt(12531, false);
    assert(starts == before_key + 1); // stop press release must not start again
    g_control_status.output_fault = CONTROL_FAULT_CAN_TX;
    keyAt(12600, true); keyAt(12631, true); keyAt(13632, true);
    assert(g_control_status.output_fault == 0 && g_key_status.last_result == 3);
    keyAt(13700, false); keyAt(13731, false);
    assert(starts == before_key + 1); // long clear doesn't start
    // A held key never retriggers, even if a pulse has finished.
    keyAt(14000, true); keyAt(14031, true); keyAt(16000, true);
    assert(starts == before_key + 1);
    std::puts("PASS: UART framing/ranges, duplicate start, STOP, overflow/error recovery, response saturation, KEY debounce/boot hold/stop/clear");
}
