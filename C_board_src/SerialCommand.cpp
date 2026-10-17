#include "SerialCommand.h"
#include "ControlTask.h"
#include "ControlConfig.h"
#include "usart.h"
#include <cstdio>
#include <cstring>

volatile SerialCommandStatus g_serial_status = {};
namespace {
constexpr unsigned kRxSize = 128, kLineSize = 48, kTxSlots = 4, kTxSize = 192;
uint8_t rx_byte;
uint8_t rx_buffer[kRxSize];
volatile unsigned rx_head = 0, rx_tail = 0;
volatile bool rx_fault = false;
char tx_buffer[kTxSlots][kTxSize];
uint16_t tx_length[kTxSlots];
volatile unsigned tx_head = 0, tx_tail = 0;
volatile bool tx_busy = false;
char line[kLineSize];
unsigned line_length = 0;
bool discard_line = false;
uint32_t last_byte_ms = 0;
ControlOutputMode previous_mode = CONTROL_OUTPUT_STOPPED;
bool plot_enabled = false;
uint32_t last_plot_ms = 0;

class Guard {
public:
    Guard() : mask(__get_PRIMASK()) { __disable_irq(); }
    ~Guard() { __set_PRIMASK(mask); }
private:
    uint32_t mask;
};

// Foreground only. Buffers remain owned by the queue until the TX interrupt completes.
void reply(const char *text) {
    const unsigned head = tx_head;
    const unsigned next = (head + 1) % kTxSlots;
    if (next == tx_tail) {
        g_serial_status.tx_dropped = g_serial_status.tx_dropped + 1U;
        return;
    }
    const size_t length = std::strlen(text);
    if (length >= kTxSize) return;
    std::memcpy(tx_buffer[head], text, length);
    tx_length[head] = static_cast<uint16_t>(length);
    const Guard guard;
    tx_head = next;
}

void reject(const char *reason) {
    ControlStopOutput();
    g_serial_status.rejected = g_serial_status.rejected + 1U;
    reply(reason);
}

bool space(char c) { return c == ' ' || c == '\t'; }
void skipSpace(const char *&p) { while (space(*p)) ++p; }
bool number(const char *&p, int &value) {
    skipSpace(p);
    int sign = 1;
    if (*p == '-' || *p == '+') { if (*p == '-') sign = -1; ++p; }
    if (*p < '0' || *p > '9') return false;
    int result = 0;
    while (*p >= '0' && *p <= '9') {
        const int digit = *p++ - '0';
        if (result > (32767 - digit) / 10) return false;
        result = result * 10 + digit;
    }
    value = sign * result;
    return true;
}

void status() {
    uint32_t online, feedback, mode, fault, serial_fault;
    int voltage;
    {
        const Guard guard;
        online = g_control_status.feedback_count != 0 &&
            HAL_GetTick() - g_control_status.last_feedback_ms < CONTROL_FEEDBACK_TIMEOUT_MS;
        feedback = g_control_status.feedback_count;
        mode = g_control_status.output_mode;
        fault = g_control_status.output_fault;
        voltage = g_control_status.output_command;
        serial_fault = g_serial_status.link_fault;
    }
    char response[kTxSize];
    std::snprintf(response, sizeof(response),
        "STATUS online=%lu mode=%lu raw=%d fault=%lu serial_fault=%lu feedback=%lu tx_enabled=%u command_current=%u next=%lu\r\n",
        static_cast<unsigned long>(online), static_cast<unsigned long>(mode), voltage,
        static_cast<unsigned long>(fault), static_cast<unsigned long>(serial_fault),
        static_cast<unsigned long>(feedback), static_cast<unsigned>(CONTROL_CAN_TX_ENABLED),
        static_cast<unsigned>(CONTROL_COMMAND_CURRENT), static_cast<unsigned long>(g_assessment_status.next_mode));
    reply(response);
}

void execute() {
    line[line_length] = '\0';
    while (line_length && space(line[line_length - 1])) line[--line_length] = '\0';
    const char *p = line;
    skipSpace(p);
    if (*p == '\0') return;
    g_serial_status.commands = g_serial_status.commands + 1U;
    if (std::strcmp(p, "STOP") == 0) {
        ControlStopOutput();
        reply("OK STOP\r\n");
    } else if (std::strcmp(p, "STATUS") == 0) {
        status();
    } else if (std::strcmp(p, "CLEAR") == 0) {
        const bool cleared = SerialCommandClearFaults();
        if (cleared) reply("OK CLEAR\r\n");
        else reply("ERR CLEAR: stop first, require fresh feedback and idle CAN mailbox\r\n");
    } else if (std::strcmp(p, "MODE SPEED") == 0 || std::strcmp(p, "MODE POSITION") == 0) {
        const uint32_t mode = std::strcmp(p, "MODE SPEED") == 0 ? CONTROL_OUTPUT_SPEED_SINE : CONTROL_OUTPUT_POSITION_SINE;
        if (ControlSelectAssessment(mode)) reply("OK MODE\r\n");
        else reply("ERR MODE: stop first\r\n");
    } else if (std::strcmp(p, "RUN") == 0) {
        bool started;
        {
            const Guard guard;
            started = !rx_fault && !g_serial_status.link_fault && ControlStartNextAssessment();
        }
        if (started) reply("OK RUN\r\n");
        else { g_serial_status.rejected = g_serial_status.rejected + 1U; reply("ERR RUN: require online, stopped, no fault\r\n"); }
    } else if (std::strcmp(p, "PLOT 1") == 0 || std::strcmp(p, "PLOT 0") == 0) {
        plot_enabled = p[5] == '1';
        reply("OK PLOT\r\n");
    } else if (std::strncmp(p, "START", 5) == 0 && space(p[5])) {
        p += 5;
        int raw = 0, duration = 0;
        if (!number(p, raw) || !space(*p) || !number(p, duration)) {
            reject("ERR FORMAT: START raw milliseconds\r\n");
            return;
        }
        skipSpace(p);
        if (*p != '\0' || raw == 0 || raw < -CONTROL_TEST_OUTPUT_LIMIT ||
            raw > CONTROL_TEST_OUTPUT_LIMIT || duration < 1 ||
            static_cast<uint32_t>(duration) > CONTROL_TEST_MAX_DURATION_MS) {
            reject("ERR RANGE: nonzero raw -1000..1000, milliseconds 1..1000\r\n");
            return;
        }
        bool started;
        {
            const Guard guard;
            started = !rx_fault && !g_serial_status.link_fault &&
                ControlStartOutputTest(static_cast<int16_t>(raw), static_cast<uint32_t>(duration));
        }
        if (started) reply("OK START\r\n");
        else {
            g_serial_status.rejected = g_serial_status.rejected + 1U;
            reply("ERR START: check STATUS; requires online, stopped, no fault\r\n");
        }
    } else {
        reject("ERR COMMAND: STATUS | MODE SPEED/POSITION | RUN | PLOT 0/1 | START raw ms | STOP | CLEAR\r\n");
    }
}

void input(uint8_t byte) {
    if (byte == '\r' || byte == '\n') {
        if (!discard_line) execute();
        line_length = 0;
        discard_line = false;
        return;
    }
    if (discard_line) return;
    if ((byte < 32 && byte != '\t') || byte > 126 || line_length >= kLineSize - 1) {
        discard_line = true;
        line_length = 0;
        reject("ERR LINE: invalid or too long; stopped\r\n");
        return;
    }
    line[line_length++] = static_cast<char>(byte);
    last_byte_ms = HAL_GetTick();
}
}

void SerialCommandInit(void) {
    if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK) rx_fault = true;
    reply("READY: MODE SPEED/POSITION | RUN | PLOT 0/1 | STATUS | STOP | CLEAR\r\n");
}

uint32_t SerialCommandClearFaults(void) {
    const Guard guard;
    if (rx_fault || !ControlClearOutputFault()) return 0;
    g_serial_status.link_fault = 0;
    return 1;
}

void SerialCommandPoll(void) {
    // Bounded work: CAN and watchdog servicing are never held up by UART traffic.
    for (unsigned n = 0; n < kRxSize; ++n) {
        bool damaged = false, available = false;
        uint8_t byte = 0;
        {
            const Guard guard;
            if (rx_fault) {
                rx_fault = false;
                rx_tail = rx_head; // Do not execute a truncated/reassembled START.
                g_serial_status.link_fault = 1;
                HAL_UART_AbortReceive(&huart6); // IRQ mode only; no DMA transfer in use.
                if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK) rx_fault = true;
                damaged = true;
            } else if (rx_tail != rx_head) {
                byte = rx_buffer[rx_tail];
                rx_tail = (rx_tail + 1) % kRxSize;
                available = true;
            }
        }
        if (damaged) {
            line_length = 0;
            discard_line = true; // A fresh newline re-establishes framing.
            reject("ERR SERIAL: stopped; send newline then CLEAR\r\n");
            break;
        }
        if (!available) break;
        input(byte);
    }
    if (line_length && HAL_GetTick() - last_byte_ms >= 500U) {
        line_length = 0;
        discard_line = true;
        reject("ERR TIMEOUT: incomplete command; stopped\r\n");
    }
    const auto mode = g_control_status.output_mode;
    if (previous_mode != CONTROL_OUTPUT_STOPPED && mode == CONTROL_OUTPUT_STOPPED) {
        reply("EVENT STOP\r\n");
    }
    previous_mode = mode;
    if (plot_enabled && g_assessment_status.running && HAL_GetTick() - last_plot_ms >= 20U) {
        // Leave room for command replies. Snapshot under guard, formatting outside IRQ masking.
        last_plot_ms = HAL_GetTick();
        float target, measured, speed_ref, speed, angle_ref, angle;
        int raw;
        {
            const Guard snapshot;
            const bool position = g_assessment_status.mode == CONTROL_OUTPUT_POSITION_SINE;
            speed_ref = g_assessment_status.target_speed_rad_s;
            speed = g_assessment_status.actual_speed_rad_s;
            angle_ref = g_assessment_status.target_angle_rad;
            angle = g_assessment_status.actual_angle_rad;
            target = position ? angle_ref : speed_ref;
            measured = position ? angle : speed;
            raw = g_control_status.output_command;
        }
        if ((tx_head + 2U) % kTxSlots != tx_tail && (tx_head + 1U) % kTxSlots != tx_tail) {
            char plot[kTxSize];
            std::snprintf(plot, sizeof(plot), "plot:%.5f,%.5f,%.5f,%d,%.5f,%.5f\n",
                          static_cast<double>(target), static_cast<double>(measured),
                          static_cast<double>(target - measured), raw,
                          static_cast<double>(speed_ref), static_cast<double>(speed));
            reply(plot);
        }
    }
    const Guard guard;
    if (!tx_busy && tx_tail != tx_head) {
        tx_busy = true;
        if (HAL_UART_Transmit_IT(&huart6, reinterpret_cast<uint8_t *>(tx_buffer[tx_tail]),
                                 tx_length[tx_tail]) != HAL_OK) {
            tx_busy = false;
        }
    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart != &huart6) return;
    g_serial_status.rx_bytes = g_serial_status.rx_bytes + 1U;
    const unsigned next = (rx_head + 1) % kRxSize;
    if (next == rx_tail) {
        rx_fault = true;
        g_serial_status.rx_errors = g_serial_status.rx_errors + 1U;
    } else {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK) rx_fault = true;
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart != &huart6 || !tx_busy) return;
    tx_tail = (tx_tail + 1) % kTxSlots;
    tx_busy = false;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart != &huart6) return;
    g_serial_status.rx_errors = g_serial_status.rx_errors + 1U;
    rx_fault = true;
}
