#include "ControlTask.h"
#include "ControlConfig.h"
#include "Gm6020.h"
#include "Pid.h"
#include "AssessmentController.h"
#include <cmath>
#include "can.h"
#include "iwdg.h"
#include "tim.h"

volatile ControlStatus g_control_status = {};
volatile CanDiagnostics g_can_diagnostics = {};
volatile SpeedTestStatus g_speed_test = {};
volatile SpeedTracePoint g_speed_trace[CONTROL_SPEED_TRACE_CAPACITY] = {};
volatile uint32_t g_speed_trace_count = 0;
volatile AssessmentStatus g_assessment_status = {};
volatile AssessmentLog g_assessment_log = {};

namespace {
static_assert(CONTROL_ASSESSMENT_CURRENT_LIMIT > 0 && CONTROL_ASSESSMENT_CURRENT_LIMIT <= 16384,
              "Assessment current outside protocol range");
static_assert(CONTROL_ASSESSMENT_PERIOD_MS * CONTROL_ASSESSMENT_CYCLES <= 20000U,
              "Assessment exceeds bounded recording duration");
static_assert(CONTROL_CAN_CHANNEL == 1 || CONTROL_CAN_CHANNEL == 2,
              "CAN channel must be 1 or 2");
static_assert(CONTROL_MOTOR_ID >= 1 && CONTROL_MOTOR_ID <= 7,
              "Motor ID must be between 1 and 7");
static_assert(CONTROL_SPEED_PERIOD_MS > 0, "Sine period must be positive");
static_assert(CONTROL_SPEED_AMPLITUDE_RAD_S >= 0.0f, "Amplitude must be nonnegative");
static_assert(CONTROL_SPEED_OUTPUT_LIMIT > 0.0f && CONTROL_SPEED_OUTPUT_LIMIT <= 25000.0f,
              "Output limit must be in (0, 25000]");
static_assert(CONTROL_SPEED_CALC_ENABLED == 0 || CONTROL_SPEED_CALC_ENABLED == 1,
              "Speed calculation switch must be 0 or 1");
static_assert(CONTROL_CAN_TX_ENABLED == 0 || CONTROL_CAN_TX_ENABLED == 1,
              "CAN transmit switch must be 0 or 1");
static_assert(CONTROL_COMMAND_CURRENT == 0 || CONTROL_COMMAND_CURRENT == 1,
              "Command mode must be 0 (voltage) or 1 (current)");
static_assert(CONTROL_TEST_OUTPUT_LIMIT <= (CONTROL_COMMAND_CURRENT ? 16384 : 25000),
              "Test command exceeds protocol limit");
static_assert(CONTROL_SPEED_OUTPUT_LIMIT <= (CONTROL_COMMAND_CURRENT ? 16384 : 25000),
              "PID command exceeds protocol limit");
static_assert(CONTROL_TEST_OUTPUT_LIMIT > 0 && CONTROL_TEST_OUTPUT_LIMIT <= 25000,
              "Test output limit must be in (0, 25000]");

Gm6020 motor(CONTROL_MOTOR_ID);
Pid speed_pid(CONTROL_SPEED_KP, CONTROL_SPEED_KI, CONTROL_SPEED_KD,
              CONTROL_SPEED_OUTPUT_LIMIT, -CONTROL_SPEED_OUTPUT_LIMIT);
constexpr float kTwoPi = 6.2831853071795864769f;
uint32_t speed_phase_ms = 0;
uint32_t processed_feedback_session = 0;
bool feedback_seen = false;  // 只在 CAN 接收中断中访问。
uint32_t last_watchdog_ms = 0;
uint32_t last_watchdog_tick = 0;
// 输出状态只由前台接口和 TIM6 更新，使用 InterruptGuard 保护。
uint32_t armed_session = 0;
uint32_t test_start_ms = 0, test_duration_ms = 0;
float speed_test_target = 0.0f;
float speed_test_sum = 0.0f, speed_test_error_sum = 0.0f, speed_test_tail_sum = 0.0f;
int32_t speed_queued_sum = 0;
int32_t test_current_sum = 0;
int16_t test_voltage = 0;
uint32_t tx_mailbox = 0, tx_started_ms = 0;
int16_t pending_voltage = 0;
bool tx_aborting = false;
// 接收回调会重建 motor，发送使用独立对象避免与接收角度状态共享。
Gm6020 tx_motor(CONTROL_MOTOR_ID);
AssessmentController assessment;
uint32_t assessment_start_ms = 0, assessment_overspeed_ticks = 0;
float assessment_center = 0, assessment_abs_sum = 0, assessment_sq_sum = 0;

bool assessmentMode() {
    return g_control_status.output_mode == CONTROL_OUTPUT_SPEED_SINE ||
           g_control_status.output_mode == CONTROL_OUTPUT_POSITION_SINE;
}

#if CONTROL_CAN_CHANNEL == 1
CAN_HandleTypeDef *const motor_can = &hcan1;
constexpr uint32_t kRxFifo = CAN_RX_FIFO0;
constexpr uint32_t kFilterFifo = CAN_FILTER_FIFO0;
constexpr uint32_t kFilterBank = 0;
constexpr uint32_t kRxNotification = CAN_IT_RX_FIFO0_MSG_PENDING;
#else
// CubeMX 当前为 CAN2 开启 RX1 中断，因此必须使用 FIFO1。
CAN_HandleTypeDef *const motor_can = &hcan2;
constexpr uint32_t kRxFifo = CAN_RX_FIFO1;
constexpr uint32_t kFilterFifo = CAN_FILTER_FIFO1;
constexpr uint32_t kFilterBank = 14;
constexpr uint32_t kRxNotification = CAN_IT_RX_FIFO1_MSG_PENDING;
#endif

// 短暂保护多字段快照，避免 CAN 中断在读取过程中更新反馈。
class InterruptGuard {
public:
    InterruptGuard() : saved_mask_(__get_PRIMASK()) { __disable_irq(); }
    ~InterruptGuard() { __set_PRIMASK(saved_mask_); }
    InterruptGuard(const InterruptGuard &) = delete;
    InterruptGuard &operator=(const InterruptGuard &) = delete;
private:
    uint32_t saved_mask_;
};

bool feedbackFresh(uint32_t now) {
    return g_control_status.feedback_count != 0 &&
           now - g_control_status.last_feedback_ms < CONTROL_FEEDBACK_TIMEOUT_MS;
}

void resetTestCurrent() {
    test_current_sum = 0;
    g_control_status.test_current_samples = 0;
    g_control_status.test_mean_current_raw = 0;
    g_control_status.test_peak_abs_current_raw = 0;
}

void recordTestMotion() {
    // Call with interrupts masked, only within the original feedback session.
    if ((g_control_status.output_mode != CONTROL_OUTPUT_TEST &&
         g_control_status.output_mode != CONTROL_OUTPUT_SPEED_TEST) ||
        armed_session != g_control_status.feedback_session || !feedbackFresh(HAL_GetTick())) return;
    g_control_status.test_delta_angle_rad = g_control_status.angle_rad - g_control_status.test_start_angle_rad;
    const float speed = std::fabs(g_control_status.speed_rad_s);
    if (speed > g_control_status.test_peak_speed_rad_s) g_control_status.test_peak_speed_rad_s = speed;
}

void stopOutput(uint32_t fault) {
    recordTestMotion();
    if (assessmentMode()) {
        g_assessment_status.running = 0;
        if (!g_assessment_status.completed) g_assessment_status.aborted = 1;
        g_assessment_status.output_raw = 0;
        assessment.reset(0);
    }
    g_control_status.output_mode = CONTROL_OUTPUT_STOPPED;
    g_control_status.voltage_command = 0;
    g_control_status.output_command = 0;
    g_control_status.output_fault = g_control_status.output_fault | fault;
    speed_pid.reset();
}

void recordAssessment(uint32_t elapsed, bool force = false) {
    const uint32_t n = g_assessment_log.count;
    if (n >= CONTROL_ASSESSMENT_LOG_CAPACITY) return;
    if (!force && n && elapsed - g_assessment_log.points[n - 1].elapsed_ms < 20U) return;
    auto &point = g_assessment_log.points[n];
    point.elapsed_ms = elapsed;
    point.target_speed = g_assessment_status.target_speed_rad_s;
    point.actual_speed = g_assessment_status.actual_speed_rad_s;
    point.target_angle = g_assessment_status.target_angle_rad;
    point.actual_angle = g_assessment_status.actual_angle_rad;
    point.output_raw = g_assessment_status.output_raw;
    g_assessment_log.count = n + 1U;
}

void runAssessment(uint32_t now, float speed) {
    const InterruptGuard guard;
    if (!assessmentMode() || !feedbackFresh(now) ||
        armed_session != g_control_status.feedback_session) return;
    const uint32_t elapsed = now - assessment_start_ms;
    const bool position = g_control_status.output_mode == CONTROL_OUTPUT_POSITION_SINE;
    g_assessment_status.elapsed_ms = elapsed;
    const float angle = g_control_status.angle_rad;
    const AssessmentOutput r = assessment.step(position, elapsed, assessment_center, angle, speed);
    if (!std::isfinite(r.output) || !std::isfinite(angle) || !std::isfinite(speed)) {
        stopOutput(CONTROL_FAULT_INVALID_OUTPUT); return;
    }
    g_assessment_status.target_speed_rad_s = r.target_speed;
    g_assessment_status.actual_speed_rad_s = speed;
    g_assessment_status.target_angle_rad = r.target_angle;
    g_assessment_status.actual_angle_rad = angle;
    g_control_status.speed_target_rad_s = r.speed_reference;
    g_control_status.speed_feedback_used_rad_s = r.filtered_speed;
    g_control_status.speed_error_rad_s = r.speed_reference - r.filtered_speed;
    g_control_status.speed_output = r.output;
    g_control_status.speed_pid_active = 1;
    if (elapsed >= g_assessment_status.duration_ms) {
        g_assessment_status.completed = 1;
        g_assessment_status.next_mode = position ? CONTROL_OUTPUT_SPEED_SINE : CONTROL_OUTPUT_POSITION_SINE;
        stopOutput(CONTROL_FAULT_NONE);
        recordAssessment(elapsed, true);
        return;
    }
    assessment_overspeed_ticks = std::fabs(speed) > CONTROL_ASSESSMENT_OVERSPEED_RAD_S
        ? assessment_overspeed_ticks + 1U : 0U;
    if (assessment_overspeed_ticks >= 20U) { stopOutput(CONTROL_FAULT_OVERSPEED); return; }
    if (g_control_status.temperature_c >= CONTROL_ASSESSMENT_MAX_TEMPERATURE_C) {
        stopOutput(CONTROL_FAULT_TEMPERATURE); return;
    }
    if (position && elapsed > 1000U && std::fabs(r.target_angle - angle) > CONTROL_ASSESSMENT_POSITION_ERROR_RAD) {
        stopOutput(CONTROL_FAULT_POSITION_ERROR); return;
    }
    const float error = position ? r.target_angle - angle : r.target_speed - speed;
    const float magnitude = std::fabs(error);
    const uint32_t samples = g_assessment_status.samples + 1U;
    assessment_abs_sum += magnitude;
    assessment_sq_sum += error * error;
    g_assessment_status.samples = samples;
    g_assessment_status.error = error;
    g_assessment_status.mean_abs_error = assessment_abs_sum / samples;
    g_assessment_status.rms_error = std::sqrt(assessment_sq_sum / samples);
    if (magnitude > g_assessment_status.max_abs_error) g_assessment_status.max_abs_error = magnitude;
    g_assessment_status.output_raw = static_cast<int16_t>(r.output);
    if (std::fabs(r.output) >= CONTROL_ASSESSMENT_CURRENT_LIMIT)
        g_assessment_status.saturated_samples = g_assessment_status.saturated_samples + 1U;
    recordAssessment(elapsed);
}

void txFault() {
    g_control_status.can_tx_errors = g_control_status.can_tx_errors + 1U;
    stopOutput(CONTROL_FAULT_CAN_TX);
}

// 单个在途帧，不积压历史控制量。所有 HAL 调用均非阻塞。
void sendVoltage(int16_t voltage, uint32_t now) {
    if (tx_mailbox != 0) {
        if (HAL_CAN_IsTxMessagePending(motor_can, tx_mailbox)) {
            if (now - tx_started_ms >= CONTROL_TX_TIMEOUT_MS && !tx_aborting) {
                txFault();
                voltage = 0;
            }
            if (tx_aborting || (pending_voltage != 0 && voltage == 0) ||
                now - tx_started_ms >= CONTROL_TX_TIMEOUT_MS) {
                if (HAL_CAN_AbortTxRequest(motor_can, tx_mailbox) != HAL_OK) {
                    txFault();
                } else if (!tx_aborting) {
                    g_control_status.can_tx_aborts = g_control_status.can_tx_aborts + 1U;
                }
                tx_aborting = true;
            }
            // 撤销是异步的，确认旧请求结束后才能排入新帧。
            if (HAL_CAN_IsTxMessagePending(motor_can, tx_mailbox)) return;
        }
        tx_mailbox = 0;
        tx_aborting = false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(motor_can) == 0) {
        txFault();
        return;
    }
    if (g_control_status.output_fault != CONTROL_FAULT_NONE) voltage = 0;

    CAN_TxHeaderTypeDef header = {};
    header.StdId = tx_motor.txId(CONTROL_COMMAND_CURRENT != 0);
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    // 此单电机工程独占这组报文；其他电机槽位全部为零。
    uint8_t data[8] = {};
    tx_motor.setCommand(voltage);
    tx_motor.encode(data);
    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(motor_can, &header, data, &mailbox) != HAL_OK) {
        txFault();
        return;
    }
    tx_mailbox = mailbox;
    tx_started_ms = now;
    pending_voltage = voltage;
    g_control_status.can_tx_queued = g_control_status.can_tx_queued + 1U;
    if (g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST) {
        const uint32_t n = g_speed_test.queued_samples;
        if (n == 0 || voltage < g_speed_test.min_queued) g_speed_test.min_queued = voltage;
        if (n == 0 || voltage > g_speed_test.max_queued) g_speed_test.max_queued = voltage;
        if (voltage > 0) g_speed_test.positive_queued = g_speed_test.positive_queued + 1U;
        if (voltage < 0) g_speed_test.negative_queued = g_speed_test.negative_queued + 1U;
        speed_queued_sum += voltage;
        g_speed_test.mean_queued = static_cast<float>(speed_queued_sum) / (n + 1U);
        g_speed_test.queued_samples = n + 1U;
        const uint32_t index = g_speed_trace_count;
        const uint32_t elapsed = now - test_start_ms;
        if (index < CONTROL_SPEED_TRACE_CAPACITY && elapsed >= index * 200U) {
            g_speed_trace[index].elapsed_ms = elapsed;
            g_speed_trace[index].queued_command = voltage;
            g_speed_trace[index].speed_rad_s = g_control_status.speed_feedback_used_rad_s;
            g_speed_trace[index].delta_angle_rad = g_control_status.angle_rad - g_control_status.test_start_angle_rad;
            g_speed_trace[index].current_feedback_raw = g_control_status.current_feedback_raw;
            g_speed_trace_count = index + 1U; // Publish after writing the point.
        }
    }
    if (voltage != 0 && (g_control_status.output_mode == CONTROL_OUTPUT_TEST ||
                        g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST)) {
        g_control_status.test_nonzero_queued = g_control_status.test_nonzero_queued + 1U;
    }
}

void updateOutput() {
    const InterruptGuard guard;
    if (!CONTROL_CAN_TX_ENABLED || !g_control_status.initialized) return;
    recordTestMotion();
    const uint32_t now = HAL_GetTick();
    if (g_control_status.output_mode != CONTROL_OUTPUT_STOPPED &&
        (!feedbackFresh(now) || armed_session != g_control_status.feedback_session)) {
        stopOutput(CONTROL_FAULT_FEEDBACK);
    }
    int16_t voltage = 0;
    if (g_control_status.output_mode == CONTROL_OUTPUT_TEST) {
        if (now - test_start_ms >= test_duration_ms) stopOutput(CONTROL_FAULT_NONE);
        else voltage = test_voltage;
    } else if (g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST &&
               now - test_start_ms >= test_duration_ms) {
        stopOutput(CONTROL_FAULT_NONE);
    } else if (g_control_status.output_mode == CONTROL_OUTPUT_SPEED_PID ||
               g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST || assessmentMode()) {
        float output = g_control_status.speed_output;
        if (!std::isfinite(output)) {
            stopOutput(CONTROL_FAULT_INVALID_OUTPUT);
        } else {
            const float limit = assessmentMode() ? CONTROL_ASSESSMENT_CURRENT_LIMIT : CONTROL_SPEED_OUTPUT_LIMIT;
            if (output > limit) output = limit;
            if (output < -limit) output = -limit;
            voltage = static_cast<int16_t>(output);
        }
    }
    if (g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST) {
        const float measured = g_control_status.speed_feedback_used_rad_s;
        const uint32_t samples = g_speed_test.samples + 1U;
        speed_test_sum += measured;
        speed_test_error_sum += std::fabs(speed_test_target - measured);
        g_speed_test.samples = samples;
        g_speed_test.mean_speed_rad_s = speed_test_sum / samples;
        g_speed_test.mean_abs_error_rad_s = speed_test_error_sum / samples;
        g_speed_test.final_speed_rad_s = measured;
        const uint32_t tail_begin = test_duration_ms > 200U ? test_duration_ms - 200U : 0U;
        if (now - test_start_ms >= tail_begin) {
            speed_test_tail_sum += measured;
            const uint32_t tail_samples = g_speed_test.tail_samples + 1U;
            g_speed_test.tail_samples = tail_samples;
            g_speed_test.tail_mean_speed_rad_s = speed_test_tail_sum / tail_samples;
        }
        const float magnitude = std::fabs(g_control_status.speed_output);
        if (magnitude > g_speed_test.peak_abs_output) g_speed_test.peak_abs_output = magnitude;
        if (magnitude >= CONTROL_SPEED_OUTPUT_LIMIT)
            g_speed_test.saturated_samples = g_speed_test.saturated_samples + 1U;
    }
    g_control_status.voltage_command = voltage;
    g_control_status.output_command = voltage;
    sendVoltage(voltage, now);
}

void receivePending(CAN_HandleTypeDef *hcan, uint32_t fifo) {
    if (hcan != motor_can || fifo != kRxFifo) {
        return;
    }

    // 硬件 FIFO 深度为 3；限制单次处理数量，不在中断里无限等待。
    for (uint32_t i = 0; i < 3 && HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0; ++i) {
        CAN_RxHeaderTypeDef header = {};
        uint8_t data[8] = {};
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) {
            g_control_status.can_receive_errors = g_control_status.can_receive_errors + 1U;
            return;
        }
        if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA) {
            g_can_diagnostics.rx_frames = g_can_diagnostics.rx_frames + 1U;
            g_can_diagnostics.last_id = header.StdId;
            g_can_diagnostics.last_dlc = header.DLC;
            g_can_diagnostics.last_rx_ms = HAL_GetTick();
            if (header.DLC == 8) CanFeedbackCallback(header.StdId, data);
        }
    }
}
}  // namespace

uint32_t ControlStartVoltageTest(int16_t voltage, uint32_t duration_ms) {
    return ControlStartOutputTest(voltage, duration_ms);
}

uint32_t ControlStartOutputTest(int16_t voltage, uint32_t duration_ms) {
    const InterruptGuard guard;
    const uint32_t now = HAL_GetTick();
    if (!CONTROL_CAN_TX_ENABLED || !g_control_status.initialized ||
        !feedbackFresh(now) || g_control_status.output_fault != 0 ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED ||
        (tx_mailbox != 0 && (pending_voltage != 0 || tx_aborting)) ||
        duration_ms == 0 || duration_ms > CONTROL_TEST_MAX_DURATION_MS) return 0;
    if (voltage > CONTROL_TEST_OUTPUT_LIMIT) voltage = CONTROL_TEST_OUTPUT_LIMIT;
    if (voltage < -CONTROL_TEST_OUTPUT_LIMIT) voltage = -CONTROL_TEST_OUTPUT_LIMIT;
    test_voltage = voltage;
    test_duration_ms = duration_ms;
    test_start_ms = now;
    armed_session = g_control_status.feedback_session;
    g_control_status.test_start_angle_rad = g_control_status.angle_rad;
    g_control_status.test_delta_angle_rad = 0.0f;
    g_control_status.test_peak_speed_rad_s = 0.0f;
    g_control_status.test_nonzero_queued = 0;
    g_control_status.output_mode = CONTROL_OUTPUT_TEST;
    resetTestCurrent();
    return 1;
}

uint32_t ControlStartSpeedOutput(void) {
    const InterruptGuard guard;
    if (!CONTROL_CAN_TX_ENABLED || !CONTROL_SPEED_CALC_ENABLED ||
        !g_control_status.initialized || !feedbackFresh(HAL_GetTick()) ||
        g_control_status.output_fault != 0 ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED ||
        (tx_mailbox != 0 && (pending_voltage != 0 || tx_aborting))) return 0;
    speed_pid.reset();
    speed_phase_ms = 0;
    armed_session = g_control_status.feedback_session;
    g_control_status.output_mode = CONTROL_OUTPUT_SPEED_PID;
    return 1;
}

uint32_t ControlStartSpeedTest(float target_rad_s, uint32_t duration_ms) {
    const InterruptGuard guard;
    const uint32_t now = HAL_GetTick();
    if (!CONTROL_CAN_TX_ENABLED || !CONTROL_SPEED_CALC_ENABLED ||
        !g_control_status.initialized || !feedbackFresh(now) || g_control_status.output_fault != 0 ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED ||
        (tx_mailbox != 0 && (pending_voltage != 0 || tx_aborting)) ||
        !std::isfinite(target_rad_s) || target_rad_s == 0.0f ||
        std::fabs(target_rad_s) > CONTROL_SPEED_TEST_MAX_TARGET_RAD_S ||
        duration_ms == 0 || duration_ms > CONTROL_SPEED_TEST_MAX_DURATION_MS) return 0;
    speed_pid.reset();
    speed_test_target = target_rad_s;
    test_start_ms = now;
    test_duration_ms = duration_ms;
    armed_session = g_control_status.feedback_session;
    g_control_status.test_start_angle_rad = g_control_status.angle_rad;
    g_control_status.test_delta_angle_rad = 0;
    g_control_status.test_peak_speed_rad_s = 0;
    g_control_status.test_nonzero_queued = 0;
    speed_test_sum = speed_test_error_sum = speed_test_tail_sum = 0;
    speed_queued_sum = 0;
    g_speed_test.queued_samples = g_speed_test.positive_queued = g_speed_test.negative_queued = 0;
    g_speed_test.min_queued = g_speed_test.max_queued = 0;
    g_speed_test.mean_queued = 0;
    g_speed_trace_count = 0; // Only entries below this count belong to this trial.
    g_speed_test.target_rad_s = target_rad_s;
    g_speed_test.kp = CONTROL_SPEED_KP;
    g_speed_test.ki = CONTROL_SPEED_KI;
    g_speed_test.kd = CONTROL_SPEED_KD;
    g_speed_test.output_limit = CONTROL_SPEED_OUTPUT_LIMIT;
    g_speed_test.duration_ms = duration_ms;
    g_speed_test.samples = g_speed_test.tail_samples = g_speed_test.saturated_samples = 0;
    g_speed_test.mean_speed_rad_s = g_speed_test.mean_abs_error_rad_s = 0;
    g_speed_test.final_speed_rad_s = g_speed_test.tail_mean_speed_rad_s = g_speed_test.peak_abs_output = 0;
    g_control_status.output_mode = CONTROL_OUTPUT_SPEED_TEST;
    resetTestCurrent();
    return 1;
}

uint32_t ControlSelectAssessment(uint32_t mode) {
    const InterruptGuard guard;
    if (g_control_status.output_mode != CONTROL_OUTPUT_STOPPED ||
        (mode != CONTROL_OUTPUT_SPEED_SINE && mode != CONTROL_OUTPUT_POSITION_SINE)) return 0;
    g_assessment_status.next_mode = mode;
    return 1;
}

uint32_t ControlStartNextAssessment(void) {
    return ControlStartAssessment(g_assessment_status.next_mode);
}

uint32_t ControlStartAssessment(uint32_t mode) {
    const InterruptGuard guard;
    const uint32_t now = HAL_GetTick();
    if (!CONTROL_CAN_TX_ENABLED || !CONTROL_COMMAND_CURRENT || !CONTROL_SPEED_CALC_ENABLED ||
        !g_control_status.initialized || !feedbackFresh(now) || g_control_status.output_fault ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED ||
        (tx_mailbox != 0 && (pending_voltage != 0 || tx_aborting)) ||
        (mode != CONTROL_OUTPUT_SPEED_SINE && mode != CONTROL_OUTPUT_POSITION_SINE)) return 0;
    assessment_start_ms = now;
    assessment_center = g_control_status.angle_rad;
    assessment_abs_sum = assessment_sq_sum = 0;
    assessment_overspeed_ticks = 0;
    assessment.reset(g_control_status.speed_rad_s);
    speed_pid.reset();
    armed_session = g_control_status.feedback_session;
    g_assessment_status.completed = g_assessment_status.aborted = g_assessment_status.elapsed_ms = 0;
    g_assessment_status.samples = g_assessment_status.saturated_samples = 0;
    g_assessment_status.error = g_assessment_status.mean_abs_error = g_assessment_status.rms_error = 0;
    g_assessment_status.max_abs_error = 0;
    g_assessment_status.target_speed_rad_s = g_assessment_status.actual_speed_rad_s = 0;
    g_assessment_status.output_raw = 0;
    g_assessment_status.next_mode = mode;
    g_assessment_status.mode = mode;
    g_assessment_status.running = 1;
    g_assessment_status.duration_ms = CONTROL_ASSESSMENT_PERIOD_MS * CONTROL_ASSESSMENT_CYCLES;
    g_assessment_status.target_angle_rad = assessment_center;
    g_assessment_status.actual_angle_rad = assessment_center;
    g_assessment_log.version = 1;
    g_assessment_log.count = 0;
    g_assessment_log.mode = mode;
    g_assessment_log.duration_ms = g_assessment_status.duration_ms;
    g_control_status.output_mode = static_cast<ControlOutputMode>(mode);
    return 1;
}

void ControlStopOutput(void) {
    const InterruptGuard guard;
    stopOutput(CONTROL_FAULT_NONE);
}

uint32_t ControlClearOutputFault(void) {
    const InterruptGuard guard;
    if (!g_control_status.initialized || !feedbackFresh(HAL_GetTick()) ||
        g_control_status.output_mode != CONTROL_OUTPUT_STOPPED ||
        (tx_mailbox != 0 && HAL_CAN_IsTxMessagePending(motor_can, tx_mailbox))) return 0;
    g_control_status.output_fault = CONTROL_FAULT_NONE;
    return 1;
}

void ControlTaskInit(void) {
    // main.c 在所有 MX_XXX_Init() 之后调用一次。
    CAN_FilterTypeDef filter = {};
    filter.FilterBank = kFilterBank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = motor.rxId() << 5;
    filter.FilterMaskIdHigh = 0x7FFU << 5;
#if CONTROL_CAN_DIAGNOSTIC_RX
    filter.FilterIdHigh = 0;
    filter.FilterMaskIdHigh = 0;
#endif
    // 比较 IDE/RTR 位，只接收标准数据帧。
    filter.FilterMaskIdLow = 0x0006U;
    filter.FilterFIFOAssignment = kFilterFifo;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(motor_can, &filter) != HAL_OK ||
        HAL_CAN_Start(motor_can) != HAL_OK ||
        HAL_CAN_ActivateNotification(motor_can, kRxNotification) != HAL_OK) {
        Error_Handler();
        return;
    }

    last_watchdog_ms = HAL_GetTick();
    // 清除初始化时可能留下的更新标志，随后每 1 ms 进入 MainTask。
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
        return;
    }
    g_control_status.initialized = 1;
    g_assessment_status.next_mode = CONTROL_OUTPUT_SPEED_SINE;
    g_control_status.command_mode = CONTROL_COMMAND_CURRENT;
    g_can_diagnostics.channel = CONTROL_CAN_CHANNEL;
    g_can_diagnostics.expected_id = motor.rxId();
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (data == nullptr || std_id != motor.rxId()) {
        return;
    }
    const uint16_t raw_angle = static_cast<uint16_t>((data[0] << 8) | data[1]);
    if (raw_angle >= 8192U) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    // 中断过久后无法恢复丢失的整圈数，重新建立角度基准。
    // 后续做位置闭环时，反馈超时必须退出控制并重新确认位置。
    if (!feedback_seen || now - g_control_status.last_feedback_ms >= CONTROL_FEEDBACK_TIMEOUT_MS) {
        motor = Gm6020(CONTROL_MOTOR_ID);
        g_control_status.feedback_session = g_control_status.feedback_session + 1U;
    }
    motor.decode(data);
    feedback_seen = true;
    g_control_status.angle_rad = motor.angle();
    g_control_status.speed_rad_s = motor.vel();
    g_control_status.temperature_c = motor.temp();
    g_control_status.current_feedback_raw = motor.currentRaw();
    // Record the motor's feedback, not the requested current. Freeze after the deadline/stop.
    if ((g_control_status.output_mode == CONTROL_OUTPUT_TEST ||
         g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST) &&
        armed_session == g_control_status.feedback_session && now - test_start_ms < test_duration_ms) {
        const int32_t current = motor.currentRaw();
        const uint32_t magnitude = static_cast<uint32_t>(current < 0 ? -current : current);
        if (magnitude > g_control_status.test_peak_abs_current_raw)
            g_control_status.test_peak_abs_current_raw = magnitude;
        test_current_sum += current;
        const uint32_t samples = g_control_status.test_current_samples + 1U;
        g_control_status.test_current_samples = samples;
        g_control_status.test_mean_current_raw = static_cast<float>(test_current_sum) / samples;
    }
    g_control_status.last_feedback_ms = now;
    g_control_status.feedback_count = g_control_status.feedback_count + 1U;
}

void MainTask(void) {
    uint32_t count;
    uint32_t last_feedback;
    uint32_t now;
    uint32_t feedback_session;
    float speed_feedback;
    {
        const InterruptGuard guard;
        count = g_control_status.feedback_count;
        last_feedback = g_control_status.last_feedback_ms;
        now = HAL_GetTick();
        feedback_session = g_control_status.feedback_session;
        speed_feedback = g_control_status.speed_rad_s;
    }

    if (count == 0) {
        g_control_status.state = CONTROL_WAITING_FEEDBACK;
    } else if (now - last_feedback >= CONTROL_FEEDBACK_TIMEOUT_MS) {
        g_control_status.state = CONTROL_FEEDBACK_TIMEOUT;
    } else {
        g_control_status.state = CONTROL_FEEDBACK_ONLINE;
    }

    // 首帧或断线重连后，从正弦零点重新开始，清除上一段 PID 历史。
    // session 计数还能发现两次定时任务之间已经断开又恢复的情况。
    if (feedback_session != processed_feedback_session) {
        speed_pid.reset();
        speed_phase_ms = 0;
        processed_feedback_session = feedback_session;
    }

    if (assessmentMode()) {
        runAssessment(now, speed_feedback);
        updateOutput();
        g_control_status.control_ticks = g_control_status.control_ticks + 1U;
        return;
    }
    // TIM6 与 Pid.cpp 都按 1 ms 周期运行。整数相位在一个周期内循环，
    // 避免浮点时间长期累加后丢失精度。未接电机时也可预览目标曲线。
    const float target = g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST ? speed_test_target :
        CONTROL_SPEED_AMPLITUDE_RAD_S *
        std::sin(kTwoPi * static_cast<float>(speed_phase_ms) /
                 static_cast<float>(CONTROL_SPEED_PERIOD_MS));
    g_control_status.speed_phase_ms = speed_phase_ms;
    g_control_status.speed_target_rad_s = target;
    g_control_status.speed_feedback_used_rad_s = speed_feedback;

    const bool active = CONTROL_SPEED_CALC_ENABLED &&
                        g_control_status.state == CONTROL_FEEDBACK_ONLINE;
    g_control_status.speed_pid_active = active ? 1U : 0U;
    if (active) {
        g_control_status.speed_error_rad_s = target - speed_feedback;
        // PID 计算与运动使能分离，updateOutput 决定是否允许实际输出。
        g_control_status.speed_output = speed_pid.calc(target, speed_feedback);
    } else {
        speed_pid.reset();
        g_control_status.speed_error_rad_s = 0.0f;
        g_control_status.speed_output = 0.0f;
    }

    speed_phase_ms = (speed_phase_ms + 1U) % CONTROL_SPEED_PERIOD_MS;
    updateOutput();

    // 放在任务末尾，表示本次定时任务已经完成。
    g_control_status.control_ticks = g_control_status.control_ticks + 1U;
}

void ControlTaskPoll(void) {
    const uint32_t now = HAL_GetTick();
    const uint32_t completed_ticks = g_control_status.control_ticks;
    if (g_control_status.initialized == 0 ||
        now - last_watchdog_ms < CONTROL_WATCHDOG_FEED_INTERVAL_MS ||
        completed_ticks == last_watchdog_tick) {
        return;
    }

    // ESR 只读快照；不清除错误、不更改 CAN 状态，每约 10 ms 更新。
    const uint32_t esr = motor_can->Instance->ESR;
    g_can_diagnostics.esr = esr;
    g_can_diagnostics.rx_error_count = (esr >> 24) & 0xFFU;
    g_can_diagnostics.tx_error_count = (esr >> 16) & 0xFFU;
    g_can_diagnostics.last_error_code = (esr >> 4) & 7U;
    g_can_diagnostics.bus_off = (esr >> 2) & 1U;

    // 主循环能到这里、定时任务也有新心跳，才允许喂狗。
    // 没有电机反馈不代表程序卡死，等待硬件时仍正常喂狗。
    if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) {
        last_watchdog_ms = now;
        last_watchdog_tick = completed_ticks;
        g_control_status.watchdog_feeds = g_control_status.watchdog_feeds + 1U;
    }
}

// HAL 弱回调的实现统一放在这里，工程中不要重复定义。
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    receivePending(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    receivePending(hcan, CAN_RX_FIFO1);
}
