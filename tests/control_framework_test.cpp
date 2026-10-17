#include "ControlTask.h"
#include "ControlConfig.h"
#include "Gm6020.h"
#include "AssessmentController.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

CAN_TypeDef can1_registers, can2_registers;
CAN_HandleTypeDef hcan1{&can1_registers}, hcan2{&can2_registers};
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim6;
namespace {
uint32_t now_ms = 0, interrupt_mask = 0, feeds = 0;
bool pending = false;
bool tx_pending = false, hold_tx = false, hold_abort = false;
bool fail_tx = false, fail_abort = false;
uint32_t free_mailboxes = 3, tx_calls = 0, abort_calls = 0;
CAN_TxHeaderTypeDef last_tx_header = {};
uint8_t last_tx_data[8] = {};
CAN_FilterTypeDef configured_filter = {};
CAN_RxHeaderTypeDef queued_header = {};
uint8_t queued_data[8] = {};
CAN_HandleTypeDef *const selected = CONTROL_CAN_CHANNEL == 1 ? &hcan1 : &hcan2;
constexpr uint32_t fifo = CONTROL_CAN_CHANNEL == 1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
void receive() {
    if (CONTROL_CAN_CHANNEL == 1) HAL_CAN_RxFifo0MsgPendingCallback(selected);
    else HAL_CAN_RxFifo1MsgPendingCallback(selected);
}
void enqueue(uint16_t angle, uint32_t id = 0x204U + CONTROL_MOTOR_ID, uint32_t dlc = 8, int16_t rpm = 60, int16_t current = 0) {
    queued_header = {id, CAN_ID_STD, CAN_RTR_DATA, dlc};
    queued_data[0] = static_cast<uint8_t>(angle >> 8);
    queued_data[1] = static_cast<uint8_t>(angle & 0xFF);
    const uint16_t raw_rpm = static_cast<uint16_t>(rpm);
    queued_data[2] = static_cast<uint8_t>(raw_rpm >> 8);
    queued_data[3] = static_cast<uint8_t>(raw_rpm & 0xFF);
    queued_data[4] = static_cast<uint16_t>(current) >> 8;
    queued_data[5] = static_cast<uint16_t>(current) & 0xFF;
    queued_data[6] = 25;
    pending = true;
    receive();
}
void tick(uint32_t time) {
    now_ms = time;
    HAL_TIM_PeriodElapsedCallback(&htim6);
}
void freshTick(uint32_t time) {
    now_ms = time;
    enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, 0);
    tick(time);
}
void expectVoltage(int16_t value) {
    assert(g_control_status.output_command == value);
    const uint32_t expected_id = CONTROL_COMMAND_CURRENT
        ? (CONTROL_MOTOR_ID <= 4 ? 0x1FEU : 0x2FEU)
        : (CONTROL_MOTOR_ID <= 4 ? 0x1FFU : 0x2FFU);
    assert(last_tx_header.StdId == expected_id);
    assert(last_tx_header.IDE == CAN_ID_STD && last_tx_header.RTR == CAN_RTR_DATA);
    assert(last_tx_header.DLC == 8 && last_tx_header.TransmitGlobalTime == 0);
    const unsigned slot = (CONTROL_MOTOR_ID <= 4 ? CONTROL_MOTOR_ID - 1 : CONTROL_MOTOR_ID - 5) * 2;
    for (unsigned i = 0; i < 8; ++i) {
        uint8_t expected = 0;
        if (i == slot) expected = static_cast<uint16_t>(value) >> 8;
        if (i == slot + 1) expected = static_cast<uint16_t>(value) & 0xFF;
        assert(last_tx_data[i] == expected);
    }
}
}

uint32_t HAL_GetTick() { return now_ms; }
uint32_t __get_PRIMASK() { return interrupt_mask; }
void __disable_irq() { interrupt_mask = 1; }
void __set_PRIMASK(uint32_t value) { interrupt_mask = value; }
void Error_Handler() { assert(false && "Unexpected HAL initialization error"); }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *handle, CAN_FilterTypeDef *filter) {
    assert(handle == selected); configured_filter = *filter; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *handle) { assert(handle == selected); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *handle, uint32_t notification) {
    assert(handle == selected);
    assert(notification == (CONTROL_CAN_CHANNEL == 1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING));
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *handle, uint32_t rx_fifo) {
    assert(handle == selected && rx_fifo == fifo); return pending ? 1U : 0U;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    *header = queued_header; std::memcpy(data, queued_data, 8); pending = false; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer) { assert(timer == &htim6); return HAL_OK; }
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *handle) { assert(handle == &hiwdg); ++feeds; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t *mailbox) {
    assert(handle == selected && !tx_pending);
    if (fail_tx) return HAL_ERROR;
    last_tx_header = *header;
    std::memcpy(last_tx_data, data, 8);
    ++tx_calls; tx_pending = true; *mailbox = 1;
    return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle, uint32_t mailbox) {
    assert(handle == selected && mailbox == 1);
    if (!hold_tx) tx_pending = false;
    return tx_pending ? 1U : 0U;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *handle) {
    assert(handle == selected); return free_mailboxes;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle, uint32_t mailbox) {
    assert(handle == selected && mailbox == 1); ++abort_calls;
    if (fail_abort) return HAL_ERROR;
    if (!hold_abort) tx_pending = false;
    return HAL_OK;
}

int main() {
    ControlTaskInit();
    assert(g_control_status.initialized == 1);
    assert(g_control_status.command_mode == CONTROL_COMMAND_CURRENT);
    assert(ControlStartVoltageTest(100, 20) == 0); // 无反馈不可使能。
    assert(ControlStartSpeedOutput() == 0);
    assert(ControlStartSpeedTest(0.5f, 1000) == 0); // 尚无反馈
    assert(configured_filter.FilterFIFOAssignment == fifo);
    assert(configured_filter.FilterBank == (CONTROL_CAN_CHANNEL == 1 ? 0U : 14U));
    assert(configured_filter.FilterIdHigh == (CONTROL_CAN_DIAGNOSTIC_RX ? 0U : ((0x204U + CONTROL_MOTOR_ID) << 5)));
    assert(configured_filter.FilterMaskIdHigh == (CONTROL_CAN_DIAGNOSTIC_RX ? 0U : (0x7FFU << 5)));
    assert(configured_filter.FilterMaskIdLow == 6U);
    assert(g_can_diagnostics.channel == CONTROL_CAN_CHANNEL);
    assert(g_can_diagnostics.expected_id == 0x204U + CONTROL_MOTOR_ID);
    // 其他 ID 能被诊断记录，但绝不能被当成本电机的反馈。
    enqueue(100, 0x123);
    assert(g_can_diagnostics.rx_frames == 1 && g_can_diagnostics.last_id == 0x123);
    assert(g_control_status.feedback_count == 0);
    selected->Instance->ESR = (9U << 24) | (3U << 16) | (2U << 4) | 4U;
    tick(10); ControlTaskPoll();
    assert(g_can_diagnostics.rx_error_count == 9 && g_can_diagnostics.tx_error_count == 3);
    assert(g_can_diagnostics.last_error_code == 2 && g_can_diagnostics.bus_off == 1);
    selected->Instance->ESR = 0;
    assert(g_control_status.state == CONTROL_WAITING_FEEDBACK && feeds == 1);
    assert(g_control_status.speed_pid_active == 0 && g_control_status.speed_output == 0.0f);
    now_ms = 20; ControlTaskPoll();
    assert(feeds == 1); // 定时任务停止推进时，不继续喂狗。
    tick(21); ControlTaskPoll(); assert(feeds == 2);
    tick(40); assert(feeds == 2); // 中断本身不喂狗，主循环也必须运行。

    enqueue(6000, 0x100); enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 7);
    enqueue(9000);
    assert(g_control_status.feedback_count == 0);
    queued_header = {0x204U + CONTROL_MOTOR_ID, 4, 0, 8}; pending = true; receive();
    assert(g_control_status.feedback_count == 0); // 忽略扩展帧。
    enqueue(6000);
    assert(g_control_status.angle_rad > 4.5f); // 首帧不能误判为反转一圈。
    assert(std::fabs(g_control_status.speed_rad_s - 6.2831853f) < 0.001f);
    tick(41); assert(g_control_status.state == CONTROL_FEEDBACK_ONLINE);
    assert(interrupt_mask == 0);
    tick(140); assert(g_control_status.state == CONTROL_FEEDBACK_TIMEOUT);
    ControlTaskPoll(); assert(feeds == 3); // 无反馈并不等于程序卡死。
    enqueue(8100); tick(141);
    assert(g_control_status.state == CONTROL_FEEDBACK_ONLINE);
    enqueue(100);
    assert(g_control_status.angle_rad > 6.28f); // 正向跨零，累计角度增加。
    enqueue(8100);
    assert(g_control_status.angle_rad < 6.28f); // 反向跨零，累计角度减少。

    // 毫秒计数溢出后，超时判断仍正确。
    now_ms = 0xFFFFFFF0U; enqueue(6000);
    tick(20); assert(g_control_status.state == CONTROL_FEEDBACK_ONLINE);
    tick(100); assert(g_control_status.state == CONTROL_FEEDBACK_TIMEOUT);

    // 没有新反馈时，仍可预览完整正弦周期，但必须保持 PID 输出为零。
    float first_target = 0.0f;
    for (uint32_t i = 0; i <= CONTROL_SPEED_PERIOD_MS; ++i) {
        tick(200 + i);
        const auto phase = g_control_status.speed_phase_ms;
        const float target = g_control_status.speed_target_rad_s;
        if (i == 0) first_target = target;
        if (i == CONTROL_SPEED_PERIOD_MS) assert(std::fabs(target - first_target) < 0.00001f);
        if (phase == 0 || phase == CONTROL_SPEED_PERIOD_MS / 2) assert(std::fabs(target) < 0.00001f);
        if (phase == CONTROL_SPEED_PERIOD_MS / 4) assert(std::fabs(target - CONTROL_SPEED_AMPLITUDE_RAD_S) < 0.00001f);
        if (phase == 3 * CONTROL_SPEED_PERIOD_MS / 4) assert(std::fabs(target + CONTROL_SPEED_AMPLITUDE_RAD_S) < 0.00001f);
        assert(g_control_status.speed_output == 0.0f && g_control_status.speed_pid_active == 0);
    }

    // 恢复零速度反馈，从零相位开始；测试参数的非零增益仅在电脑测试中使用。
    now_ms = 10000; enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, 0); tick(now_ms);
    assert(g_control_status.speed_phase_ms == 0 && g_control_status.speed_output == 0.0f);
    for (uint32_t i = 1; i <= 200; ++i) {
        now_ms = 10000 + i;
        enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, 0); tick(now_ms);
        assert(std::fabs(g_control_status.speed_output) <= CONTROL_SPEED_OUTPUT_LIMIT);
        assert(g_control_status.speed_pid_active == (CONTROL_SPEED_CALC_ENABLED ? 1U : 0U));
        if (CONTROL_SPEED_CALC_ENABLED) {
            assert(std::fabs(g_control_status.speed_error_rad_s - g_control_status.speed_target_rad_s) < 0.00001f);
        } else {
            assert(g_control_status.speed_output == 0.0f && g_control_status.speed_error_rad_s == 0.0f);
        }
    }
    if (CONTROL_SPEED_CALC_ENABLED && CONTROL_SPEED_KP > 0.0f) assert(g_control_status.speed_output > 0.0f);
    tick(now_ms + CONTROL_FEEDBACK_TIMEOUT_MS);
    assert(g_control_status.state == CONTROL_FEEDBACK_TIMEOUT);
    assert(g_control_status.speed_output == 0.0f && g_control_status.speed_pid_active == 0);
    enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, 0); tick(now_ms + 1);
    assert(g_control_status.speed_phase_ms == 0 && g_control_status.speed_output == 0.0f);

    // 测试非零 P 参数下的正负限幅，并验证误差使用的是同一拍反馈。
    const int16_t test_rpms[] = {30000, -30000};
    for (const int16_t rpm : test_rpms) {
        now_ms += 1;
        enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, rpm); tick(now_ms);
        assert(std::fabs(g_control_status.speed_output) <= CONTROL_SPEED_OUTPUT_LIMIT);
        if (CONTROL_SPEED_CALC_ENABLED && CONTROL_SPEED_KP >= 4.0f && CONTROL_SPEED_OUTPUT_LIMIT == 5.0f) {
            assert(g_control_status.speed_output == (rpm > 0 ? -5.0f : 5.0f));
        }
    }
    // 接收端先发现断线重连，即使 MainTask 没看到超时状态，也应重置积分与相位。
    now_ms += CONTROL_FEEDBACK_TIMEOUT_MS;
    enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, 0); tick(now_ms);
    assert(g_control_status.speed_phase_ms == 0 && g_control_status.speed_output == 0.0f);
    std::printf("PASS: CAN%d, feedback, sine period/amplitude, PID limits/reset, timeout, watchdog\n", CONTROL_CAN_CHANNEL);

    if (!CONTROL_CAN_TX_ENABLED) {
        assert(ControlStartSpeedTest(0.5f, 1000) == 0);
        assert(tx_calls == 0 && ControlStartVoltageTest(100, 20) == 0);
        assert(ControlStartSpeedOutput() == 0);
        std::puts("PASS: transmit disabled, no CAN commands");
        return 0;
    }
    expectVoltage(0); // 开启发送也不会自动运动。
    assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
    assert(ControlStartVoltageTest(100, 0) == 0);
    assert(ControlStartVoltageTest(100, CONTROL_TEST_MAX_DURATION_MS + 1) == 0);
    uint32_t t = now_ms;
    assert(ControlStartVoltageTest(32767, 20) == 1);
    assert(g_control_status.test_nonzero_queued == 0);
    const float start_angle = g_control_status.test_start_angle_rad;
    assert(ControlStartVoltageTest(10, 100) == 0); // 不允许重复调用延长测试。
    now_ms = t + 1; enqueue(6100, 0x204U + CONTROL_MOTOR_ID, 8, -60); tick(now_ms);
    expectVoltage(CONTROL_TEST_OUTPUT_LIMIT);
    assert(g_control_status.test_nonzero_queued == 1);
    const float expected_delta = g_control_status.angle_rad - start_angle;
    assert(std::fabs(g_control_status.test_delta_angle_rad - expected_delta) < 0.00001f);
    assert(std::fabs(g_control_status.test_peak_speed_rad_s - 6.2831853f) < 0.00001f);
    freshTick(t + 19); expectVoltage(CONTROL_TEST_OUTPUT_LIMIT);
    freshTick(t + 20); expectVoltage(0);
    assert(g_control_status.test_nonzero_queued == 2);
    assert(std::fabs(g_control_status.test_peak_speed_rad_s - 6.2831853f) < 0.00001f);
    const float saved_delta = g_control_status.test_delta_angle_rad;
    enqueue(6200); // Stopped feedback must not overwrite the completed measurement.
    assert(g_control_status.test_delta_angle_rad == saved_delta);
    assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
    assert(ControlStartVoltageTest(-32768, 20) == 1);
    assert(g_control_status.test_nonzero_queued == 0 && g_control_status.test_peak_speed_rad_s == 0.0f);
    freshTick(t + 21); expectVoltage(-CONTROL_TEST_OUTPUT_LIMIT);

    // 停止时撤销尚未发出的非零帧，撤销完成前不能加入新的帧。
    hold_tx = true; hold_abort = true;
    ControlStopOutput();
    const auto before_stop = tx_calls;
    freshTick(t + 22);
    assert(tx_calls == before_stop && abort_calls != 0);
    assert(g_control_status.voltage_command == 0);
    assert(ControlStartVoltageTest(100, 20) == 0);
    hold_abort = false;
    freshTick(t + 23); expectVoltage(0);
    hold_tx = false;

    // 反馈丢失锁存故障；恢复只发零，不自动运动。
    assert(ControlStartVoltageTest(100, 500) == 1);
    freshTick(t + 24); expectVoltage(100);
    tick(t + 124); expectVoltage(0);
    assert(g_control_status.output_fault & CONTROL_FAULT_FEEDBACK);
    assert(ControlClearOutputFault() == 0);
    freshTick(t + 125); expectVoltage(0);
    assert(ControlStartVoltageTest(100, 20) == 0);
    assert(ControlClearOutputFault() == 1);

    // 接收中断先发现重连，即使未经历 TIMEOUT 状态也应停止。
    assert(ControlStartVoltageTest(100, 500) == 1);
    freshTick(t + 126);
    freshTick(t + 226); expectVoltage(0);
    assert(g_control_status.output_fault & CONTROL_FAULT_FEEDBACK);
    assert(ControlClearOutputFault() == 1);

    // 邮箱一直 pending（例如未获 ACK）会在期限后撤销并锁存故障。
    assert(ControlStartVoltageTest(100, 500) == 1);
    freshTick(t + 227); expectVoltage(100);
    hold_tx = true;
    const auto before_timeout = tx_calls;
    freshTick(t + 228); assert(tx_calls == before_timeout);
    freshTick(t + 232); expectVoltage(0);
    assert(g_control_status.output_fault & CONTROL_FAULT_CAN_TX);
    hold_tx = false;
    freshTick(t + 233); assert(ControlClearOutputFault() == 1);

    // HAL 提交失败、邮箱满都撤销使能。
    fail_tx = true;
    assert(ControlStartVoltageTest(100, 500) == 1);
    freshTick(t + 234);
    assert(g_control_status.output_fault & CONTROL_FAULT_CAN_TX);
    assert(g_control_status.voltage_command == 0);
    fail_tx = false;
    freshTick(t + 235); expectVoltage(0); assert(ControlClearOutputFault() == 1);
    free_mailboxes = 0;
    freshTick(t + 236); assert(g_control_status.output_fault & CONTROL_FAULT_CAN_TX);
    free_mailboxes = 3;
    freshTick(t + 237); assert(ControlClearOutputFault() == 1);

    // 撤销失败不会重启；下次撤销成功后只能提交零输出。
    assert(ControlStartVoltageTest(100, 500) == 1);
    freshTick(t + 238); hold_tx = true; fail_abort = true;
    ControlStopOutput();
    const auto before_abort_failure = tx_calls;
    freshTick(t + 239);
    assert(tx_calls == before_abort_failure && (g_control_status.output_fault & CONTROL_FAULT_CAN_TX));
    fail_abort = false;
    freshTick(t + 240); expectVoltage(0);
    hold_tx = false;
    freshTick(t + 241); assert(ControlClearOutputFault() == 1);

    // 速度输出必须单独使能；停止后 PID 预览不会重新驱动电机。
    assert(ControlStartSpeedOutput() == (CONTROL_SPEED_CALC_ENABLED ? 1U : 0U));
    for (unsigned i = 0; i < 500; ++i) {
        freshTick(t + 242 + i);
        const int16_t expected = CONTROL_SPEED_CALC_ENABLED ? static_cast<int16_t>(g_control_status.speed_output) : 0;
        expectVoltage(expected);
    }
    ControlStopOutput(); freshTick(now_ms + 1); expectVoltage(0);
    assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);

    // 计时器回绕不延长电压测试。
    freshTick(0xFFFFFFF0U);
    assert(ControlStartVoltageTest(100, 20) == 1);
    freshTick(0xFFFFFFF1U); expectVoltage(100);
    freshTick(4); expectVoltage(0);
    assert(g_control_status.can_tx_queued == tx_calls);
    assert(ControlStartSpeedTest(0.5f, 0) == 0);
    assert(ControlStartSpeedTest(0.5f, CONTROL_SPEED_TEST_MAX_DURATION_MS + 1) == 0);
    assert(ControlStartSpeedTest(0, 20) == 0);
    assert(ControlStartSpeedTest(2.0f, 20) == 0);
    assert(ControlStartSpeedTest(std::numeric_limits<float>::quiet_NaN(), 20) == 0);
    assert(ControlStartSpeedTest(std::numeric_limits<float>::infinity(), 20) == 0);
    if (CONTROL_SPEED_CALC_ENABLED) {
        assert(ControlStartSpeedTest(0.5f, 1000) == 1);
        assert(ControlStartSpeedTest(-0.5f, 1000) == 0); // 不允许续时/变更目标
        const uint32_t speed_start = now_ms;
        int32_t queued_sum = 0;
        uint32_t positive_frames = 0, negative_frames = 0;
        int16_t queued_min = 32767, queued_max = -32768;
        for (uint32_t i = 1; i <= 1000; ++i) {
            now_ms = speed_start + i;
            enqueue(6200, 0x204U + CONTROL_MOTOR_ID, 8, i < 800 ? 0 : 6, i < 800 ? 120 : -80);
            tick(now_ms);
            if (i < 1000) {
                const unsigned slot = (CONTROL_MOTOR_ID <= 4 ? CONTROL_MOTOR_ID - 1 : CONTROL_MOTOR_ID - 5) * 2;
                const int16_t sent = static_cast<int16_t>((last_tx_data[slot] << 8) | last_tx_data[slot + 1]);
                queued_sum += sent;
                positive_frames += sent > 0;
                negative_frames += sent < 0;
                if (sent < queued_min) queued_min = sent;
                if (sent > queued_max) queued_max = sent;
                assert(g_speed_test.queued_samples == i);
                assert(g_speed_test.positive_queued == positive_frames && g_speed_test.negative_queued == negative_frames);
                assert(g_speed_test.min_queued == queued_min && g_speed_test.max_queued == queued_max);
                assert(std::fabs(g_speed_test.mean_queued - static_cast<float>(queued_sum) / i) < 0.0001f);
                expectVoltage(static_cast<int16_t>(g_control_status.speed_output));
                if (i == 1 || i == 200 || i == 400 || i == 600 || i == 800) {
                    const auto &point = g_speed_trace[g_speed_trace_count - 1];
                    assert(point.elapsed_ms == i && point.queued_command == sent);
                    assert(point.current_feedback_raw == (i < 800 ? 120 : -80));
                    assert(std::fabs(point.speed_rad_s - (i < 800 ? 0.0f : 0.6283185f)) < 0.0001f);
                }
                assert(g_control_status.speed_target_rad_s == 0.5f);
                assert(g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST);
                assert(std::fabs(g_control_status.output_command) <= CONTROL_SPEED_OUTPUT_LIMIT);
                if (i == 1 && CONTROL_SPEED_KP > 0) assert(g_control_status.speed_output > 0);
                if (i == 800 && CONTROL_SPEED_KI == 0 && CONTROL_SPEED_KD == 0 && CONTROL_SPEED_KP > 0)
                    assert(g_control_status.speed_output < 0); // 超过目标时反馈方向正确
            }
        }
        expectVoltage(0);
        assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
        assert(g_speed_test.samples == 999 && g_speed_test.tail_samples == 200);
        assert(std::fabs(g_speed_test.tail_mean_speed_rad_s - 0.6283185f) < 0.0001f);
        const float saved_mean = g_speed_test.mean_speed_rad_s;
        assert(g_speed_trace_count == 5 && g_speed_test.queued_samples == 999);
        assert(g_control_status.test_current_samples == 999);
        assert(g_control_status.test_peak_abs_current_raw == 120);
        const float expected_current = (799.0f * 120 - 200.0f * 80) / 999.0f;
        assert(std::fabs(g_control_status.test_mean_current_raw - expected_current) < 0.0001f);
        const float saved_queued_mean = g_speed_test.mean_queued;
        freshTick(now_ms + 1);
        assert(g_speed_test.samples == 999 && g_speed_test.mean_speed_rad_s == saved_mean);
        assert(g_speed_test.queued_samples == 999 && g_speed_test.mean_queued == saved_queued_mean);
        assert(g_speed_trace_count == 5 && g_speed_trace[4].elapsed_ms == 800);
        assert(g_control_status.test_current_samples == 999);
        assert(std::fabs(g_control_status.test_mean_current_raw - expected_current) < 0.0001f);
        assert(ControlStartSpeedTest(-0.5f, 100) == 1);
        assert(g_speed_trace_count == 0 && g_speed_test.queued_samples == 0);
        assert(g_control_status.test_current_samples == 0 && g_control_status.test_peak_abs_current_raw == 0);
        freshTick(now_ms + 1);
        assert(g_control_status.speed_output < 0 && g_speed_test.samples == 1);
        if (g_control_status.output_command < 0) assert(g_speed_test.negative_queued == 1);
        ControlStopOutput(); freshTick(now_ms + 1); expectVoltage(0);
        assert(g_speed_test.samples == 1); // 手动停止后统计冻结
        assert(ControlStartSpeedTest(0.5f, 1000) == 1);
        freshTick(now_ms + 1);
        tick(now_ms + CONTROL_FEEDBACK_TIMEOUT_MS);
        expectVoltage(0); assert(g_control_status.output_fault & CONTROL_FAULT_FEEDBACK);
        freshTick(now_ms + 1); assert(ControlClearOutputFault() == 1);
        freshTick(0xFFFFFFF0U);
        assert(ControlStartSpeedTest(0.5f, 20) == 1);
        freshTick(0xFFFFFFF1U); freshTick(4); expectVoltage(0);
        assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
        assert(g_speed_trace_count == 1 && g_speed_trace[0].elapsed_ms == 1);
        assert(ControlStartSpeedTest(0.5f, 100) == 1);
        fail_tx = true;
        freshTick(now_ms + 1);
        fail_tx = false;
        assert(g_speed_test.queued_samples == 0 && g_speed_trace_count == 0);
        assert(g_control_status.output_fault & CONTROL_FAULT_CAN_TX);
        freshTick(now_ms + 1); assert(ControlClearOutputFault() == 1);
        assert(ControlStartOutputTest(500, 20) == 1);
        enqueue(6200, 0x204U + CONTROL_MOTOR_ID, 8, 0, -32768);
        enqueue(6200, 0x204U + CONTROL_MOTOR_ID, 8, 0, 32767);
        assert(g_control_status.current_feedback_raw == 32767);
        assert(g_control_status.test_current_samples == 2);
        assert(g_control_status.test_peak_abs_current_raw == 32768U);
        assert(g_control_status.test_mean_current_raw == -0.5f);
        ControlStopOutput(); freshTick(now_ms + 1);
        assert(g_control_status.current_feedback_raw == 0);
        assert(g_control_status.test_current_samples == 2 && g_control_status.test_mean_current_raw == -0.5f);
        expectVoltage(0);
        // 3 秒速度测试独立于原始给定 1 秒上限，验证整个窗口和强制截止。
        assert(ControlStartOutputTest(500, CONTROL_TEST_MAX_DURATION_MS + 1) == 0);
        assert(ControlStartSpeedTest(0.5f, CONTROL_SPEED_TEST_MAX_DURATION_MS) == 1);
        const uint32_t long_start = now_ms;
        for (uint32_t i = 1; i <= CONTROL_SPEED_TEST_MAX_DURATION_MS; ++i) {
            now_ms = long_start + i;
            enqueue(6200, 0x204U + CONTROL_MOTOR_ID, 8, 6, 120);
            tick(now_ms);
            if (i < CONTROL_SPEED_TEST_MAX_DURATION_MS) {
                assert(g_control_status.output_mode == CONTROL_OUTPUT_SPEED_TEST);
                assert(std::fabs(g_control_status.output_command) <= CONTROL_SPEED_OUTPUT_LIMIT);
            }
        }
        expectVoltage(0);
        assert(g_control_status.output_mode == CONTROL_OUTPUT_STOPPED);
        assert(g_speed_test.duration_ms == 3000 && g_speed_test.samples == 2999);
        assert(g_speed_test.tail_samples == 200);
        assert(std::fabs(g_speed_test.tail_mean_speed_rad_s - 0.6283185f) < 0.0001f);
        assert(g_speed_trace_count == 15 && g_speed_trace[14].elapsed_ms == 2800);
        assert(g_control_status.test_current_samples == 2999);
        assert(std::fabs(g_control_status.test_mean_current_raw - 120.0f) < 0.0001f);
        freshTick(now_ms + 1);
        assert(g_speed_test.samples == 2999 && g_speed_trace_count == 15);
        std::puts("PASS: bounded speed test, feedback sign, summary/tail samples, explicit stop, feedback loss, timer wrap");
    } else {
        assert(ControlStartSpeedTest(0.5f, 1000) == 0);
    }
    std::puts("PASS: CAN encoding, explicit enable, bounded test, stop/abort, timeout, reconnect, TX errors, speed output");
    if (CONTROL_COMMAND_CURRENT && CONTROL_SPEED_CALC_ENABLED) {
        AssessmentController controller;
        controller.reset(0);
        // 经过短暂量化零速时不立即触发起动补偿。
        AssessmentOutput early = {};
        const float first = controller.step(false, 2000, 0, 0, 0).output;
        controller.reset(0);
        for (unsigned i = 0; i < 50; ++i) early = controller.step(false, 2000, 0, 0, 0);
        const float integration_only = CONTROL_ASSESSMENT_SPEED_KI * 3.0f * 0.049f;
        assert(std::fabs((early.output - first) - integration_only) < 0.01f);
        // 持续静止时补偿有延迟、有界增长，重置后不会继承上一轮补偿。
        controller.reset(0);
        float final_boost = 0;
        const float stationary_base = CONTROL_ASSESSMENT_SPEED_KP * 3.0f +
            CONTROL_ASSESSMENT_FRICTION_RAW + AssessmentAngleFeedforward(0, 3.0f);
        for (unsigned i = 1; i <= 400; ++i) {
            const auto stationary = controller.step(false, 2000, 0, 0, 0);
            const float pi_only = stationary_base + CONTROL_ASSESSMENT_SPEED_KI * 3.0f * i * 0.001f;
            final_boost = stationary.output - pi_only;
            assert(final_boost >= -0.02f && final_boost <= CONTROL_ASSESSMENT_BREAKAWAY_RAW + 0.02f);
            if (i < CONTROL_ASSESSMENT_BREAKAWAY_DELAY_MS) assert(std::fabs(final_boost) < 0.02f);
            if (i == CONTROL_ASSESSMENT_BREAKAWAY_DELAY_MS)
                assert(std::fabs(final_boost - CONTROL_ASSESSMENT_BREAKAWAY_RAMP_RAW_PER_MS) < 0.02f);
        }
        assert(std::fabs(final_boost - CONTROL_ASSESSMENT_BREAKAWAY_RAW) < 0.02f);
        controller.reset(0);
        assert(std::fabs(controller.step(false, 2000, 0, 0, 0).output - first) < 0.01f);
        for (int i = -500; i <= 500; ++i) {
            const float angle = i * 0.07f;
            const float feed = AssessmentAngleFeedforward(angle, 1.0f);
            assert(std::fabs(feed) <= CONTROL_ASSESSMENT_ANGLE_FF_LIMIT);
            assert(std::fabs(feed - AssessmentAngleFeedforward(angle + 6.28318530718f, 1.0f)) < 0.01f);
            assert(std::fabs(feed - AssessmentAngleFeedforward(angle, -1.0f)) < 0.0001f);
            assert(AssessmentAngleFeedforward(angle, 0.0f) == 0.0f);
            assert(std::fabs(AssessmentAngleFeedforward(angle, 0.15f) - feed * 0.5f) < 0.001f);
        }
        assert(AssessmentAngleFeedforward(std::numeric_limits<float>::quiet_NaN(), 1) == 0);
        assert(AssessmentAngleFeedforward(0, std::numeric_limits<float>::infinity()) == 0);
        // 目标突变且实际速度不变时，不产生D冲击。
        controller.reset(0);
        const uint32_t reference_steps[] = {0U, 2000U, 6000U, 0U};
        for (const uint32_t phase_ms : reference_steps) {
            assert(controller.step(false, phase_ms, 0, 0, 0).derivative_output == 0);
        }
        // 速度上升产生负向D给定，速度下降产生正向D给定；限幅且重置清除历史。
        controller.reset(0);
        const auto rising = controller.step(false, 2000, 0, 0, 100);
        assert(std::isfinite(rising.output));
        assert(std::fabs(rising.derivative_output) <= CONTROL_ASSESSMENT_SPEED_D_LIMIT);
        controller.reset(100);
        const auto falling = controller.step(false, 2000, 0, 0, 0);
        assert(std::fabs(falling.derivative_output) <= CONTROL_ASSESSMENT_SPEED_D_LIMIT);
        if (CONTROL_ASSESSMENT_SPEED_KD > 0) {
            assert(rising.derivative_output < 0 && falling.derivative_output > 0);
            assert(std::fabs(rising.derivative_output) == CONTROL_ASSESSMENT_SPEED_D_LIMIT);
        } else {
            assert(rising.derivative_output == 0 && falling.derivative_output == 0);
        }
        for (unsigned i = 0; i < 500; ++i) {
            const auto settled = controller.step(false, 2000, 0, 0, 0);
            assert(std::fabs(settled.derivative_output) <= CONTROL_ASSESSMENT_SPEED_D_LIMIT);
        }
        assert(std::fabs(controller.step(false, 2000, 0, 0, 0).derivative_output) < 0.001f);
        controller.reset(0);
        assert(controller.step(false, 2000, 0, 0, 0).derivative_output == 0);
        controller.reset(0);
        for (unsigned i = 0; i < 4000; ++i) {
            const auto v = controller.step(false, 2000, 0, 0, 0);
            assert(std::fabs(v.target_speed - 3.0f) < 0.0001f);
            assert(std::fabs(v.output) <= CONTROL_ASSESSMENT_CURRENT_LIMIT);
        }
        AssessmentOutput unwind = {};
        for (unsigned i = 0; i < 4000; ++i) unwind = controller.step(false, 6000, 0, 0, 0);
        assert(unwind.output < 0); // 持续反向误差可退出原来的正饱和。
        controller.reset(0);
        const auto pos = controller.step(true, 2000, 10.0f, 10.0f, 0);
        assert(std::fabs(pos.target_angle - 11.0f) < 0.0001f);
        assert(std::fabs(pos.speed_reference) <= CONTROL_ASSESSMENT_SPEED_LIMIT);
        freshTick(now_ms + 1);
        assert(ControlClearOutputFault() == 1);
        assert(ControlStartAssessment(999) == 0);
        assert(ControlSelectAssessment(CONTROL_OUTPUT_SPEED_SINE) == 1);
        assert(ControlStartNextAssessment() == 1);
        assert(ControlStartNextAssessment() == 0);
        assert(ControlSelectAssessment(CONTROL_OUTPUT_POSITION_SINE) == 0);
        const uint32_t assessment_start = now_ms;
        for (uint32_t i = 1; i <= 16000; ++i) {
            now_ms = assessment_start + i;
            const float target = 3.0f * std::sin(6.28318530718f * i / 8000.0f);
            enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, static_cast<int16_t>(target * 60 / 6.28318530718f));
            tick(now_ms);
            assert(std::fabs(g_control_status.output_command) <= CONTROL_ASSESSMENT_CURRENT_LIMIT);
        }
        expectVoltage(0);
        assert(g_assessment_status.completed == 1 && g_assessment_status.aborted == 0);
        assert(g_assessment_status.running == 0 && g_assessment_log.count >= 800);
        assert(g_assessment_status.next_mode == CONTROL_OUTPUT_POSITION_SINE);
        assert(g_assessment_log.points[g_assessment_log.count - 1].output_raw == 0);
        assert(g_assessment_status.rms_error < 0.11f); // 这是注入反馈的量化误差，不模拟电机性能。
        const float center = g_control_status.angle_rad;
        assert(ControlStartNextAssessment() == 1);
        const uint32_t position_start = now_ms;
        for (uint32_t i = 1; i <= 16000; ++i) {
            now_ms = position_start + i;
            const float phase = 6.28318530718f * i / 8000.0f;
            const float angle = center + std::sin(phase);
            int raw = static_cast<int>(std::lround(angle * 8192 / 6.28318530718f)) % 8192;
            if (raw < 0) raw += 8192;
            enqueue(static_cast<uint16_t>(raw), 0x204U + CONTROL_MOTOR_ID, 8,
                    static_cast<int16_t>(0.78539816f * std::cos(phase) * 60 / 6.28318530718f));
            tick(now_ms);
        }
        expectVoltage(0);
        assert(g_assessment_status.completed == 1 && g_assessment_status.rms_error < 0.002f);
        assert(g_assessment_status.next_mode == CONTROL_OUTPUT_SPEED_SINE);
        assert(ControlStartNextAssessment() == 1);
        freshTick(now_ms + 1); ControlStopOutput(); freshTick(now_ms + 1);
        assert(g_assessment_status.aborted == 1 && g_assessment_status.completed == 0);
        assert(ControlStartNextAssessment() == 1);
        freshTick(now_ms + 1); tick(now_ms + CONTROL_FEEDBACK_TIMEOUT_MS);
        assert(g_control_status.output_fault & CONTROL_FAULT_FEEDBACK);
        assert(g_assessment_status.aborted == 1);
        freshTick(now_ms + 1); assert(ControlClearOutputFault() == 1);
        assert(ControlStartNextAssessment() == 1);
        for (unsigned i = 0; i < 20; ++i) {
            now_ms += 1; enqueue(6000, 0x204U + CONTROL_MOTOR_ID, 8, 150); tick(now_ms);
        }
        assert(g_control_status.output_fault & CONTROL_FAULT_OVERSPEED);
        expectVoltage(0);
        freshTick(now_ms + 1); assert(ControlClearOutputFault() == 1);
        assert(ControlStartNextAssessment() == 1);
        now_ms += 1; enqueue(6000); g_control_status.temperature_c = 85; tick(now_ms);
        assert(g_control_status.output_fault & CONTROL_FAULT_TEMPERATURE);
        expectVoltage(0);
        freshTick(now_ms + 1); assert(ControlClearOutputFault() == 1);
        assert(ControlSelectAssessment(CONTROL_OUTPUT_POSITION_SINE) == 1);
        assert(ControlStartNextAssessment() == 1);
        for (unsigned i = 0; i < 1200; ++i) freshTick(now_ms + 1);
        now_ms += 1; enqueue(1719, 0x204U + CONTROL_MOTOR_ID, 8, 0); tick(now_ms);
        assert(g_control_status.output_fault & CONTROL_FAULT_POSITION_ERROR);
        assert(g_assessment_status.aborted == 1);
        expectVoltage(0);
        freshTick(now_ms + 1); assert(ControlClearOutputFault() == 1);
        freshTick(0xFFFFFFF0U); assert(ControlSelectAssessment(CONTROL_OUTPUT_SPEED_SINE) == 1);
        assert(ControlStartNextAssessment() == 1);
        freshTick(5); assert(g_assessment_status.elapsed_ms == 21 && g_assessment_status.running == 1);
        ControlStopOutput(); freshTick(6); expectVoltage(0);
        std::puts("PASS: both assessment sine modes, bounded current, automatic selection, log, stop and protections");
    } else {
        assert(ControlStartAssessment(CONTROL_OUTPUT_SPEED_SINE) == 0);
    }
}
