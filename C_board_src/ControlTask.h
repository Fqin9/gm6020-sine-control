#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>

// 供 C 中断处理(stm32f4xx_it.c 及 HAL 弱回调)调用的接口
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_WAITING_FEEDBACK = 0,
    CONTROL_FEEDBACK_ONLINE,
    CONTROL_FEEDBACK_TIMEOUT
} ControlState;

typedef enum {
    CONTROL_OUTPUT_STOPPED = 0,
    CONTROL_OUTPUT_TEST,
    CONTROL_OUTPUT_SPEED_PID,
    CONTROL_OUTPUT_SPEED_TEST,
    CONTROL_OUTPUT_SPEED_SINE,
    CONTROL_OUTPUT_POSITION_SINE
} ControlOutputMode;

typedef enum {
    CONTROL_FAULT_NONE = 0,
    CONTROL_FAULT_FEEDBACK = 1,
    CONTROL_FAULT_CAN_TX = 2,
    CONTROL_FAULT_INVALID_OUTPUT = 4,
    CONTROL_FAULT_OVERSPEED = 8,
    CONTROL_FAULT_POSITION_ERROR = 16,
    CONTROL_FAULT_TEMPERATURE = 32
} ControlOutputFault;

// 调试器只读观察这些字段；不要通过修改这些值控制电机。
typedef struct {
    uint32_t initialized;
    uint32_t control_ticks;
    uint32_t feedback_count;
    uint32_t last_feedback_ms;
    uint32_t feedback_session; // 首次连接或超时重连时递增
    uint32_t watchdog_feeds;
    uint32_t can_receive_errors;
    ControlState state;
    float angle_rad;
    float speed_rad_s;
    float temperature_c;
    uint32_t speed_phase_ms; // 当前正弦周期内的时间 [0, 周期)
    uint32_t speed_pid_active; // 本拍是否计算 PID，不代表已发送电机指令
    float speed_target_rad_s;
    float speed_feedback_used_rad_s; // 本拍计算用的反馈快照
    float speed_error_rad_s; // 未进入 PID 时为 0
    float speed_output; // 限幅后的 PID 计算结果，不等于已施加到电机
    ControlOutputMode output_mode;
    uint32_t output_fault; // 锁存故障位，停止/重连都不会自动清除
    int16_t voltage_command; // 本拍要求发送的协议值；不是实际电机电压
    uint32_t can_tx_queued; // 成功提交邮箱次数，不等于电机已执行
    uint32_t can_tx_errors;
    uint32_t can_tx_aborts;
    uint32_t command_mode; // 1 电流给定，0 电压给定
    int16_t output_command; // 协议原始给定；voltage_command 保留作为历史兼容字段
    // 本次限时测试数据，结束后保留，下一次成功启动时清零/重设。
    float test_start_angle_rad;
    float test_delta_angle_rad; // 测试结束时的净位移，不是总路程；不含结束后的滑行
    float test_peak_speed_rad_s; // 测试期间采样到的最大速度绝对值
    uint32_t test_nonzero_queued; // 非零帧提交次数，不等于电机执行/ACK 次数
    int16_t current_feedback_raw; // 最新电机电流反馈原始值，不是发送给定
    uint32_t test_current_samples; // 只累计测试时间窗内收到的反馈帧
    float test_mean_current_raw;
    uint32_t test_peak_abs_current_raw;
} ControlStatus;

extern volatile ControlStatus g_control_status;

typedef struct {
    float target_rad_s;
    float kp, ki, kd;
    float output_limit;
    uint32_t duration_ms;
    uint32_t samples;
    float mean_speed_rad_s;
    float final_speed_rad_s; // 最后一次测试采样，停止后保留
    uint32_t tail_samples;
    float tail_mean_speed_rad_s; // 计划结束前 200 ms 窗口；提前停止时可能没有样本
    float mean_abs_error_rad_s;
    float peak_abs_output;
    uint32_t saturated_samples;
    // 仅统计本次速度测试成功提交 CAN 邮箱的值，不代表总线 ACK 或实测电流。
    uint32_t queued_samples, positive_queued, negative_queued;
    int16_t min_queued, max_queued;
    float mean_queued;
} SpeedTestStatus;
extern volatile SpeedTestStatus g_speed_test;

// 每约 200 ms 保留一个成功提交的帧及当时反馈，停止后冻结。
typedef struct {
    uint32_t elapsed_ms;
    int16_t queued_command;
    float speed_rad_s;
    float delta_angle_rad;
    int16_t current_feedback_raw;
} SpeedTracePoint;
#define CONTROL_SPEED_TRACE_CAPACITY 15U
extern volatile SpeedTracePoint g_speed_trace[CONTROL_SPEED_TRACE_CAPACITY];
extern volatile uint32_t g_speed_trace_count;

typedef struct {
    uint32_t next_mode, mode, running, completed, aborted, elapsed_ms, duration_ms, samples;
    float target_speed_rad_s, actual_speed_rad_s, target_angle_rad, actual_angle_rad;
    float error, mean_abs_error, rms_error, max_abs_error;
    int16_t output_raw;
    uint32_t saturated_samples;
} AssessmentStatus;
extern volatile AssessmentStatus g_assessment_status;

typedef struct {
    uint32_t elapsed_ms;
    float target_speed, actual_speed, target_angle, actual_angle;
    int32_t output_raw;
} AssessmentLogPoint;
#define CONTROL_ASSESSMENT_LOG_CAPACITY 1001U
typedef struct {
    uint32_t version, count, mode, duration_ms;
    AssessmentLogPoint points[CONTROL_ASSESSMENT_LOG_CAPACITY];
} AssessmentLog;
extern volatile AssessmentLog g_assessment_log;

// 调试只读；标准数据帧计数包含其他 ID，不代表有效电机反馈。
typedef struct {
    uint32_t channel;
    uint32_t expected_id;
    uint32_t rx_frames;
    uint32_t last_id;
    uint32_t last_dlc;
    uint32_t last_rx_ms;
    uint32_t esr;
    uint32_t rx_error_count;
    uint32_t tx_error_count;
    uint32_t last_error_code;
    uint32_t bus_off;
} CanDiagnostics;
extern volatile CanDiagnostics g_can_diagnostics;

// 上电初始化: 配置 CAN 过滤器、启动 CAN 与 1kHz 控制定时器。
// CAN 发送受 ControlConfig.h 中的开关控制，上电默认停止。
// 在 main.c 中 MX_XXX_Init() 全部执行完后调用一次
void ControlTaskInit(void);

// 在 main.c 的 while (1) 内反复调用，检查定时任务心跳并喂狗。
void ControlTaskPoll(void);

// 只在主循环/前台调用，不在其他中断中调用。
// 返回 1 表示接受请求，0 表示未初始化、未开启发送、反馈不新鲜、
// 已在运动、故障未清除或参数无效。接受请求不代表电机已经运动。
// 测试值会限幅；duration_ms 必须在 1..CONTROL_TEST_MAX_DURATION_MS。
uint32_t ControlStartVoltageTest(int16_t voltage, uint32_t duration_ms);
// 使用当前 CONTROL_COMMAND_CURRENT 模式；旧 VoltageTest 名称为兼容包装。
uint32_t ControlStartOutputTest(int16_t raw, uint32_t duration_ms);
uint32_t ControlStartSpeedOutput(void);
// 恒速闭环限时试验，最长 CONTROL_SPEED_TEST_MAX_DURATION_MS，结束撤销输出。
uint32_t ControlStartSpeedTest(float target_rad_s, uint32_t duration_ms);
uint32_t ControlStartAssessment(uint32_t mode);
uint32_t ControlStartNextAssessment(void);
uint32_t ControlSelectAssessment(uint32_t mode);
// 立即撤销软件使能；下一次 1 ms 定时任务尝试撤销旧帧并发送零输出。
void ControlStopOutput(void);
// 停止且反馈新鲜、旧发送请求已结束时允许清除故障；不会启动运动。
uint32_t ControlClearOutputFault(void);

// 1kHz 控制入口, 在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用
void MainTask(void);

// 由本模块的 HAL CAN 回调在检查帧类型和长度后调用。
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // CONTROLTASK_H
