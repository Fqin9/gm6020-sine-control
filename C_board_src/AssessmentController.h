#ifndef ASSESSMENT_CONTROLLER_H
#define ASSESSMENT_CONTROLLER_H
#include <stdint.h>

struct AssessmentOutput {
    float target_speed, target_angle, speed_reference, filtered_speed, output;
};

// 转子绝对机械角度相关的电流前馈；零参考速度时淡出，不是位置目标偏置。
float AssessmentAngleFeedforward(float angle, float speed_reference);

// 1 ms 控制器：纯正弦轨迹，位置模式采用位置 P 外环、速度 PI 内环。
class AssessmentController {
public:
    void reset(float speed);
    AssessmentOutput step(bool position, uint32_t elapsed_ms, float center,
                          float angle, float speed);
private:
    float integral_ = 0, filtered_speed_ = 0, breakaway_ = 0;
    uint32_t stalled_ticks_ = 0;
};
#endif
