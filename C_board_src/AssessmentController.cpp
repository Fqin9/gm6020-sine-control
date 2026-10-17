#include "AssessmentController.h"
#include "ControlConfig.h"
#include <cmath>

namespace {
float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
constexpr float kTwoPi = 6.2831853071795864769f;
}

void AssessmentController::reset(float speed) {
    integral_ = breakaway_ = 0;
    filtered_speed_ = speed;
}

AssessmentOutput AssessmentController::step(bool position, uint32_t elapsed_ms,
                                           float center, float angle, float speed) {
    const float omega = kTwoPi * 1000.0f / CONTROL_ASSESSMENT_PERIOD_MS;
    const float phase = omega * (elapsed_ms % CONTROL_ASSESSMENT_PERIOD_MS) * 0.001f;
    AssessmentOutput r = {};
    r.target_angle = center + CONTROL_ASSESSMENT_POSITION_AMPLITUDE * std::sin(phase);
    r.target_speed = position
        ? CONTROL_ASSESSMENT_POSITION_AMPLITUDE * omega * std::cos(phase)
        : CONTROL_ASSESSMENT_SPEED_AMPLITUDE * std::sin(phase);
    r.speed_reference = position
        ? clamp(r.target_speed + CONTROL_ASSESSMENT_POSITION_KP * (r.target_angle - angle),
                -CONTROL_ASSESSMENT_SPEED_LIMIT, CONTROL_ASSESSMENT_SPEED_LIMIT)
        : r.target_speed;
    filtered_speed_ += 0.1f * (speed - filtered_speed_); // 约 10 ms，减小整 rpm 量化影响。
    r.filtered_speed = filtered_speed_;
    const float error = r.speed_reference - filtered_speed_;
    const float sign = clamp(r.speed_reference / 0.3f, -1.0f, 1.0f);
    // 卡在低速位置时逐渐增加起动补偿，转动后平滑撤去，避免长期固定踢转。
    const bool stalled = std::fabs(r.speed_reference) > 0.2f &&
                         std::fabs(filtered_speed_) < 0.15f && std::fabs(error) > 0.2f;
    if (stalled) breakaway_ = clamp(breakaway_ + 2.0f, 0, CONTROL_ASSESSMENT_BREAKAWAY_RAW);
    else breakaway_ *= 0.98f;
    const float base = CONTROL_ASSESSMENT_SPEED_KP * error +
                       sign * (CONTROL_ASSESSMENT_FRICTION_RAW + breakaway_);
    const float limit = CONTROL_ASSESSMENT_CURRENT_LIMIT;
    const float candidate = clamp(integral_ + CONTROL_ASSESSMENT_SPEED_KI * error * 0.001f,
                                  -limit, limit);
    const float tentative = base + candidate;
    // 允许使输出退出饱和的积分方向，抑制继续推向饱和的积分。
    if (!((tentative > limit && error > 0) || (tentative < -limit && error < 0)))
        integral_ = candidate;
    r.output = clamp(base + integral_, -limit, limit);
    return r;
}
