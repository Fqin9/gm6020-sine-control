#include "AssessmentController.h"
#include "ControlConfig.h"
#include <cmath>

namespace {
static_assert(CONTROL_ASSESSMENT_SPEED_FILTER_ALPHA > 0 && CONTROL_ASSESSMENT_SPEED_FILTER_ALPHA <= 1,
              "Speed filter alpha must be in (0, 1]");
static_assert(CONTROL_ASSESSMENT_SPEED_KD >= 0, "Derivative gain must be nonnegative");
static_assert(CONTROL_ASSESSMENT_SPEED_D_FILTER_MS > 0, "Derivative filtering must be enabled");
static_assert(CONTROL_ASSESSMENT_SPEED_D_LIMIT > 0 &&
              CONTROL_ASSESSMENT_SPEED_D_LIMIT <= CONTROL_ASSESSMENT_CURRENT_LIMIT,
              "Derivative command must stay within the total current limit");
static_assert(CONTROL_ASSESSMENT_ANGLE_FF_ENABLED == 0 || CONTROL_ASSESSMENT_ANGLE_FF_ENABLED == 1,
              "Angle feedforward switch must be 0 or 1");
static_assert(CONTROL_ASSESSMENT_ANGLE_FF_LIMIT > 0 &&
              CONTROL_ASSESSMENT_ANGLE_FF_LIMIT <= CONTROL_ASSESSMENT_CURRENT_LIMIT,
              "Angle feedforward must stay within the current command limit");
float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
constexpr float kTwoPi = 6.2831853071795864769f;
}

float AssessmentAngleFeedforward(float angle, float speed_reference) {
#if CONTROL_ASSESSMENT_ANGLE_FF_ENABLED
    if (!std::isfinite(angle) || !std::isfinite(speed_reference)) return 0;
    const float phase = std::remainder(angle, kTwoPi);
    const float raw = CONTROL_ASSESSMENT_ANGLE_FF_SIN10 * std::sin(10.0f * phase) +
                      CONTROL_ASSESSMENT_ANGLE_FF_COS10 * std::cos(10.0f * phase) +
                      CONTROL_ASSESSMENT_ANGLE_FF_SIN20 * std::sin(20.0f * phase) +
                      CONTROL_ASSESSMENT_ANGLE_FF_COS20 * std::cos(20.0f * phase);
    // 保持扰动补偿与旋转方向无关；参考接近零时淡出。
    return clamp(raw, -CONTROL_ASSESSMENT_ANGLE_FF_LIMIT, CONTROL_ASSESSMENT_ANGLE_FF_LIMIT) *
           clamp(std::fabs(speed_reference) / 0.3f, 0.0f, 1.0f);
#else
    (void)angle; (void)speed_reference;
    return 0;
#endif
}

void AssessmentController::reset(float speed) {
    integral_ = breakaway_ = 0;
    stalled_ticks_ = 0;
    filtered_speed_ = speed;
    filtered_acceleration_ = 0;
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
    // 首轮实测显示快速速度波动，缩短反馈滤波延迟，避免继续增加相位滞后。
    const float previous_filtered_speed = filtered_speed_;
    filtered_speed_ += CONTROL_ASSESSMENT_SPEED_FILTER_ALPHA * (speed - filtered_speed_);
    r.filtered_speed = filtered_speed_;
    if (CONTROL_ASSESSMENT_SPEED_KD > 0.0f) {
        // 只对实测速度微分，目标变化不会直接引起微分冲击。
        // 后向欧拉一阶滤波：alpha=dt/(tau+dt)，dt=1ms。
        const float acceleration = (filtered_speed_ - previous_filtered_speed) * 1000.0f;
        filtered_acceleration_ += (acceleration - filtered_acceleration_) /
                                  (1.0f + CONTROL_ASSESSMENT_SPEED_D_FILTER_MS);
        r.derivative_output = clamp(-CONTROL_ASSESSMENT_SPEED_KD * filtered_acceleration_,
                                   -CONTROL_ASSESSMENT_SPEED_D_LIMIT, CONTROL_ASSESSMENT_SPEED_D_LIMIT);
    }
    const float error = r.speed_reference - filtered_speed_;
    const float sign = clamp(r.speed_reference / 0.3f, -1.0f, 1.0f);
    // 卡在低速位置时逐渐增加起动补偿，转动后平滑撤去，避免长期固定踢转。
    const bool stalled = std::fabs(r.speed_reference) > 0.2f &&
                         std::fabs(filtered_speed_) < 0.15f && std::fabs(error) > 0.2f;
    stalled_ticks_ = stalled ? stalled_ticks_ + 1U : 0U;
    // 连续低速至少 100 ms 才补偿；经过零速或短暂的 RPM 量化零值不立即踢转。
    if (stalled_ticks_ >= CONTROL_ASSESSMENT_BREAKAWAY_DELAY_MS)
        breakaway_ = clamp(breakaway_ + 0.5f, 0, CONTROL_ASSESSMENT_BREAKAWAY_RAW);
    else breakaway_ *= 0.99f;
    const float base = CONTROL_ASSESSMENT_SPEED_KP * error +
                       sign * (CONTROL_ASSESSMENT_FRICTION_RAW + breakaway_) +
                       AssessmentAngleFeedforward(angle, r.speed_reference) + r.derivative_output;
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
