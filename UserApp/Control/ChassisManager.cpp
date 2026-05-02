#include "ChassisManager.hpp"
#include "main.h"
#include <algorithm>

namespace auv {
namespace control {

ChassisManager::ChassisManager() {
  for (int i = 0; i < 4; i++) {
    profiles_[i].setLimits(0.5f, 0.2f);
    PID_Controller::Config pos_cfg;
    pos_cfg.kp = 1.0f;
    pos_pids_[i].setConfig(pos_cfg);
    PID_Controller::Config vel_cfg;
    vel_cfg.kp = 2.0f;
    vel_cfg.ki = 0.5f;
    vel_cfg.kd = 0.1f;
    vel_cfg.dt = 0.01f;
    vel_pids_[i].setConfig(vel_cfg);
  }
}

void ChassisManager::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  if (axis < 0 || axis >= 4)
    return;
  PID_Controller::Config cfg;
  cfg.kp = kp;
  cfg.ki = ki;
  cfg.kd = kd;
  cfg.i_limit = i_limit;
  cfg.output_limit = out_limit;
  cfg.dt = 0.01f;
  if (is_pos_ring)
    pos_pids_[axis].setConfig(cfg);
  else
    vel_pids_[axis].setConfig(cfg);
}

void ChassisManager::setControlLevel(ControlLevel new_level,
                                     const float actual_p[4],
                                     const float actual_v[4]) {
  if (new_level == level_)
    return;
  // 切换到 POSITION：需要将影子平滑器与真实传感器对齐，且对 Z 轴采用积分继承策略
  if (new_level == ControlLevel::POSITION) {
    for (int i = 0; i < 4; i++) {
      profiles_[i].align(actual_p[i], actual_v[i]);
      if (i == 2) {
        // 对 Z 轴采用无扰动继承：将上一周期输出作为积分初值（由 PID 内部限幅）
        vel_pids_[i].setIntegral(last_output_forces_[i]);
      } else {
        vel_pids_[i].reset();
      }
    }
  }

  // 切换到 VELOCITY：对齐影子状态并清除速度环积分
  else if (new_level == ControlLevel::VELOCITY) {
    for (int i = 0; i < 4; i++) {
      profiles_[i].align(actual_p[i], actual_v[i]);
      vel_pids_[i].reset();
    }
  }

  // 切换到 ACTUATOR：只改变层级，不清空 target_forces_
  else if (new_level == ControlLevel::ACTUATOR) {
    // 保持 target_forces_ 不变，使外部直接写入的力能立即生效
  }

  // 切换到 NONE（安全停机）：清空目标推力
  else if (new_level == ControlLevel::NONE) {
    target_forces_.fill(0.0f);
  }

  level_ = new_level;
}

std::array<float, 4> ChassisManager::update(const float actual_p[4],
                                            const float actual_v[4],
                                            const float target_p[4]) {
  std::array<float, 4> output_forces = {0};
  uint32_t now = HAL_GetTick();
  float dt = (last_update_tick_ == 0)
                 ? 0.01f
                 : (float)(now - last_update_tick_) / 1000.0f;
  last_update_tick_ = now;

  // 防止 dt 异常：限定在 [1ms, 100ms] 范围
  if (dt > 0.1f) dt = 0.1f;
  if (dt <= 0.0f) dt = 0.001f;

  if (level_ == ControlLevel::NONE)
    return output_forces;

  for (int i = 0; i < 4; i++) {
    ProfileState d = profiles_[i].update(target_p[i], dt);
    float v_ref = 0.0f;
    if (level_ == ControlLevel::POSITION) {
      v_ref = pos_pids_[i].compute(d.p - actual_p[i], dt) + d.v;
    } else {
      v_ref = d.v;
    }

    float f_base = 0.0f;
    if (level_ == ControlLevel::POSITION || level_ == ControlLevel::VELOCITY) {
      float accel_ff = 1.0f * d.a;
      f_base = vel_pids_[i].compute(v_ref - actual_v[i], dt) + accel_ff;
    }

    output_forces[i] = f_base + target_forces_[i];
  }

  last_z_thrust_ = output_forces[2];
  last_output_forces_ = output_forces; // 更新快照
  return output_forces;
}

} // namespace control
} // namespace auv
