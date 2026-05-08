#pragma once
#include <array>
#include "SystemConfig.hpp"

namespace auv {
namespace control {

/**
 * @class AuvSimulator
 * @brief 运行在单片机内部的物理引擎 (HITL 模式)
 * 旁路传感器数据，而代之以数学模型。
 */
class AuvSimulator {
public:
    AuvSimulator(float dt = 0.01f) : dt_(dt) {}

    /**
     * @brief 推进物理引擎一步
     * @param forces 归一化推力 [-1.0, 1.0]
     * @param mass 质量
     * @param drag 线性阻力系数
     * @param k 推力增益 (1.0 推力对应多少牛顿)
     */
    void step(const std::array<float, 4>& forces, float mass, float drag, float k) {
        if (mass <= 0.01f) mass = 20.0f; // 安全冗余

        for (int i = 0; i < 4; i++) {
            // F_net = F_thrust - F_drag
            float push = forces[i] * k;
            float resist = velocity_[i] * drag;
            float accel = (push - resist) / mass;

            // 欧拉积分
            velocity_[i] += accel * dt_;
            position_[i] += velocity_[i] * dt_;
        }
    }

    const std::array<float, 4>& getPosition() const { return position_; }
    const std::array<float, 4>& getVelocity() const { return velocity_; }
    
    void reset(const float p[4]) {
        for(int n=0; n<4; n++) { 
            position_[n] = p[n]; 
            velocity_[n] = 0.0f; 
        }
    }

private:
    float dt_;
    std::array<float, 4> position_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 4> velocity_{0.0f, 0.0f, 0.0f, 0.0f};
};

} // namespace control
} // namespace auv
