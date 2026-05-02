#include "INS_Driver.hpp"
#include <cmath>

namespace auv {

void INS_Driver::init() {
    rx_port_.startReceive();
    // 初始化：位置增量清零
    resetPosition();
}

void INS_Driver::sendCommand(uint8_t cmd_id, uint8_t value) {
    uint8_t cmd[14] = {0xFC, 0xCF, cmd_id, value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0xDF};
    cmd[11] = checkData(cmd, 11);
    HAL_UART_Transmit(tx_uart_, cmd, 14, 20);
}

void INS_Driver::resetPosition() {
    sendCommand(0x02, 0x00);
}

void INS_Driver::setDvlPower(bool on) {
    sendCommand(0x03, on ? 0x01 : 0x00);
}

void INS_Driver::restart() {
    sendCommand(0x04, 0x00);
}

bool INS_Driver::update(NavState& state) {
    uint8_t temp_buf[256];
    uint16_t len = rx_port_.read(temp_buf, 256);

    if (len > 0) {
        for (uint16_t i = 0; i < len; i++) {
            if (parseByte(temp_buf[i]) && validateFrame()) {
                decodePacket(state);
                state = state_; // 同步到传入的引用
                return true;
            }
        }
    }
    return false;
}

uint8_t INS_Driver::checkData(const uint8_t* data, uint8_t size) {
    uint8_t v = data[0];
    for (uint8_t i = 1; i < size; ++i) {
        v ^= data[i];
    }
    return v;
}

bool INS_Driver::parseByte(uint8_t b) {
    if (frame_len_ == 0) {
        if (b == 0xFA) {
            packet_buf_[frame_len_++] = b;
        }
        return false;
    }
    
    if (frame_len_ == 1) {
        if (b == 0xAF) {
            packet_buf_[frame_len_++] = b;
        } else if (b == 0xFA) {
            packet_buf_[0] = 0xFA;
            frame_len_ = 1;
        } else {
            frame_len_ = 0;
        }
        return false;
    }
    
    if (packet_buf_[frame_len_ - 1] == 0xFA && b == 0xAF) {
        packet_buf_[0] = 0xFA;
        packet_buf_[1] = 0xAF;
        frame_len_ = 2;
        return false;
    }
    
    if (frame_len_ >= kMaxFrameSize) {
        frame_len_ = 0;
        return false;
    }
    
    packet_buf_[frame_len_++] = b;
    
    return frame_len_ >= 2 &&
           packet_buf_[frame_len_ - 2] == 0xFB &&
           packet_buf_[frame_len_ - 1] == 0xBF;
}

bool INS_Driver::validateFrame() {
    if (frame_len_ < kMinFrameSize) {
        frame_len_ = 0;
        return false;
    }
    if (packet_buf_[0] != 0xFA || packet_buf_[1] != 0xAF) {
        frame_len_ = 0;
        return false;
    }
    if (checkData(packet_buf_, 130) != packet_buf_[130]) {
        frame_len_ = 0;
        return false;
    }
    return true;
}

void INS_Driver::decodePacket(NavState& s) {
    const float kDeg2Rad = 0.0174532925f;

    // 1. 姿态与角速度 (单位转换：度 -> 弧度)
    float yaw_deg, wz_deg;
    memcpy(&yaw_deg, packet_buf_ + 10, 4);
    memcpy(&wz_deg, packet_buf_ + 22, 4);
    
    state_.yaw = yaw_deg * kDeg2Rad;
    state_.vyaw = wz_deg * kDeg2Rad;

    // 2. 机体系线速度 (直接从协议读取，无需微分)
    // Offset 26: Vx (Surge), 30: Vy (Sway), 34: Vz (Heave)
    memcpy(&state_.vx, packet_buf_ + 26, 4);
    memcpy(&state_.vy, packet_buf_ + 30, 4);
    memcpy(&state_.vz, packet_buf_ + 34, 4);

    // 3. 位置信息
    // Offset 103: 北向增量 (North -> X), 107: 东向增量 (East -> Y), 46: 深度 (Depth -> Z)
    memcpy(&state_.x, packet_buf_ + 103, 4);
    memcpy(&state_.y, packet_buf_ + 107, 4);
    memcpy(&state_.z, packet_buf_ + 46, 4);

    // 4. 状态位
    // Offset 129: 导航模式 (imu_state)
    state_.imu_state = packet_buf_[129];
    
    // Offset 115: 传感器状态。Bit 1 为 DVL 有效位 (Valid)
    state_.dvl_state = (packet_buf_[115] & 0x02) ? 1 : 0;
    
    state_.timestamp = HAL_GetTick();

    s = state_; // 同步输出
    frame_len_ = 0;
}

} // namespace auv
