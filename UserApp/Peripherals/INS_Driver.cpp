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
    NavState prev = state_;
    float raw_data[12];
    memcpy(&(raw_data[3]), packet_buf_ + 2, 12);   
    memcpy(&(raw_data[0]), packet_buf_ + 54, 8);   
    memcpy(&(raw_data[2]), packet_buf_ + 62, 4);   

    state_.y = raw_data[0];   
    state_.x = -raw_data[1];  
    state_.z = raw_data[2];   
    state_.yaw = raw_data[5]; 

    state_.imu_state = packet_buf_[129];       
    state_.dvl_state = packet_buf_[115] >> 7;  
    state_.timestamp = HAL_GetTick();

    if (has_prev_state_) {
        uint32_t dt_ms = state_.timestamp - prev.timestamp;
        if (dt_ms > 0) {
            float dt = static_cast<float>(dt_ms) * 0.001f;
            float world_dx = state_.x - prev.x;
            float world_dy = state_.y - prev.y;
            float world_dz = state_.z - prev.z;
            float dyaw = state_.yaw - prev.yaw;

            const float kPi = 3.1415926535f;
            const float kTwoPi = 6.283185307f;
            while (dyaw > kPi) dyaw -= kTwoPi;
            while (dyaw < -kPi) dyaw += kTwoPi;

            float cos_y = std::cos(state_.yaw);
            float sin_y = std::sin(state_.yaw);
            state_.vx = (world_dx * cos_y + world_dy * sin_y) / dt;
            state_.vy = (-world_dx * sin_y + world_dy * cos_y) / dt;
            state_.vz = world_dz / dt;
            state_.vyaw = dyaw / dt;
        }
    } else {
        state_.vx = 0.0f;
        state_.vy = 0.0f;
        state_.vz = 0.0f;
        state_.vyaw = 0.0f;
        has_prev_state_ = true;
    }

    prev_state_ = state_;

    s = state_; // 同步
    frame_len_ = 0;
}

} // namespace auv
