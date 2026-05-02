#include "INS_Driver.hpp"
#include <cmath>

namespace auv {

void INS_Driver::init() {
    rx_port_.startReceive();
    // 初始化：位置增量清零
    resetPosition();
}

void INS_Driver::sendCommand(uint8_t cmd_id, uint8_t value) {
    // 根据 UNAV-IP 指令协议 (FC CF ... FD DF)
    uint8_t cmd[14] = {0xFC, 0xCF, cmd_id, value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0xDF};
    
    // 指令校验（异或校验）
    uint8_t v = cmd[0];
    for (uint8_t i = 1; i < 11; ++i) {
        v ^= cmd[i];
    }
    cmd[11] = v;
    
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
    uint8_t temp_buf[128];
    uint16_t len = rx_port_.read(temp_buf, 128);
    bool has_new_frame = false;
    
    for (int i = 0; i < len; i++) {
        if (parseByte(temp_buf[i])) {
            if (validateFrame()) {
                decodePacket(state);
                has_new_frame = true;
            }
        }
    }
    return has_new_frame;
}

bool INS_Driver::parseByte(uint8_t b) {
    if (frame_len_ == 0 && b != 0x55) return false;
    if (frame_len_ == 1 && b != 0xAA) {
        frame_len_ = 0;
        return false;
    }
    
    if (frame_len_ < kMaxFrameSize) {
        packet_buf_[frame_len_++] = b;
    } else {
        frame_len_ = 0;
        return false;
    }
    
    // UNAV-IP 协议为固定 117 字节长度
    return frame_len_ == 117;
}

bool INS_Driver::validateFrame() {
    if (frame_len_ != 117) {
        frame_len_ = 0;
        return false;
    }
    
    // 校验规则：从第 3 字节 (index 2) 到第 115 字节 (index 114) 的累加和
    uint8_t ck1 = 0, ck2 = 0;
    for (int i = 2; i < 115; i++) {
        ck1 += packet_buf_[i];
        ck2 += ck1;
    }
    
    if (ck1 == packet_buf_[115] && ck2 == packet_buf_[116]) {
        return true;
    }
    
    frame_len_ = 0;
    return false;
}

void INS_Driver::decodePacket(NavState& s) {
    const float kDeg2Rad = 0.0174532925f;

    // 1. 姿态 (Offset 46, 50, 54 为 Roll, Pitch, Yaw, 类型为 float, 单位 deg)
    float roll_deg, pitch_deg, yaw_deg;
    memcpy(&roll_deg, packet_buf_ + 46, 4);
    memcpy(&pitch_deg, packet_buf_ + 50, 4);
    memcpy(&yaw_deg, packet_buf_ + 54, 4);
    
    state_.roll = roll_deg * kDeg2Rad;
    state_.pitch = pitch_deg * kDeg2Rad;
    state_.yaw = yaw_deg * kDeg2Rad;

    // 2. 机体系速度 (Offset 70, 74, 78 为 Vx, Vy, Vz, 类型为 float, 单位 m/s)
    memcpy(&state_.vx, packet_buf_ + 70, 4);
    memcpy(&state_.vy, packet_buf_ + 74, 4);
    memcpy(&state_.vz, packet_buf_ + 78, 4);

    // 3. 经纬度信息 (用于相对位置换算)
    // Offset 34: Lat, 38: Lon (int32, 单位 1e-7 deg)
    int32_t lat_int, lon_int;
    memcpy(&lat_int, packet_buf_ + 34, 4);
    memcpy(&lon_int, packet_buf_ + 38, 4);
    
    state_.lat = lat_int * 1e-7;
    state_.lon = lon_int * 1e-7;

    // 4. 状态位 (Offset 114 为系统状态)
    state_.imu_state = packet_buf_[114];
    
    state_.timestamp = HAL_GetTick();
    s = state_; 
    frame_len_ = 0; 
}

} // namespace auv
