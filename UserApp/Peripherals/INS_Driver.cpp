#include "INS_Driver.hpp"
#include <cmath>

namespace auv {

void INS_Driver::init() {
    rx_port_.startReceive();
    resetPosition();
}

void INS_Driver::sendCommand(uint8_t cmd_id, uint8_t value) {
    uint8_t data[8] = {value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCommand(cmd_id, data, 8);
}

void INS_Driver::sendCommand(uint8_t cmd_id, const uint8_t* data, uint8_t data_len) {
    uint8_t cmd[14] = {0xFC, 0xCF, cmd_id};
    
    // 拷贝负载数据（最多8字节）
    uint8_t copy_len = (data_len > 8) ? 8 : data_len;
    if (data != nullptr && copy_len > 0) {
        memcpy(&cmd[3], data, copy_len);
    }
    
    // 填充剩余字节为 0
    for (uint8_t i = 3 + copy_len; i < 11; ++i) {
        cmd[i] = 0x00;
    }
    
    // 帧尾
    cmd[12] = 0xFD;
    cmd[13] = 0xDF;

    // 计算 XOR 校验和 (从 byte 0 到 byte 10)
    uint8_t v = 0;
    for (uint8_t i = 0; i < 11; ++i) {
        v ^= cmd[i];
    }
    cmd[11] = v;
    
    // 发送到 tx_uart_ (尝试发送3次以确保可靠性)
    for (int i = 0; i < 3; i++) {
        HAL_UART_Transmit(tx_uart_, cmd, 14, 50);
        if (i < 2) HAL_Delay(10); 
    }
}

void INS_Driver::resetPosition() {
    sendCommand(0x02, 0x00);
}

void INS_Driver::setDvlPower(bool on) {
    // 手册：03 为 DVL 电源控制位, 01 为开, 00 为关
    sendCommand(0x03, on ? 0x01 : 0x00);
}

void INS_Driver::restart() {
    // 手册：04 为重启指令
    sendCommand(0x04, 0x00);
}

void INS_Driver::setInitialPosition(double lat, double lon) {
    // 手册：纬度/经度 * 10^7, 强制转 int, 高位在前 (Big Endian)
    int32_t lat_fixed = (int32_t)(lat * 1e7);
    int32_t lon_fixed = (int32_t)(lon * 1e7);
    
    uint8_t data[8];
    // 纬度 (Big Endian)
    data[0] = (uint8_t)((lat_fixed >> 24) & 0xFF);
    data[1] = (uint8_t)((lat_fixed >> 16) & 0xFF);
    data[2] = (uint8_t)((lat_fixed >> 8) & 0xFF);
    data[3] = (uint8_t)(lat_fixed & 0xFF);
    
    // 经度 (Big Endian)
    data[4] = (uint8_t)((lon_fixed >> 24) & 0xFF);
    data[5] = (uint8_t)((lon_fixed >> 16) & 0xFF);
    data[6] = (uint8_t)((lon_fixed >> 8) & 0xFF);
    data[7] = (uint8_t)(lon_fixed & 0xFF);
    
    sendCommand(0x20, data, 8);
}

bool INS_Driver::update(NavState& state) {
    uint8_t temp_buf[256];
    uint16_t len = rx_port_.read(temp_buf, 256);
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
    if (frame_len_ == 0 && b != 0xFA) return false;
    if (frame_len_ == 1 && b != 0xAF) {
        frame_len_ = 0;
        return false;
    }
    
    if (frame_len_ < kMaxFrameSize) {
        packet_buf_[frame_len_++] = b;
    } else {
        frame_len_ = 0;
        return false;
    }
    
    // UNAV-IP 系列 V2.0 协议：总长度 133 字节
    return frame_len_ == 133;
}

bool INS_Driver::validateFrame() {
    if (frame_len_ != 133) {
        frame_len_ = 0;
        return false;
    }
    
    // 异或校验位在 130，校验范围 0-129
    uint8_t v = 0;
    for (int i = 0; i < 130; i++) {
        v ^= packet_buf_[i];
    }
    
    if (v == packet_buf_[130]) {
        // 检查帧尾 0xFB 0xBF
        if (packet_buf_[131] == 0xFB && packet_buf_[132] == 0xBF) {
            return true;
        }
    }
    
    frame_len_ = 0;
    return false;
}

void INS_Driver::decodePacket(NavState& s) {
    const float kDeg2Rad = 0.0174532925f;

    // 1. 姿态 (Offset 2, 6, 10 为 Roll, Pitch, Yaw, 类型为 float, 单位 deg)
    float roll_deg, pitch_deg, yaw_deg;
    memcpy(&roll_deg, packet_buf_ + 2, 4);
    memcpy(&pitch_deg, packet_buf_ + 6, 4);
    memcpy(&yaw_deg, packet_buf_ + 10, 4);
    
    state_.roll = roll_deg * kDeg2Rad;
    state_.pitch = pitch_deg * kDeg2Rad;
    state_.yaw = yaw_deg * kDeg2Rad;

    // 1.1 角速度 (Offset 14, 18, 22 为 Gyro X, Y, Z, 类型为 float, 单位 deg/s)
    float gx, gy, gz;
    memcpy(&gx, packet_buf_ + 14, 4);
    memcpy(&gy, packet_buf_ + 18, 4);
    memcpy(&gz, packet_buf_ + 22, 4);
    state_.vroll = gx * kDeg2Rad;
    state_.vpitch = gy * kDeg2Rad;
    state_.vyaw = gz * kDeg2Rad;

    // 2. 机体系线速度 (Offset 26, 30, 34 为 Vx, Vy, Vz, 类型为 float, 单位 m/s)
    memcpy(&state_.vx, packet_buf_ + 26, 4);
    memcpy(&state_.vy, packet_buf_ + 30, 4);
    memcpy(&state_.vz, packet_buf_ + 34, 4);

    // 3. 经纬度信息 (Offset 38, 42, int32, 单位 1e-7 deg)
    int32_t lat_int, lon_int;
    memcpy(&lat_int, packet_buf_ + 38, 4);
    memcpy(&lon_int, packet_buf_ + 42, 4);
    
    state_.lat = lat_int * 1e-7;
    state_.lon = lon_int * 1e-7;

    // 4. 深度 (Offset 46, float, 单位 m)
    // 注意：手册中 Offset 46 是 SINS 深度，Offset 107 是压力计深度
    // 通常使用压力计深度更为准确
    memcpy(&state_.z, packet_buf_ + 107, 4);

    // 5. 状态位与模式
    state_.imu_state = packet_buf_[129]; // 导航模式 (0:Init, 1:SINS, 2:SINS+GPS, 3:SINS+DVL, 4:SINS+GPS+DVL)
    state_.dvl_state = (packet_buf_[115] & 0x02) ? 1 : 0; // bit 1 为 DVL 有效位
    
    state_.timestamp = HAL_GetTick();
    s = state_; 
    frame_len_ = 0; 
}

} // namespace auv
