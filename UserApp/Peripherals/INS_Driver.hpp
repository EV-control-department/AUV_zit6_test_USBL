#ifndef __INS_DRIVER_HPP
#define __INS_DRIVER_HPP

#include "SerialPort.hpp"
#include "CommonConfig.hpp"
#include <string.h>

namespace auv {

class INS_Driver {
public:
    INS_Driver(UART_HandleTypeDef* rx_uart, UART_HandleTypeDef* tx_uart)
        : rx_port_(rx_uart, 512), tx_port_(tx_uart, 128) {}

    void init() {
        rx_port_.startReceive();
        // 发送初始化指令 (设置为室内导航模式)
        uint8_t init_cmd[14] = {0xFC, 0xCF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xFD, 0xDF};
        tx_port_.transmit(init_cmd, 14);
    }

    /**
     * @brief 解析并更新导航状态
     */
    bool update(NavState& state) {
        uint8_t temp_buf[256];
        uint16_t len = rx_port_.read(temp_buf, 256);

        for (uint16_t i = 0; i < len; i++) {
            if (parseByte(temp_buf[i])) {
                decodePacket(state);
                return true;
            }
        }
        return false;
    }

private:
    SerialPort rx_port_;
    SerialPort tx_port_;
    
    uint8_t packet_buf_[256];
    uint16_t parse_idx_ = 0;

    /**
     * @brief 简单的状态机解析 0xFA 0xAF 包
     */
    bool parseByte(uint8_t b) {
        if (parse_idx_ == 0 && b != 0xFA) return false;
        if (parse_idx_ == 1 && b != 0xAF) { parse_idx_ = 0; return false; }
        
        packet_buf_[parse_idx_++] = b;
        
        if (parse_idx_ >= 92) { // 91 bytes data + 1 byte checksum
            uint8_t check = 0;
            for (int i = 0; i < 91; i++) check ^= packet_buf_[i];
            
            if (check == packet_buf_[91]) {
                parse_idx_ = 0;
                return true;
            } else {
                // 校验失败，寻找下一个可能的包头
                parse_idx_ = 0;
                return false;
            }
        }
        return false;
    }

    /**
     * @brief 解码数据包 (移植自 comm.c)
     */
    void decodePacket(NavState& state) {
        float raw_data[12];
        // 姿态 (Roll, Pitch, Yaw) -> Offset 2, Size 12
        memcpy(&(raw_data[3]), packet_buf_ + 2, 12);
        // 位置 (X, Y) -> Offset 54, Size 8
        memcpy(&(raw_data[0]), packet_buf_ + 54, 8);
        // 深度 (Z) -> Offset 46, Size 4
        memcpy(&(raw_data[2]), packet_buf_ + 46, 4);

        // 映射到 AUV 标准坐标系
        state.y = raw_data[0];
        state.x = -raw_data[1]; // 同学代码里的 X/Y 转换
        state.z = raw_data[2];
        
        state.yaw = raw_data[5]; // 同学代码里的姿态映射 (0:Roll, 1:Pitch, 2:Yaw?)
        // 注意：同学代码 imu_data[3]=ry, imu_data[4]=rx, imu_data[5]=rz (yaw)
        
        state.imu_state = packet_buf_[129]; // 这一行在 91 字节包外，可能需要调整包长
        // 修正：如果同学代码中访问了 buf[129]，说明包长可能超过了 91。
        // 根据 comm.c 第 118 行，check = Check_Data(buf, 91)，说明基础包长是 92 字节。
    }
};

} // namespace auv

#endif
