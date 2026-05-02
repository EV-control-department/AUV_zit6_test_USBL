/**
 * @file INS_Driver.hpp
 * @brief NAV-300 集成惯导驱动类
 *
 * 职责：
 * 1. 通过串口（UART+DMA）接收并解析 NAV-300 的 100Hz 导航数据包。
 * 2. 按照“室外 GPS/SINS/DVL”模式协议提取位姿、速度及传感器状态。
 * 3. 向上层提供线程安全的 NavState 访问接口。
 * 4. 封装控制指令（DVL电源、位置清零、系统重启）。
 *
 * 协议参考 (UNAV-IP Series)：
 * - 帧头：0x55 0xAA (2 bytes)
 * - 长度：117 bytes (固定)
 * - 校验：双字节累加和 (CK1, CK2)
 */

#ifndef __INS_DRIVER_HPP
#define __INS_DRIVER_HPP

#include "SerialPort.hpp"
#include "CommonConfig.hpp"
#include <string.h>

namespace auv {

/**
 * @class INS_Driver
 * @brief 惯导驱动实现类
 */
class INS_Driver {
public:
    /**
     * @brief 构造函数
     * @param rx_uart 接收数据的 UART 句柄 (通常开启 DMA)
     * @param tx_uart 发送指令的 UART 句柄
     */
    INS_Driver(UART_HandleTypeDef* rx_uart, UART_HandleTypeDef* tx_uart)
        : rx_port_(rx_uart, 512), tx_uart_(tx_uart) {}

    /**
     * @brief 初始化驱动，启动非阻塞接收
     */
    void init();

    /**
     * @brief 发送控制指令给惯导硬件
     * @param cmd_id 指令ID（0x02:位置清零, 0x03:DVL电源, 0x04:重启）
     * @param value 指令参数
     */
    void sendCommand(uint8_t cmd_id, uint8_t value);

    /**
     * @brief 全局位置增量清零
     */
    void resetPosition();

    /**
     * @brief 控制 DVL 模块电源状态
     * @param on true 表示开启 DVL (0x01), false 表示关闭 (0x00)
     */
    void setDvlPower(bool on);

    /**
     * @brief 触发惯导系统重启
     */
    void restart();

    /**
     * @brief 驱动更新函数，建议在 100Hz+ 循环中调用
    *
     * 计算流程：
     * 1. 从环形缓冲区读取原始字节。
     * 2. 执行状态机解析（检测帧头、帧尾）。
     * 3. 校验合格后解码数据并更新内部 state_ 副本。
    *
     * @param[out] state 输出解码后的导航状态
     * @return true 表示成功解析到一个完整的新帧
     */
    bool update(NavState& state);

    /**
     * @brief 获取最新解析到的导航状态快照
     */
    NavState getNavState() const { return state_; }

private:
    SerialPort rx_port_;          ///< 串口接收驱动
    UART_HandleTypeDef* tx_uart_; ///< 指令发送串口句柄

    static constexpr uint16_t kMaxFrameSize = 128;
    static constexpr uint16_t kMinFrameSize = 117;
    uint8_t packet_buf_[kMaxFrameSize] = {0};      ///< 帧解析临时缓冲区
    uint16_t frame_len_ = 0;                       ///< 当前解析长度

    NavState state_{}; ///< 缓存的最新有效位姿状态
    NavState prev_state_{}; ///< 上一帧状态，用于速度差分估计
    bool has_prev_state_ = false;

    // 内部私有方法
    uint8_t checkData(const uint8_t* data, uint8_t size);
    bool parseByte(uint8_t b);
    bool validateFrame();
    void decodePacket(NavState& state);
};

} // namespace auv

#endif
