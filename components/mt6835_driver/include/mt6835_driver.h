//
// MT6835 SPI 磁编码器驱动
// 21-bit 绝对角度 (截取高 16-bit 使用), 连读模式, CRC8 校验
//

#ifndef FOC_MT6835_DRIVER_H
#define FOC_MT6835_DRIVER_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "hal/spi_ll.h"
#include "soc/gpio_struct.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

#define SPI_MT6835_RESOLUTION 65536       // 21-bit 截取高 16-bit
#define MT6835_INTERNAL_DELAY_S 0.00001f  // MT6835 固有延迟 ~10µs

class MT6835 {
public:
    /// 故障回调函数类型 (连续读取失败超过阈值时触发)
    using FaultCallback = void (*)(void *user_ctx);

    /**
     * @brief 初始化 SPI 总线和 MT6835 设备
     */
    MT6835();

    /**
     * @brief 注册故障回调（连续读取失败 ≥20 次时触发）
     *
     * 由 FOC Driver 在构造时注册，编码器掉线时直接回调强制停机，
     * 无需等到 FOC 层面检测错误。
     *
     * @param cb 回调函数指针
     * @param user_ctx 传给回调的用户上下文（通常是 FocMotor*）
     */
    void register_fault_callback(FaultCallback cb, void *user_ctx) {
        fault_cb_ = cb;
        fault_cb_ctx_ = user_ctx;
    }

    /**
     * @brief 直接读取机械角度（弧度），不更新速度（用于校准）
     * @return 机械角度 [0, 2π)。读取失败返回 NAN。
     */
    float IRAM_ATTR read_angle_no_update();

    /**
     * @brief 读取机械角度（弧度），并更新速度和低通滤波
     * @return 机械角度 [0, 2π)。读取失败返回 NAN。
     */
    float IRAM_ATTR read_angle_raw();

    /**
     * @brief 读取延迟补偿后的电角度（弧度）
     *
     * 补偿算法:
     *   补偿后角度 = 读取角度 + (编码器延迟 10µs + MCPWM 周期 × 0.5T) × 滤波角速度
     *   电角度 = 补偿后角度 × 极对数 × 方向 - 零电角度偏移
     *
     * @param pole_pairs   电机极对数
     * @param direction    旋转方向 (1.0 或 -1.0)
     * @param zero_offset  零电角度偏移（弧度）
     * @return 补偿后的电角度（弧度），如果读取失败则返回 NAN
     */
    float IRAM_ATTR read_electrical_angle_compensated(int pole_pairs, float direction, float zero_offset);

    /**
     * @brief 获取低通滤波后的角速度（弧度/秒）
     */
    [[nodiscard]] float IRAM_ATTR get_velocity_filtered() const { return velocity_filtered_; }

    /**
     * @brief 检查弱磁报警标志 (STATUS[1])
     */
    [[nodiscard]] bool IRAM_ATTR has_magnet_warning() const { return no_mag_warning_; }

    /**
     * @brief 检查过速报警标志 (STATUS[0])，超过 12 万转/分钟
     */
    [[nodiscard]] bool IRAM_ATTR has_over_speed_warning() const { return over_speed_warning_; }

    /**
     * @brief 检查芯片供电欠压报警标志 (STATUS[2])
     */
    [[nodiscard]] bool IRAM_ATTR has_undervoltage_warning() const { return undervoltage_warning_; }

    /**
     * @brief 获取开机以来 CRC 校验错误的总次数
     *
     * 如果连续错误超过阈值（断线），则不再增加此计数。
     */
    [[nodiscard]] uint32_t IRAM_ATTR get_total_crc_errors() const { return total_crc_errors_; }

private:
    spi_device_handle_t spi_dev_{};
    gpio_num_t cs_pin_{}; // 手动控制的 CS 引脚
    spi_dev_t *spi_hw_{}; // SPI 硬件外设指针（直接寄存器操作）
    uint32_t cs_bit_mask_{}; // CS GPIO 位掩码（用于直接寄存器写）

    float prev_angle_{}; // 上一次的机械角度（弧度）
    float velocity_{}; // 即时角速度（弧度/秒）
    float velocity_filtered_{}; // 低通滤波后角速度
    bool no_mag_warning_{}; // 弱磁报警标志
    bool over_speed_warning_{}; // 过速报警标志
    bool undervoltage_warning_{}; // 欠压报警标志
    bool first_read_{true}; // 首次读取标志

    /// 连续读取失败计数器及故障回调
    uint32_t consecutive_errors_{0};
    uint32_t total_crc_errors_{0}; // 开机以来的 CRC 错误总数
    bool fault_triggered_{false};
    FaultCallback fault_cb_{nullptr};
    void *fault_cb_ctx_{nullptr};

    static constexpr uint32_t FAULT_THRESHOLD = 20;

    /**
     * @brief 连读命令读取角度寄存器 0x003-0x006
     *
     * 使用直接硬件寄存器操作，48-bit SPI 事务:
     *   MOSI: [1010_0000] [0000_0011] [00000000] [00000000] [00000000] [00000000]
     *          连读命令     起始地址     dummy       dummy       dummy       dummy
     *   MISO: [Hi-Z     ] [Hi-Z     ] [reg 0x03] [reg 0x04] [reg 0x05] [reg 0x06]
     *                                  ANGLE[20:13] ANGLE[12:5] ANGLE[4:0]|STATUS  CRC8
     *
     * CSN 下降沿锁存编码器内部角度数据。
     *
     * @param rx4 输出 4 字节: [reg03, reg04, reg05, reg06]
     */
    void IRAM_ATTR spi_burst_read(uint8_t *rx4);

    /**
     * @brief 读取 16-bit 角度值（21-bit 截取高 16 位）并校验 CRC8
     *
     * 流程: 连读 → CRC8 校验 → 状态检查 → 截取 ANGLE[20:5]
     * 连续失败 ≥20 次触发注册的故障回调。
     *
     * @param out_angle 成功时输出 16-bit 角度值 [0, 65535]
     * @return 成功返回 true，CRC 错误或弱磁时返回 false
     */
    bool IRAM_ATTR read_angle(uint16_t &out_angle);

    /**
     * @brief 更新角速度和低通滤波
     */
    void IRAM_ATTR update_velocity(float current_angle);

    /**
     * @brief CRC8 计算 (多项式 X⁸+X²+X+1 = 0x07)
     * 使用 DRAM 查找表，3 次表查询完成 24-bit 校验
     */
    static uint8_t IRAM_ATTR crc8_calc(const uint8_t *data, size_t len);
};

#endif // FOC_MT6835_DRIVER_H
