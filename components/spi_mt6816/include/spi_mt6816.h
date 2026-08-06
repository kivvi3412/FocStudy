//
// MT6816 SPI 磁编码器驱动
// 14-bit 绝对角度, 4线SPI协议
//

#ifndef FOC_SPI_MT6816_H
#define FOC_SPI_MT6816_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "hal/spi_ll.h"
#include "soc/gpio_struct.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

class MT6816 {
public:
    /**
     * @brief 初始化 SPI 总线和 MT6816 设备
     */
    MT6816();

    /**
     * @brief 直接读取机械角度（弧度），不更新速度（用于校准）
     * @return 机械角度 [0, 2π)
     */
    float IRAM_ATTR read_angle_no_update();

    /**
     * @brief 读取机械角度（弧度），并更新速度和低通滤波
     * @return 机械角度 [0, 2π)。如果读取失败返回 NAN。
     */
    float IRAM_ATTR read_angle_raw();

    /**
     * @brief 读取延迟补偿后的电角度（弧度）
     *
     * 补偿算法:
     *   补偿后角度 = 读取角度 + (编码器延迟 + MCPWM周期) × 滤波角速度
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
    [[nodiscard]] float IRAM_ATTR get_velocity_filtered() const {
        return velocity_filtered_;
    }

    /**
     * @brief 检查弱磁报警标志 (No_Mag_Warning)
     */
    [[nodiscard]] bool IRAM_ATTR has_magnet_warning() const { return no_mag_warning_; }

private:
    spi_device_handle_t spi_dev_{};
    gpio_num_t cs_pin_{};        // 手动控制的 CS 引脚
    spi_dev_t *spi_hw_{};        // SPI 硬件外设指针（直接寄存器操作）
    uint32_t cs_bit_mask_{};     // CS GPIO 位掩码（用于直接寄存器写）

    float prev_angle_{}; // 上一次的机械角度（弧度）
    int64_t prev_time_us_{}; // 上一次读取的时间戳（µs）
    float velocity_{}; // 即时角速度（弧度/秒）
    float velocity_filtered_{}; // 低通滤波后角速度
    bool no_mag_warning_{}; // 弱磁报警标志
    bool first_read_{true}; // 首次读取标志

    /**
     * @brief SPI 读取单个寄存器
     *
     * MT6816 4线SPI协议: 16-bit 帧
     * MOSI: [R/W=1 | A6:A0 | 0x00]  (读操作)
     * MISO: [garbage  | DO7:DO0]     (寄存器数据)
     *
     * @param reg_addr 寄存器地址 (7-bit)
     * @param out_value 读出的寄存器数据 (8-bit)
     * @return 传输成功返回 true，否则返回 false
     */
    bool IRAM_ATTR spi_read_register(uint8_t reg_addr, uint8_t &out_value);

    /**
     * @brief 极速 SPI 读取（直接硬件寄存器 + GPIO 寄存器 CS）
     * 绕过 ESP-IDF SPI master 驱动，每次事务仅需 ~4µs @ 5MHz
     */
    bool IRAM_ATTR spi_read_register_fast(uint8_t reg_addr, uint8_t &out_value);

    /**
     * @brief 读取 14-bit 原始角度值并进行奇偶校验
     *
     * 读取寄存器 0x03 (Angle<13:6>) 和 0x04 (Angle<5:0> | No_Mag_Warning | PC)
     * @param out_angle 成功时输出的 14-bit 角度值 [0, 16383]
     * @return 成功返回 true，失败（弱磁或校验错误）返回 false
     */
    bool IRAM_ATTR read_14bit_angle(uint16_t &out_angle);

    /**
     * @brief 更新角速度和低通滤波
     */
    void IRAM_ATTR update_velocity(float current_angle, int64_t now_us);
};

#endif // FOC_SPI_MT6816_H
