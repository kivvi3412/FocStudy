//
// MT6816 SPI 磁编码器驱动实现
//

#include "spi_mt6816.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "project_conf.h"
#include <cmath>

#ifndef M_TWOPI
#define M_TWOPI (2.0f * (float)M_PI)
#endif

static const char *TAG = "MT6816";

// MT6816 寄存器地址
#define MT6816_REG_ANGLE_H 0x03 // Angle<13:6>
#define MT6816_REG_ANGLE_L 0x04 // [Angle<5:0> | No_Mag_Warning | PC]

MT6816::MT6816() {
    // 初始化 SPI 总线
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SPI_MT6816_MOSI_IO;
    bus_cfg.miso_io_num = SPI_MT6816_MISO_IO;
    bus_cfg.sclk_io_num = SPI_MT6816_SCK_IO;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4; // 最多 2 字节传输
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED));

    // SPI Mode 3: CPOL=1 (SCK空闲高), CPHA=1 (上升沿采样)
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = SPI_MT6816_FREQ_HZ;
    dev_cfg.mode = 3;
    dev_cfg.spics_io_num = -1; // 禁用硬件 CS，改为手动 GPIO 控制以减少事务间隔
    dev_cfg.queue_size = 1;
    // 不再需要 cs_ena_pretrans/posttrans，因为 CS 由 GPIO 手动控制 (自动必须加不然会暴毙)
    // dev_cfg.cs_ena_pretrans = 1;
    // dev_cfg.cs_ena_posttrans = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev_));

    // 配置 CS 引脚为 GPIO 输出，初始高电平（空闲态）
    cs_pin_ = SPI_MT6816_CSN_IO;
    gpio_config_t cs_conf = {};
    cs_conf.pin_bit_mask = (1ULL << cs_pin_);
    cs_conf.mode = GPIO_MODE_OUTPUT;
    cs_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&cs_conf);
    gpio_set_level(cs_pin_, 1);
    // 关键：启用 MISO 内部上拉。如果 SPI 断开，将读到全 1 (0xFF)，从而触发弱磁报警
    gpio_set_pull_mode(SPI_MT6816_MISO_IO, GPIO_PULLUP_ONLY);

    // 预先锁定总线，消除每次事务的 mutex 开销，FOC 场景下 SPI 总线独占，不需要与其他设备共享
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_dev_, portMAX_DELAY));

    // 获取 SPI 硬件外设指针和 CS 位掩码，用于直接寄存器操作
    spi_hw_ = SPI_LL_GET_HW(SPI2_HOST);
    cs_bit_mask_ = (1UL << cs_pin_);

    // 执行一次 ESP-IDF 事务来初始化所有 SPI 硬件寄存器（时钟/模式/数据长度等）
    // CS 未连接硬件所以 MT6816 不响应，仅为配置寄存器
    {
        spi_transaction_t prime = {};
        prime.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        prime.length = 16;
        prime.tx_data[0] = 0x83;
        prime.tx_data[1] = 0x00;
        spi_device_polling_transmit(spi_dev_, &prime);
    }

    ESP_LOGI(TAG, "MT6816 SPI initialized @ %d Hz, Mode 3, direct HW access, CS on GPIO %d", SPI_MT6816_FREQ_HZ,
             cs_pin_);
}

bool MT6816::spi_read_register(uint8_t reg_addr, uint8_t &out_value) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16; // 16-bit 事务

    // MT6816 4线SPI协议:
    // MOSI byte 0: [R/W=1 | A6:A0]  R/W=1 表示读操作
    // MOSI byte 1: [DI7:DI0]         读操作时无意义
    t.tx_data[0] = 0x80U | (reg_addr & 0x7FU);
    t.tx_data[1] = 0x00;

    // 手动 CS: 拉低 → 传输 → 拉高
    gpio_set_level(cs_pin_, 0);
    esp_err_t err = spi_device_polling_transmit(spi_dev_, &t);
    gpio_set_level(cs_pin_, 1);

    if (err != ESP_OK) {
        return false;
    }

    // MISO byte 0: 无效 (地址阶段)
    // MISO byte 1: [DO7:DO0] 寄存器数据
    out_value = t.rx_data[1];
    return true;
}

bool MT6816::spi_read_register_fast(uint8_t reg_addr, uint8_t &out_value) {
    // 写 TX 数据到硬件缓冲区
    // ESP32 little-endian: byte[0] → data_buf[0] bits[7:0], byte[1] → bits[15:8]
    uint8_t tx[2] = {(uint8_t) (0x80U | (reg_addr & 0x7FU)), 0x00};
    spi_ll_write_buffer(spi_hw_, tx, 16);

    // 清除上次事务的完成标志
    spi_ll_clear_int_stat(spi_hw_);

    // CS 拉低（直接寄存器写，~4ns）
    GPIO.out_w1tc = cs_bit_mask_;

    // 100ns 建立时间 (MT6816 TL ≥ 100ns)
    // 24 NOP @ 160MHz = 150ns
    __asm__ __volatile__(
        "nop; nop; nop; nop; nop; nop; nop; nop;"
        "nop; nop; nop; nop; nop; nop; nop; nop;"
        "nop; nop; nop; nop; nop; nop; nop; nop;"
        ::: "memory"
    );

    // 启动 SPI 传输
    spi_ll_user_start(spi_hw_);

    // 忙等完成 (16 bits @ 5MHz ≈ 3.2µs)
    while (!spi_ll_usr_is_done(spi_hw_)) {
    }

    // CS 拉高
    GPIO.out_w1ts = cs_bit_mask_;

    // 读取 RX 数据
    uint8_t rx[2];
    spi_ll_read_buffer(spi_hw_, rx, 16);
    out_value = rx[1];
    return true;
}

bool MT6816::read_14bit_angle(uint16_t &out_angle) {
    uint8_t reg03_1;
    uint8_t reg04;
    uint8_t reg03_2; // 用于防撕裂二次校验

    // 允许最多 2 次重试（应对撕裂现象）
    for (int retry = 0; retry < 2; ++retry) {
        // 连续读取 高 -> 低 -> 高
        spi_read_register_fast(MT6816_REG_ANGLE_H, reg03_1);
        spi_read_register_fast(MT6816_REG_ANGLE_L, reg04);
        spi_read_register_fast(MT6816_REG_ANGLE_H, reg03_2);

        // 防撕裂检查：如果两次读取的高字节不一致，说明发生了进位/借位撕裂
        if (reg03_1 != reg03_2) {
            continue;
        }

        // 提取弱磁报警标志
        no_mag_warning_ = (reg04 >> 1) & 0x01;

        // 奇偶校验: 0x03[7:0] 和 0x04[7:1] 1 的总个数的奇偶性，需等于 0x04[0] (偶校验)
        uint8_t ones_count = __builtin_popcount(reg03_1) + __builtin_popcount(reg04 >> 1);
        bool parity_ok = ((ones_count & 1) == (reg04 & 0x01));

        if (parity_ok && !no_mag_warning_) {
            // 组合 14-bit 角度: reg03 = Angle<13:6>, reg04 的高 6 位 = Angle<5:0>
            out_angle = ((uint16_t) reg03_1 << 6) | ((reg04 >> 2) & 0x3F);
            return true;
        }

        // 如果奇偶校验失败或弱磁，直接返回 false，不重试
        return false;
    }

    return false; // 重试超限
}

void MT6816::update_velocity(float current_angle, int64_t now_us) {
    if (first_read_) {
        prev_angle_ = current_angle;
        prev_time_us_ = now_us;
        first_read_ = false;
        return;
    }

    // 计算角度增量，处理 0/2π 边界跨越
    float delta = current_angle - prev_angle_;
    if (delta > (float) M_PI)
        delta -= M_TWOPI;
    else if (delta < -(float) M_PI)
        delta += M_TWOPI;

    // 使用真实时间间隔计算速度（不假设固定周期）
    float dt = (float) (now_us - prev_time_us_) * 1e-6f;

    if (dt > 1e-6f) {
        // 防止除零
        velocity_ = delta / dt;
        // 低通滤波: alpha * 当前 + (1-alpha) * 上次
        velocity_filtered_ =
                FOC_LOW_PASS_FILTER_ALPHA * velocity_ +
                (1.0f - FOC_LOW_PASS_FILTER_ALPHA) * velocity_filtered_;
    }

    prev_angle_ = current_angle;
    prev_time_us_ = now_us;
}

float MT6816::read_angle_no_update() {
    uint16_t raw;
    if (!read_14bit_angle(raw)) {
        return NAN;
    }
    return (float) raw * M_TWOPI / (float) SPI_MT6816_RESOLUTION;
}

float MT6816::read_angle_raw() {
    uint16_t raw;
    if (!read_14bit_angle(raw)) {
        return NAN;
    }
    float angle = (float) raw * M_TWOPI / (float) SPI_MT6816_RESOLUTION;

    int64_t now_us = esp_timer_get_time();
    update_velocity(angle, now_us);

    return angle;
}

float MT6816::read_electrical_angle_compensated(int pole_pairs, float direction, float zero_offset) {
    // 谷底采样 1.5T 补偿
    float mech_angle = read_angle_raw();
    if (std::isnan(mech_angle)) {
        return NAN;
    }
    constexpr float mcpwm_period_s = (FOC_MCPWM_PERIOD / (float) FOC_MCPWM_TIMER_RESOLUTION_HZ) * 1.5f;
    constexpr float total_delay_s = MT6816_INTERNAL_DELAY_S + mcpwm_period_s;
    float compensated_mech = mech_angle + velocity_filtered_ * total_delay_s;

    // 机械角度 → 电角度
    return compensated_mech * (float) pole_pairs * direction - zero_offset;
}
