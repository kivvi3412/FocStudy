//
// MT6835 SPI 磁编码器驱动实现
// 连读模式 (Burst Read) + CRC8 校验 + 故障回调
//

#include "mt6835_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "project_conf.h"
#include <cmath>

#ifndef M_TWOPI
#define M_TWOPI (2.0f * (float)M_PI)
#endif

static const char *TAG = "MT6835";

// ============================================================================
// CRC-8 查找表 (多项式 X⁸+X²+X+1 = 0x07, 初始值 0x00, 无反射/异或)
// DRAM_ATTR 确保表在 DRAM 中，允许 IRAM 函数访问（避免 Flash cache miss）
// ============================================================================
static const DRAM_ATTR uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

uint8_t MT6835::crc8_calc(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

// ============================================================================
// 构造函数：初始化 SPI 总线 + 硬件寄存器配置
// ============================================================================
MT6835::MT6835() {
    // 初始化 SPI 总线
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SPI_MT6835_MOSI_IO;
    bus_cfg.miso_io_num = SPI_MT6835_MISO_IO;
    bus_cfg.sclk_io_num = SPI_MT6835_SCK_IO;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 8; // 连读最多 6 字节 (48-bit 事务)
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED));

    // SPI Mode 3: CPOL=1 (SCK空闲高), CPHA=1 (上升沿采样)
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = SPI_MT6835_FREQ_HZ;
    dev_cfg.mode = 3;
    dev_cfg.spics_io_num = -1; // 禁用硬件 CS，改为手动 GPIO 控制以减少事务间隔
    // 不再需要 cs_ena_pretrans/posttrans，因为 CS 由 GPIO 手动控制 (自动必须加不然MT6835会暴毙)
    // dev_cfg.cs_ena_pretrans = 1;
    // dev_cfg.cs_ena_posttrans = 1;
    dev_cfg.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev_));

    // 配置 CS 引脚为 GPIO 输出，初始高电平（空闲态）
    cs_pin_ = SPI_MT6835_CSN_IO;
    gpio_config_t cs_conf = {};
    cs_conf.pin_bit_mask = (1ULL << cs_pin_);
    cs_conf.mode = GPIO_MODE_OUTPUT;
    cs_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&cs_conf);
    gpio_set_level(cs_pin_, 1);

    // 预先锁定总线，消除每次事务的 mutex 开销，FOC 场景下 SPI 总线独占
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_dev_, portMAX_DELAY));

    // 获取 SPI 硬件外设指针和 CS 位掩码，用于直接寄存器操作
    spi_hw_ = SPI_LL_GET_HW(SPI2_HOST);
    cs_bit_mask_ = (1UL << cs_pin_);

    // 执行一次 ESP-IDF 事务来初始化所有 SPI 硬件寄存器（时钟/模式/数据长度=48bit）
    // CS 未连接硬件所以 MT6835 不响应，仅为配置寄存器
    {
        uint8_t prime_tx[8] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint8_t prime_rx[8] = {};
        spi_transaction_t prime = {};
        prime.length = 48;
        prime.tx_buffer = prime_tx;
        prime.rx_buffer = prime_rx;
        spi_device_polling_transmit(spi_dev_, &prime);
    }

    ESP_LOGI(TAG, "MT6835 SPI initialized @ %d Hz, Mode 3, burst read, CS on GPIO %d", SPI_MT6835_FREQ_HZ, cs_pin_);
}

// ============================================================================
// 连读命令：一次 48-bit SPI 事务读取 reg 0x003~0x006
// ============================================================================
void MT6835::spi_burst_read(uint8_t *rx4) {
    // 写 TX 数据到硬件缓冲区
    // 连读命令格式: [CMD(4bit)=1010 | ADDR(12bit)=0x003]
    //   byte0 = 1010_0000 = 0xA0  (CMD | ADDR[11:8])
    //   byte1 = 0000_0011 = 0x03  (ADDR[7:0])
    //   byte2~5 = dummy (时钟供 MISO 输出数据)
    uint8_t tx[6] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
    spi_ll_write_buffer(spi_hw_, tx, 48);

    // 清除上次事务的完成标志
    spi_ll_clear_int_stat(spi_hw_);

    // CS 拉低（直接寄存器写，~4ns）
    // CSN 下降沿锁存 MT6835 内部角度寄存器 0x003~0x006
    GPIO.out_w1tc = cs_bit_mask_;

    // CS 建立时间 ~150ns (24 NOP @ 160MHz)
    __asm__ __volatile__(
        "nop; nop; nop; nop; nop; nop; nop; nop;"
        "nop; nop; nop; nop; nop; nop; nop; nop;"
        "nop; nop; nop; nop; nop; nop; nop; nop;"
        ::: "memory"
    );

    // 启动 SPI 传输 (48 bits @ 5MHz ≈ 9.6µs)
    spi_ll_user_start(spi_hw_);

    // 忙等完成
    while (!spi_ll_usr_is_done(spi_hw_)) {
    }

    // CS 拉高（结束连读，只读取一组数据）
    GPIO.out_w1ts = cs_bit_mask_;

    // 读取 RX 数据
    uint8_t rx[8];
    spi_ll_read_buffer(spi_hw_, rx, 48);

    // rx[0..1] = 命令阶段 MISO Hi-Z 垃圾数据
    // rx[2] = reg 0x003: ANGLE[20:13]
    // rx[3] = reg 0x004: ANGLE[12:5]
    // rx[4] = reg 0x005: ANGLE[4:0] | STATUS[2:0]
    // rx[5] = reg 0x006: CRC[7:0]
    rx4[0] = rx[2];
    rx4[1] = rx[3];
    rx4[2] = rx[4];
    rx4[3] = rx[5];
}

// ============================================================================
// 读取 16-bit 角度值 + CRC8 校验 + 故障检测
// ============================================================================
bool MT6835::read_angle(uint16_t &out_angle) {
    uint8_t data[4]; // [reg03, reg04, reg05, reg06]
    spi_burst_read(data);

    // CRC8 校验: 对 ANGLE[20:0] + STATUS[2:0] = 24 bit (data[0..2]) 校验
    // CRC 多项式 X⁸+X²+X+1, ANGLE[20] (data[0] MSB) 最先移位进入
    uint8_t crc_calc = crc8_calc(data, 3);
    if (crc_calc != data[3]) {
        // CRC 失败：数据损坏或硬件断线 (全 0xFF 时 CRC 必然不匹配)
        consecutive_errors_++;
        if (consecutive_errors_ >= FAULT_THRESHOLD && fault_cb_ && !fault_triggered_) {
            fault_triggered_ = true;
            fault_cb_(fault_cb_ctx_);
        }
        return false;
    }

    // 提取 STATUS[2:0] 报警标志
    uint8_t status = data[2] & 0x07;
    over_speed_warning_ = (status >> 0) & 1; // STATUS[0]: 转速过快 (>12万转/分钟)
    no_mag_warning_ = (status >> 1) & 1; // STATUS[1]: 外加磁场太弱
    undervoltage_warning_ = (status >> 2) & 1; // STATUS[2]: 芯片供电欠压

    if (no_mag_warning_) {
        // 弱磁报警：磁铁脱落或距离过远，角度数据不可信
        consecutive_errors_++;
        if (consecutive_errors_ >= FAULT_THRESHOLD && fault_cb_ && !fault_triggered_) {
            fault_triggered_ = true;
            fault_cb_(fault_cb_ctx_);
        }
        return false;
    }

    // 读取成功，重置错误计数
    consecutive_errors_ = 0;
    fault_triggered_ = false;

    // 21-bit 角度截取高 16 位: ANGLE[20:5] = (reg03 << 8) | reg04
    // 分辨率 0~65535 对应 0°~360°
    out_angle = ((uint16_t) data[0] << 8) | data[1];

    return true;
}

// ============================================================================
// 速度估算 + 低通滤波
// ============================================================================
void MT6835::update_velocity(float current_angle, int64_t now_us) {
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
                FOC_LOW_PASS_FILTER_ALPHA * velocity_ + (1.0f - FOC_LOW_PASS_FILTER_ALPHA) * velocity_filtered_;
    }

    prev_angle_ = current_angle;
    prev_time_us_ = now_us;
}

// ============================================================================
// 公共接口：角度读取
// ============================================================================
float MT6835::read_angle_no_update() {
    uint16_t raw;
    if (!read_angle(raw)) {
        return NAN;
    }
    return (float) raw * M_TWOPI / (float) SPI_MT6835_RESOLUTION;
}

float MT6835::read_angle_raw() {
    uint16_t raw;
    if (!read_angle(raw)) {
        return NAN;
    }
    float angle = (float) raw * M_TWOPI / (float) SPI_MT6835_RESOLUTION;

    int64_t now_us = esp_timer_get_time();
    update_velocity(angle, now_us);

    return angle;
}

float MT6835::read_electrical_angle_compensated(int pole_pairs, float direction, float zero_offset) {
    float mech_angle = read_angle_raw();
    if (std::isnan(mech_angle)) {
        return NAN;
    }
    // 谷底采样 1T 补偿（计数器归零时采样，不再是 1.5T）
    constexpr float mcpwm_period_s = FOC_MCPWM_PERIOD / (float) FOC_MCPWM_TIMER_RESOLUTION_HZ;
    constexpr float total_delay_s = MT6835_INTERNAL_DELAY_S + mcpwm_period_s;
    float compensated_mech = mech_angle + velocity_filtered_ * total_delay_s;

    // 机械角度 → 电角度
    return compensated_mech * (float) pole_pairs * direction - zero_offset;
}
