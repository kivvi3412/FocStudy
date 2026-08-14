// ============================================================
// mt6835_calib.h —— MT6835 磁编码器自校准（外部拖动版）
//
// 用法：
//   #include "mt6835_calib.h"
//   mt6835_calib::run_calibration();   // 在 app_main() 中调用一次
//
// 本版本不驱动电机：电机/负载由外部拖动，程序只做三件事：
//   1. SPI 读取 MT6835 角度并做 CRC 校验
//   2. 持续测速，直到转速稳定在目标值附近连续 N 秒
//   3. 配置 AUTO_CAL_FREQ 后拉高 CAL_EN，轮询 0x113 直到自校准结束
//
// 芯片的 AUTO_CAL_FREQ 窗口按转速（RPM）定义，出厂默认 0x3=400~800 RPM。
// 如需修改窗口，配置 MT6835_CALIB_AUTO_CAL_FREQ（0~7）即可；默认 -1 不改芯片。
// ============================================================
#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/spi_ll.h"
#include "soc/gpio_struct.h"
#include <cmath>

namespace mt6835_calib {
    // ================= 参数配置区（直接在这里修改） =================
    namespace config {
        // —— 外部拖动测速参数 ——
        static constexpr int MT6835_CALIB_TARGET_RPM = 600; // 目标机械转速（外部拖动）
        static constexpr int MT6835_CALIB_TARGET_BAND_RPM = 50; // ±50 RPM 视为“目标附近”
        static constexpr int MT6835_CALIB_STABLE_MS = 5000; // 连续稳定时间（毫秒）
        static constexpr int MT6835_CALIB_MONITOR_TIMEOUT_MS = 300000; // 等待稳定总超时（5 分钟）
        static constexpr int MT6835_CALIB_TIMEOUT_MS = 60000; // 自校准轮询超时（毫秒）
        static constexpr int MT6835_CALIB_COMM_FAIL_MAX = 10; // 连续通讯失败多少次判定掉线

        // 芯片自校准转速窗口（AUTO_CAL_FREQ，寄存器 0x00E[6:4]）：
        //   0x0=3200~6400, 0x1=1600~3200, 0x2=800~1600, 0x3=400~800（出厂默认）,
        //   0x4=200~400, 0x5=100~200, 0x6=50~100, 0x7=25~50（单位 RPM）
        // 默认 -1 = 不改芯片设置，使用芯片当前窗口（出厂 0x3 覆盖 400~800 RPM）。
        // 如确需修改，填 0~7：程序会只改 RAM 副本（不烧 EEPROM）并回读校验。
        static constexpr int MT6835_CALIB_AUTO_CAL_FREQ = -1;

        // —— 引脚 ——
        static constexpr gpio_num_t MT6835_CALIB_MOSI_GPIO = GPIO_NUM_18;
        static constexpr gpio_num_t MT6835_CALIB_MISO_GPIO = GPIO_NUM_22;
        static constexpr gpio_num_t MT6835_CALIB_SCK_GPIO = GPIO_NUM_19;
        static constexpr gpio_num_t MT6835_CALIB_CSN_GPIO = GPIO_NUM_4;
        static constexpr gpio_num_t MT6835_CALIB_CAL_GPIO = GPIO_NUM_15;

        // —— SPI ——
        static constexpr int MT6835_CALIB_SPI_FREQ_HZ = 5000000;
    } // namespace config

    // 让文件内部直接使用上面的参数名
    using namespace config;

    namespace {
        static constexpr const char *TAG = "MT6835校准";

        static constexpr float kPiF = 3.14159265358979323846f;
        static constexpr float kTwoPiF = 6.28318530717958647692f;

        // ---------------- 内部状态 ----------------
        static spi_device_handle_t s_spi = nullptr;
        static spi_dev_t *s_spi_hw = nullptr; // SPI2 硬件外设指针（直接寄存器操作）
        static uint32_t s_cs_mask = 0; // CSN GPIO 位掩码（直接寄存器写）
        static uint32_t s_total_comm_errors = 0; // 开机以来通讯错误总数（超时+CRC）
        static uint32_t s_consecutive_errors = 0; // 连续读取失败次数

        // ---------------- CRC-8：多项式 X^8+X^2+X+1 = 0x07，初值 0x00 ----------------
        inline uint8_t crc8_07(const uint8_t *data, size_t len) {
            uint8_t crc = 0;
            for (size_t i = 0; i < len; i++) {
                crc ^= data[i];
                for (int b = 0; b < 8; b++) {
                    crc = (crc & 0x80U)
                              ? (uint8_t) ((crc << 1) ^ 0x07U)
                              : (uint8_t) (crc << 1);
                }
            }
            return crc;
        }

        // ---------------- SPI ----------------
        inline void init_spi() {
            spi_bus_config_t bus_cfg = {};
            bus_cfg.mosi_io_num = MT6835_CALIB_MOSI_GPIO;
            bus_cfg.miso_io_num = MT6835_CALIB_MISO_GPIO;
            bus_cfg.sclk_io_num = MT6835_CALIB_SCK_GPIO;
            bus_cfg.quadwp_io_num = -1;
            bus_cfg.quadhd_io_num = -1;
            bus_cfg.max_transfer_sz = 32;
            ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED));

            // MISO 上拉：编码器断线时 MISO 读到全 0xFF，CRC 必定失败，实现“断线必停机”
            gpio_pullup_en(MT6835_CALIB_MISO_GPIO);

            spi_device_interface_config_t dev_cfg = {};
            dev_cfg.clock_speed_hz = MT6835_CALIB_SPI_FREQ_HZ;
            dev_cfg.mode = 3; // CPOL=1, CPHA=1
            dev_cfg.spics_io_num = -1; // 禁用硬件 CS，改用手动 GPIO 控制（6007 分支验证过的方案）
            dev_cfg.queue_size = 1;
            ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi));

            // CSN 手动控制：输出 + 上拉，空闲保持高电平
            gpio_config_t cs_cfg = {};
            cs_cfg.pin_bit_mask = 1ULL << MT6835_CALIB_CSN_GPIO;
            cs_cfg.mode = GPIO_MODE_OUTPUT;
            cs_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_config(&cs_cfg);
            gpio_set_level(MT6835_CALIB_CSN_GPIO, 1);

            // 独占 SPI 总线，消除每次事务的互斥开销
            ESP_ERROR_CHECK(spi_device_acquire_bus(s_spi, portMAX_DELAY));

            s_spi_hw = SPI_LL_GET_HW(SPI2_HOST);
            s_cs_mask = 1UL << MT6835_CALIB_CSN_GPIO;

            // 用一次标准事务把 SPI 硬件寄存器（时钟/模式/数据长度=48bit）初始化好，
            // CS 未接硬件，编码器不会响应，仅为配置外设
            uint8_t prime_tx[6] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
            uint8_t prime_rx[6] = {};
            spi_transaction_t prime = {};
            prime.length = 48;
            prime.tx_buffer = prime_tx;
            prime.rx_buffer = prime_rx;
            ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &prime));
        }

        // CSN 建立/保持时间：36 个 NOP @240MHz ≈ 150ns，
        // 满足手册 TL≥100ns、TH≥0.5·TSCK（5MHz 时 100ns）
        inline void cs_timing_nops() {
            __asm__ __volatile__(
                "nop; nop; nop; nop; nop; nop; nop; nop; nop;"
                "nop; nop; nop; nop; nop; nop; nop; nop; nop;"
                "nop; nop; nop; nop; nop; nop; nop; nop; nop;"
                "nop; nop; nop; nop; nop; nop; nop; nop; nop;"
                ::: "memory");
        }

        // 手动 CS + 直接寄存器 SPI 事务（与 6007 分支完全一致的时序）
        // 返回 false 表示事务超时（正常 48-bit @5MHz 约 10µs，超时阈值 1ms 只防挂死）
        inline bool spi_raw_transact(const uint8_t *tx, uint8_t *rx, int bits) {
            spi_ll_write_buffer(s_spi_hw, tx, bits);
            spi_ll_clear_int_stat(s_spi_hw);

            // CSN 下降沿锁存编码器内部角度寄存器
            GPIO.out_w1tc = s_cs_mask;
            cs_timing_nops();

            spi_ll_user_start(s_spi_hw);
            const int64_t t_start = esp_timer_get_time();
            while (!spi_ll_usr_is_done(s_spi_hw)) {
                if ((esp_timer_get_time() - t_start) > 1000) { // 1ms
                    GPIO.out_w1ts = s_cs_mask; // 超时也要把 CS 拉回空闲态
                    ESP_LOGE(TAG, "SPI 事务超时（%d bit）！编码器无响应？", bits);
                    return false;
                }
            }
            cs_timing_nops();

            // CSN 上升沿结束通信
            GPIO.out_w1ts = s_cs_mask;
            spi_ll_read_buffer(s_spi_hw, rx, bits);
            return true;
        }

        // 单字节读寄存器；失败时 out 置 0xFF
        inline bool spi_read_reg(uint16_t addr, uint8_t &out) {
            uint8_t tx[3] = {
                (uint8_t) (0x30U | (addr >> 8)),
                (uint8_t) (addr & 0xFFU), 0x00
            };
            uint8_t rx[3] = {};
            if (!spi_raw_transact(tx, rx, 24)) {
                out = 0xFF;
                return false;
            }
            out = rx[2];
            return true;
        }

        // 单字节写寄存器（0x00E 的 AUTO_CAL_FREQ 用；只写 RAM 副本，不烧 EEPROM）
        inline bool spi_write_reg(uint16_t addr, uint8_t value) {
            uint8_t tx[3] = {
                (uint8_t) (0x60U | (addr >> 8)), // 0110 = 写寄存器
                (uint8_t) (addr & 0xFFU),
                value
            };
            uint8_t rx[3] = {};
            return spi_raw_transact(tx, rx, 24);
        }

        struct AngleReadResult {
            bool ok = false; // CRC 校验通过
            bool tx_ok = true; // SPI 事务是否完成（超时=false）
            uint8_t status = 0; // STATUS[2:0]
            uint32_t angle21 = 0; // 21 位绝对角度
            uint8_t rx[4] = {}; // 原始 REG03..REG06
            uint8_t crc_calc = 0; // 本地计算的 CRC
        };

        // 连读 0x003~0x006：一次 48-bit 事务，返回 21 位角度 + 状态，并做 CRC8 校验
        inline bool spi_read_angle_diag(AngleReadResult &r) {
            uint8_t tx[6] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
            uint8_t rx[6] = {};
            if (!spi_raw_transact(tx, rx, 48)) {
                r.tx_ok = false;
                s_total_comm_errors++;
                s_consecutive_errors++;
                return false;
            }

            // rx[2..5] = 0x003 ANGLE[20:13], 0x004 ANGLE[12:5],
            //            0x005 ANGLE[4:0]|STATUS[2:0], 0x006 CRC
            r.rx[0] = rx[2];
            r.rx[1] = rx[3];
            r.rx[2] = rx[4];
            r.rx[3] = rx[5];
            r.crc_calc = crc8_07(&rx[2], 3);
            if (r.crc_calc != rx[5]) {
                s_total_comm_errors++;
                s_consecutive_errors++;
                return false;
            }

            s_consecutive_errors = 0;
            r.status = rx[4] & 0x07U;
            r.angle21 = ((uint32_t) rx[2] << 13) |
                        ((uint32_t) rx[3] << 5) |
                        ((uint32_t) rx[4] >> 3);
            r.ok = true;
            return true;
        }

        // 读取角度并输出详细失败原因（ctx 用于区分调用场景）
        inline bool read_angle_rad(float &rad, uint8_t &status3, const char *ctx) {
            AngleReadResult r;
            if (!spi_read_angle_diag(r)) {
                if (!r.tx_ok) {
                    ESP_LOGE(TAG, "%s：编码器读取失败（SPI 事务超时）！", ctx);
                } else {
                    ESP_LOGE(TAG, "%s：编码器读取失败（CRC 校验错误）！", ctx);
                    ESP_LOGE(TAG, "  REG03=0x%02X REG04=0x%02X REG05=0x%02X REG06(CRC)=0x%02X，"
                             "本地计算 CRC=0x%02X（多项式 0x07）",
                             r.rx[0], r.rx[1], r.rx[2], r.rx[3], r.crc_calc);
                }
                ESP_LOGE(TAG, "  连续失败=%u，开机累计通讯错误=%u",
                         (unsigned) s_consecutive_errors, (unsigned) s_total_comm_errors);
                if (r.tx_ok && r.rx[0] == 0xFF && r.rx[1] == 0xFF && r.rx[2] == 0xFF) {
                    ESP_LOGE(TAG, "  MISO 全为 0xFF：编码器无响应/断线/未供电");
                } else if (r.tx_ok && r.rx[0] == 0x00 && r.rx[1] == 0x00 && r.rx[2] == 0x00) {
                    ESP_LOGE(TAG, "  MISO 全为 0x00：编码器未驱动 MISO，检查接线/供电");
                } else if (r.tx_ok) {
                    ESP_LOGE(TAG, "  MISO 有数据但 CRC 不符：传输被干扰（PWM 噪声/供电跌落），检查布线");
                }
                return false;
            }
            status3 = r.status;
            rad = (float) r.angle21 * (kTwoPiF / 2097152.0f); // 21 位角度 -> 弧度
            return true;
        }

        // 启动前检查编码器通讯（CRC 校验连续 5 次）
        inline bool encoder_comm_ok() {
            int ok = 0;
            constexpr int kTries = 5;
            for (int i = 0; i < kTries; i++) {
                float rad;
                uint8_t s;
                if (read_angle_rad(rad, s, "启动前检查")) {
                    ok++;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (ok < kTries) {
                ESP_LOGE(TAG, "\033[1;31m编码器通讯失败：%d/%d 次读取 CRC 校验失败，"
                         "请检查 SPI 接线、供电和磁铁安装！\033[0m",
                         kTries - ok, kTries);
                return false;
            }
            ESP_LOGI(TAG, "编码器通讯正常（%d/%d 次读取校验全部通过）", ok, kTries);
            return true;
        }

        // ---------------- 速度测量（用编码器角度） ----------------
        // 成功返回 true，rpm 为带符号转速（负值=相对编码器正方向反转）；
        // 读取失败返回 false。
        inline bool measure_rpm(float &rpm, uint8_t *out_status) {
            float a1 = 0, a2 = 0;
            uint8_t s1 = 0, s2 = 0;
            int64_t t1 = esp_timer_get_time();
            if (!read_angle_rad(a1, s1, "测速")) {
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            if (!read_angle_rad(a2, s2, "测速")) {
                return false;
            }
            int64_t t2 = esp_timer_get_time();
            if (out_status != nullptr) {
                *out_status = s2;
            }

            float delta = a2 - a1;
            if (delta > kPiF) {
                delta -= kTwoPiF;
            } else if (delta < -kPiF) {
                delta += kTwoPiF;
            }
            float dt = (float) (t2 - t1) * 1e-6f;
            if (dt <= 0.0f) {
                return false;
            }
            rpm = delta / dt * 60.0f / kTwoPiF;
            return true;
        }

        // 芯片 AUTO_CAL_FREQ[2:0]（寄存器 0x00E[6:4]）对应的自校准转速窗口
        inline const char *autocal_freq_desc(uint8_t acf) {
            switch (acf) {
                case 0x0: return "3200<=RPM<6400";
                case 0x1: return "1600<=RPM<3200";
                case 0x2: return "800<=RPM<1600";
                case 0x3: return "400<=RPM<800";
                case 0x4: return "200<=RPM<400";
                case 0x5: return "100<=RPM<200";
                case 0x6: return "50<=RPM<100";
                default: return "25<=RPM<50";
            }
        }

    } // namespace

    // ============================================================
    // 对外唯一接口：MT6835 自校准主流程（外部拖动版）
    // ============================================================
    inline void run_calibration() {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "MT6835 磁编码器自校准程序启动（外部拖动版）");
        ESP_LOGI(TAG, "目标转速=%d±%d RPM，连续稳定 %d 秒后开始自校准",
                 MT6835_CALIB_TARGET_RPM, MT6835_CALIB_TARGET_BAND_RPM,
                 MT6835_CALIB_STABLE_MS / 1000);
        ESP_LOGI(TAG, "========================================");

        // 1. 初始化 SPI 并检查编码器通讯
        ESP_LOGI(TAG, "初始化 SPI...");
        init_spi();
        if (!encoder_comm_ok()) {
            ESP_LOGE(TAG, "\033[1;31m编码器通讯失败，已中止校准！\033[0m");
            return;
        }

        // 2. 初始化 CAL_EN 引脚（默认低电平）
        gpio_config_t cal_cfg = {};
        cal_cfg.pin_bit_mask = 1ULL << MT6835_CALIB_CAL_GPIO;
        cal_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&cal_cfg);
        gpio_set_level(MT6835_CALIB_CAL_GPIO, 0);

        // 3. 持续测速，等待外部拖动转速稳定
        ESP_LOGI(TAG, "请外部拖动电机：转速稳定在 %d±%d RPM 连续 %d 秒后自动开始自校准...",
                 MT6835_CALIB_TARGET_RPM, MT6835_CALIB_TARGET_BAND_RPM,
                 MT6835_CALIB_STABLE_MS / 1000);

        int comm_fail = 0;
        int64_t stable_since = -1;
        float rpm_min = 0.0f;
        float rpm_max = 0.0f;
        const int64_t monitor_start = esp_timer_get_time();
        int64_t last_log = 0;

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(200));

            uint8_t status = 0;
            float rpm = 0.0f;
            if (!measure_rpm(rpm, &status)) {
                comm_fail++;
                ESP_LOGW(TAG, "编码器读取失败（第 %d 次，累计通讯错误=%u）",
                         comm_fail, (unsigned) s_total_comm_errors);
                if (comm_fail >= MT6835_CALIB_COMM_FAIL_MAX) {
                    ESP_LOGE(TAG, "\033[1;31m编码器通讯失败（累计通讯错误=%u），已中止！\033[0m",
                             (unsigned) s_total_comm_errors);
                    return;
                }
                stable_since = -1;
                continue;
            }
            comm_fail = 0;
            rpm = fabsf(rpm);

            if ((status & 0x04U) != 0) {
                ESP_LOGE(TAG, "\033[1;31m编码器供电欠压报警，已中止！\033[0m");
                return;
            }
            if ((status & 0x02U) != 0) {
                ESP_LOGW(TAG, "编码器磁场过弱报警，请检查磁铁安装！");
            }

            const int64_t now = esp_timer_get_time();
            if (rpm >= MT6835_CALIB_TARGET_RPM - MT6835_CALIB_TARGET_BAND_RPM &&
                rpm <= MT6835_CALIB_TARGET_RPM + MT6835_CALIB_TARGET_BAND_RPM) {
                if (stable_since < 0) {
                    stable_since = now;
                    rpm_min = rpm_max = rpm;
                } else {
                    if (rpm < rpm_min) rpm_min = rpm;
                    if (rpm > rpm_max) rpm_max = rpm;
                }
                if (now - stable_since >= (int64_t) MT6835_CALIB_STABLE_MS * 1000) {
                    break;
                }
            } else {
                if (stable_since >= 0) {
                    ESP_LOGW(TAG, "转速 %.0f RPM 离开 %d±%d 窗口，稳定计时清零",
                             (double) rpm, MT6835_CALIB_TARGET_RPM,
                             MT6835_CALIB_TARGET_BAND_RPM);
                }
                stable_since = -1;
            }

            if (now - last_log >= 1000000) {
                const double stable_sec =
                        (stable_since >= 0) ? (now - stable_since) * 1e-6 : 0.0;
                ESP_LOGI(TAG, "当前转速 %.0f RPM，已连续稳定 %.1f/%d 秒",
                         (double) rpm, stable_sec, MT6835_CALIB_STABLE_MS / 1000);
                last_log = now;
            }

            if (now - monitor_start >=
                (int64_t) MT6835_CALIB_MONITOR_TIMEOUT_MS * 1000) {
                ESP_LOGE(TAG, "\033[1;31m等待转速稳定超时（%d 秒），"
                         "请确认外部拖动转速是否达到 %d±%d RPM！\033[0m",
                         MT6835_CALIB_MONITOR_TIMEOUT_MS / 1000,
                         MT6835_CALIB_TARGET_RPM, MT6835_CALIB_TARGET_BAND_RPM);
                return;
            }
        }

        const float stable_rpm = (rpm_min + rpm_max) / 2.0f;
        ESP_LOGI(TAG, "转速已稳定：%.0f~%.0f RPM，连续 %d 秒",
                 (double) rpm_min, (double) rpm_max,
                 MT6835_CALIB_STABLE_MS / 1000);

        // 4. 检查芯片自校准转速窗口（默认不改动芯片，保持出厂 0x3=400~800 RPM）
        uint8_t acf_reg = 0;
        if (!spi_read_reg(0x00E, acf_reg)) {
            ESP_LOGE(TAG, "\033[1;31m读取 AUTO_CAL_FREQ 失败，已中止！\033[0m");
            return;
        }
        const uint8_t cur_acf = (acf_reg >> 4) & 0x07U;
        ESP_LOGI(TAG, "芯片当前 AUTO_CAL_FREQ=%d（%s）",
                 cur_acf, autocal_freq_desc(cur_acf));
        if (MT6835_CALIB_AUTO_CAL_FREQ >= 0 &&
            cur_acf != (uint8_t) MT6835_CALIB_AUTO_CAL_FREQ) {
            const uint8_t want_acf = (uint8_t) MT6835_CALIB_AUTO_CAL_FREQ;
            const uint8_t new_reg = (uint8_t) ((acf_reg & 0x8FU) | ((want_acf & 0x07U) << 4));
            if (!spi_write_reg(0x00E, new_reg)) {
                ESP_LOGE(TAG, "\033[1;31m写入 AUTO_CAL_FREQ 失败，已中止！\033[0m");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            spi_read_reg(0x00E, acf_reg);
            if (((acf_reg >> 4) & 0x07U) != want_acf) {
                ESP_LOGE(TAG, "\033[1;31mAUTO_CAL_FREQ 回读校验失败，已中止！\033[0m");
                return;
            }
            ESP_LOGI(TAG, "AUTO_CAL_FREQ 已改为 %d（%s）",
                     want_acf, autocal_freq_desc(want_acf));
        } else if (MT6835_CALIB_AUTO_CAL_FREQ < 0) {
            ESP_LOGI(TAG, "保持芯片当前自校准窗口（如需修改可配置 MT6835_CALIB_AUTO_CAL_FREQ）");
        }

        // 5. 拉高 CAL_EN 开始自校准
        ESP_LOGI(TAG, "转速稳定，开始自校准（CAL_EN 拉高）...");
        gpio_set_level(MT6835_CALIB_CAL_GPIO, 1);

        // 6. 轮询 0x113[7:6] 校准状态，带超时和通讯检查
        int cal_result = 0; // 0=超时, 1=成功, 2=失败, -1=通讯错误
        comm_fail = 0;
        int last_cal_state = -1;
        int64_t last_progress_log = 0;
        const int64_t cal_start = esp_timer_get_time();
        while (esp_timer_get_time() - cal_start <
               (int64_t) MT6835_CALIB_TIMEOUT_MS * 1000) {
            vTaskDelay(pdMS_TO_TICKS(100));

            uint8_t status = 0;
            if (!spi_read_reg(0x113, status)) {
                comm_fail++;
                ESP_LOGW(TAG, "校准过程中 0x113 状态寄存器读取失败（第 %d 次）", comm_fail);
                if (comm_fail >= MT6835_CALIB_COMM_FAIL_MAX) {
                    cal_result = -1;
                    break;
                }
                continue;
            }
            const uint8_t cal_state = (status >> 6) & 0x03U;
            if (cal_state != last_cal_state) {
                ESP_LOGI(TAG, "校准状态变化：0x113=0x%02X，状态=%d"
                         "（0=未运行 1=校准中 2=失败 3=成功）",
                         status, cal_state);
                last_cal_state = cal_state;
            }

            // 每次轮询同时做一次 CRC 校验读，确保编码器在线
            float rad;
            uint8_t s;
            if (!read_angle_rad(rad, s, "校准轮询")) {
                comm_fail++;
                ESP_LOGW(TAG, "校准过程中编码器读取失败（第 %d 次，累计通讯错误=%u）",
                         comm_fail, (unsigned) s_total_comm_errors);
                if (comm_fail >= MT6835_CALIB_COMM_FAIL_MAX) {
                    cal_result = -1;
                    break;
                }
                continue;
            }
            comm_fail = 0;

            if (cal_state == 1) {
                const int64_t now = esp_timer_get_time();
                if (now - last_progress_log >= 1000000) { // 每秒一次
                    ESP_LOGI(TAG, "正在校准中...（已 %.1f 秒）",
                             (double) ((now - cal_start) * 1e-6));
                    last_progress_log = now;
                }
            } else if (cal_state == 2) {
                cal_result = 2;
                break;
            } else if (cal_state == 3) {
                cal_result = 1;
                break;
            }
        }
        if (cal_result == 0) {
            ESP_LOGE(TAG, "\033[1;31m校准超时（%d 秒），已中止！\033[0m",
                     MT6835_CALIB_TIMEOUT_MS / 1000);
        }

        // 7. 结束自校准（CAL_EN 拉低）
        gpio_set_level(MT6835_CALIB_CAL_GPIO, 0);
        ESP_LOGI(TAG, "自校准流程结束，CAL_EN 已拉低");

        if (cal_result == 1) {
            ESP_LOGI(TAG, "自校准成功！按手册要求等待 6 秒让 EEPROM 写入完成...");
            vTaskDelay(pdMS_TO_TICKS(6000));
            ESP_LOGE(TAG, "\033[1;31m==========================================\033[0m");
            ESP_LOGE(TAG, "\033[1;31m  【允许断电】校准数据已写入 EEPROM！      \033[0m");
            ESP_LOGE(TAG, "\033[1;31m  现在可以安全断开电源了！                \033[0m");
            ESP_LOGE(TAG, "\033[1;31m==========================================\033[0m");
        } else if (cal_result == 2) {
            ESP_LOGE(TAG, "\033[1;31m芯片报告自校准失败（0x113[7:6]=10）！\033[0m");
            ESP_LOGE(TAG, "这是 MT6835 芯片内部的校准算法判定失败，不是 SPI 通讯错误。");
            ESP_LOGE(TAG, "芯片当前 AUTO_CAL_FREQ=%d（%s），请确认目标转速在该窗口内。",
                     cur_acf, autocal_freq_desc(cur_acf));
            ESP_LOGE(TAG, "常见原因：转速超出 AUTO_CAL_FREQ 窗口、"
                     "转速不够恒定（芯片要求匀速转满 64 圈）、磁场信号弱、或负载突变。");
            ESP_LOGE(TAG, "请回看上方“当前转速/稳定计时”是否持续稳定，再重新上电重试。");
        } else if (cal_result == -1) {
            ESP_LOGE(TAG, "\033[1;31m校准过程中编码器通讯丢失，已中止！请检查 SPI 接线。\033[0m");
        }

        // 8. 校准结束，主线程挂起
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
} // namespace mt6835_calib
