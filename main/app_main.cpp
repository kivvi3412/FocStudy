#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "debug_console.h"
#include "foc_driver.h"
#include "project_conf.h"
#include "mt6835_driver.h"
#include "littlefs_module.h"
#include "midi_player.h"

static const char *TAG = "Main";

struct MotorInitParams {
    FocMotor *motor0;
    MT6835 *mt6835;
    TaskHandle_t main_task;
};

static void motor_init_task(void *arg) {
    auto *params = static_cast<MotorInitParams *>(arg);

    params->mt6835 = new MT6835();
    params->motor0 = new FocMotor(
        params->mt6835, FOC_MOTOR_POLE_PAIRS, FOC_MCPWM_U0_GPIO,
        FOC_MCPWM_V0_GPIO, FOC_MCPWM_W0_GPIO, FOC_DRV_EN_GPIO
    );

    // 如果已经知道参数，可以跳过自动校准，直接设置：
    params->motor0->set_calibration_params(-1.0f, 0.647268474f); // 接线:黑黄橘
    // params->motor0->calibrate();
    params->motor0->enable(); // 校准完成后重新使能驱动

    // 通知主任务初始化完成
    xTaskNotifyGive(params->main_task);

    vTaskDelete(nullptr);
}

extern "C" void app_main() {
    littleFS::init();

    MotorInitParams init_params = {nullptr, nullptr, xTaskGetCurrentTaskHandle()};

    // 在 Core 1 上创建初始化任务，以确保 MCPWM ISR 注册在 Core 1
    xTaskCreatePinnedToCore(motor_init_task, "motor_init", 4096, &init_params, 5, nullptr, 1);

    // 等待初始化完成
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    FocMotor *motor0 = init_params.motor0;

    ESP_LOGI(TAG, "FOC loop active in high-priority task - open loop voltage control (Ud/Uq: 0~%d)", FOC_MCPWM_OUTPUT_LIMIT);

    // 播放一首歌：main 中只需写入 littlefs 内的 MIDI 文件名
    midi_player_play(motor0, "Flower_Dance_DJ_Okawari.mid");

    float params[2] = {0, 50}; // [Ud, Uq]
    auto *console = new DebugConsole(params, init_params.mt6835);
    (void) console;

    while (true) {
        motor0->set_voltage(params[0], params[1]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
