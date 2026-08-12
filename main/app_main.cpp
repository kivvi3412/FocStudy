#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "debug_console.h"
#include "foc_driver.h"
#include "project_conf.h"
#include "mt6835_driver.h"

static const char *TAG = "Main";

extern "C" void app_main() {
    auto *mt6835 = new MT6835();

    auto *motor0 = new FocMotor(
        mt6835, FOC_MOTOR_POLE_PAIRS, FOC_MCPWM_U0_GPIO,
        FOC_MCPWM_V0_GPIO, FOC_MCPWM_W0_GPIO, FOC_DRV_EN_GPIO
    );

    // 如果已经知道参数，可以跳过自动校准，直接设置：
    // motor0->set_calibration_params(-1.0f, -83.07349396f);
    motor0->calibrate();
    motor0->enable(); // 校准完成后重新使能驱动

    ESP_LOGI(TAG, "FOC loop active - open loop voltage control (Ud/Uq: 0~%d)", FOC_MCPWM_OUTPUT_LIMIT);

    float params[2] = {0, 50}; // [Ud, Uq]
    auto *console = new DebugConsole(params);
    (void) console;

    while (true) {
        motor0->set_voltage(params[0], params[1]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
