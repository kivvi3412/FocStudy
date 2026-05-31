#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "motor_foc_driver.h"
#include "project_conf.h"

extern "C" void app_main() {
    auto driver = FocDriver(FOC_MCPWM_U_GPIO, FOC_MCPWM_V_GPIO, FOC_MCPWM_W_GPIO, FOC_MCPWM_EN_GPIO,
                            FOC_MCPWM_POLE_PAIRS);
    driver.driver_enable(true);
    driver.open_loop_1_cycle(0, 300);
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
