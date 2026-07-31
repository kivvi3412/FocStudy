#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

#include "motor_foc_driver.h"
#include "motor_pid_controller.h"
#include "iic_as5600.h"
#include "iic_master.h"
#include "project_conf.h"
#include "debug_console.h"

extern "C" void app_main() {
    auto *iic_master = new IICMaster(IIC_MASTER_NUM, IIC_MASTER_SDA0_IO, IIC_MASTER_SCL0_IO);
    auto *as5600 = new AS5600(iic_master->iic_master_get_bus_handle(), IIC_AS5600_ADDR);
    auto *foc_driver = new FocDriver(FOC_MCPWM_U0_GPIO,
                                     FOC_MCPWM_V0_GPIO,
                                     FOC_MCPWM_W0_GPIO,
                                     FOC_DRV_EN_GPIO,
                                     as5600,
                                     14
    ); // 6007是14

    foc_driver->bsp_bridge_driver_enable(true);
    foc_driver->foc_motor_calibrate();

    // 参数数组: Ud, Uq, V(圈/s), P, I, D
    // 电机功率极大，所以初始 P, I 给得很小
    float params[6] = {0, 0, 0, 0.05, 0.001, 0};
    auto *console = new DebugConsole(params);

    // 初始化速度 PID，输出限幅(300), 积分限幅(300) 可以根据实际调
    auto *pid_velocity = new PIDController(params[3], params[4], params[5], 1000, 1000, 0);

    while (true) {
        if (console->current_mode == DebugConsole::Mode::Torque) {
            foc_driver->set_dq(params[0], params[1]);
        } else if (console->current_mode == DebugConsole::Mode::Velocity) {
            pid_velocity->setPID(params[3], params[4], params[5]);
            // 将圈/s 转换为 弧度/s
            float target_speed_rad_s = params[2] * 2.0f * M_PI;
            foc_driver->set_velocity(target_speed_rad_s, pid_velocity);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
