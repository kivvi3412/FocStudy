#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "motor_foc_driver.h"
#include "iic_as5600.h"
#include "iic_master.h"
#include "project_conf.h"

static const char *TAG = "app_main";

extern "C" void app_main() {
    auto *iic_master = new IICMaster(IIC_MASTER_NUM, IIC_MASTER_SDA1_IO, IIC_MASTER_SCL1_IO);
    auto *as5600 = new AS5600(iic_master->iic_master_get_bus_handle(), IIC_AS5600_ADDR);
    auto *foc_driver = new FocDriver(FOC_MCPWM_U1_GPIO,
                                     FOC_MCPWM_V1_GPIO,
                                     FOC_MCPWM_W1_GPIO,
                                     FOC_DRV_EN_GPIO,
                                     as5600,
                                     FOC_MOTOR_POLE_PAIRS
    );

    foc_driver->bsp_bridge_driver_enable(true);
    foc_driver->foc_motor_calibrate();
    foc_driver->set_dq(0, 1000);
}
