//
// Created by HAIRONG ZHU on 25-1-14.
//

#ifndef FOCKNOB_IIC_MASTER_H
#define FOCKNOB_IIC_MASTER_H

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

class IICMaster {
public:
    IICMaster(i2c_port_num_t i2c_port_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num);

    i2c_master_bus_handle_t iic_master_get_bus_handle();

private:
    esp_err_t iic_master_init(i2c_port_num_t i2c_port_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num);

    i2c_master_bus_handle_t iic_bus_handle{};
};


#endif //FOCKNOB_IIC_MASTER_H
