//
// Created by HAIRONG ZHU on 25-1-14.
//

#include "debug_console.h"

#include "esp_console.h"
#include "esp_log.h"


struct {
    struct arg_dbl *ud = arg_dbln("d", "ud", "<float>", 0, 1, "设置 Ud (进入力矩模式)");
    struct arg_dbl *uq = arg_dbln("q", "uq", "<float>", 0, 1, "设置 Uq (进入力矩模式)");
    struct arg_dbl *v = arg_dbln("v", "v", "<float>", 0, 1, "设置速度 圈/s (进入速度模式)");
    struct arg_dbl *p = arg_dbln("p", "p", "<float>", 0, 1, "设置 PID P");
    struct arg_dbl *i = arg_dbln("i", "i", "<float>", 0, 1, "设置 PID I");
    struct arg_dbl *k = arg_dbln("k", "k", "<float>", 0, 1, "设置 PID D");
    struct arg_end *end = arg_end(20);
} set_params_args;

float *m_parm_list[6];
DebugConsole *g_console = nullptr;

DebugConsole::DebugConsole(float parm_list[6]) {
    g_console = this;
    for (int j = 0; j < 6; j++) {
        m_parm_list[j] = &parm_list[j];
    }

    const esp_console_cmd_t cmd = {
            .command = "set",
            .help = "设置电机参数并切换模式",
            .hint = nullptr,
            .func = &DebugConsole::set_params_cmd,
            .argtable = &set_params_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

    // 启动 REPL，这时你就可以通过串口进行交互了
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "debug_console>"; // 自定义提示符
    repl_config.max_cmdline_length = 256;

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

    // 启动 REPL，这时你就可以通过串口进行交互了
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

int DebugConsole::set_params_cmd(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **) &set_params_args);
    if (nerrors != 0) {
        arg_print_errors(stdout, set_params_args.end, "set");
        return 1;
    }
    
    bool mode_changed_to_torque = false;
    bool mode_changed_to_velocity = false;

    if (set_params_args.ud->count > 0) {
        *m_parm_list[0] = (float) set_params_args.ud->dval[0];
        mode_changed_to_torque = true;
    }
    if (set_params_args.uq->count > 0) {
        *m_parm_list[1] = (float) set_params_args.uq->dval[0];
        mode_changed_to_torque = true;
    }
    if (set_params_args.v->count > 0) {
        *m_parm_list[2] = (float) set_params_args.v->dval[0];
        mode_changed_to_velocity = true;
    }
    if (set_params_args.p->count > 0) {
        *m_parm_list[3] = (float) set_params_args.p->dval[0];
    }
    if (set_params_args.i->count > 0) {
        *m_parm_list[4] = (float) set_params_args.i->dval[0];
    }
    if (set_params_args.k->count > 0) {
        *m_parm_list[5] = (float) set_params_args.k->dval[0];
    }
    
    if (g_console) {
        if (mode_changed_to_velocity) {
            g_console->current_mode = Mode::Velocity;
        } else if (mode_changed_to_torque) {
            g_console->current_mode = Mode::Torque;
        }
    }

    ESP_LOGI("set_parm", "Mode: %s | Ud: %.2f, Uq: %.2f, V: %.2f, P: %.4f, I: %.4f, D: %.4f",
             (g_console && g_console->current_mode == Mode::Velocity) ? "VELOCITY" : "TORQUE",
             *m_parm_list[0], *m_parm_list[1], *m_parm_list[2], *m_parm_list[3], *m_parm_list[4], *m_parm_list[5]);
    return 0;
}