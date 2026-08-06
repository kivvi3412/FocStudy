//
// Created by HAIRONG ZHU on 25-1-14.
//

#include "debug_console.h"

#include "esp_console.h"
#include "esp_log.h"


struct {
    struct arg_dbl *ud = arg_dbln("d", "ud", "<float>", 0, 1, "设置 Ud (D轴电压)");
    struct arg_dbl *uq = arg_dbln("q", "uq", "<float>", 0, 1, "设置 Uq (Q轴电压)");
    struct arg_end *end = arg_end(20);
} set_params_args;

float *m_parm_list[2];

DebugConsole::DebugConsole(float parm_list[2]) {
    for (int j = 0; j < 2; j++) {
        m_parm_list[j] = &parm_list[j];
    }

    const esp_console_cmd_t cmd = {
        .command = "set",
        .help = "设置开环电压 Ud/Uq (范围 0~1000)",
        .hint = nullptr,
        .func = &DebugConsole::set_params_cmd,
        .argtable = &set_params_args,
        .func_w_context = nullptr,
        .context = nullptr,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

    // 启动 REPL
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "foc>";
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

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

int DebugConsole::set_params_cmd(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **) &set_params_args);
    if (nerrors != 0) {
        arg_print_errors(stdout, set_params_args.end, "set");
        return 1;
    }

    if (set_params_args.ud->count > 0) {
        *m_parm_list[0] = (float) set_params_args.ud->dval[0];
    }
    if (set_params_args.uq->count > 0) {
        *m_parm_list[1] = (float) set_params_args.uq->dval[0];
    }

    ESP_LOGI("set_parm", "Ud: %.2f, Uq: %.2f", *m_parm_list[0], *m_parm_list[1]);
    return 0;
}
