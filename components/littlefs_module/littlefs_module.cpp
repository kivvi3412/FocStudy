//
// Created by HAIRONG ZHU on 2024/3/28.
//
#include "littlefs_module.h"

#include <esp_log.h>
#include "esp_littlefs.h"
#include "project_conf.h"

static const char *TAG = "LittleFS";

void littleFS::init() {
    esp_vfs_littlefs_conf_t conf = {};
    conf.base_path = LITTLEFS_MOUNT_POINT;
    conf.partition_label = LITTLEFS_PARTITION_LABEL;
    conf.format_if_mount_failed = true;
    conf.dont_mount = false;

    // 使用给定的配置初始化和挂载LittleFS。
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    ESP_ERROR_CHECK(ret);

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    ESP_ERROR_CHECK(ret);
}

std::string littleFS::readFile(const char *relativePath) {
    std::string absolutePath = LITTLEFS_MOUNT_POINT + std::string("/") + std::string(relativePath);
    FILE *file = fopen(absolutePath.c_str(), "r");
    if (file == nullptr) {
        ESP_LOGI(TAG, "No such file: %s", relativePath);
        return {};
    }

    fseek(file, 0, SEEK_END);
    size_t fileSize = ftell(file);
    rewind(file);

    std::string content;
    content.resize(fileSize);

    size_t bytesRead = fread(&content[0], 1, fileSize, file);
    if (bytesRead != fileSize) {
        ESP_LOGE(TAG, "Failed to read file");
        fclose(file);
        return {};
    }

    fclose(file);
    return content;
}
