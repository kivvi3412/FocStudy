//
// Created by HAIRONG ZHU on 2024/3/28.
//

#ifndef IN12_LITTLEFS_MODULE_H
#define IN12_LITTLEFS_MODULE_H

#include <string>
#include "project_config.h"

class littleFS {
public:
    static void init();

    // 例：littleFS::readFile("index.html")
    static std::string readFile(const char *relativePath);
};

#endif // IN12_LITTLEFS_MODULE_H
