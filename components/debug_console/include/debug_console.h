//
// Created by HAIRONG ZHU on 25-1-14.
//

#ifndef FOCKNOB_DEBUG_CONSOLE_H
#define FOCKNOB_DEBUG_CONSOLE_H

#include "argtable3/argtable3.h"

class MT6835;

class DebugConsole {
public:
    /**
     * @param parm_list Ud, Uq 两个参数指针（由 main 持有）
     * @param encoder   MT6835 编码器指针，用于 `set -i/--info` 读取速度和 CRC 错误
     */
    explicit DebugConsole(float parm_list[2], MT6835 *encoder);

private:
    static int set_params_cmd(int argc, char **argv);
};


#endif //FOCKNOB_DEBUG_CONSOLE_H
