//
// Created by HAIRONG ZHU on 25-1-14.
//

#ifndef FOCKNOB_DEBUG_CONSOLE_H
#define FOCKNOB_DEBUG_CONSOLE_H

#include "argtable3/argtable3.h"


class DebugConsole {
public:
    enum class Mode {
        Torque,
        Velocity
    };
    explicit DebugConsole(float parm_list[6]); //需要修改的6个全局变量
    
    Mode current_mode = Mode::Torque;

private:
    static int set_params_cmd(int argc, char **argv); //设置参数的命令
};


#endif //FOCKNOB_DEBUG_CONSOLE_H
