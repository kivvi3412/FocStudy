//
// Created by HAIRONG ZHU on 25-1-14.
//

#ifndef FOCKNOB_DEBUG_CONSOLE_H
#define FOCKNOB_DEBUG_CONSOLE_H

#include "argtable3/argtable3.h"


class DebugConsole {
public:
    explicit DebugConsole(float parm_list[2]); // Ud, Uq 两个参数

private:
    static int set_params_cmd(int argc, char **argv);
};


#endif //FOCKNOB_DEBUG_CONSOLE_H
