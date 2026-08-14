// MT6835 编码器自校准入口：
// 所有逻辑都打包在 mt6835_calib.h 中，可在 include 前用宏覆盖参数
#include "mt6835_calib.h"

extern "C" void app_main() {
    mt6835_calib::run_calibration();
}
