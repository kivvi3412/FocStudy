//
// 电机 MIDI 播放器：从 littlefs 加载 MIDI 文件，
// 在 FOC 主循环中以真实时间调度音符，
// 用多路正弦叠加（和弦）合成复合音注入 Q 轴。
//

#ifndef MIDI_PLAYER_H
#define MIDI_PLAYER_H

#include "foc_driver.h"

// 开始整首播放（播完即停，不循环）。
// path 为相对 littlefs 根目录的路径，例如 "Unravel.mid"。
// 内部会自动向 motor 注册采样回调，无需额外调用。
void midi_player_play(FocMotor *motor, const char *path);

// 是否正在播放中（供调试/主循环查询）。
bool midi_player_active();

#endif  // MIDI_PLAYER_H
