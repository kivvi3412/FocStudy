//
// 纯 C++ 标准 MIDI 文件 (SMF) 解析器
//
// 不依赖 ESP-IDF，可在主机侧独立编译测试。
// 支持: format 0/1、VLQ 增量时间、running status、
//       Note On/Off（力度 0 视为 Off）、tempo 元事件 (0xFF 0x51)。
// 忽略: sysex、CC/PC/Aftertouch/PitchBend、其他 meta 事件。
// 输出: 合并所有音轨、按绝对微秒时间排序的音符事件数组。
//

#ifndef MIDI_PARSER_H
#define MIDI_PARSER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace midi {

struct MidiEvent {
    uint32_t time_us;  // 距歌曲起始的绝对时间 (us)
    uint8_t note;      // MIDI 音高 0..127
    uint8_t velocity;  // 力度 0..127
    uint8_t on;        // 1 = Note On, 0 = Note Off
};

struct ParseResult {
    bool ok = false;
    std::vector<MidiEvent> events;  // 按 time_us 升序
    uint32_t duration_us = 0;       // 最后一个事件的时间
    int note_on_count = 0;
    int note_off_count = 0;
    int tempo_change_count = 0;     // 文件中的 tempo 元事件数
};

// 解析一段完整 MIDI 文件内容。失败时 ok=false。
ParseResult parse_midi(const uint8_t *data, size_t size);

}  // namespace midi

#endif  // MIDI_PARSER_H
