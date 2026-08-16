//
// 标准 MIDI 文件 (SMF) 解析实现，纯 C++，无 ESP-IDF 依赖。
//
// 内存策略：两遍扫描。
//   第一遍：收集 tempo 事件、统计音符事件总数（不分配大数组）。
//   第二遍：按精确数量 reserve 后直接填充事件数组。
// 这样避免 std::vector 动态扩容造成的峰值内存翻倍 ——
// ESP32 堆较小，数千事件的 MIDI（如 Unravel 6852 条、c.mid 8402 条）
// 若逐条 push_back 扩容容易 OOM 触发 abort。
//

#include "midi_parser.h"

#include <algorithm>

namespace midi {
namespace {

struct TempoPoint {
    uint32_t tick;
    uint32_t us_per_qn;
};

inline uint16_t read_u16be(const uint8_t *p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

inline uint32_t read_u32be(const uint8_t *p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

// 读取 MIDI 变长数量 (VLQ)，最多 4 字节；成功返回 true 并推进 pos。
bool read_vlq(const uint8_t *data, size_t end, size_t &pos, uint32_t &out) {
    out = 0;
    for (int i = 0; i < 4; i++) {
        if (pos >= end) {
            return false;
        }
        const uint8_t b = data[pos++];
        out = (out << 7) | static_cast<uint32_t>(b & 0x7f);
        if (!(b & 0x80)) {
            return true;
        }
    }
    return false;
}

// tick -> 绝对微秒。tempos 必须按 tick 升序，且首个元素 tick==0。
uint32_t tick_to_us(uint32_t tick, uint32_t tpqn, const std::vector<TempoPoint> &tempos) {
    uint64_t us = 0;
    uint32_t prev_tick = 0;
    uint32_t prev_tempo = 500000;  // 默认 120 BPM

    for (const TempoPoint &tp : tempos) {
        if (tp.tick > tick) {
            break;
        }
        if (tp.tick > prev_tick) {
            us += (static_cast<uint64_t>(tp.tick - prev_tick) * prev_tempo) / tpqn;
            prev_tick = tp.tick;
        }
        prev_tempo = tp.us_per_qn;
    }
    if (tick > prev_tick) {
        us += (static_cast<uint64_t>(tick - prev_tick) * prev_tempo) / tpqn;
    }
    return us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(us);
}

struct WalkContext {
    std::vector<TempoPoint> tempos;
    std::vector<MidiEvent> *events = nullptr;  // 第二遍填充
    int note_on_count = 0;
    int note_off_count = 0;
    int tempo_meta_count = 0;
    bool collect_tempos = true;
};

// 顺序遍历全部音轨；返回 false 表示文件损坏。
bool walk_tracks(const uint8_t *data, size_t size, size_t start, uint16_t ntrks,
                 WalkContext &ctx) {
    size_t pos = start;
    for (uint16_t t = 0; t < ntrks; t++) {
        if (pos + 8 > size || data[pos] != 'M' || data[pos + 1] != 'T' || data[pos + 2] != 'r' ||
            data[pos + 3] != 'k') {
            return false;
        }
        const uint32_t track_len = read_u32be(data + pos + 4);
        const size_t track_end = pos + 8 + static_cast<size_t>(track_len);
        if (track_end > size) {
            return false;
        }
        pos += 8;

        uint32_t tick = 0;
        uint8_t running = 0;
        bool end_of_track = false;

        while (pos < track_end && !end_of_track) {
            uint32_t delta;
            if (!read_vlq(data, track_end, pos, delta)) {
                return false;
            }
            tick += delta;
            if (pos >= track_end) {
                return false;
            }

            const uint8_t b = data[pos];
            if (b == 0xFF) {
                // meta 事件
                pos++;
                if (pos >= track_end) {
                    return false;
                }
                const uint8_t type = data[pos++];
                uint32_t len;
                if (!read_vlq(data, track_end, pos, len) || pos + len > track_end) {
                    return false;
                }
                if (type == 0x51 && len == 3) {
                    const uint32_t us_per_qn =
                        (static_cast<uint32_t>(data[pos]) << 16) |
                        (static_cast<uint32_t>(data[pos + 1]) << 8) | data[pos + 2];
                    if (ctx.collect_tempos) {
                        ctx.tempos.push_back({tick, us_per_qn});
                    }
                    ctx.tempo_meta_count++;
                } else if (type == 0x2F) {
                    end_of_track = true;
                }
                pos += len;
            } else if (b == 0xF0 || b == 0xF7) {
                // sysex：跳过
                pos++;
                uint32_t len;
                if (!read_vlq(data, track_end, pos, len) || pos + len > track_end) {
                    return false;
                }
                pos += len;
            } else {
                if (b & 0x80u) {
                    running = b;
                    pos++;
                }
                if (running == 0) {
                    return false;
                }
                const uint8_t status = running & 0xF0u;
                if (status == 0x80 || status == 0x90) {
                    if (pos + 2 > track_end) {
                        return false;
                    }
                    const uint8_t note = data[pos];
                    const uint8_t velocity = data[pos + 1];
                    pos += 2;
                    const bool on = (status == 0x90 && velocity > 0);
                    if (on) {
                        ctx.note_on_count++;
                    } else {
                        ctx.note_off_count++;
                    }
                    if (ctx.events != nullptr) {
                        MidiEvent ev;
                        ev.time_us = tick;  // 暂存 tick，解析完后统一换算为微秒
                        ev.note = note;
                        ev.velocity = velocity;
                        ev.on = on ? 1 : 0;
                        ctx.events->push_back(ev);
                    }
                } else if (status == 0xC0 || status == 0xD0) {
                    if (pos + 1 > track_end) {
                        return false;
                    }
                    pos += 1;
                } else {
                    // 0xA0 Poly Aftertouch / 0xB0 CC / 0xE0 Pitch Bend：各 2 字节
                    if (pos + 2 > track_end) {
                        return false;
                    }
                    pos += 2;
                }
            }
        }
    }
    return true;
}

}  // namespace

ParseResult parse_midi(const uint8_t *data, size_t size) {
    ParseResult result;

    if (size < 14 || data[0] != 'M' || data[1] != 'T' || data[2] != 'h' || data[3] != 'd') {
        return result;
    }

    const uint32_t header_len = read_u32be(data + 4);
    if (header_len < 6 || static_cast<size_t>(8 + header_len) > size) {
        return result;
    }

    const uint16_t format = read_u16be(data + 8);
    const uint16_t ntrks = read_u16be(data + 10);
    const uint16_t division = read_u16be(data + 12);

    // 仅支持 format 0/1 与 ticks-per-quarter-note 制式
    if (format > 1 || (division & 0x8000u) != 0 || (division & 0x7fffu) == 0) {
        return result;
    }
    const uint32_t tpqn = division & 0x7fffu;
    const size_t track_start = 8 + header_len;

    // 第一遍：收集 tempo、统计音符事件总数
    WalkContext first;
    if (!walk_tracks(data, size, track_start, ntrks, first)) {
        return result;
    }

    // 构造 tempo 时间轴：保证按 tick 升序且起点为 0
    std::sort(first.tempos.begin(), first.tempos.end(),
              [](const TempoPoint &a, const TempoPoint &b) { return a.tick < b.tick; });
    if (first.tempos.empty() || first.tempos.front().tick != 0) {
        first.tempos.insert(first.tempos.begin(), {0, 500000});
    }

    // 第二遍：按精确数量预分配后填充（避免扩容峰值）
    const size_t total_notes =
        static_cast<size_t>(first.note_on_count + first.note_off_count);
    result.events.reserve(total_notes);
    WalkContext second;
    second.events = &result.events;
    second.collect_tempos = false;
    if (!walk_tracks(data, size, track_start, ntrks, second)) {
        return result;
    }

    // tick -> 绝对微秒
    for (MidiEvent &e : result.events) {
        e.time_us = tick_to_us(e.time_us, tpqn, first.tempos);
    }

    std::stable_sort(result.events.begin(), result.events.end(),
                     [](const MidiEvent &a, const MidiEvent &b) { return a.time_us < b.time_us; });

    result.note_on_count = second.note_on_count;
    result.note_off_count = second.note_off_count;
    result.tempo_change_count = second.tempo_meta_count;
    result.duration_us = result.events.empty() ? 0 : result.events.back().time_us;
    result.ok = true;
    return result;
}

}  // namespace midi
