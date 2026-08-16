//
// MIDI 播放器实现：文件加载 + 实时事件调度 + 12 复音正弦合成。
//
// 性能约束：midi_player_sample() 由 FOC 任务以 10~20kHz 调用，
// 必须保持 IRAM、无 malloc/fopen/日志等重操作；
// 事件与声部状态全部在启动时（play）准备为裸指针/定长数组。
//

#include "midi_player.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fast_trig.h"
#include "midi_parser.h"
#include "project_conf.h"

static const char *TAG = "MidiPlayer";

namespace {

constexpr int kVoices = 12;            // 复音上限（Unravel 合并后最大同时 8 音，留余量）
constexpr float kAmplitude = 100.0f;   // 音乐注入振幅 ±50
constexpr float kAttackS = 0.004f;     // 起音 4ms
constexpr float kReleaseS = 0.030f;    // 释音 30ms
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kMaxDt = 0.005f;       // dt 上限，防止长时间停滞后相位跳变
constexpr float kSilenceGain = 0.5f;
constexpr long kMaxFileSize = 512 * 1024;

struct Voice {
    uint8_t note;      // 0xFF = 已完全释放/空闲
    uint8_t sounding;  // 1 = 按键保持中（目标增益 > 0）
    uint32_t seq;      // note_on 顺序号，note_off 取最新实例
    float freq;
    float velocity;
    float phase;
    float gain;
    float target_gain;
};

// ---- 播放状态（全部为 DRAM 数据；sample 只访问裸指针与定长数组）----
volatile bool s_playing = false;
bool s_finished = false;
int64_t s_start_us = 0;
int64_t s_last_sample_us = 0;
uint32_t s_seq = 0;
size_t s_event_index = 0;
const midi::MidiEvent *s_events = nullptr;
size_t s_event_count = 0;
uint32_t s_duration_us = 0;
midi::ParseResult s_parsed;
float s_freq_table[128];
Voice s_voice[kVoices];

void voice_reset() {
    for (Voice &v : s_voice) {
        v.note = 0xFF;
        v.sounding = 0;
        v.seq = 0;
        v.freq = 0.0f;
        v.velocity = 0.0f;
        v.phase = 0.0f;
        v.gain = 0.0f;
        v.target_gain = 0.0f;
    }
}

void voice_note_on(uint8_t note, uint8_t velocity) {
    Voice *slot = nullptr;

    // 1) 优先空闲声部，允许同音叠加
    for (Voice &v : s_voice) {
        if (v.note == 0xFF) {
            slot = &v;
            break;
        }
    }
    // 2) 其次复用正在保持的同音声部（连续快速同音）
    if (slot == nullptr) {
        for (Voice &v : s_voice) {
            if (v.note == note && v.sounding) {
                slot = &v;
                break;
            }
        }
    }
    // 3) 全忙则偷掉当前增益最小的声部
    if (slot == nullptr) {
        float min_gain = 1e30f;
        for (Voice &v : s_voice) {
            if (v.gain < min_gain) {
                min_gain = v.gain;
                slot = &v;
            }
        }
    }

    slot->note = note;
    slot->sounding = 1;
    slot->seq = ++s_seq;
    slot->freq = s_freq_table[note];
    slot->velocity = static_cast<float>(velocity);
    slot->phase = 0.0f;
}

void voice_note_off(uint8_t note) {
    Voice *best = nullptr;
    uint32_t best_seq = 0;
    for (Voice &v : s_voice) {
        if (v.note == note && v.sounding && v.seq > best_seq) {
            best_seq = v.seq;
            best = &v;
        }
    }
    if (best != nullptr) {
        best->sounding = 0;  // 进入释放；note 保留到增益归零后才可复用
    }
}

}  // namespace

float IRAM_ATTR midi_player_sample() {
    if (!s_playing) {
        return 0.0f;
    }

    const int64_t now = esp_timer_get_time();
    const int64_t elapsed = now - s_start_us;
    if (elapsed < 0) {
        return 0.0f;
    }

    float dt = static_cast<float>(now - s_last_sample_us) * 1e-6f;
    s_last_sample_us = now;
    if (dt < 0.0f) {
        dt = 0.0f;
    }
    if (dt > kMaxDt) {
        dt = kMaxDt;
    }

    // ---- 实时事件调度：到点即应用 Note On/Off ----
    while (s_event_index < s_event_count &&
           s_events[s_event_index].time_us <= static_cast<uint32_t>(elapsed)) {
        const midi::MidiEvent &e = s_events[s_event_index];
        if (e.on) {
            voice_note_on(e.note, e.velocity);
        } else {
            voice_note_off(e.note);
        }
        s_event_index++;
    }

    // ---- 目标增益：按力度加权归一化，复合峰值 <= ±100 ----
    float sum_vel = 0.0f;
    for (int i = 0; i < kVoices; i++) {
        if (s_voice[i].sounding) {
            sum_vel += s_voice[i].velocity;
        }
    }
    const float norm = (sum_vel > 0.0f) ? (kAmplitude / sum_vel) : 0.0f;
    for (int i = 0; i < kVoices; i++) {
        Voice &v = s_voice[i];
        v.target_gain = v.sounding ? (v.velocity * norm) : 0.0f;
    }

    // ---- 包络 + 相位累加 + 正弦叠加 ----
    float out = 0.0f;
    bool any_voice = false;
    for (int i = 0; i < kVoices; i++) {
        Voice &v = s_voice[i];
        if (v.note == 0xFF) {
            continue;
        }
        any_voice = true;

        // 线性包络（attack 4ms / release 30ms）
        if (v.gain < v.target_gain) {
            v.gain += (kAmplitude / kAttackS) * dt;
            if (v.gain > v.target_gain) {
                v.gain = v.target_gain;
            }
        } else if (v.gain > v.target_gain) {
            v.gain -= (kAmplitude / kReleaseS) * dt;
            if (v.gain < v.target_gain) {
                v.gain = v.target_gain;
            }
        }

        // 释放完成，声部归位
        if (!v.sounding && v.gain < kSilenceGain) {
            v.gain = 0.0f;
            v.note = 0xFF;
            continue;
        }

        // 相位累加并回绕到 [0, 2π)，避免长时间播放后 float 精度丢失
        v.phase += kTwoPi * v.freq * dt;
        if (v.phase >= kTwoPi) {
            v.phase -= kTwoPi;
        }
        if (v.phase >= kTwoPi) {
            v.phase -= kTwoPi;
        }
        out += v.gain * fast_sinf(v.phase);
    }

    // ---- 播完即停：事件全部处理完且所有声部衰减归零 ----
    if (s_event_index >= s_event_count && !any_voice) {
        s_playing = false;
        s_finished = true;
        ESP_LOGI(TAG, "Playback finished");
    }

    // 安全限幅（正常归一化下不会触发）
    if (out > kAmplitude) {
        out = kAmplitude;
    } else if (out < -kAmplitude) {
        out = -kAmplitude;
    }
    return out;
}

void midi_player_play(FocMotor *motor, const char *path) {
    if (motor == nullptr || path == nullptr) {
        ESP_LOGE(TAG, "invalid args");
        return;
    }

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s/%s", LITTLEFS_MOUNT_POINT, path);

    FILE *f = fopen(full_path, "rb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "cannot open %s", full_path);
        return;
    }
    fseek(f, 0, SEEK_END);
    const long file_size = ftell(f);
    rewind(f);
    if (file_size <= 0 || file_size > kMaxFileSize) {
        fclose(f);
        ESP_LOGE(TAG, "bad file size %ld", file_size);
        return;
    }

    std::vector<uint8_t> buffer(static_cast<size_t>(file_size));
    const size_t read_bytes = fread(buffer.data(), 1, buffer.size(), f);
    fclose(f);
    if (read_bytes != buffer.size()) {
        ESP_LOGE(TAG, "short read %u/%u", static_cast<unsigned>(read_bytes),
                 static_cast<unsigned>(buffer.size()));
        return;
    }

    ESP_LOGI(TAG, "free heap before parse: %u bytes", (unsigned)esp_get_free_heap_size());
    midi::ParseResult parsed = midi::parse_midi(buffer.data(), buffer.size());
    if (!parsed.ok) {
        ESP_LOGE(TAG, "MIDI parse failed: %s", path);
        return;
    }

    // 预计算音符频率表 (A4 = 440 Hz, MIDI note 69)
    for (int i = 0; i < 128; i++) {
        s_freq_table[i] = 440.0f * powf(2.0f, static_cast<float>(i - 69) / 12.0f);
    }

    // 重置播放状态（s_playing 最后置位，保证状态可见性顺序）
    voice_reset();
    s_parsed = std::move(parsed);
    s_events = s_parsed.events.data();
    s_event_count = s_parsed.events.size();
    s_duration_us = s_parsed.duration_us;
    s_event_index = 0;
    s_seq = 0;
    s_finished = false;
    s_start_us = esp_timer_get_time();
    s_last_sample_us = s_start_us;

    motor->set_music_source(midi_player_sample);
    s_playing = true;

    ESP_LOGI(TAG, "playing %s: %d note-on, %d note-off, %d tempo changes, %.2f s", path,
             s_parsed.note_on_count, s_parsed.note_off_count, s_parsed.tempo_change_count,
             static_cast<double>(s_duration_us) / 1e6);
}

bool midi_player_active() {
    return s_playing;
}
