//
// 主机侧解析测试（不参与 IDF 构建）：
//
//   clang++ -std=c++17 -I components/midi_player/include \
//       components/midi_player/midi_parser.cpp \
//       components/midi_player/tools/midi_parser_test.cpp \
//       -o /tmp/midi_parser_test
//   /tmp/midi_parser_test littlefs_root/Unravel.mid
//

#include "midi_parser.h"

#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>

static int failures = 0;

static void check(bool cond, const char *msg) {
    if (cond) {
        printf("ok  : %s\n", msg);
    } else {
        printf("FAIL: %s\n", msg);
        failures++;
    }
}

int main(int argc, char **argv) {
    const char *path = (argc > 1) ? argv[1] : "littlefs_root/Unravel.mid";

    FILE *f = fopen(path, "rb");
    if (f == nullptr) {
        printf("cannot open %s\n", path);
        return 2;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    rewind(f);
    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    const size_t rd = fread(buffer.data(), 1, buffer.size(), f);
    fclose(f);
    if (rd != buffer.size()) {
        printf("short read\n");
        return 2;
    }

    const midi::ParseResult r = midi::parse_midi(buffer.data(), buffer.size());
    check(r.ok, "parse ok");
    if (!r.ok) {
        printf("FAILURES: %d\n", failures);
        return 1;
    }

    printf("summary: on=%d off=%d tempo=%d duration=%.6f s events=%zu\n", r.note_on_count,
           r.note_off_count, r.tempo_change_count, static_cast<double>(r.duration_us) / 1e6,
           r.events.size());

    check(r.note_on_count == 3426, "3426 note-on");
    check(r.note_off_count == 3426, "3426 note-off");
    check(r.tempo_change_count == 3, "3 tempo changes");
    check(r.duration_us >= 236000000u && r.duration_us <= 237500000u, "duration ~236.6s");

    bool sorted = true;
    uint8_t min_note = 255, max_note = 0;
    for (size_t i = 1; i < r.events.size(); i++) {
        if (r.events[i].time_us < r.events[i - 1].time_us) {
            sorted = false;
            break;
        }
    }
    for (const midi::MidiEvent &e : r.events) {
        if (e.note < min_note) {
            min_note = e.note;
        }
        if (e.note > max_note) {
            max_note = e.note;
        }
    }
    check(sorted, "events sorted by time_us");
    check(min_note == 26 && max_note == 103, "note range 26..103");
    check(r.events.front().time_us == 222222u, "first note event at 240 ticks == 222222 us");

    std::set<uint8_t> active;
    size_t max_poly = 0;
    for (const midi::MidiEvent &e : r.events) {
        if (e.on) {
            active.insert(e.note);
        } else {
            active.erase(e.note);
        }
        if (active.size() > max_poly) {
            max_poly = active.size();
        }
    }
    printf("max simultaneous distinct notes: %zu\n", max_poly);
    check(max_poly >= 1 && max_poly <= 8, "max distinct polyphony 8, voice cap 12 has margin");

    if (failures == 0) {
        printf("PASS: all checks passed\n");
        return 0;
    }
    printf("FAILURES: %d\n", failures);
    return 1;
}
