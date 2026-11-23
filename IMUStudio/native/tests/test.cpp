#include "log.h"
#include "update.h"
#include "win.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <fstream>
#include <iostream>

namespace {
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

std::vector<Byte> sample_frame(std::uint32_t seq, std::uint32_t stamp, Byte mask) {
    std::vector<Byte> payload(frame_size(mask) - 10);
    const auto put = [&](std::size_t at, std::uint32_t value) {
        for (int i = 0; i < 4; ++i)
            payload[at + i] = static_cast<Byte>(value >> (8 * i));
    };
    put(0, stamp);
    payload[4] = output_on | valid_attitude;
    payload[5] = mask;
    int at = 6;
    for (unsigned group = 0; group < group_size.size(); ++group)
        if (mask & (1 << group))
            for (int i = 0; i < group_size[group]; ++i, at += 4)
                put(at, std::bit_cast<std::uint32_t>(static_cast<float>(group_start[group] + i + 1)));
    return encode(1, seq, payload);
}

void protocol() {
    constexpr std::array<Byte, 18> fixture{0xa5, 0x5a, 0x10, 8, 5, 0, 0, 0, 2, 0x80, 0x84,
        0x1e, 0, 0xe8, 3, 0x0f, 0x66, 0xfa};
    const auto command = command_frame(5, configure({2000000, 1000, 15}));
    require(std::ranges::equal(command, fixture), "firmware configure vector");
    require(config_error({1000000, 1000, 15}) && !config_error({2000000, 1000, 15}), "80% bandwidth");

    Parser parser;
    int parsed = 0;
    for (Byte mask = 1; mask <= 15; ++mask) {
        auto bad = sample_frame(mask, mask * 1000, mask);
        const auto wire = bad;
        bad.back() ^= 1;
        parser.feed(bad, [&](Frame) { require(false, "bad CRC accepted"); });
        for (const auto b : wire)
            parser.feed(Bytes(&b, 1), [&](Frame f) {
                Sample row;
                require(decode_sample(f, row) && row.mask == mask && row.seq == mask, "split telemetry");
                for (unsigned group = 0; group < group_size.size(); ++group)
                    for (int i = group_start[group]; i < group_start[group] + group_size[group]; ++i)
                        require(mask & (1 << group) ? row.values[i] == i + 1 : std::isnan(row.values[i]), "mask layout");
                ++parsed;
            });
    }
    require(parsed == 15 && parser.counts.crc == 15, "stream resync");

    SampleClock clock;
    Sample before, after, lost;
    before.seq = 0xfffffffe;
    before.stamp = 0xfffff800;
    after.seq = 2;
    after.stamp = 1952;
    lost.seq = 14;
    lost.stamp = 13952;
    clock.stamp(before, 0, 4);
    require(clock.stamp(after, 0.004, 4) == 0 && !after.gap && after.device_us - before.device_us == 4000,
        "wrap at 250 Hz");
    require(clock.stamp(lost, 0.016, 4) == 2 && lost.gap, "true gap");
    clock.boundary();
    lost.seq = 100;
    lost.stamp = 100000;
    require(clock.stamp(lost, 0.1, 4) == 0 && lost.gap, "pause boundary");
}

template<class Predicate>
void wait_until(Predicate predicate) {
    for (int i = 0; i < 500; ++i) {
        if (predicate())
            return;
        std::this_thread::sleep_for(10ms);
    }
    throw std::runtime_error("worker did not finish");
}

void logs() {
    const auto directory = fs::temp_directory_path() /
        (L"IMUStudio 日志 " + std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(now_ns()));
    fs::create_directory(directory);
    const auto log = directory / L"实测.imulog", csv = directory / L"实测.csv";
    {
        Model model;
        Recorder recorder;
        recorder.start(log, model.state, L"COM99");
        const auto start = now_ns();
        for (std::uint32_t i = 0; i < 140; ++i)
            recorder.frame(start + i * 1000000, false, sample_frame(i, i * 1000, 1));
        recorder.event(start + 50000000, L"测试，含 \" 引号", "INFO");
        recorder.stop();
        wait_until([&] { return !recorder.status().running; });
        require(recorder.status().error.empty() && recorder.status().frames == 140, "record flush");

        Replay replay(model);
        replay.start(log);
        wait_until([&] { return replay.status().ready || !replay.status().running; });
        require(replay.status().ready && replay.status().complete, "replay metadata");
        replay.seek(0.129);
        wait_until([&] { return replay.status().position > 0.128; });
        {
            std::lock_guard lock(model.mutex);
            require(model.state.latest && model.state.latest->seq == 129, "indexed seek");
        }
        replay.stop();
        wait_until([&] { return !replay.status().running; });

        Export exporter;
        exporter.start(log, csv, 0.03, 0.08);
        wait_until([&] { return !exporter.status().running; });
        require(exporter.status().rows == 51 && fs::exists(csv), "CSV range");
        std::ifstream file(csv, std::ios::binary);
        std::string header;
        std::getline(file, header);
        require(header.starts_with("\xef\xbb\xbf" "session_s,device_us,seq,flags,mask,roll_deg"), "CSV columns");
    }
    fs::remove(csv);
    fs::remove(directory / L"实测_events.csv");
    fs::remove(log);
    fs::remove(directory / L"实测.imulog-shm");
    fs::remove(directory / L"实测.imulog-wal");
    fs::remove(directory);
}

void update() {
    const auto digest = sha256("abc");
    require(digest == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "SHA-256");
    const auto entry = [&](std::string tag, bool prerelease = false, std::string checksum = {}) {
        return "{\"tag_name\":\"" + tag + "\",\"draft\":false,\"prerelease\":" + (prerelease ? "true" : "false") +
            ",\"assets\":[{\"name\":\"IMUStudio.exe\",\"state\":\"uploaded\",\"size\":3,\"digest\":\"sha256:" +
            (checksum.empty() ? digest : checksum) + "\"}]}";
    };
    const auto json = "[" + entry("imustudio-v1.10.0") + "," + entry("imustudio-v1.9.0") + "," +
        entry("v99.0.0") + "," + entry("imustudio-v2.0.0", true) + "," + entry("imustudio-v9.0.0", false, "bad") + "]";
    const auto latest = latest_release(json, "1.9.0");
    require(latest && latest->tag == "imustudio-v1.10.0" && latest->digest == digest, "release selection");
    require(!latest_release(json, "1.10.0"), "version must increase");
}
}

int main() {
    try {
        protocol();
        logs();
        update();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
