#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

using Byte = std::uint8_t;
using Bytes = std::span<const Byte>;
enum Op : Byte { Query = 1, Configure, Output, Calibrate, Save, Profile, ClearProfile, Diagnose, Parameters, Reinit };
inline constexpr std::array baud_rates{1000000, 2000000, 3000000, 6000000, 115200};
inline constexpr std::array sample_rates{10, 25, 50, 100, 200, 250, 500, 1000};
inline constexpr std::array group_start{0, 3, 7, 13};
inline constexpr std::array group_size{3, 4, 6, 6};
inline constexpr std::array columns{
    "roll_deg", "pitch_deg", "yaw_deg", "qw", "qx", "qy", "qz",
    "gx_dps", "gy_dps", "gz_dps", "ax_g", "ay_g", "az_g",
    "filtered_gx_dps", "filtered_gy_dps", "filtered_gz_dps",
    "filtered_ax_g", "filtered_ay_g", "filtered_az_g"
};
inline constexpr std::array status_keys{
    "version", "baud", "rate", "mask", "flags", "frames", "time_us", "heartbeat", "id",
    "sensor_phase", "sensor_error", "missed", "late", "no_data", "invalid", "dma_timeouts",
    "dma_errors", "overlaps", "tx_dropped", "tx_errors", "rx_errors", "crc_errors",
    "cal_mode", "cal_state", "poses", "samples", "stable", "total_samples", "cal_error",
    "solve_steps", "save_result", "fit_rms", "max_norm_error"
};
inline constexpr std::array diag_keys{
    "diagnostic_version", "boot_error", "core_hz", "sensor_phase", "sensor_error", "who_am_i",
    "drdy_edges", "fallback_reads", "last_status", "pb0_level", "sample_age_us", "flags",
    "config_index", "frames"
};
inline constexpr std::array stage_names{
    L"采集延迟", L"采集驱动", L"换算与校正", L"滤波", L"Mahony", L"姿态发布", L"标定采样",
    L"求解步骤", L"完整求解", L"整帧延迟", L"主循环", L"中断", L"Flash 保存", L"串口处理"
};
inline constexpr Byte valid_attitude = 1, sensor_fault = 32, warming_up = 64, output_on = 128;

struct Config {
    int baud = 2000000, rate = 1000, mask = 1;
    bool operator==(const Config&) const = default;
};

struct Command {
    Op op;
    std::array<Byte, 7> args{};
};

struct Frame {
    std::uint32_t seq() const;
    Byte kind() const { return wire[2]; }
    Bytes payload() const { return wire.subspan(8, wire[3]); }
    Bytes wire;
};

struct ParseCounts {
    std::uint64_t crc{}, length{}, discarded{};
};

class Parser {
public:
    Parser() { buffered_.reserve(8192); }
    void feed(Bytes bytes, const std::function<void(Frame)>& receive);
    void boundary() { buffered_.clear(); }
    ParseCounts counts;

private:
    std::vector<Byte> buffered_;
};

struct Sample {
    double time{};
    std::uint64_t device_us{};
    std::uint32_t seq{}, stamp{};
    Byte flags{}, mask{};
    bool gap{};
    std::array<float, columns.size()> values{};
};

int frame_size(int mask);
const wchar_t* config_error(Config config);
const wchar_t* result_text(unsigned result);
bool decode_sample(Frame frame, Sample& sample);
bool valid_reply(Bytes payload);
std::vector<Byte> encode(Byte kind, std::uint32_t seq, Bytes payload);
std::vector<Byte> command_frame(std::uint32_t seq, const Command& command);
Command configure(Config config);
Config command_config(Bytes args);
std::uint16_t crc16(Bytes bytes);
std::uint32_t u32(Bytes bytes, std::size_t at);
std::uint64_t u64(Bytes bytes, std::size_t at);
float f32(Bytes bytes, std::size_t at);
