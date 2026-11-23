#pragma once

#include "protocol.h"

#include <deque>
#include <mutex>
#include <optional>
#include <string>

using Ns = std::int64_t;
Ns now_ns();

struct Status {
    enum Field { Version, Baud, Rate, Mask, Flags, Frames, Stamp, Heartbeat, Id, Phase, Error,
                 Missed, Late, NoData, Invalid, DmaTimeouts, DmaErrors, Overlaps, TxDropped, TxErrors,
                 RxErrors, CrcErrors, CalMode, CalState, Poses, Samples, Stable, TotalSamples, CalError,
                 SolveSteps, SaveResult };
    std::array<std::uint32_t, 31> fields{};
    float rms{}, max_error{};

    auto operator[](Field field) const { return fields[field]; }
    Config config() const;
};

struct Diagnostic {
    enum Field { Version, BootError, CoreHz, Phase, Error, Id, Edges, Fallback, LastStatus,
                 Pb0, SampleAge, Flags, ConfigIndex, Frames };
    std::array<std::uint32_t, diag_keys.size()> fields{};
    auto operator[](Field field) const { return fields[field]; }
};

struct Calibration {
    std::uint32_t flags{};
    std::array<float, 15> values{};
};

struct Timing {
    std::uint32_t core{}, elapsed{}, count{}, last{}, min{}, max{};
    std::uint64_t idle{}, total{};
};

struct Event {
    std::uint64_t id;
    Ns ns;
    std::string level;
    std::wstring text;
};

struct Snapshot {
    std::uint64_t generation{}, total{}, missing{}, invalid{}, status_serial{};
    Ns origin{}, last_data{}, last_rx{};
    double rate{}, cpu{};
    ParseCounts parse{};
    std::optional<Sample> latest;
    std::optional<Status> status;
    std::optional<Diagnostic> diagnostic;
    std::optional<Calibration> calibration;
    std::array<Timing, stage_names.size()> timing{};
    std::wstring link{L"未连接"}, saved{L"保存状态未确认"};
    std::wstring interrupt;
    bool port_open{}, config_confirmed{}, replay{};
};

class SampleClock {
public:
    void boundary() { gap_ = true; }
    std::uint32_t stamp(Sample& sample, double host_seconds, unsigned step);

private:
    std::uint32_t sequence_{}, stamp_{};
    std::uint64_t expanded_{};
    double time_{};
    bool started_{}, gap_{true};
};

class Model {
public:
    static constexpr std::size_t capacity = 65536;

    Model() { reset(); }

    // The producer holds mutex for a batch of frames. Disk and serial I/O stay outside the lock.
    std::mutex mutex;
    Snapshot state;
    std::deque<Event> events;
    void reset(Ns origin = now_ns());
    void boundary() { clock_.boundary(); }
    bool consume(Frame frame, Ns ns);
    void note(std::wstring text, Ns ns, const char* level = "INFO");
    void clear_timing();
    void copy_since(std::uint64_t generation, std::uint64_t& cursor, std::vector<Sample>& out) const;

private:
    std::vector<Sample> samples_ = std::vector<Sample>(capacity);
    SampleClock clock_;
    std::optional<std::pair<std::uint32_t, Config>> requested_;
    std::optional<Timing> last_profile_;
    Ns rate_since_{};
    std::uint64_t rate_count_{}, event_id_{};
    void apply(Op op, Bytes data);
};
