#pragma once

#include "model.h"

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <thread>

namespace fs = std::filesystem;

struct RecordStatus {
    bool active{}, running{};
    std::uint64_t frames{};
    std::wstring error;
};

class Recorder {
public:
    ~Recorder();
    void start(fs::path path, const Snapshot& snapshot, std::wstring port);
    void stop();
    void frame(Ns ns, bool outgoing, Bytes wire, bool accepted = true);
    void event(Ns ns, const std::wstring& text, const char* level);
    RecordStatus status();

private:
    struct Entry {
        Ns ns{};
        std::array<Byte, 144> wire{};
        std::uint16_t size{};
        bool outgoing{}, accepted{};
        std::string level;
        std::wstring text;
    };
    void put(Entry entry);
    void run(fs::path path, Snapshot snapshot, std::wstring port);
    std::mutex mutex_;
    std::condition_variable ready_;
    std::vector<Entry> queue_;
    std::size_t bytes_{};
    RecordStatus state_;
    std::atomic<bool> active_{};
    std::thread thread_;
};

struct ReplayStatus {
    double position{}, duration{};
    bool ready{}, playing{}, running{}, complete{};
    std::wstring error;
};

class Replay {
public:
    explicit Replay(Model& model) : model_(model) {}
    ~Replay();
    void start(fs::path path);
    void stop();
    void toggle();
    void seek(double seconds);
    void speed(double rate);
    ReplayStatus status();

private:
    void run(std::stop_token stop, fs::path path);
    Model& model_;
    std::mutex mutex_;
    std::condition_variable_any ready_;
    ReplayStatus state_;
    std::optional<double> seek_;
    double speed_ = 1;
    std::jthread thread_;
};

struct ExportStatus {
    std::uint64_t rows{};
    bool running{};
    std::wstring message;
};

class Export {
public:
    void start(fs::path source, fs::path csv, double from, double to);
    void stop() { thread_.request_stop(); }
    ExportStatus status();

private:
    void run(std::stop_token stop, fs::path source, fs::path csv, double from, double to);
    std::mutex mutex_;
    ExportStatus state_;
    std::jthread thread_;
};
