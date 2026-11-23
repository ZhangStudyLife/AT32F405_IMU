#pragma once

#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct Release {
    std::string tag, digest;
    std::size_t bytes{};
};

enum class UpdatePhase { Idle, Checking, Current, Available, Downloading, Ready, Failed };

struct UpdateStatus {
    UpdatePhase phase{};
    std::optional<Release> release;
    std::wstring message;
};

class Updater {
public:
    ~Updater();
    void check();
    void download();
    bool restart();
    void stop() { worker_.request_stop(); }
    UpdateStatus status();

private:
    void set(UpdatePhase phase, std::wstring message = {});
    std::mutex mutex_;
    UpdateStatus state_;
    std::filesystem::path staged_;
    std::jthread worker_;
};

std::optional<Release> latest_release(std::string_view json, std::string_view current, std::size_t* count = nullptr);
std::string sha256(std::string_view data);
int apply_update(const wchar_t* parent, const std::filesystem::path& target);
void remove_update(const wchar_t* parent, const std::filesystem::path& staged);
