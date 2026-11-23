#pragma once

#include "model.h"

#include <atomic>
#include <thread>

class Recorder;

struct Port {
    std::wstring name, label;
    bool ch340{};
};

std::vector<Port> serial_ports();

class Serial {
public:
    Serial(Model& model, Recorder& recorder) : model_(model), recorder_(recorder) {}
    void start(Port port, int baud, bool probe = false);
    void stop() { thread_.request_stop(); }
    bool request(Command command);
    bool running() const { return running_; }
    bool busy() const { return busy_; }
    int probe_result() const { return found_; }
    std::atomic<bool> profiling{};

private:
    void run(std::stop_token stop, Port port, int baud);
    void probe(std::stop_token stop, const Port& port);
    void note(std::wstring text, const char* level = "INFO");
    Model& model_;
    Recorder& recorder_;
    std::mutex mutex_;
    std::optional<Command> requested_;
    std::atomic<bool> running_{}, busy_{};
    std::atomic<int> found_{};
    std::jthread thread_;
};
