#include "serial.h"
#include "log.h"
#include "win.h"

#include <algorithm>
#include <setupapi.h>

namespace {
class PortIo {
public:
    PortIo(const std::wstring& name, int speed) {
        const auto path = L"\\\\.\\" + name;
        port_.reset(CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr));
        if (!port_)
            win_error("CreateFile");
        SetupComm(port_.get(), 64 * 1024, 4096);
        baud(speed);
        COMMTIMEOUTS timeouts{};
        timeouts.ReadTotalTimeoutConstant = 8;
        timeouts.WriteTotalTimeoutConstant = 50;
        if (!SetCommTimeouts(port_.get(), &timeouts))
            win_error("SetCommTimeouts");
        PurgeComm(port_.get(), PURGE_RXCLEAR | PURGE_TXCLEAR);
    }

    void baud(int speed) {
        DCB settings{};
        settings.DCBlength = sizeof(settings);
        settings.BaudRate = static_cast<DWORD>(speed);
        settings.ByteSize = 8;
        settings.Parity = NOPARITY;
        settings.StopBits = ONESTOPBIT;
        settings.fBinary = TRUE;
        settings.XonChar = 0x11;
        settings.XoffChar = 0x13;
        if (!SetCommState(port_.get(), &settings))
            win_error("SetCommState");
        PurgeComm(port_.get(), PURGE_RXCLEAR);
    }

    Bytes read() {
        DWORD count{};
        if (!ReadFile(port_.get(), input_.data(), static_cast<DWORD>(input_.size()), &count, nullptr))
            win_error("ReadFile");
        return Bytes(input_).first(count);
    }

    void send(Bytes data) {
        DWORD count{};
        if (!WriteFile(port_.get(), data.data(), static_cast<DWORD>(data.size()), &count, nullptr))
            win_error("WriteFile");
        if (count != data.size())
            throw std::runtime_error("UART write timed out");
    }

private:
    Handle port_;
    std::array<Byte, 4096> input_{};
};

bool changes_device(Op op) {
    return op == Configure || op == Output || op == Calibrate || op == Save || op == ClearProfile || op == Reinit;
}

constexpr Ns ms = 1000000;
constexpr std::array op_names{L"", L"查询状态", L"应用通信设置", L"遥测输出", L"标定", L"保存到 Flash",
                              L"计时查询", L"清零计时", L"硬件诊断", L"读取标定", L"重新初始化"};
}

std::vector<Port> serial_ports() {
    constexpr GUID ports_guid{0x4d36e978, 0xe325, 0x11ce, {0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18}};
    const auto devices = SetupDiGetClassDevsW(&ports_guid, nullptr, nullptr, DIGCF_PRESENT);
    std::vector<Port> result;
    if (devices == INVALID_HANDLE_VALUE)
        return result;

    SP_DEVINFO_DATA dev{};
    dev.cbSize = sizeof(dev);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(devices, i, &dev); ++i) {
        wchar_t name[32]{}, label[256]{}, identity[1024]{};
        const auto key = SetupDiOpenDevRegKey(devices, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
        if (key == INVALID_HANDLE_VALUE)
            continue;
        DWORD size = sizeof(name);
        const auto read = RegGetValueW(key, nullptr, L"PortName", RRF_RT_REG_SZ, nullptr, name, &size);
        RegCloseKey(key);
        if (read != ERROR_SUCCESS || !std::wstring_view(name).starts_with(L"COM"))
            continue;
        SetupDiGetDeviceRegistryPropertyW(devices, &dev, SPDRP_FRIENDLYNAME, nullptr,
            reinterpret_cast<BYTE*>(label), sizeof(label), nullptr);
        SetupDiGetDeviceRegistryPropertyW(devices, &dev, SPDRP_HARDWAREID, nullptr,
            reinterpret_cast<BYTE*>(identity), sizeof(identity), nullptr);
        CharUpperBuffW(identity, static_cast<DWORD>(std::wcslen(identity)));
        result.push_back({name, *label ? label : name, std::wstring_view(identity).find(L"VID_1A86") != std::wstring_view::npos});
    }
    SetupDiDestroyDeviceInfoList(devices);
    std::ranges::sort(result, [](const Port& a, const Port& b) {
        if (a.ch340 != b.ch340)
            return a.ch340;
        return std::wcstol(a.name.c_str() + 3, nullptr, 10) < std::wcstol(b.name.c_str() + 3, nullptr, 10);
    });
    return result;
}

void Serial::start(Port port, int baud, bool detecting) {
    if (running_)
        return;
    if (thread_.joinable())
        thread_.join();
    requested_.reset();
    found_ = 0;
    busy_ = false;
    running_ = true;
    thread_ = std::jthread([this, port = std::move(port), baud, detecting](std::stop_token stop) {
        if (detecting)
            probe(stop, port);
        else
            run(stop, port, baud);
        running_ = false;
    });
}

bool Serial::request(Command command) {
    if (!running_ || busy_.exchange(true))
        return false;
    std::lock_guard lock(mutex_);
    requested_ = command;
    return true;
}

void Serial::note(std::wstring text, const char* level) {
    const auto ns = now_ns();
    recorder_.event(ns, text, level);
    std::lock_guard lock(model_.mutex);
    model_.note(std::move(text), ns, level);
}

void Serial::run(std::stop_token stop, Port port, int baud) {
    struct Pending {
        Command command;
        std::uint32_t seq;
        Ns deadline;
        bool user;
    };
    std::optional<Pending> pending;
    std::optional<Config> verifying;
    Ns status_due{}, diag_due{}, timing_due{}, baud_ready{};
    std::uint32_t seq = static_cast<std::uint32_t>(now_ns());
    Byte stage{};
    bool params_due = true;
    std::wstring close_text = L"串口已断开";

    try {
        PortIo io(port.name, baud);
        Parser parser;
        {
            std::lock_guard lock(model_.mutex);
            model_.state.port_open = true;
            model_.state.link = port.name + L" · 等待协议应答";
        }
        note(L"打开 " + port.name + L" · " + std::to_wstring(baud) + L" baud");
        while (!stop.stop_requested()) {
            const auto input = io.read();
            const auto now = now_ns();
            std::optional<Pending> completed;
            Byte result{};
            unsigned cal_before{}, cal_after{}, cal_error{};
            std::optional<Status> status;
            bool confirmed{};
            {
                std::lock_guard lock(model_.mutex);
                auto& state = model_.state;
                if (state.status)
                    cal_before = (*state.status)[Status::CalState];
                parser.feed(input, [&](Frame frame) {
                    const auto p = frame.payload();
                    bool accepted = frame.kind() == 1 || frame.kind() == 0x12;
                    if (frame.kind() == 0x11 && pending && !completed && now <= pending->deadline &&
                        frame.seq() == pending->seq && p[0] == pending->command.op) {
                        if (valid_reply(p)) {
                            completed = pending;
                            result = p[1];
                            accepted = true;
                        } else {
                            ++state.invalid;
                        }
                    }
                    if (accepted && !model_.consume(frame, now)) {
                        ++state.invalid;
                        accepted = false;
                    }
                    recorder_.frame(now, false, frame.wire, accepted);
                });
                state.parse = parser.counts;
                if (state.status) {
                    status = state.status;
                    cal_after = (*status)[Status::CalState];
                    cal_error = (*status)[Status::CalError];
                }
                confirmed = state.config_confirmed;
            }

            if (completed) {
                pending.reset();
                const auto op = completed->command.op;
                bool hold_busy = false;
                if (result) {
                    if (completed->user)
                        note(std::wstring(op_names[op]) + L" · " + result_text(result), "WARN");
                    if (verifying && op == Query) {
                        verifying.reset();
                        busy_ = false;
                        note(L"通信设置读回失败", "WARN");
                    }
                } else if (op == Query && status) {
                    const bool v1 = (*status)[Status::Version] == 1;
                    const auto config = status->config();
                    const bool matches = config.baud == baud && (!verifying || config == *verifying);
                    {
                        std::lock_guard lock(model_.mutex);
                        model_.state.config_confirmed = v1 && matches;
                        model_.state.link = port.name + (v1 ? L" · 协议 v1" : L" · 协议版本不支持");
                        if (cal_before != cal_after && cal_after == 5)
                            model_.state.saved = L"标定参数尚未保存";
                    }
                    if (verifying) {
                        note(matches ? L"通信设置读回一致" : L"通信设置读回不符", matches ? "INFO" : "WARN");
                        verifying.reset();
                        busy_ = false;
                    }
                    if (cal_before != cal_after && (cal_after == 5 || cal_after == 6)) {
                        params_due = true;
                        note(cal_after == 5 ? L"标定完成" : std::wstring(L"标定失败 · ") + result_text(cal_error));
                    }
                } else if (op == Configure) {
                    verifying = command_config(completed->command.args);
                    hold_busy = true;
                    {
                        std::lock_guard lock(model_.mutex);
                        model_.state.config_confirmed = false;
                        model_.state.saved = L"通信设置尚未保存";
                    }
                    if (verifying->baud != baud) {
                        baud = verifying->baud;
                        io.baud(baud);
                        parser.boundary();
                        baud_ready = now_ns() + 30 * ms;
                    }
                    status_due = 0;
                    note(L"通信设置已接受，正在读回");
                } else if (changes_device(op)) {
                    status_due = 0;
                    if (op == Save) {
                        std::lock_guard lock(model_.mutex);
                        model_.state.saved = L"Flash 已保存并校验";
                        params_due = true;
                    }
                    note(std::wstring(op_names[op]) + (op == Calibrate ? L" · 命令已接受" : L" · 成功"));
                } else if (completed->user) {
                    note(std::wstring(op_names[op]) + L" · 成功");
                }
                if (completed->user && !hold_busy)
                    busy_ = false;
            }

            if (pending && now > pending->deadline) {
                const auto op = pending->command.op;
                if (pending->user || verifying) {
                    note(std::wstring(op_names[op]) + L" · 应答超时，结果未确认", "WARN");
                    busy_ = false;
                    if (op == Save || op == Configure) {
                        std::lock_guard lock(model_.mutex);
                        model_.state.saved = L"应答超时，保存状态未确认";
                        if (op == Configure)
                            model_.state.config_confirmed = false;
                    }
                    verifying.reset();
                    status_due = 0;
                }
                pending.reset();
            }
            if (pending || now < baud_ready)
                continue;

            std::optional<Command> next;
            {
                std::lock_guard lock(mutex_);
                next = std::exchange(requested_, std::nullopt);
            }
            const bool user = next.has_value();
            if (next && changes_device(next->op)) {
                const wchar_t* rejection = nullptr;
                if (!status || (*status)[Status::Version] != 1)
                    rejection = L"等待协议 v1 应答";
                else if (next->op == Save && !confirmed)
                    rejection = L"通信设置尚未读回确认";
                else if (next->op == Configure) {
                    const auto config = command_config(next->args);
                    rejection = config_error(config);
                    if (!rejection && port.ch340 && config.baud > 2000000)
                        rejection = L"CH340 支持 1 或 2 Mbps";
                }
                if (rejection) {
                    note(rejection, "WARN");
                    busy_ = false;
                    continue;
                }
            }
            if (!next && now >= status_due) {
                next = Command{Query};
                status_due = now + (cal_after >= 1 && cal_after <= 4 ? 200 : 1000) * ms;
            } else if (!next && now >= diag_due) {
                next = Command{Diagnose};
                diag_due = now + 1000 * ms;
            } else if (!next && params_due) {
                next = Command{Parameters};
                params_due = false;
            } else if (!next && profiling && now >= timing_due) {
                next = Command{Profile, {stage}};
                stage = static_cast<Byte>((stage + 1) % stage_names.size());
                timing_due = now + 200 * ms;
            }
            if (next) {
                const auto wire = command_frame(++seq, *next);
                io.send(wire);
                const auto sent = now_ns();
                {
                    std::lock_guard lock(model_.mutex);
                    model_.consume({wire}, sent);
                }
                recorder_.frame(sent, true, wire);
                pending = Pending{*next, seq, sent + (next->op == Save ? 1000 : 100) * ms, user};
            }
        }
    } catch (const std::exception& error) {
        close_text = L"串口错误 · " + wide(error.what());
        note(close_text, "ERROR");
    }
    note(L"串口已关闭");
    std::lock_guard lock(model_.mutex);
    recorder_.stop();
    model_.state.port_open = model_.state.config_confirmed = false;
    model_.state.link = std::move(close_text);
    busy_ = false;
}

void Serial::probe(std::stop_token stop, const Port& port) {
    constexpr std::array speeds{2000000, 1000000, 3000000, 6000000, 115200};
    std::wstring error;
    for (const auto speed : speeds) {
        if (stop.stop_requested())
            return;
        if (port.ch340 && speed > 2000000)
            continue;
        try {
            PortIo io(port.name, speed);
            Parser parser;
            const auto seq = static_cast<std::uint32_t>(now_ns());
            io.send(command_frame(seq, {Query}));
            const auto until = now_ns() + 320 * ms;
            while (!stop.stop_requested() && now_ns() < until && !found_) {
                parser.feed(io.read(), [&](Frame frame) {
                    const auto p = frame.payload();
                    if (frame.kind() == 0x11 && frame.seq() == seq && p[0] == Query && !p[1] &&
                        valid_reply(p) && u32(p, 2) == 1 && u32(p, 6) == static_cast<unsigned>(speed))
                        found_ = speed;
                });
            }
            if (found_) {
                note(L"探测到 " + port.name + L" · " + std::to_wstring(speed) + L" baud");
                return;
            }
        } catch (const std::exception& e) {
            error = wide(e.what());
        }
    }
    note(error.empty() ? L"未探测到协议 v1" : L"未探测到协议 v1 · " + error, "WARN");
}
