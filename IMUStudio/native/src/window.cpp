#include "draw.h"
#include "log.h"
#include "serial.h"
#include "update.h"
#include "version.h"

#include <algorithm>
#include <cmath>
#include <commctrl.h>
#include <commdlg.h>
#include <iomanip>
#include <limits>
#include <shellapi.h>
#include <shlobj.h>
#include <sstream>
#include <uxtheme.h>
#include <windowsx.h>

namespace {
enum Page { Live, Wave, Cal, Diagnostics, Logs };
enum Id {
    VersionButton = 100, LiveTab, WaveTab, CalTab, DiagTab, LogTab,
    PortBox, LinkBaud, Refresh, Probe, Connect,
    DeviceBaud, DeviceRate, Euler, Quaternion, Raw, Filtered, Apply, PauseOutput, SaveFlash,
    PlotGroup, TimeBox, Freeze, Follow, ViewReset,
    GyroCal, AccelCal, CapturePose, SolveCal, CancelCal, ReadParams,
    ReadStatus, ReadDiag, ResetTiming, Reinitialize, FieldsBox, TimingBox,
    RecordLog, OpenLog, CloseLog, PlayLog, PlaySpeed, Position, From, To, ExportCsv, CancelExport, EventBox, EndId
};
constexpr std::array baud_labels{L"1 Mbps", L"2 Mbps", L"3 Mbps", L"6 Mbps", L"115200"};
constexpr std::array windows_sec{2, 5, 10, 20, 30, 60};
constexpr std::array replay_speeds{0.25, 0.5, 1.0, 2.0, 4.0};
constexpr auto read_only_edit = WS_VSCROLL | ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY;
constexpr UINT single = DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS;

std::wstring get_text(HWND hwnd) {
    std::wstring text(static_cast<std::size_t>(GetWindowTextLengthW(hwnd)) + 1, L'\0');
    text.resize(GetWindowTextW(hwnd, text.data(), static_cast<int>(text.size())));
    return text;
}

void text_if_changed(HWND hwnd, const std::wstring& text) {
    if (get_text(hwnd) != text)
        SetWindowTextW(hwnd, text.c_str());
}

void edit_text(HWND hwnd, const std::wstring& text) {
    if (get_text(hwnd) == text)
        return;
    const auto line = SendMessageW(hwnd, EM_GETFIRSTVISIBLELINE, 0, 0);
    SetWindowTextW(hwnd, text.c_str());
    SendMessageW(hwnd, EM_LINESCROLL, 0, line);
}

fs::path documents() {
    PWSTR path{};
    if (FAILED(SHGetKnownFolderPath(FOLDERID_Documents, 0, nullptr, &path)))
        return {};
    const fs::path logs = fs::path(path) / L"IMUStudio" / L"Logs";
    CoTaskMemFree(path);
    std::error_code error;
    fs::create_directories(logs, error);
    return error ? fs::path() : logs;
}

fs::path pick_file(HWND owner, bool save, bool csv, const fs::path& suggested = {}) {
    const auto name = suggested.filename().wstring();
    const auto folder = suggested.parent_path().wstring();
    std::wstring buffer(32768, L'\0');
    std::ranges::copy(name, buffer.begin());
    OPENFILENAMEW options{};
    options.lStructSize = sizeof(options);
    options.hwndOwner = owner;
    options.lpstrFilter = csv ? L"CSV (*.csv)\0*.csv\0\0" : L"IMUStudio 日志 (*.imulog)\0*.imulog\0\0";
    options.lpstrFile = buffer.data();
    options.nMaxFile = static_cast<DWORD>(buffer.size());
    options.lpstrInitialDir = folder.empty() ? nullptr : folder.c_str();
    options.lpstrDefExt = csv ? L"csv" : L"imulog";
    options.Flags = OFN_EXPLORER | OFN_NOCHANGEDIR | OFN_PATHMUSTEXIST |
        (save ? (csv ? OFN_OVERWRITEPROMPT : 0) : OFN_FILEMUSTEXIST);
    if (!(save ? GetSaveFileNameW(&options) : GetOpenFileNameW(&options)))
        return {};
    return fs::path(buffer.c_str());
}

std::wstring log_name() {
    SYSTEMTIME now{};
    GetLocalTime(&now);
    wchar_t name[48]{};
    std::swprintf(name, std::size(name), L"IMU-%04u%02u%02u-%02u%02u%02u.imulog",
                  now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond);
    return name;
}

class App {
public:
    explicit App(HINSTANCE instance) : module_(instance), board_(instance) { delta_.reserve(Model::capacity); }
    int run(int show, bool smoke);

private:
    static LRESULT CALLBACK dispatch(HWND window, UINT msg, WPARAM w, LPARAM l);
    LRESULT handle(UINT msg, WPARAM w, LPARAM l);
    HWND ctrl(Id id) const { return controls_[id - VersionButton]; }
    void create();
    void fonts(int dpi);
    void layout();
    void tab(Page page);
    void refresh_ports();
    void tick(bool force = false);
    void set_refresh();
    void sync_controls();
    void diagnostics();
    void log_events();
    void paint(HDC dc, RECT affected);
    void draw_button(const DRAWITEMSTRUCT& item);
    void command(Id id, int code);
    void send(Command request);
    void notice(std::wstring message);
    void close(bool restart = false);
    void mouse_move(POINT at);
    void mouse_down(POINT at);
    void wheel(POINT at, int delta, bool vertical);
    void seek();
    void invalidate() { InvalidateRect(hwnd_, nullptr, FALSE); }
    void change(Id id, const std::wstring& label) { text_if_changed(ctrl(id), label); }
    void enable(Id id, bool yes) {
        if (static_cast<bool>(IsWindowEnabled(ctrl(id))) != yes)
            EnableWindow(ctrl(id), yes);
    }
    void select(Id id, int index) { SendMessageW(ctrl(id), CB_SETCURSEL, index, 0); }
    int selected(Id id) const { return static_cast<int>(SendMessageW(ctrl(id), CB_GETCURSEL, 0, 0)); }
    bool checked(Id id) const { return SendMessageW(ctrl(id), BM_GETCHECK, 0, 0) == BST_CHECKED; }
    void check(Id id, bool value) { SendMessageW(ctrl(id), BM_SETCHECK, value ? BST_CHECKED : BST_UNCHECKED, 0); }
    void add(Id id, const wchar_t* label) { SendMessageW(ctrl(id), CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label)); }
    Config edited_config() const;

    HINSTANCE module_;
    HWND hwnd_{};
    std::array<HWND, EndId - VersionButton> controls_{};
    Fonts fonts_;
    Handle refresh_tick_;
    Buffer buffer_;
    Model model_;
    Recorder recorder_;
    Serial serial_{model_, recorder_};
    Replay replay_{model_};
    Export exporter_;
    Updater updater_;
    Snapshot view_;
    ReplayStatus playing_;
    RecordStatus recording_;
    ExportStatus exporting_;
    UpdateStatus updating_;
    History history_;
    Charts charts_;
    Board board_;
    std::vector<Sample> delta_;
    std::vector<Port> ports_;
    std::uint64_t generation_{}, consumed_{}, latest_event_{}, rendered_event_{}, event_generation_{}, displayed_status_{};
    std::optional<Config> displayed_config_;
    fs::path log_path_;
    std::wstring notice_, update_message_;
    Page page_ = Live;
    RECT board_card_{}, contents_{};
    POINT drag_at_{};
    Ns next_controls_{};
    int plot_group_{}, dragged_ = -1, interval_ = 125, width_{}, height_{};
    bool config_dirty_{}, probing_{}, has_seek_{}, closing_{}, restart_{}, stale_{true};
};

LRESULT CALLBACK App::dispatch(HWND window, UINT msg, WPARAM w, LPARAM l) {
    auto* app = reinterpret_cast<App*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    if (msg == WM_NCCREATE) {
        app = static_cast<App*>(reinterpret_cast<const CREATESTRUCTW*>(l)->lpCreateParams);
        app->hwnd_ = window;
        SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
    }
    return app ? app->handle(msg, w, l) : DefWindowProcW(window, msg, w, l);
}

int App::run(int show, bool smoke) {
    INITCOMMONCONTROLSEX common{sizeof(common), ICC_BAR_CLASSES};
    InitCommonControlsEx(&common);
    WNDCLASSEXW cls{};
    cls.cbSize = sizeof(cls);
    cls.style = CS_DBLCLKS;
    cls.lpfnWndProc = dispatch;
    cls.hInstance = module_;
    cls.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    cls.hIcon = LoadIconW(module_, MAKEINTRESOURCEW(100));
    cls.lpszClassName = L"IMUStudio";
    if (!RegisterClassExW(&cls))
        win_error("RegisterClassEx");
    RECT work{};
    SystemParametersInfoW(SPI_GETWORKAREA, 0, &work, 0);
    const int dpi = static_cast<int>(GetDpiForSystem());
    const int width = std::min(static_cast<int>(work.right - work.left), MulDiv(1200, dpi, 96));
    const int height = std::min(static_cast<int>(work.bottom - work.top), MulDiv(820, dpi, 96));
    if (!CreateWindowExW(WS_EX_CONTROLPARENT, cls.lpszClassName, L"IMUStudio",
            WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN, work.left + (work.right - work.left - width) / 2,
            work.top + (work.bottom - work.top - height) / 2, width, height,
            nullptr, nullptr, module_, this))
        win_error("CreateWindowEx");
    if (smoke) {
        if (!board_.ready() || std::ranges::find(controls_, nullptr) != controls_.end())
            return 1;
        const auto dc = GetDC(hwnd_);
        RECT r{};
        GetClientRect(hwnd_, &r);
        for (const auto page : {Live, Wave, Cal, Diagnostics, Logs}) {
            tab(page);
            paint(dc, r);
        }
        ReleaseDC(hwnd_, dc);
        DestroyWindow(hwnd_);
        return 0;
    }
    ShowWindow(hwnd_, show);
    updater_.check();
    const HANDLE timer = refresh_tick_.get();
    const DWORD count = refresh_tick_ ? 1 : 0;
    MSG msg{};
    for (;;) {
        const auto result = MsgWaitForMultipleObjectsEx(count, &timer, INFINITE, QS_ALLINPUT, MWMO_INPUTAVAILABLE);
        if (result == WAIT_FAILED)
            win_error("MsgWaitForMultipleObjectsEx");
        if (count && result == WAIT_OBJECT_0)
            tick();
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT)
                return static_cast<int>(msg.wParam);
            if (!IsDialogMessageW(hwnd_, &msg)) {
                TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
        }
    }
}

void App::create() {
    const auto make = [&](Id id, const wchar_t* type, const wchar_t* label, DWORD style) {
        controls_[id - VersionButton] = CreateWindowExW(0, type, label, WS_CHILD | WS_TABSTOP | style,
            0, 0, 0, 0, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), module_, nullptr);
    };
    const auto button = [&](Id id, const wchar_t* label) { make(id, L"BUTTON", label, BS_OWNERDRAW); };
    for (const auto& [id, label] : std::initializer_list<std::pair<Id, const wchar_t*>>{
        {VersionButton, L"检查更新"}, {LiveTab, L"实时"}, {WaveTab, L"波形"}, {CalTab, L"标定"},
        {DiagTab, L"诊断"}, {LogTab, L"日志"}, {Refresh, L"刷新"}, {Probe, L"探测速率"}, {Connect, L"连接"},
        {Apply, L"应用配置"}, {PauseOutput, L"暂停遥测"}, {SaveFlash, L"保存 Flash"}, {Freeze, L"冻结波形"},
        {Follow, L"实时跟随"}, {ViewReset, L"恢复视角"}, {GyroCal, L"陀螺标定"}, {AccelCal, L"加速度标定"},
        {CapturePose, L"采集姿态"}, {SolveCal, L"求解"}, {CancelCal, L"取消标定"}, {ReadParams, L"读参数"},
        {ReadStatus, L"查询状态"}, {ReadDiag, L"硬件诊断"}, {ResetTiming, L"清零计时"}, {Reinitialize, L"重新初始化"},
        {RecordLog, L"开始记录"}, {OpenLog, L"打开日志"}, {CloseLog, L"关闭回放"}, {PlayLog, L"播放"},
        {ExportCsv, L"导出 CSV"}, {CancelExport, L"取消导出"}})
        button(id, label);
    constexpr std::array group_labels{L"欧拉角", L"四元数", L"原始六轴", L"滤波六轴"};
    for (int i = 0; i < 4; ++i) {
        const auto id = static_cast<Id>(Euler + i);
        make(id, L"BUTTON", group_labels[i], BS_AUTOCHECKBOX);
        SetWindowTheme(ctrl(id), L"", L"");
    }
    for (const Id id : {PortBox, LinkBaud, DeviceBaud, DeviceRate, PlotGroup, TimeBox, PlaySpeed})
        make(id, L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_VSCROLL);
    for (unsigned i = 0; i < baud_labels.size(); ++i) {
        add(LinkBaud, baud_labels[i]);
        if (i < baud_labels.size() - 1)
            add(DeviceBaud, baud_labels[i]);
    }
    for (const auto rate : sample_rates)
        add(DeviceRate, (std::to_wstring(rate) + L" Hz").c_str());
    for (const auto seconds : windows_sec)
        add(TimeBox, (std::to_wstring(seconds) + L" s").c_str());
    add(TimeBox, L"自定义");
    for (const auto* group : {L"欧拉角", L"四元数", L"角速度", L"加速度"})
        add(PlotGroup, group);
    for (const auto* speed : {L"0.25×", L"0.5×", L"1×", L"2×", L"4×"})
        add(PlaySpeed, speed);
    select(LinkBaud, 1);
    select(DeviceBaud, 1);
    select(DeviceRate, static_cast<int>(sample_rates.size()) - 1);
    select(TimeBox, 2);
    select(PlotGroup, 0);
    select(PlaySpeed, 2);
    check(Euler, true);
    make(Position, TRACKBAR_CLASSW, L"", TBS_NOTICKS);
    SetWindowTheme(ctrl(Position), L"", L"");
    SendMessageW(ctrl(Position), TBM_SETRANGE, TRUE, MAKELPARAM(0, 10000));
    make(From, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL);
    make(To, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL);
    SendMessageW(ctrl(From), EM_SETLIMITTEXT, 18, 0);
    SendMessageW(ctrl(To), EM_SETLIMITTEXT, 18, 0);
    SendMessageW(ctrl(To), EM_SETCUEBANNER, FALSE, reinterpret_cast<LPARAM>(L"结束"));
    for (const auto id : {FieldsBox, TimingBox, EventBox})
        make(id, L"EDIT", L"", read_only_edit);
    fonts(static_cast<int>(GetDpiForWindow(hwnd_)));
    refresh_ports();
    tab(Live);
    refresh_tick_.reset(CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS));
    set_refresh();
}

void App::set_refresh() {
    LARGE_INTEGER due{};
    due.QuadPart = -interval_ * 10000LL;
    if (!refresh_tick_ || !SetWaitableTimer(refresh_tick_.get(), &due, interval_, nullptr, nullptr, FALSE))
        SetTimer(hwnd_, 1, interval_, nullptr);
}

void App::fonts(int dpi) {
    fonts_.set(dpi);
    for (const auto hwnd : controls_)
        SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(fonts_.body), FALSE);
    for (const Id id : {FieldsBox, TimingBox, EventBox})
        SendMessageW(ctrl(id), WM_SETFONT, reinterpret_cast<WPARAM>(fonts_.mono), FALSE);
    for (const Id id : {PortBox, LinkBaud, DeviceBaud, DeviceRate, PlotGroup, TimeBox, PlaySpeed})
        SendMessageW(ctrl(id), CB_SETITEMHEIGHT, static_cast<WPARAM>(-1), fonts_.px(22));
}

void App::layout() {
    RECT client{};
    GetClientRect(hwnd_, &client);
    const auto px = [&](int n) { return fonts_.px(n); };
    width_ = MulDiv(client.right, 96, fonts_.dpi);
    height_ = MulDiv(client.bottom, 96, fonts_.dpi);
    const int right = width_ - 22;
    const auto place = [&](Id id, int x, int y, int width, int height = 30) {
        MoveWindow(ctrl(id), px(x), px(y), px(width), px(height), FALSE);
    };
    const auto field = [&](Id id, int x, int y, int width) { place(id, x, y, width, 210); };
    place(VersionButton, right - 112, 23, 112);
    for (int i = 0; i < 5; ++i)
        place(static_cast<Id>(LiveTab + i), 22 + i * 78, 68, 70);
    field(PortBox, 67, 120, 184);
    field(LinkBaud, 262, 120, 94);
    place(Refresh, 368, 118, 54);
    place(Probe, 432, 118, 82);
    place(Connect, 524, 118, 82);
    field(DeviceBaud, 22, 170, 89);
    field(DeviceRate, 121, 170, 82);
    int x = 218;
    for (int i = 0; i < 4; ++i) {
        place(static_cast<Id>(Euler + i), x, 168, i < 2 ? 66 : 80);
        x += i < 2 ? 67 : 81;
    }
    place(Apply, 526, 168, 78);
    place(PauseOutput, 614, 168, 83);
    place(SaveFlash, 707, 168, 98);
    const int plot_y = page_ == Live ? 225 : 171;
    field(PlotGroup, 22, plot_y + 2, 99);
    const int time_x = page_ == Live ? 132 : 22;
    field(TimeBox, time_x, plot_y + 2, 74);
    place(Freeze, time_x + 85, plot_y, 94);
    place(Follow, time_x + 189, plot_y, 94);
    place(ViewReset, right - 92, plot_y, 92);
    contents_ = {px(22), px(plot_y + 45), px(right), px(height_ - 48)};
    board_card_ = {contents_.left + (contents_.right - contents_.left) * 57 / 100 + px(6),
                   contents_.top, contents_.right, contents_.bottom};

    x = 22;
    for (const auto [id, w] : std::initializer_list<std::pair<Id, int>>{
        {GyroCal, 92}, {AccelCal, 108}, {CapturePose, 92}, {SolveCal, 68}, {CancelCal, 92}, {ReadParams, 84}})
        { place(id, x, 170, w); x += w + 9; }
    x = 22;
    for (const auto [id, w] : std::initializer_list<std::pair<Id, int>>{
        {ReadStatus, 92}, {ReadDiag, 92}, {ResetTiming, 92}, {Reinitialize, 108}})
        { place(id, x, 170, w); x += w + 9; }
    const int half = width_ / 2;
    place(FieldsBox, 36, 291, half - 57, std::max(30, height_ - 354));
    place(TimingBox, half + 21, 291, half - 57, std::max(30, height_ - 354));

    place(RecordLog, 22, 170, 96);
    place(OpenLog, 128, 170, 96);
    place(CloseLog, 234, 170, 96);
    place(PlayLog, 340, 170, 76);
    field(PlaySpeed, 427, 172, 74);
    place(Position, 22, 215, right - 44, 29);
    place(From, 111, 292, 85, 27);
    place(To, 221, 292, 85, 27);
    place(ExportCsv, 320, 290, 95);
    place(CancelExport, 425, 290, 88);
    place(EventBox, 36, 368, right - 50, std::max(30, height_ - 431));
}

void App::tab(Page page) {
    page_ = page;
    for (int id = DeviceBaud; id < EndId; ++id) {
        bool show = false;
        if (id <= SaveFlash || id == PlotGroup || id == ViewReset)
            show = page == Live;
        else if (id <= ViewReset)
            show = page == Live || page == Wave;
        else if (id <= ReadParams)
            show = page == Cal;
        else if (id <= TimingBox)
            show = page == Diagnostics;
        else
            show = page == Logs;
        ShowWindow(ctrl(static_cast<Id>(id)), show ? SW_SHOWNA : SW_HIDE);
    }
    for (int id = VersionButton; id <= Connect; ++id)
        ShowWindow(ctrl(static_cast<Id>(id)), SW_SHOWNA);
    serial_.profiling = page == Diagnostics;
    layout();
    tick(true);
}

void App::refresh_ports() {
    const int old = selected(PortBox);
    const auto name = old >= 0 && old < static_cast<int>(ports_.size()) ? ports_[old].name : L"";
    ports_ = serial_ports();
    SendMessageW(ctrl(PortBox), CB_RESETCONTENT, 0, 0);
    int selected = 0;
    for (int i = 0; i < static_cast<int>(ports_.size()); ++i) {
        add(PortBox, ports_[i].label.c_str());
        if (ports_[i].name == name)
            selected = i;
    }
    select(PortBox, ports_.empty() ? -1 : selected);
}

Config App::edited_config() const {
    const auto baud = selected(DeviceBaud), rate = selected(DeviceRate);
    Config config{baud >= 0 ? baud_rates[baud] : 0, rate >= 0 ? sample_rates[rate] : 0, 0};
    for (int i = 0; i < 4; ++i)
        if (checked(static_cast<Id>(Euler + i)))
            config.mask |= 1 << i;
    return config;
}

void App::tick(bool force) {
    const auto now = now_ns();
    const bool charts_visible = (page_ == Live || page_ == Wave) && !IsIconic(hwnd_);
    bool redraw = force;
    {
        std::lock_guard lock(model_.mutex);
        if (generation_ != model_.state.generation) {
            generation_ = model_.state.generation;
            consumed_ = 0;
            history_.clear();
            charts_.follow();
            board_.clear_pose();
            notice_.clear();
            latest_event_ = 0;
            displayed_config_.reset();
            config_dirty_ = false;
            redraw = true;
        }
        if (charts_visible && !charts_.frozen && model_.state.total != consumed_) {
            model_.copy_since(generation_, consumed_, delta_);
        } else {
            delta_.clear();
        }
        redraw |= charts_visible && view_.total != model_.state.total && (page_ == Live || !charts_.frozen);
        if (!model_.events.empty() && model_.events.back().id != latest_event_) {
            notice_ = model_.events.back().text;
            latest_event_ = model_.events.back().id;
            redraw = true;
        }
        view_ = model_.state;
    }
    if (!delta_.empty()) {
        history_.append(delta_);
        redraw = true;
    }
    if (view_.status && !config_dirty_ && displayed_config_ != view_.status->config()) {
        const auto config = view_.status->config();
        const auto baud = std::ranges::find(baud_rates, config.baud);
        const auto rate = std::ranges::find(sample_rates, config.rate);
        select(DeviceBaud, baud >= baud_rates.end() - 1 ? -1 : static_cast<int>(baud - baud_rates.begin()));
        select(DeviceRate, rate == sample_rates.end() ? -1 : static_cast<int>(rate - sample_rates.begin()));
        for (int i = 0; i < 4; ++i)
            check(static_cast<Id>(Euler + i), config.mask & (1 << i));
        displayed_config_ = config;
    }
    const bool stale = !view_.replay && (!view_.latest || now - view_.last_data > 500000000);
    redraw |= stale != stale_;
    stale_ = stale;
    if (force || now >= next_controls_ || closing_) {
        const bool update_details = (page_ != Live && page_ != Wave) || recording_.running || playing_.running;
        const auto old_phase = updating_.phase;
        const auto old_link = probing_;
        const bool state_changed = view_.status_serial != displayed_status_;
        displayed_status_ = view_.status_serial;
        sync_controls();
        next_controls_ = now + 250000000;
        redraw |= update_details || old_phase != updating_.phase || old_link != probing_ || state_changed;
    }
    if (closing_ && !serial_.running() && !recording_.running && !playing_.running && !exporting_.running &&
        updating_.phase != UpdatePhase::Checking && updating_.phase != UpdatePhase::Downloading) {
        if (!restart_ || updater_.restart()) {
            DestroyWindow(hwnd_);
            return;
        }
        closing_ = restart_ = false;
    }
    if (redraw && !IsIconic(hwnd_))
        invalidate();
    const int interval = charts_visible && (view_.port_open || playing_.playing) ? 16 : 125;
    if (interval != interval_) {
        interval_ = interval;
        set_refresh();
    }
}

void App::sync_controls() {
    recording_ = recorder_.status();
    playing_ = replay_.status();
    exporting_ = exporter_.status();
    updating_ = updater_.status();
    const auto& status = view_.status;
    const bool live = serial_.running();
    if (probing_ && !live) {
        probing_ = false;
        const auto rate = std::ranges::find(baud_rates, serial_.probe_result());
        if (rate != baud_rates.end())
            select(LinkBaud, static_cast<int>(rate - baud_rates.begin()));
    }
    const bool ready = !closing_ && live && view_.port_open && !probing_;
    const bool writable = ready && !serial_.busy() && status && (*status)[Status::Version] == 1 &&
        now_ns() - view_.last_rx <= 3000000000LL;
    const auto diag = view_.diagnostic.value_or(Diagnostic{});
    const bool ch340 = selected(PortBox) >= 0 && ports_[selected(PortBox)].ch340;
    const auto draft = edited_config();
    const bool normal_clock = !diag[Diagnostic::BootError];
    const unsigned mode = status ? (*status)[Status::CalMode] : 0, state = status ? (*status)[Status::CalState] : 0;
    const bool calibrating = state >= 1 && state <= 4;
    const bool fresh = status && (*status)[Status::Phase] == 6 && !((*status)[Status::Flags] & sensor_fault) &&
        view_.diagnostic && diag[Diagnostic::SampleAge] < 10000;
    const unsigned poses = status ? (*status)[Status::Poses] : 0;
    change(Connect, closing_ ? L"关闭中" : probing_ ? L"停止探测" : live ? L"断开" : L"连接");
    for (const Id id : {PortBox, LinkBaud, Refresh, Probe})
        enable(id, !closing_ && !live && !playing_.running && (id == Refresh || !ports_.empty()));
    enable(Connect, !closing_ && !playing_.running && (live || !ports_.empty()));
    change(PauseOutput, status && !((*status)[Status::Flags] & output_on) ? L"恢复遥测" : L"暂停遥测");
    enable(Apply, writable && normal_clock && !config_error(draft) && !(ch340 && draft.baud > 2000000));
    enable(PauseOutput, writable);
    enable(SaveFlash, writable && normal_clock && view_.config_confirmed && !calibrating);
    for (const Id id : {GyroCal, AccelCal})
        enable(id, writable && fresh && !calibrating);
    enable(CapturePose, writable && fresh && mode == 2 && state == 1 && poses < 128);
    enable(SolveCal, writable && mode == 2 && state == 1 && poses >= 12);
    enable(CancelCal, writable && calibrating);
    enable(ResetTiming, writable);
    enable(Reinitialize, writable && normal_clock);
    for (const Id id : {ReadStatus, ReadDiag, ReadParams})
        enable(id, ready && !serial_.busy());
    change(Freeze, charts_.frozen ? L"继续波形" : L"冻结波形");
    change(RecordLog, recording_.active ? L"停止记录" : recording_.running ? L"写入中" : L"开始记录");
    change(PlayLog, playing_.playing ? L"暂停" : L"播放");
    enable(RecordLog, !closing_ && (recording_.active || (ready && !recording_.running)));
    enable(OpenLog, !closing_ && !live && !playing_.running && !recording_.running);
    enable(CloseLog, playing_.running);
    enable(PlayLog, playing_.ready);
    enable(PlaySpeed, playing_.ready);
    enable(Position, playing_.ready);
    enable(ExportCsv, !closing_ && !log_path_.empty() && !recording_.running && !exporting_.running);
    enable(CancelExport, exporting_.running);
    if (playing_.ready && !has_seek_)
        SendMessageW(ctrl(Position), TBM_SETPOS, TRUE, playing_.duration > 0
            ? static_cast<int>(playing_.position / playing_.duration * 10000) : 0);
    using enum UpdatePhase;
    const auto phase = updating_.phase;
    change(VersionButton, phase == Available ? L"下载 " + wide(updating_.release->tag.substr(11)) :
        phase == Ready ? L"重启更新" : phase == Downloading ? L"正在下载" : phase == Checking ? L"正在检查" : L"检查更新");
    enable(VersionButton, !closing_ && phase != Checking && phase != Downloading);
    if (page_ == Diagnostics)
        diagnostics();
    if (page_ == Logs)
        log_events();
}

void App::diagnostics() {
    std::wostringstream fields, timings;
    fields << std::left;
    if (view_.status) {
        fields << L"状态\r\n";
        const auto& st = *view_.status;
        for (unsigned i = 0; i < st.fields.size(); ++i)
            fields << std::setw(20) << wide(status_keys[i]) << st.fields[i] << L"\r\n";
        fields << std::setw(20) << L"fit_rms" << st.rms << L"\r\n"
               << std::setw(20) << L"max_norm_error" << st.max_error << L"\r\n\r\n";
    }
    if (view_.diagnostic) {
        fields << L"硬件诊断\r\n";
        for (unsigned i = 0; i < diag_keys.size(); ++i)
            fields << std::setw(20) << wide(diag_keys[i]) << view_.diagnostic->fields[i] << L"\r\n";
    }
    timings << L"阶段                 次数 / 最近 / 最小 / 平均 / 最大 (μs)\r\n";
    for (unsigned i = 0; i < stage_names.size(); ++i) {
        const auto& t = view_.timing[i];
        const double scale = t.core ? 1e6 / t.core : 0;
        timings << stage_names[i] << L"\r\n  ";
        if (t.count && scale)
            timings << t.count << L"  /  " << fixed(t.last * scale, 3) << L"  /  " << fixed(t.min * scale, 3) <<
                L"  /  " << fixed(t.total * scale / t.count, 3) << L"  /  " << fixed(t.max * scale, 3);
        else
            timings << L"—";
        timings << L"\r\n";
    }
    edit_text(ctrl(FieldsBox), fields.str());
    edit_text(ctrl(TimingBox), timings.str());
}

void App::log_events() {
    std::vector<Event> events;
    {
        std::lock_guard lock(model_.mutex);
        const auto last = model_.events.empty() ? 0 : model_.events.back().id;
        if (event_generation_ == view_.generation && rendered_event_ == last)
            return;
        events.assign(model_.events.begin(), model_.events.end());
    }
    event_generation_ = view_.generation;
    rendered_event_ = events.empty() ? 0 : events.back().id;
    std::wstring text;
    for (const auto& event : events)
        text += fixed((event.ns - view_.origin) * 1e-9, 3) + L"   " + wide(event.level) + L"   " + event.text + L"\r\n";
    const auto hwnd = ctrl(EventBox);
    if (text == get_text(hwnd))
        return;
    const auto old = SendMessageW(hwnd, EM_GETLINECOUNT, 0, 0), line = SendMessageW(hwnd, EM_GETFIRSTVISIBLELINE, 0, 0);
    RECT box{};
    GetClientRect(hwnd, &box);
    const bool end = old - line <= box.bottom / fonts_.px(16) + 2;
    SetWindowTextW(hwnd, text.c_str());
    SendMessageW(hwnd, EM_LINESCROLL, 0, end ? SendMessageW(hwnd, EM_GETLINECOUNT, 0, 0) : line);
}

void App::draw_button(const DRAWITEMSTRUCT& item) {
    const auto id = item.CtlID;
    const bool nav = id >= LiveTab && id <= LogTab;
    const bool active = nav && id == LiveTab + static_cast<UINT>(page_);
    const bool disabled = item.itemState & ODS_DISABLED;
    const bool primary = id == Connect || id == Apply || id == RecordLog || id == PlayLog ||
        (id == VersionButton && updating_.phase == UpdatePhase::Ready);
    const bool dark = !disabled && (primary || active);
    const auto p = Paint{item.hDC, fonts_};
    const auto rect = item.rcItem;
    p.fill(rect, dark ? gray((item.itemState & ODS_SELECTED) ? 55 : 22) :
          gray(disabled ? 247 : (item.itemState & ODS_SELECTED) ? 230 : 255));
    if (!dark)
        p.border(rect);
    p.text(rect, get_text(item.hwndItem), active ? fonts_.strong : fonts_.body,
           dark ? paper : disabled ? gray(117) : ink, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    if (item.itemState & ODS_FOCUS) {
        auto focus = inset(rect, p.px(3), p.px(3));
        DrawFocusRect(p.dc, &focus);
    }
}

void App::paint(HDC dc, RECT affected) {
    RECT client{};
    GetClientRect(hwnd_, &client);
    auto memory = buffer_.get(dc, client.right, client.bottom);
    const Paint p{memory ? memory : dc, fonts_};
    const auto bounds = [&](int l, int t, int r, int b) { return RECT{p.px(l), p.px(t), p.px(r), p.px(b)}; };
    const auto px = [&](int n) { return p.px(n); };
    const int right = width_ - 22;
    const int saved = SaveDC(p.dc);
    IntersectClipRect(p.dc, affected.left, affected.top, affected.right, affected.bottom);
    p.fill(affected, background);
    p.fill(bounds(0, 0, width_, 158), paper);
    p.text(bounds(22, 10, 230, 43), L"IMUStudio", fonts_.title);
    p.text(bounds(23, 44, 280, 61), L"AT32F405  /  LSM6DSV16X", fonts_.caption, muted);
    p.text(bounds(right - 238, 23, right - 124, 53), L"v" + std::wstring(app_version_w),
           fonts_.mono, muted, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    p.line(0, px(107), client.right, px(107));
    p.line(0, px(157), client.right, px(157));
    p.text(bounds(22, 117, 62, 147), L"串口", fonts_.strong);
    auto link = probing_ ? L"正在探测速率" : view_.link;
    if (view_.port_open && now_ns() - std::max(view_.last_rx, view_.origin) > 3000000000LL)
        link = L"串口已打开 · 无有效回复";
    p.text(bounds(625, 117, right, 148), link, fonts_.caption, ink, DT_RIGHT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);

    if (page_ == Live || page_ == Wave) {
        charts_.begin();
        if (page_ == Live) {
            const auto draft = edited_config();
            std::wstring config_text = L"当前设置 —";
            if (view_.status) {
                const auto cfg = view_.status->config();
                config_text = L"当前  " + std::to_wstring(cfg.baud) + L" baud  /  " + std::to_wstring(cfg.rate) +
                    L" Hz  /  mask " + std::to_wstring(cfg.mask);
            }
            p.text(bounds(22, 197, width_ / 2, 223), config_text, fonts_.caption, muted);
            const bool ch340 = selected(PortBox) >= 0 && ports_[selected(PortBox)].ch340;
            const auto warning = ch340 && draft.baud > 2000000 ? L"CH340 支持 1 或 2 Mbps" : config_error(draft);
            const auto budget = draft.baud ? fixed(frame_size(draft.mask) * draft.rate * 1000.0 / draft.baud, 1) + L"%" : L"—";
            const auto config_hint = warning ? std::wstring(warning) : L"线路占用 " + budget + L"   ·   " + view_.saved;
            p.text(bounds(width_ / 2, 197, right, 223), config_hint, fonts_.caption, muted,
                   DT_RIGHT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
            RECT left = contents_;
            left.right = board_card_.left - px(12);
            auto values = left;
            values.bottom = values.top + px(83);
            p.fill(values, paper);
            p.border(values);
            const int span = (values.right - values.left) / 3;
            for (int i = 0; i < 3; ++i) {
                constexpr std::array axes{L"Roll", L"Pitch", L"Yaw"};
                const auto x = values.left + i * span;
                if (i)
                    p.line(x, values.top + px(15), x, values.bottom - px(15));
                p.text({x + px(17), values.top + px(9), x + span - px(12), values.top + px(31)}, axes[i], fonts_.caption, muted);
                const auto angle = view_.latest ? view_.latest->values[i] : std::numeric_limits<float>::quiet_NaN();
                const auto text = std::isfinite(angle) && (view_.latest->flags & valid_attitude) ? fixed(angle, 2) + L"°" : L"—";
                p.text({x + px(15), values.top + px(31), x + span - px(10), values.bottom - px(7)}, text,
                       fonts_.number, stale_ ? muted : ink);
            }
            left.top = values.bottom + px(12);
            charts_.draw(p, left, plot_group_, history_);
            board_.draw(p, board_card_, view_, stale_);
        } else {
            const auto card = contents_;
            const auto mid_x = (card.left + card.right) / 2, mid_y = (card.top + card.bottom) / 2;
            const int gap = px(6);
            charts_.draw(p, {card.left, card.top, mid_x - gap, mid_y - gap}, 0, history_);
            charts_.draw(p, {mid_x + gap, card.top, card.right, mid_y - gap}, 1, history_);
            charts_.draw(p, {card.left, mid_y + gap, mid_x - gap, card.bottom}, 2, history_);
            charts_.draw(p, {mid_x + gap, mid_y + gap, card.right, card.bottom}, 3, history_);
        }
    } else if (page_ == Cal) {
        const auto status = view_.status.value_or(Status{});
        const auto state = status[Status::CalState], mode = status[Status::CalMode];
        constexpr std::array states{L"空闲", L"等待摆放", L"等待静止", L"采集中", L"求解中", L"完成", L"失败"};
        auto label = view_.status ? std::wstring(state < states.size() ? states[state] : L"未知状态") : L"等待状态";
        if (status[Status::CalError])
            label += L" · " + std::wstring(result_text(status[Status::CalError]));
        p.fill(bounds(22, 214, right, 335), paper);
        p.border(bounds(22, 214, right, 335));
        p.text(bounds(38, 222, right - 15, 246), label, fonts_.strong);
        const auto stable_max = mode == 1 ? 1500u : 1250u, sample_max = mode == 1 ? 60000u : 5000u;
        const auto progress = [&](int x, int w, int value, int max, const wchar_t* name) {
            p.text(bounds(x, 252, x + w, 272), std::wstring(name) + L"  " + std::to_wstring(value) +
                L" / " + std::to_wstring(max), fonts_.caption, muted);
            p.fill(bounds(x, 276, x + w, 280), gray(233));
            p.fill(bounds(x, 276, x + MulDiv(w, std::clamp(value, 0, max), max), 280), ink);
        };
        const auto progress_width = (right - 104) / 2;
        progress(38, progress_width, status[Status::Stable], stable_max, L"稳定样本");
        progress(60 + progress_width, progress_width, status[Status::Samples], sample_max, L"有效样本");
        p.text(bounds(38, 291, right - 14, 319), L"已接受 " + std::to_wstring(status[Status::Poses]) +
            L" / 128 姿态  ·  至少 12 个可求解  ·  RMS " + significant(status.rms) +
            L"  ·  最大模长误差 " + significant(status.max_error) + L"  ·  步骤 " + std::to_wstring(status[Status::SolveSteps]),
            fonts_.body);
        p.text(bounds(22, 341, right, 369), L"陀螺标定保持静止约 60 秒。加速度标定从 ±X、±Y、±Z 与各斜向分散采点，每点静置约 6 秒。", fonts_.caption, muted);
        p.fill(bounds(22, 380, right, height_ - 48), paper);
        p.border(bounds(22, 380, right, height_ - 48));
        p.text(bounds(38, 386, right - 16, 411), L"当前标定参数", fonts_.strong);
        if (view_.calibration) {
            const auto flags = view_.calibration->flags;
            const auto text = std::wstring(L"陀螺 ") + (flags & 1 ? L"已标定" : L"未标定") +
                L"   /   加速度 " + (flags & 2 ? L"已标定" : L"未标定");
            p.text(bounds(220, 386, right - 16, 411), text, fonts_.caption, muted,
                DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
        }
        const auto step = std::min(29, std::max(17, (height_ - 48 - 418) / 5));
        const int span = (right - 200) / 3;
        constexpr std::array names{L"陀螺零偏  /  °/s", L"加速度偏置  /  g", L"矩阵第 1 行", L"矩阵第 2 行", L"矩阵第 3 行"};
        for (int row = 0; row < 5; ++row) {
            const int y = 418 + step * row;
            p.line(px(38), px(y), px(right - 16), px(y), gray(237));
            p.text(bounds(38, y, 197, y + step), names[row], fonts_.caption, muted);
            for (int col = 0; col < 3; ++col)
                p.text(bounds(200 + span * col, y, 200 + span * (col + 1), y + step),
                    view_.calibration ? significant(view_.calibration->values[row * 3 + col], 8) : L"—", fonts_.mono);
        }
    } else if (page_ == Diagnostics) {
        const int half = width_ / 2;
        const auto& c = view_.parse;
        p.text(bounds(22, 205, right, 231), L"电脑接收   CRC " + std::to_wstring(c.crc) + L"  ·  长度异常 " +
            std::to_wstring(c.length) + L"  ·  无效 " + std::to_wstring(view_.invalid) + L"  ·  缺帧 " +
            std::to_wstring(view_.missing) + L"  ·  丢弃 " + std::to_wstring(c.discarded) + L" B", fonts_.caption, muted);
        p.text(bounds(22, 228, right, 250), view_.interrupt, fonts_.caption, muted);
        for (const auto card : {bounds(22, 255, half - 7, height_ - 48), bounds(half + 7, 255, right, height_ - 48)}) {
            p.fill(card, paper);
            p.border(card);
        }
        p.text(bounds(36, 258, half - 20, 288), L"模块状态与硬件诊断", fonts_.strong);
        p.text(bounds(half + 21, 258, right - 14, 288), L"阶段计时  ·  MCU CPU " + fixed(view_.cpu, 2) + L"%",
               fonts_.strong);
    } else {
        auto log = playing_.running ? L"回放  " + fixed(playing_.position) + L" / " + fixed(playing_.duration) + L" s" :
            recording_.active ? L"记录中  ·  " + std::to_wstring(recording_.frames) + L" 帧" :
            recording_.running ? L"正在写入日志" : recording_.error;
        if (!playing_.error.empty())
            log = playing_.error;
        p.text(bounds(516, 170, right, 200), log, fonts_.caption, muted);
        p.text(bounds(22, 251, right, 279), log_path_.wstring(), fonts_.caption, muted, single | DT_PATH_ELLIPSIS);
        p.text(bounds(22, 289, 109, 320), L"CSV 范围 / s", fonts_.caption, muted);
        p.text(bounds(197, 289, 221, 320), L"至", fonts_.caption, muted, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        p.text(bounds(527, 289, right, 320), exporting_.message.empty() ? L"" :
            exporting_.message + L"  ·  " + std::to_wstring(exporting_.rows) + L" 行", fonts_.caption, muted);
        p.fill(bounds(22, 332, right, height_ - 48), paper);
        p.border(bounds(22, 332, right, height_ - 48));
        p.text(bounds(36, 337, right - 16, 366), L"通信事件", fonts_.strong);
    }
    const int footer = height_ - 37;
    p.fill(bounds(0, footer, width_, height_), paper);
    p.line(0, px(footer), client.right, px(footer));
    const auto cursor = page_ == Live || page_ == Wave ? charts_.cursor(history_) : L"";
    const auto left = !cursor.empty() ? cursor : fixed(view_.port_open && stale_ ? 0 : view_.rate, 1) +
        L" 帧/s  ·  " + std::to_wstring(view_.total) + L" 帧" +
        ((page_ == Live || page_ == Wave) && charts_.frozen ? L"  ·  波形已冻结" : L"");
    p.text(bounds(22, footer, width_ / 2 + 56, height_), left, fonts_.caption, muted);
    auto message = closing_ ? L"正在关闭" : !update_message_.empty() ? updating_.message : notice_;
    if (updating_.phase == UpdatePhase::Available || updating_.phase == UpdatePhase::Ready)
        message = updating_.message;
    p.text(bounds(width_ / 2 + 64, footer, right, height_), message, fonts_.caption, muted,
           DT_RIGHT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
    RestoreDC(p.dc, saved);
    if (memory)
        BitBlt(dc, affected.left, affected.top, affected.right - affected.left, affected.bottom - affected.top,
               memory, affected.left, affected.top, SRCCOPY);
}

void App::notice(std::wstring message) {
    update_message_.clear();
    notice_ = std::move(message);
    invalidate();
}

void App::send(Command request) {
    if (!serial_.request(request))
        notice(L"串口未连接或命令处理中");
}

void App::command(Id id, int code) {
    if (closing_)
        return;
    try {
        if (id >= LiveTab && id <= LogTab) {
            tab(static_cast<Page>(id - LiveTab));
            return;
        }
        if (id >= GyroCal && id <= CancelCal) {
            send({Calibrate, {static_cast<Byte>(id - GyroCal + 1)}});
        } else switch (id) {
        case VersionButton:
            update_message_ = L"更新";
            if (updating_.phase == UpdatePhase::Ready)
                close(true);
            else if (updating_.phase == UpdatePhase::Available)
                updater_.download();
            else
                updater_.check();
            break;
        case Refresh: refresh_ports(); break;
        case Probe:
        case Connect: {
            if (serial_.running()) {
                serial_.stop();
                break;
            }
            const int port = selected(PortBox), baud = selected(LinkBaud);
            if (port < 0 || baud < 0 || playing_.running)
                break;
            probing_ = id == Probe;
            if (!probing_) {
                std::lock_guard lock(model_.mutex);
                model_.reset();
            }
            serial_.start(ports_[port], baud_rates[baud], probing_);
            break;
        }
        case DeviceBaud:
        case DeviceRate:
            if (code == CBN_SELCHANGE)
                config_dirty_ = true;
            break;
        case Euler: case Quaternion: case Raw: case Filtered:
            config_dirty_ = true;
            break;
        case Apply: send(configure(edited_config())); break;
        case PauseOutput:
            if (view_.status)
                send({Output, {static_cast<Byte>(((*view_.status)[Status::Flags] & output_on) == 0)}});
            break;
        case SaveFlash:
            if (MessageBoxW(hwnd_, L"将通信设置和标定参数写入 Flash，模块会短暂停止采样并重新预热。",
                L"保存到模块", MB_OKCANCEL | MB_ICONQUESTION | MB_DEFBUTTON2) == IDOK)
                send({Save});
            break;
        case PlotGroup:
            if (code == CBN_SELCHANGE && selected(PlotGroup) >= 0)
                plot_group_ = selected(PlotGroup);
            break;
        case TimeBox:
            if (code == CBN_SELCHANGE && selected(TimeBox) >= 0 && selected(TimeBox) < static_cast<int>(windows_sec.size()))
                charts_.seconds = windows_sec[selected(TimeBox)];
            break;
        case Freeze:
            if (charts_.frozen)
                charts_.follow();
            else {
                charts_.frozen = true;
                if (!history_.empty() && !charts_.end)
                    charts_.end = history_.latest().time;
            }
            break;
        case Follow: charts_.follow(); break;
        case ViewReset: board_.reset(); break;
        case ReadParams: send({Parameters}); break;
        case ReadStatus: send({Query}); break;
        case ReadDiag: send({Diagnose}); break;
        case ResetTiming: send({ClearProfile}); break;
        case Reinitialize: send({Reinit}); break;
        case RecordLog:
            if (recording_.active) {
                recorder_.stop();
            } else {
                const auto path = pick_file(hwnd_, true, false, documents() / log_name());
                if (!path.empty()) {
                    std::lock_guard lock(model_.mutex);
                    if (model_.state.port_open) {
                        log_path_ = path;
                        recorder_.start(path, model_.state, ports_[selected(PortBox)].name);
                    }
                }
            }
            break;
        case OpenLog: {
            const auto path = pick_file(hwnd_, false, false, documents() / L"");
            if (!path.empty()) {
                log_path_ = path;
                select(PlaySpeed, 2);
                replay_.start(path);
            }
            break;
        }
        case CloseLog: replay_.stop(); break;
        case PlayLog: replay_.toggle(); break;
        case PlaySpeed:
            if (code == CBN_SELCHANGE && selected(PlaySpeed) >= 0)
                replay_.speed(replay_speeds[selected(PlaySpeed)]);
            break;
        case ExportCsv: {
            const auto number = [&](Id field, double blank) {
                const auto value = get_text(ctrl(field));
                if (value.empty())
                    return blank;
                wchar_t* end{};
                const auto seconds = std::wcstod(value.c_str(), &end);
                if (*end || !std::isfinite(seconds) || seconds < 0)
                    throw std::runtime_error("请输入有效的 CSV 秒数");
                return seconds;
            };
            const double from = number(From, 0), to = number(To, std::numeric_limits<double>::infinity());
            if (from > to)
                throw std::runtime_error("CSV 起点应小于或等于终点");
            const auto csv = pick_file(hwnd_, true, true, log_path_.parent_path() / (log_path_.stem().wstring() + L".csv"));
            if (!csv.empty() && _wcsicmp(csv.extension().c_str(), L".csv") != 0)
                throw std::runtime_error("导出文件扩展名应为 .csv");
            if (!csv.empty())
                exporter_.start(log_path_, csv, from, to);
            break;
        }
        case CancelExport: exporter_.stop(); break;
        default: return;
        }
    } catch (const std::exception& error) {
        notice(wide(error.what()));
    }
    tick(true);
}

void App::close(bool restart) {
    if (closing_)
        return;
    closing_ = true;
    restart_ = restart;
    serial_.stop();
    if (!serial_.running())
        recorder_.stop();
    replay_.stop();
    exporter_.stop();
    if (!restart)
        updater_.stop();
    tick(true);
}

void App::mouse_move(POINT at) {
    if (page_ != Live && page_ != Wave)
        return;
    charts_.mouse = at;
    TRACKMOUSEEVENT track{sizeof(track), TME_LEAVE, hwnd_, 0};
    TrackMouseEvent(&track);
    if (dragged_ == 4)
        board_.rotate(at.x - drag_at_.x, at.y - drag_at_.y);
    else if (dragged_ >= 0)
        charts_.pan(history_, dragged_, at.x - drag_at_.x);
    drag_at_ = at;
    invalidate();
}

void App::mouse_down(POINT at) {
    if (page_ != Live && page_ != Wave)
        return;
    if (charts_.toggle(at)) {
        invalidate();
        return;
    }
    drag_at_ = at;
    dragged_ = page_ == Live && PtInRect(&board_card_, at) ? 4 : charts_.panel_at(at);
    if (dragged_ >= 0)
        SetCapture(hwnd_);
}

void App::wheel(POINT at, int delta, bool vertical) {
    if (page_ != Live && page_ != Wave)
        return;
    ScreenToClient(hwnd_, &at);
    const auto factor = std::pow(1.2, -delta / static_cast<double>(WHEEL_DELTA));
    const auto plot = charts_.panel_at(at);
    if (plot >= 0) {
        charts_.zoom(history_, plot, at, factor, vertical);
        if (!vertical)
            select(TimeBox, static_cast<int>(windows_sec.size()));
    } else if (page_ == Live && PtInRect(&board_card_, at)) {
        board_.zoom(1 / factor);
    }
    invalidate();
}

void App::seek() {
    if (playing_.ready) {
        const auto position = SendMessageW(ctrl(Position), TBM_GETPOS, 0, 0);
        replay_.seek(position * playing_.duration / 10000);
    }
}

LRESULT App::handle(UINT msg, WPARAM w, LPARAM l) {
    switch (msg) {
    case WM_CREATE: create(); return 0;
    case WM_DESTROY: KillTimer(hwnd_, 1); PostQuitMessage(0); return 0;
    case WM_CLOSE: close(); return 0;
    case WM_TIMER: tick(); return 0;
    case WM_SIZE: if (w != SIZE_MINIMIZED) { layout(); invalidate(); } return 0;
    case WM_DPICHANGED: {
        fonts(HIWORD(w));
        const auto& r = *reinterpret_cast<const RECT*>(l);
        SetWindowPos(hwnd_, nullptr, r.left, r.top, r.right - r.left, r.bottom - r.top, SWP_NOZORDER | SWP_NOACTIVATE);
        return 0;
    }
    case WM_GETMINMAXINFO: {
        const int dpi = std::max(96, static_cast<int>(GetDpiForWindow(hwnd_)));
        auto& info = *reinterpret_cast<MINMAXINFO*>(l);
        RECT r{0, 0, MulDiv(830, dpi, 96), MulDiv(566, dpi, 96)};
        AdjustWindowRectExForDpi(&r, WS_OVERLAPPEDWINDOW, FALSE, 0, dpi);
        info.ptMinTrackSize = {r.right - r.left, r.bottom - r.top};
        return 0;
    }
    case WM_ERASEBKGND: return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps{};
        const auto dc = BeginPaint(hwnd_, &ps);
        if (!IsRectEmpty(&ps.rcPaint))
            paint(dc, ps.rcPaint);
        EndPaint(hwnd_, &ps);
        return 0;
    }
    case WM_DRAWITEM: draw_button(*reinterpret_cast<const DRAWITEMSTRUCT*>(l)); return TRUE;
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORLISTBOX:
    case WM_CTLCOLORBTN: {
        const auto dc = reinterpret_cast<HDC>(w);
        SetTextColor(dc, ink);
        const auto id = GetDlgCtrlID(reinterpret_cast<HWND>(l));
        SetBkColor(dc, id >= Euler && id <= Filtered ? background : paper);
        SetDCBrushColor(dc, id >= Euler && id <= Filtered ? background : paper);
        return reinterpret_cast<LRESULT>(GetStockObject(DC_BRUSH));
    }
    case WM_COMMAND: command(static_cast<Id>(LOWORD(w)), HIWORD(w)); return 0;
    case WM_MOUSEMOVE: mouse_move({GET_X_LPARAM(l), GET_Y_LPARAM(l)}); return 0;
    case WM_MOUSELEAVE: charts_.mouse = {-1, -1}; invalidate(); return 0;
    case WM_LBUTTONDOWN: mouse_down({GET_X_LPARAM(l), GET_Y_LPARAM(l)}); return 0;
    case WM_LBUTTONDBLCLK:
        if (page_ == Live && PtInRect(&board_card_, {GET_X_LPARAM(l), GET_Y_LPARAM(l)})) {
            board_.reset();
            invalidate();
        }
        return 0;
    case WM_LBUTTONUP:
        if (dragged_ >= 0)
            ReleaseCapture();
        dragged_ = -1;
        return 0;
    case WM_CAPTURECHANGED: dragged_ = -1; return 0;
    case WM_MOUSEWHEEL:
        wheel({GET_X_LPARAM(l), GET_Y_LPARAM(l)}, GET_WHEEL_DELTA_WPARAM(w), (GET_KEYSTATE_WPARAM(w) & MK_CONTROL) != 0);
        return 0;
    case WM_HSCROLL:
        if (reinterpret_cast<HWND>(l) == ctrl(Position)) {
            has_seek_ = LOWORD(w) == TB_THUMBTRACK;
            if (!has_seek_)
                seek();
            return 0;
        }
        break;
    }
    return DefWindowProcW(hwnd_, msg, w, l);
}
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show) {
    int argc{};
    const auto argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    int result = 1;
    bool smoke = false;
    try {
        smoke = argc == 2 && std::wstring_view(argv[1]) == L"--smoke-test";
        if (argc == 4 && std::wstring_view(argv[1]) == L"--apply-update") {
            result = apply_update(argv[2], argv[3]);
        } else {
            if (argc == 4 && std::wstring_view(argv[1]) == L"--remove-update")
                remove_update(argv[2], argv[3]);
            result = App(instance).run(show, smoke);
        }
    } catch (const std::exception& error) {
        if (!smoke)
            MessageBoxW(nullptr, wide(error.what()).c_str(), L"IMUStudio", MB_ICONERROR);
    }
    LocalFree(argv);
    return result;
}
