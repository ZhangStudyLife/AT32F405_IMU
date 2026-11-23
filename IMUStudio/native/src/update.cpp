#include "update.h"
#include "model.h"
#include "version.h"
#include "win.h"

#include <bcrypt.h>
#include <charconv>
#include <nlohmann/json.hpp>
#include <winhttp.h>

namespace fs = std::filesystem;

namespace {
constexpr std::string_view prefix = "imustudio-v";
constexpr std::wstring_view staging_prefix = L".imustudio-update-";
constexpr std::size_t max_update_size = 32 * 1024 * 1024;

using Version = std::array<unsigned, 3>;

std::optional<Version> version(std::string_view value) {
    Version result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        const auto dot = value.find('.');
        const auto part = value.substr(0, dot);
        if (part.empty() || (part.size() > 1 && part.front() == '0') ||
            (dot == value.npos) != (i == result.size() - 1))
            return {};
        const auto end = part.data() + part.size();
        const auto parsed = std::from_chars(part.data(), end, result[i]);
        if (parsed.ec != std::errc{} || parsed.ptr != end)
            return {};
        if (dot != value.npos)
            value.remove_prefix(dot + 1);
    }
    return result;
}

struct Internet {
    const HINTERNET handle;
    ~Internet() { if (handle) WinHttpCloseHandle(handle); }
};

struct RateLimited : std::runtime_error {
    RateLimited() : std::runtime_error("GitHub 匿名请求已达上限") {}
};

std::string get(std::stop_token stop, const wchar_t* host, const std::wstring& path, std::size_t limit,
                const wchar_t* headers = WINHTTP_NO_ADDITIONAL_HEADERS, DWORD proxy = WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY) {
    const auto agent = L"IMUStudio/" + std::wstring(app_version_w);
    const Internet session{WinHttpOpen(agent.c_str(), proxy, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0)};
    if (!session.handle)
        win_error("WinHttpOpen");
    WinHttpSetTimeouts(session.handle, 4000, 4000, 5000, 5000);
    const Internet server{WinHttpConnect(session.handle, host, INTERNET_DEFAULT_HTTPS_PORT, 0)};
    if (!server.handle)
        win_error("WinHttpConnect");
    const Internet request{WinHttpOpenRequest(server.handle, L"GET", path.c_str(), nullptr, WINHTTP_NO_REFERER,
                                              WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE)};
    if (!request.handle)
        win_error("WinHttpOpenRequest");
    DWORD redirects = WINHTTP_OPTION_REDIRECT_POLICY_DISALLOW_HTTPS_TO_HTTP;
    if (!WinHttpSetOption(request.handle, WINHTTP_OPTION_REDIRECT_POLICY, &redirects, sizeof(redirects)) ||
        !WinHttpSendRequest(request.handle, headers, headers ? static_cast<DWORD>(-1) : 0,
            WINHTTP_NO_REQUEST_DATA, 0, 0, 0) || !WinHttpReceiveResponse(request.handle, nullptr))
        win_error("GitHub 请求");
    DWORD code{}, size = sizeof(code);
    if (!WinHttpQueryHeaders(request.handle, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
            WINHTTP_HEADER_NAME_BY_INDEX, &code, &size, WINHTTP_NO_HEADER_INDEX))
        win_error("GitHub 响应");
    if (code == 403 || code == 429) {
        DWORD remaining{};
        if (WinHttpQueryHeaders(request.handle, WINHTTP_QUERY_CUSTOM | WINHTTP_QUERY_FLAG_NUMBER,
                L"X-RateLimit-Remaining", &remaining, &size, WINHTTP_NO_HEADER_INDEX) && remaining == 0)
            throw RateLimited();
    }
    if (code != 200)
        throw std::runtime_error("GitHub HTTP " + std::to_string(code));

    std::string data;
    std::array<char, 16384> buffer{};
    while (!stop.stop_requested()) {
        DWORD received{};
        if (!WinHttpReadData(request.handle, buffer.data(), static_cast<DWORD>(buffer.size()), &received))
            win_error("GitHub 下载");
        if (!received)
            return data;
        if (data.size() + received > limit)
            throw std::runtime_error("GitHub 响应超出预期大小");
        data.append(buffer.data(), received);
    }
    throw std::runtime_error("更新已取消");
}

std::string releases_page(std::stop_token stop, int page) {
    const auto path = L"/repos/" + wide(github_repo) + L"/releases?per_page=100&page=" + std::to_wstring(page);
    constexpr auto headers = L"Accept: application/vnd.github+json\r\nX-GitHub-Api-Version: 2022-11-28\r\n";
    constexpr auto limit = 8 * 1024 * 1024;
    try {
        return get(stop, L"api.github.com", path, limit, headers);
    } catch (const RateLimited&) {
        if (!stop.stop_requested())
            return get(stop, L"api.github.com", path, limit, headers, WINHTTP_ACCESS_TYPE_NO_PROXY);
        throw;
    }
}

fs::path executable_path() {
    std::wstring path(32768, L'\0');
    const DWORD size = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (!size || size == path.size())
        win_error("GetModuleFileName");
    path.resize(size);
    return path;
}

std::wstring quote(const fs::path& path) { return L"\"" + path.wstring() + L"\""; }

void launch(const fs::path& file, std::wstring arguments) {
    auto command = quote(file) + L" " + arguments;
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION child{};
    if (!CreateProcessW(file.c_str(), command.data(), nullptr, nullptr, FALSE, 0, nullptr,
                        file.parent_path().c_str(), &startup, &child))
        win_error("启动 IMUStudio");
    CloseHandle(child.hThread);
    CloseHandle(child.hProcess);
}

void wait_for_exit(const wchar_t* pid) {
    wchar_t* end{};
    const auto id = std::wcstoul(pid, &end, 10);
    if (!id || *end)
        throw std::runtime_error("更新进程参数无效");
    const Handle process(OpenProcess(SYNCHRONIZE, FALSE, id));
    if (!process) {
        const auto error = GetLastError();
        if (error != ERROR_INVALID_PARAMETER)
            win_error("OpenProcess", error);
    } else if (WaitForSingleObject(process.get(), 30000) != WAIT_OBJECT_0) {
        throw std::runtime_error("等待 IMUStudio 退出超时");
    }
}

bool is_staged(const fs::path& staged, const fs::path& installed) {
    return staged.parent_path() == installed.parent_path() &&
        staged.filename().wstring().starts_with(staging_prefix) && staged.extension() == L".exe";
}

DWORD replace_exe(const fs::path& source, const fs::path& target) {
    for (int attempt = 0;; ++attempt) {
        if (MoveFileExW(source.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            return ERROR_SUCCESS;
        const auto error = GetLastError();
        if (attempt == 10 || (error != ERROR_SHARING_VIOLATION && error != ERROR_ACCESS_DENIED))
            return error;
        Sleep(50);
    }
}
}

std::optional<Release> latest_release(std::string_view json, std::string_view current, std::size_t* count) {
    auto latest = version(current).value();
    const auto releases = nlohmann::json::parse(json);
    if (!releases.is_array())
        throw std::runtime_error("GitHub Release 数据格式无效");
    if (count)
        *count = releases.size();
    std::optional<Release> found;
    for (const auto& release : releases) {
        const auto tag = release.value("tag_name", std::string());
        if (!tag.starts_with(prefix) || release.value("draft", true) || release.value("prerelease", true))
            continue;
        const auto candidate = version(std::string_view(tag).substr(prefix.size()));
        if (!candidate || *candidate <= latest)
            continue;
        for (const auto& asset : release.at("assets")) {
            if (asset.value("name", "") != std::string_view("IMUStudio.exe") || asset.value("state", "") != std::string_view("uploaded") ||
                !asset.contains("digest") || !asset["digest"].is_string())
                continue;
            const auto digest = asset["digest"].get<std::string>();
            const auto bytes = asset.value("size", std::size_t{});
            if (digest.size() != 71 || !digest.starts_with("sha256:") ||
                digest.find_first_not_of("0123456789abcdef", 7) != std::string::npos || !bytes || bytes > max_update_size)
                continue;
            latest = *candidate;
            found = Release{tag, digest.substr(7), bytes};
        }
    }
    return found;
}

std::string sha256(std::string_view data) {
    BCRYPT_ALG_HANDLE algorithm{};
    if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) != 0)
        throw std::runtime_error("无法初始化 SHA-256");
    BCRYPT_HASH_HANDLE hash{};
    std::array<UCHAR, 32> digest{};
    auto code = BCryptCreateHash(algorithm, &hash, nullptr, 0, nullptr, 0, 0);
    if (!code)
        code = BCryptHashData(hash, reinterpret_cast<PUCHAR>(const_cast<char*>(data.data())),
                              static_cast<ULONG>(data.size()), 0);
    if (!code)
        code = BCryptFinishHash(hash, digest.data(), static_cast<ULONG>(digest.size()), 0);
    if (hash)
        BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(algorithm, 0);
    if (code != 0)
        throw std::runtime_error("SHA-256 计算失败");
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(64);
    for (const auto byte : digest) {
        result += digits[byte >> 4];
        result += digits[byte & 15];
    }
    return result;
}

Updater::~Updater() {
    stop();
    if (worker_.joinable())
        worker_.join();
    if (!staged_.empty())
        DeleteFileW(staged_.c_str());
}

UpdateStatus Updater::status() {
    std::lock_guard lock(mutex_);
    return state_;
}

void Updater::set(UpdatePhase phase, std::wstring message) {
    std::lock_guard lock(mutex_);
    state_.phase = phase;
    state_.message = std::move(message);
}

void Updater::check() {
    const auto phase = status().phase;
    if (phase == UpdatePhase::Checking || phase == UpdatePhase::Downloading || phase == UpdatePhase::Ready)
        return;
    if (worker_.joinable())
        worker_.join();
    set(UpdatePhase::Checking, L"正在检查 GitHub 更新");
    worker_ = std::jthread([this](std::stop_token stop) {
        try {
            std::optional<Release> latest;
            for (int page = 1; !stop.stop_requested(); ++page) {
                const auto body = releases_page(stop, page);
                std::size_t count{};
                const auto candidate = latest_release(body,
                    latest ? std::string_view(latest->tag).substr(prefix.size()) : app_version, &count);
                if (candidate)
                    latest = candidate;
                if (count < 100)
                    break;
            }
            std::lock_guard lock(mutex_);
            state_ = {latest ? UpdatePhase::Available : UpdatePhase::Current, latest,
                latest ? L"发现 " + wide(latest->tag.substr(prefix.size())) : L"当前已是最新版本"};
        } catch (const std::exception& error) {
            set(UpdatePhase::Failed, L"检查更新失败 · " + wide(error.what()));
        }
    });
}

void Updater::download() {
    const auto state = status();
    if (state.phase != UpdatePhase::Available || !state.release)
        return;
    if (worker_.joinable())
        worker_.join();
    set(UpdatePhase::Downloading, L"正在下载并校验更新");
    worker_ = std::jthread([this, release = *state.release](std::stop_token stop) {
        fs::path path;
        try {
            const auto data = get(stop, L"github.com",
                L"/" + wide(github_repo) + L"/releases/download/" + wide(release.tag) + L"/IMUStudio.exe", release.bytes);
            if (data.size() != release.bytes || sha256(data) != release.digest)
                throw std::runtime_error("更新文件 SHA-256 校验失败");
            const auto target = executable_path();
            path = target.parent_path() / (std::wstring(staging_prefix) + std::to_wstring(GetCurrentProcessId()) +
                L"-" + std::to_wstring(now_ns()) + L".exe");
            const Handle file(CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr));
            if (!file)
                win_error("暂存更新");
            DWORD written{};
            if (!WriteFile(file.get(), data.data(), static_cast<DWORD>(data.size()), &written, nullptr) ||
                written != data.size() || !FlushFileBuffers(file.get()))
                win_error("写入更新");
            staged_ = path;
            set(UpdatePhase::Ready, L"更新已校验，重启后生效");
        } catch (const std::exception& error) {
            if (!path.empty())
                DeleteFileW(path.c_str());
            set(UpdatePhase::Failed, L"下载失败 · " + wide(error.what()));
        }
    });
}

bool Updater::restart() {
    if (status().phase != UpdatePhase::Ready)
        return false;
    if (worker_.joinable())
        worker_.join();
    try {
        launch(staged_, L"--apply-update " + std::to_wstring(GetCurrentProcessId()) + L" " + quote(executable_path()));
        staged_.clear();
        return true;
    } catch (const std::exception& error) {
        set(UpdatePhase::Ready, L"重启失败 · " + wide(error.what()));
        return false;
    }
}

int apply_update(const wchar_t* parent, const fs::path& target) {
    const auto staged = executable_path();
    const fs::path replacement = staged.wstring() + L".new", previous = staged.wstring() + L".old";
    bool backed_up = false, replaced = false;
    try {
        if (!is_staged(staged, target))
            throw std::runtime_error("更新路径无效");
        wait_for_exit(parent);
        if (!CopyFileW(staged.c_str(), replacement.c_str(), TRUE) || !CopyFileW(target.c_str(), previous.c_str(), TRUE))
            win_error("准备替换");
        backed_up = true;
        if (const auto error = replace_exe(replacement, target))
            win_error("替换 EXE", error);
        replaced = true;
        launch(target, L"--remove-update " + std::to_wstring(GetCurrentProcessId()) + L" " + quote(staged));
        DeleteFileW(previous.c_str());
        return 0;
    } catch (const std::exception& error) {
        DeleteFileW(replacement.c_str());
        auto message = L"更新失败 · " + wide(error.what());
        if (replaced) {
            if (replace_exe(previous, target))
                message += L"\n旧版位置 " + previous.wstring();
        } else if (backed_up) {
            DeleteFileW(previous.c_str());
        }
        MessageBoxW(nullptr, message.c_str(), L"IMUStudio", MB_ICONERROR);
        return 1;
    }
}

void remove_update(const wchar_t* parent, const fs::path& staged) {
    try {
        if (is_staged(staged, executable_path())) {
            wait_for_exit(parent);
            DeleteFileW(staged.c_str());
        }
    } catch (const std::exception&) {
    }
}
