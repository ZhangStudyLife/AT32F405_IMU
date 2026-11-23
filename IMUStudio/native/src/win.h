#pragma once

#include <windows.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

class Handle {
public:
    explicit Handle(HANDLE value = nullptr) : value_(value) {}
    ~Handle() { reset(); }
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    Handle(Handle&& other) noexcept : value_(std::exchange(other.value_, nullptr)) {}
    Handle& operator=(Handle&& other) noexcept {
        reset(std::exchange(other.value_, nullptr));
        return *this;
    }
    HANDLE get() const { return value_; }
    explicit operator bool() const { return value_ && value_ != INVALID_HANDLE_VALUE; }
    void reset(HANDLE value = nullptr) {
        if (*this)
            CloseHandle(value_);
        value_ = value;
    }

private:
    HANDLE value_;
};

inline std::string utf8(std::wstring_view value) {
    const int size = static_cast<int>(value.size());
    std::string text(WideCharToMultiByte(CP_UTF8, 0, value.data(), size, nullptr, 0, nullptr, nullptr), '\0');
    if (!text.empty())
        WideCharToMultiByte(CP_UTF8, 0, value.data(), size, text.data(), static_cast<int>(text.size()), nullptr, nullptr);
    return text;
}

inline std::wstring wide(std::string_view value) {
    const int size = static_cast<int>(value.size());
    std::wstring text(MultiByteToWideChar(CP_UTF8, 0, value.data(), size, nullptr, 0), L'\0');
    if (!text.empty())
        MultiByteToWideChar(CP_UTF8, 0, value.data(), size, text.data(), static_cast<int>(text.size()));
    return text;
}

[[noreturn]] inline void win_error(const char* operation, DWORD code = GetLastError()) {
    wchar_t message[384]{};
    FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code, 0,
                   message, static_cast<DWORD>(std::size(message)), nullptr);
    std::wstring_view detail(message);
    const auto last = detail.find_last_not_of(L" \r\n");
    throw std::runtime_error(std::string(operation) + " (" + std::to_string(code) + ") " +
                             utf8(detail.substr(0, last == detail.npos ? 0 : last + 1)));
}
