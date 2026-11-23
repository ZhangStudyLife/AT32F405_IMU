#include "protocol.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace {
constexpr auto crc_table = [] {
    std::array<std::uint16_t, 256> table{};
    for (unsigned i = 0; i < table.size(); ++i) {
        unsigned crc = i << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        table[i] = static_cast<std::uint16_t>(crc);
    }
    return table;
}();

std::uint16_t u16(Bytes bytes, std::size_t at) {
    return static_cast<std::uint16_t>(bytes[at] | (bytes[at + 1] << 8));
}

void write_u32(std::span<Byte> bytes, std::size_t at, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i)
        bytes[at + i] = static_cast<Byte>(value >> (8 * i));
}
}

std::uint32_t u32(Bytes bytes, std::size_t at) {
    return u16(bytes, at) | (static_cast<std::uint32_t>(u16(bytes, at + 2)) << 16);
}

std::uint64_t u64(Bytes bytes, std::size_t at) {
    return u32(bytes, at) | (static_cast<std::uint64_t>(u32(bytes, at + 4)) << 32);
}

float f32(Bytes bytes, std::size_t at) { return std::bit_cast<float>(u32(bytes, at)); }
std::uint32_t Frame::seq() const { return u32(wire, 4); }

std::uint16_t crc16(Bytes bytes) {
    std::uint16_t crc = 0xffff;
    for (const Byte byte : bytes)
        crc = static_cast<std::uint16_t>((crc << 8) ^ crc_table[(crc >> 8) ^ byte]);
    return crc;
}

void Parser::feed(Bytes bytes, const std::function<void(Frame)>& receive) {
    buffered_.insert(buffered_.end(), bytes.begin(), bytes.end());
    const Bytes data(buffered_);
    std::size_t pos = 0;
    while (data.size() - pos >= 2) {
        if (data[pos] != 0xa5 || data[pos + 1] != 0x5a) {
            ++counts.discarded;
            ++pos;
            continue;
        }
        if (data.size() - pos < 4)
            break;
        const Byte kind = data[pos + 2], length = data[pos + 3];
        if ((kind == 1 && (length < 18 || length > 82)) || (kind == 0x12 && length != 56) ||
            (kind == 0x11 && (length < 2 || length > 134)) || (kind == 0x10 && (length < 1 || length > 8))) {
            ++counts.length;
            ++pos;
            continue;
        }
        const std::size_t size = length + 10;
        if (data.size() - pos < size)
            break;
        const auto raw = data.subspan(pos, size);
        if (crc16(raw.subspan(2, size - 4)) != u16(raw, size - 2)) {
            ++counts.crc;
            ++pos;
            continue;
        }
        if (kind == 1 || (kind >= 0x10 && kind <= 0x12))
            receive({raw});
        pos += size;
    }
    buffered_.erase(buffered_.begin(), buffered_.begin() + pos);
}

int frame_size(int mask) {
    int size = 16;
    for (std::size_t group = 0; group < group_size.size(); ++group)
        if (mask & (1 << group))
            size += group_size[group] * 4;
    return size;
}

const wchar_t* config_error(Config config) {
    if (std::find(baud_rates.begin(), baud_rates.end() - 1, config.baud) == baud_rates.end() - 1 ||
        std::ranges::find(sample_rates, config.rate) == sample_rates.end() || config.mask < 1 || config.mask > 15)
        return L"波特率、频率或数据组无效";
    if (frame_size(config.mask) * config.rate * 100 > config.baud * 8)
        return L"超过串口 80% 带宽";
    return nullptr;
}

const wchar_t* result_text(unsigned result) {
    constexpr std::array messages{
        L"成功", L"忙", L"标定姿态不足", L"运动干扰", L"重复姿态", L"矩阵奇异", L"标定质量不合格",
        L"标定超时", L"命令或操作顺序错误", L"参数非法", L"带宽不足", L"Flash 保存失败", L"当前不可用"
    };
    return result < messages.size() ? messages[result] : L"未知结果";
}

bool decode_sample(Frame frame, Sample& sample) {
    const Bytes bytes = frame.payload();
    if (bytes.size() < 6)
        return false;
    const Byte mask = bytes[5];
    if (!mask || mask > 15 || static_cast<int>(frame.wire.size()) != frame_size(mask))
        return false;
    sample.seq = frame.seq();
    sample.stamp = u32(bytes, 0);
    sample.flags = bytes[4];
    sample.mask = mask;
    sample.values.fill(std::numeric_limits<float>::quiet_NaN());
    std::size_t offset = 6;
    for (std::size_t group = 0; group < group_size.size(); ++group) {
        if (!(mask & (1 << group)))
            continue;
        for (int i = 0; i < group_size[group]; ++i, offset += 4) {
            const float value = f32(bytes, offset);
            if (!std::isfinite(value))
                return false;
            sample.values[group_start[group] + i] = value;
        }
    }
    return true;
}

bool valid_reply(Bytes p) {
    constexpr std::array sizes{0, 132, 0, 0, 0, 0, 41, 0, 56, 64, 0};
    if (p.size() < 2 || p[0] < Query || p[0] > Reinit)
        return false;
    if (p.size() != static_cast<std::size_t>(2 + (p[1] ? 0 : sizes[p[0]])))
        return false;
    if (!p[1] && p[0] == Parameters)
        for (std::size_t i = 6; i < p.size(); i += 4)
            if (!std::isfinite(f32(p, i)))
                return false;
    return true;
}

std::vector<Byte> encode(Byte kind, std::uint32_t seq, Bytes payload) {
    std::vector<Byte> wire(payload.size() + 10);
    wire[0] = 0xa5;
    wire[1] = 0x5a;
    wire[2] = kind;
    wire[3] = static_cast<Byte>(payload.size());
    write_u32(wire, 4, seq);
    std::ranges::copy(payload, wire.begin() + 8);
    const auto check = crc16(Bytes(wire).subspan(2, payload.size() + 6));
    wire[wire.size() - 2] = static_cast<Byte>(check);
    wire[wire.size() - 1] = static_cast<Byte>(check >> 8);
    return wire;
}

std::vector<Byte> command_frame(std::uint32_t seq, const Command& command) {
    std::array<Byte, 8> payload{command.op};
    std::ranges::copy(command.args, payload.begin() + 1);
    constexpr std::array lengths{0, 1, 8, 2, 2, 1, 2, 1, 1, 1, 1};
    return encode(0x10, seq, Bytes(payload).first(lengths[command.op]));
}

Command configure(Config config) {
    Command request{Configure};
    write_u32(request.args, 0, config.baud);
    request.args[4] = static_cast<Byte>(config.rate);
    request.args[5] = static_cast<Byte>(config.rate >> 8);
    request.args[6] = static_cast<Byte>(config.mask);
    return request;
}

Config command_config(Bytes args) {
    return {static_cast<int>(u32(args, 0)), u16(args, 4), args[6]};
}
