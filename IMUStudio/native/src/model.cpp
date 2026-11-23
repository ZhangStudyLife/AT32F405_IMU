#include "model.h"

#include <algorithm>
#include <chrono>

Ns now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

Config Status::config() const {
    return {static_cast<int>((*this)[Baud]), static_cast<int>((*this)[Rate]), static_cast<int>((*this)[Mask])};
}

std::uint32_t SampleClock::stamp(Sample& sample, double host_seconds, unsigned step) {
    const auto delta = sample.seq - sequence_, elapsed = sample.stamp - stamp_;
    std::uint32_t missing = 0;
    if (!started_ || delta == 0 || delta >= 0x80000000 || elapsed >= 0x80000000) {
        expanded_ = sample.stamp;
        time_ = started_ ? std::max(time_ + 1e-6, host_seconds) : host_seconds;
        gap_ = true;
    } else {
        expanded_ += elapsed;
        time_ += std::max(1u, elapsed) * 1e-6;
        if (!gap_ && step && delta != step) {
            if (delta > step && delta % step == 0)
                missing = delta / step - 1;
            gap_ = true;
        }
    }
    sample.time = time_;
    sample.device_us = expanded_;
    sample.gap = gap_;
    sequence_ = sample.seq;
    stamp_ = sample.stamp;
    started_ = true;
    gap_ = false;
    return missing;
}

void Model::reset(Ns origin) {
    const auto generation = state.generation + 1;
    state = {};
    state.generation = generation;
    state.origin = rate_since_ = origin;
    clock_ = {};
    requested_.reset();
    last_profile_.reset();
    rate_count_ = event_id_ = 0;
    events.clear();
}

void Model::note(std::wstring text, Ns ns, const char* level) {
    if (events.size() == 2000)
        events.pop_front();
    events.push_back({++event_id_, ns, level, std::move(text)});
}

void Model::clear_timing() {
    state.timing = {};
    state.cpu = 0;
    last_profile_.reset();
}

void Model::copy_since(std::uint64_t generation, std::uint64_t& cursor, std::vector<Sample>& out) const {
    const auto first = state.total - std::min<std::uint64_t>(capacity, state.total);
    cursor = generation == state.generation ? std::max(cursor, first) : first;
    out.clear();
    if (cursor < state.total)
        out.reserve(static_cast<std::size_t>(state.total - cursor));
    while (cursor < state.total)
        out.push_back(samples_[cursor++ % capacity]);
}

bool Model::consume(Frame frame, Ns ns) {
    const auto p = frame.payload();
    switch (frame.kind()) {
    case 1: {
        Sample sample;
        if (!decode_sample(frame, sample))
            return false;
        const auto rate = state.status ? (*state.status)[Status::Rate] : 0;
        if (state.latest && sample.mask != state.latest->mask)
            boundary();
        state.missing += clock_.stamp(sample, (ns - state.origin) * 1e-9, rate ? 1000 / rate : 0);
        samples_[state.total++ % capacity] = sample;
        state.latest = sample;
        state.last_data = ns;
        ++rate_count_;
        const auto elapsed = ns - rate_since_;
        if (elapsed >= 1000000000) {
            state.rate = rate_count_ * 1e9 / elapsed;
            rate_since_ = ns;
            rate_count_ = 0;
        }
        break;
    }
    case 0x10:
        if (p.size() == 8 && p[0] == Configure)
            requested_ = {frame.seq(), command_config(p.subspan(1))};
        return true;
    case 0x11:
        if (!valid_reply(p))
            return false;
        if (!p[1]) {
            if (p[0] == Configure && requested_ && requested_->first == frame.seq()) {
                if (state.status) {
                    const auto cfg = requested_->second;
                    state.status->fields[Status::Baud] = cfg.baud;
                    state.status->fields[Status::Rate] = cfg.rate;
                    state.status->fields[Status::Mask] = cfg.mask;
                }
                requested_.reset();
                boundary();
            }
            if (p[0] == Output || p[0] == Save || p[0] == Reinit)
                boundary();
            if (p[0] == ClearProfile)
                clear_timing();
            apply(static_cast<Op>(p[0]), p.subspan(2));
        }
        break;
    case 0x12:
        apply(Diagnose, p);
        break;
    }
    state.last_rx = ns;
    return true;
}

void Model::apply(Op op, Bytes data) {
    switch (op) {
    case Query: {
        Status status;
        for (std::size_t i = 0; i < status.fields.size(); ++i)
            status.fields[i] = u32(data, i * 4);
        status.rms = f32(data, 124);
        status.max_error = f32(data, 128);
        if (!state.status || status.config() != state.status->config() ||
            ((status[Status::Flags] ^ (*state.status)[Status::Flags]) & output_on))
            boundary();
        state.status = status;
        ++state.status_serial;
        break;
    }
    case Diagnose: {
        Diagnostic diag;
        for (std::size_t i = 0; i < diag.fields.size(); ++i)
            diag.fields[i] = u32(data, i * 4);
        if (state.diagnostic) {
            if (diag[Diagnostic::Edges] != (*state.diagnostic)[Diagnostic::Edges])
                state.interrupt = L"INT1 中断计数增长";
            else if (diag[Diagnostic::Fallback] != (*state.diagnostic)[Diagnostic::Fallback])
                state.interrupt = L"INT1 未增长，正在使用状态补读";
            else
                state.interrupt = L"中断与补读计数无变化";
        }
        state.diagnostic = diag;
        break;
    }
    case Parameters: {
        Calibration values;
        values.flags = u32(data, 0);
        for (std::size_t i = 0; i < values.values.size(); ++i)
            values.values[i] = f32(data, (i + 1) * 4);
        state.calibration = values;
        break;
    }
    case Profile: {
        const unsigned stage = data[0];
        if (stage >= stage_names.size())
            break;
        Timing timing{u32(data, 1), u32(data, 5), u32(data, 17), u32(data, 21),
                      u32(data, 25), u32(data, 29), u64(data, 9), u64(data, 33)};
        if (last_profile_ && timing.core) {
            const auto elapsed = timing.elapsed - last_profile_->elapsed;
            if (elapsed && elapsed < 10000000 && timing.idle >= last_profile_->idle)
                state.cpu = std::clamp(100.0 * (1 - (timing.idle - last_profile_->idle) * 1e6 /
                    (static_cast<double>(elapsed) * timing.core)), 0.0, 100.0);
        }
        state.timing[stage] = timing;
        last_profile_ = timing;
        break;
    }
    default:
        break;
    }
}
