#include "log.h"
#include "version.h"
#include "win.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sqlite3.h>

namespace {
using Json = nlohmann::json;
using namespace std::chrono_literals;
constexpr Ns second = 1000000000;

class Db {
public:
    Db(const fs::path& path, int flags) {
        const int code = sqlite3_open_v2(utf8(path.wstring()).c_str(), &db_, flags, nullptr);
        if (code != SQLITE_OK) {
            const std::string message = sqlite3_errmsg(db_);
            sqlite3_close(db_);
            throw std::runtime_error(message);
        }
        sqlite3_busy_timeout(db_, 1000);
    }
    ~Db() { sqlite3_close(db_); }
    Db(const Db&) = delete;
    void check(int code) const {
        if (code != SQLITE_OK)
            throw std::runtime_error(sqlite3_errmsg(db_));
    }
    void exec(const char* sql) { check(sqlite3_exec(db_, sql, nullptr, nullptr, nullptr)); }
    sqlite3* get() const { return db_; }

private:
    sqlite3* db_{};
};

class Statement {
public:
    Statement(Db& db, const char* sql) : db_(db) {
        db_.check(sqlite3_prepare_v2(db_.get(), sql, -1, &stmt_, nullptr));
    }
    ~Statement() { sqlite3_finalize(stmt_); }
    Statement(const Statement&) = delete;

    bool next() {
        const int code = sqlite3_step(stmt_);
        if (code != SQLITE_DONE && code != SQLITE_ROW)
            db_.check(code);
        return code == SQLITE_ROW;
    }
    void reset() {
        db_.check(sqlite3_reset(stmt_));
        db_.check(sqlite3_clear_bindings(stmt_));
    }
    void insert() { next(); reset(); }
    void bind(int column, Ns value) { db_.check(sqlite3_bind_int64(stmt_, column, value)); }
    void bind(int column, Bytes value) {
        db_.check(sqlite3_bind_blob(stmt_, column, value.data(), static_cast<int>(value.size()), SQLITE_TRANSIENT));
    }
    void bind(int column, std::string_view value) {
        db_.check(sqlite3_bind_text(stmt_, column, value.data(), static_cast<int>(value.size()), SQLITE_TRANSIENT));
    }
    Ns number(int column) { return sqlite3_column_int64(stmt_, column); }
    bool null(int column) { return sqlite3_column_type(stmt_, column) == SQLITE_NULL; }
    std::string text(int column) {
        const auto value = reinterpret_cast<const char*>(sqlite3_column_text(stmt_, column));
        return value ? std::string(value, sqlite3_column_bytes(stmt_, column)) : std::string();
    }
    Bytes blob(int column) {
        return {static_cast<const Byte*>(sqlite3_column_blob(stmt_, column)),
                static_cast<std::size_t>(sqlite3_column_bytes(stmt_, column))};
    }

private:
    Db& db_;
    sqlite3_stmt* stmt_{};
};

template<class Keys, class Fields>
Json object(const Keys& keys, const Fields& fields) {
    auto out = Json::object();
    for (std::size_t i = 0; i < fields.size(); ++i)
        out[keys[i]] = fields[i];
    return out;
}

Json metadata(const Snapshot& state, const std::wstring& port) {
    Json meta{{"format", 1}, {"app_version", app_version}, {"complete", false}, {"port", utf8(port)},
              {"origin_ns", state.origin}};
    if (state.status) {
        auto fields = object(status_keys, state.status->fields);
        fields["fit_rms"] = state.status->rms;
        fields["max_norm_error"] = state.status->max_error;
        meta["status"] = std::move(fields);
    }
    if (state.diagnostic)
        meta["diag"] = object(diag_keys, state.diagnostic->fields);
    if (state.calibration) {
        const auto& p = state.calibration->values;
        meta["params"] = {{"flags", state.calibration->flags},
            {"gyro_bias", std::vector<float>(p.begin(), p.begin() + 3)},
            {"accel_bias", std::vector<float>(p.begin() + 3, p.begin() + 6)},
            {"matrix", std::vector<float>(p.begin() + 6, p.end())}};
    }
    return meta;
}

struct LogReader {
    explicit LogReader(const fs::path& path) : db(path, SQLITE_OPEN_READONLY) {
        Statement values(db, "SELECT key,value FROM meta");
        while (values.next()) {
            auto value = Json::parse(values.text(1), nullptr, false);
            if (!value.is_discarded())
                meta[values.text(0)] = std::move(value);
        }
        if (meta.value("format", 0) != 1)
            throw std::runtime_error("不支持的 .imulog 格式");
        Statement first_frame(db, "SELECT host_ns FROM records ORDER BY host_ns,id LIMIT 1");
        Statement last_frame(db, "SELECT host_ns FROM records ORDER BY host_ns DESC,id DESC LIMIT 1");
        if (!first_frame.next() || !last_frame.next())
            throw std::runtime_error("日志没有帧");
        first = first_frame.number(0);
        last = last_frame.number(0);
    }

    void initialize(Model& model) const {
        model.reset(meta.value("origin_ns", first));
        if (const auto fields = meta.value("status", Json{}); fields.is_object()) {
            Status status;
            for (std::size_t i = 0; i < status.fields.size(); ++i)
                status.fields[i] = fields.value(status_keys[i], 0u);
            const auto real = [&](const char* name) {
                const auto field = fields.find(name);
                return field != fields.end() && field->is_number() ? field->get<float>() : 0.0f;
            };
            status.rms = real("fit_rms");
            status.max_error = real("max_norm_error");
            model.state.status = status;
            ++model.state.status_serial;
        }
        if (const auto fields = meta.value("diag", Json{}); fields.is_object()) {
            Diagnostic diagnostic;
            for (std::size_t i = 0; i < diagnostic.fields.size(); ++i)
                diagnostic.fields[i] = fields.value(diag_keys[i], 0u);
            model.state.diagnostic = diagnostic;
        }
        if (const auto fields = meta.value("params", Json{}); fields.is_object()) {
            Calibration calibration;
            calibration.flags = fields.value("flags", 0u);
            const auto gyro = fields.value("gyro_bias", std::array<float, 3>{});
            const auto accel = fields.value("accel_bias", std::array<float, 3>{});
            const auto matrix = fields.value("matrix", std::array<float, 9>{});
            std::ranges::copy(gyro, calibration.values.begin());
            std::ranges::copy(accel, calibration.values.begin() + 3);
            std::ranges::copy(matrix, calibration.values.begin() + 6);
            model.state.calibration = calibration;
        }
    }

    Db db;
    Json meta = Json::object();
    Ns first{}, last{};
};

constexpr char record_query[] = "SELECT host_ns,raw FROM records WHERE accepted=1 AND host_ns>=? ORDER BY host_ns,id";
constexpr char event_query[] = "SELECT host_ns,level,text FROM events WHERE host_ns>=? ORDER BY host_ns,id";

struct WireRecord {
    Ns ns;
    std::array<Byte, 144> bytes;
    std::size_t size;
};

void consume(Model& model, Parser& parser, Bytes raw, Ns ns) {
    if (raw.size() > 144 || raw.size() < 10)
        return;
    parser.boundary();
    parser.feed(raw, [&](Frame frame) {
        if (!model.consume(frame, ns))
            ++model.state.invalid;
    });
}

std::string quoted_field(std::string_view text) {
    std::string field = "\"";
    for (auto c : text) {
        if (c == '"')
            field += '"';
        field += c;
    }
    return field + '"';
}
}

Recorder::~Recorder() {
    stop();
    if (thread_.joinable())
        thread_.join();
}

void Recorder::start(fs::path path, const Snapshot& snapshot, std::wstring port) {
    if (status().running)
        return;
    if (thread_.joinable())
        thread_.join();
    {
        std::lock_guard lock(mutex_);
        state_ = {true, true, 0, {}};
        active_ = true;
    }
    thread_ = std::thread(&Recorder::run, this, std::move(path), snapshot, std::move(port));
}

void Recorder::stop() {
    {
        std::lock_guard lock(mutex_);
        state_.active = false;
        active_ = false;
    }
    ready_.notify_one();
}

RecordStatus Recorder::status() {
    std::lock_guard lock(mutex_);
    return state_;
}

void Recorder::frame(Ns ns, bool outgoing, Bytes wire, bool accepted) {
    if (!active_)
        return;
    Entry row;
    row.ns = ns;
    row.size = static_cast<std::uint16_t>(wire.size());
    row.outgoing = outgoing;
    row.accepted = accepted;
    std::ranges::copy(wire, row.wire.begin());
    put(std::move(row));
}

void Recorder::event(Ns ns, const std::wstring& text, const char* level) {
    if (!active_)
        return;
    Entry row;
    row.ns = ns;
    row.text = text;
    row.level = level;
    put(std::move(row));
}

void Recorder::put(Entry entry) {
    std::lock_guard lock(mutex_);
    if (!active_)
        return;
    bytes_ += sizeof(Entry) + entry.text.size() * sizeof(wchar_t) + entry.level.size();
    if (bytes_ > 8 * 1024 * 1024) {
        state_.error = L"日志队列已满，记录不完整";
        state_.active = false;
        active_ = false;
        ready_.notify_one();
        return;
    }
    queue_.push_back(std::move(entry));
}

void Recorder::run(fs::path path, Snapshot snapshot, std::wstring port) {
    try {
        {
            const Handle new_file(CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
                                              CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr));
            if (!new_file)
                win_error("新建日志");
        }
        Db db(path, SQLITE_OPEN_READWRITE);
        db.exec("PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL; BEGIN;"
            "CREATE TABLE meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);"
            "CREATE TABLE records(id INTEGER PRIMARY KEY,host_ns INTEGER NOT NULL,direction INTEGER NOT NULL,"
            "raw BLOB NOT NULL,accepted INTEGER NOT NULL);"
            "CREATE INDEX record_time ON records(host_ns,id);"
            "CREATE INDEX reply_time ON records(substr(raw,9,1),host_ns DESC,id DESC)"
            " WHERE direction=0 AND accepted=1 AND substr(raw,3,1)=x'11';"
            "CREATE TABLE events(id INTEGER PRIMARY KEY,host_ns INTEGER,level TEXT,text TEXT);"
            "CREATE INDEX event_time ON events(host_ns,id);");
        Statement meta(db, "INSERT OR REPLACE INTO meta VALUES(?,?)");
        const auto data = metadata(snapshot, port);
        for (auto it = data.begin(); it != data.end(); ++it) {
            meta.bind(1, it.key());
            meta.bind(2, it.value().dump());
            meta.insert();
        }
        db.exec("COMMIT;");
        Statement record(db, "INSERT INTO records(host_ns,direction,raw,accepted) VALUES(?,?,?,?)");
        Statement event(db, "INSERT INTO events(host_ns,level,text) VALUES(?,?,?)");
        std::vector<Entry> batch;
        Ns next_disk_check{};
        for (;;) {
            {
                std::unique_lock lock(mutex_);
                ready_.wait_for(lock, 100ms, [&] { return !active_; });
                if (queue_.empty() && !active_)
                    break;
                batch.swap(queue_);
                bytes_ = 0;
            }
            if (batch.empty())
                continue;
            const auto now = now_ns();
            if (now >= next_disk_check) {
                if (fs::space(path.parent_path()).available < 256ULL * 1024 * 1024)
                    throw std::runtime_error("磁盘剩余空间不足 256 MiB");
                next_disk_check = now + second;
            }
            db.exec("BEGIN;");
            std::uint64_t frames = 0;
            for (const auto& row : batch) {
                if (row.size) {
                    record.bind(1, row.ns);
                    record.bind(2, row.outgoing);
                    record.bind(3, Bytes(row.wire).first(row.size));
                    record.bind(4, row.accepted);
                    record.insert();
                    ++frames;
                } else {
                    event.bind(1, row.ns);
                    event.bind(2, row.level);
                    event.bind(3, utf8(row.text));
                    event.insert();
                }
            }
            db.exec("COMMIT;");
            batch.clear();
            std::lock_guard lock(mutex_);
            state_.frames += frames;
        }
        const auto stopped = status();
        db.exec("BEGIN;");
        meta.bind(1, "complete");
        meta.bind(2, stopped.error.empty() ? "true" : "false");
        meta.insert();
        meta.bind(1, "error");
        meta.bind(2, Json(utf8(stopped.error)).dump());
        meta.insert();
        db.exec("COMMIT; PRAGMA wal_checkpoint(TRUNCATE);");
    } catch (const std::exception& error) {
        std::lock_guard lock(mutex_);
        state_.error = L"记录失败 · " + wide(error.what());
    }
    std::lock_guard lock(mutex_);
    active_ = state_.active = state_.running = false;
    queue_.clear();
    bytes_ = 0;
}

Replay::~Replay() {
    stop();
}

void Replay::start(fs::path path) {
    if (status().running)
        return;
    if (thread_.joinable())
        thread_.join();
    {
        std::lock_guard lock(mutex_);
        state_ = {};
        state_.running = true;
        seek_.reset();
        speed_ = 1;
    }
    thread_ = std::jthread([this, path = std::move(path)](std::stop_token stop) { run(stop, path); });
}

void Replay::stop() {
    thread_.request_stop();
    ready_.notify_all();
}

void Replay::toggle() {
    std::lock_guard lock(mutex_);
    if (!state_.ready)
        return;
    if (!state_.playing && state_.position >= state_.duration)
        seek_ = 0;
    state_.playing = !state_.playing;
    ready_.notify_one();
}

void Replay::seek(double seconds) {
    std::lock_guard lock(mutex_);
    if (state_.ready)
        seek_ = std::clamp(seconds, 0.0, state_.duration);
    ready_.notify_one();
}

void Replay::speed(double rate) {
    std::lock_guard lock(mutex_);
    speed_ = rate;
}

ReplayStatus Replay::status() {
    std::lock_guard lock(mutex_);
    return state_;
}

void Replay::run(std::stop_token stop, fs::path path) {
    try {
        LogReader reader(path);
        Statement records(reader.db, record_query), events(reader.db, event_query);
        Statement previous(reader.db, "SELECT raw FROM records WHERE host_ns<? AND direction=0 AND accepted=1"
            " AND substr(raw,3,1)=x'11' AND substr(raw,9,1)=? ORDER BY host_ns DESC,id DESC LIMIT 1");
        Parser parser;
        std::vector<WireRecord> batch;
        batch.reserve(256);
        const bool complete = reader.meta.value("complete", false);
        const double duration = (reader.last - reader.first) * 1e-9;
        double position = 0;
        bool have_record{}, have_event{}, was_playing{};
        Ns tick = now_ns();
        {
            std::lock_guard lock(mutex_);
            state_.complete = complete;
            state_.duration = duration;
            state_.ready = true;
            seek_ = 0;
        }
        while (!stop.stop_requested()) {
            bool playing{};
            double speed{};
            std::optional<double> seek;
            {
                std::unique_lock lock(mutex_);
                const auto changing = [&] { return seek_.has_value() || state_.playing != was_playing; };
                if (was_playing)
                    ready_.wait_for(lock, stop, 8ms, changing);
                else
                    ready_.wait(lock, stop, changing);
                seek = std::exchange(seek_, std::nullopt);
                playing = state_.playing;
                speed = speed_;
            }
            if (stop.stop_requested())
                break;
            const auto now = now_ns();
            if (seek) {
                position = *seek;
                const Ns end = reader.first + std::llround(position * second);
                const Ns begin = std::max(reader.first, end - 60 * second);
                {
                    std::lock_guard lock(model_.mutex);
                    reader.initialize(model_);
                    model_.state.replay = true;
                    model_.state.link = complete ? L"日志回放" : L"日志回放 · 记录不完整";
                }
                for (const Byte op : {Query, Diagnose, Parameters}) {
                    previous.bind(1, begin);
                    previous.bind(2, Bytes(&op, 1));
                    if (previous.next()) {
                        std::lock_guard lock(model_.mutex);
                        consume(model_, parser, previous.blob(0), begin);
                    }
                    previous.reset();
                }
                records.reset();
                records.bind(1, begin);
                have_record = records.next();
                events.reset();
                events.bind(1, begin);
                have_event = events.next();
            } else if (playing && was_playing) {
                position = std::min(duration, position + (now - tick) * 1e-9 * speed);
            }
            was_playing = playing;
            tick = now;
            const Ns target = position >= duration ? reader.last : reader.first + std::llround(position * second);
            while (!stop.stop_requested() && have_record && records.number(0) <= target) {
                const auto raw = records.blob(1);
                if (raw.size() <= WireRecord{}.bytes.size()) {
                    auto& item = batch.emplace_back();
                    item.ns = records.number(0);
                    item.size = raw.size();
                    std::ranges::copy(raw, item.bytes.begin());
                }
                have_record = records.next();
                if (batch.size() == 256 || !have_record || records.number(0) > target) {
                    std::lock_guard lock(model_.mutex);
                    for (const auto& item : batch)
                        consume(model_, parser, Bytes(item.bytes).first(item.size), item.ns);
                    batch.clear();
                }
            }
            while (have_event && events.number(0) <= target && !stop.stop_requested()) {
                const auto level = events.text(1);
                const auto message = wide(events.text(2));
                {
                    std::lock_guard lock(model_.mutex);
                    model_.note(message, events.number(0), level.c_str());
                }
                have_event = events.next();
            }
            if (seek)
                tick = now_ns();
            std::lock_guard lock(mutex_);
            state_.position = position;
            if (position >= duration)
                state_.playing = false;
        }
    } catch (const std::exception& error) {
        std::lock_guard lock(mutex_);
        state_.error = L"回放失败 · " + wide(error.what());
    }
    {
        std::lock_guard lock(model_.mutex);
        model_.state.replay = false;
        model_.state.link = L"回放已关闭";
    }
    std::lock_guard lock(mutex_);
    state_.ready = state_.running = state_.playing = false;
}

void Export::start(fs::path source, fs::path csv, double from, double to) {
    if (status().running)
        return;
    if (thread_.joinable())
        thread_.join();
    {
        std::lock_guard lock(mutex_);
        state_ = {0, true, L"正在导出"};
    }
    thread_ = std::jthread([this, source = std::move(source), csv = std::move(csv), from, to](std::stop_token stop) {
        run(stop, source, csv, from, to);
    });
}

ExportStatus Export::status() {
    std::lock_guard lock(mutex_);
    return state_;
}

void Export::run(std::stop_token stop, fs::path source, fs::path csv, double from, double to) {
    const auto event_csv = csv.parent_path() / (csv.stem().wstring() + L"_events.csv");
    const auto suffix = L"." + std::to_wstring(GetCurrentProcessId()) + L".part";
    const fs::path partial = csv.wstring() + suffix, event_partial = event_csv.wstring() + suffix;
    std::wstring message;
    std::uint64_t rows = 0;
    try {
        LogReader reader(source);
        Model model;
        reader.initialize(model);
        Parser parser;
        {
            std::ofstream file(partial, std::ios::binary);
            file.exceptions(std::ios::badbit | std::ios::failbit);
            file << "\xef\xbb\xbfsession_s,device_us,seq,flags,mask";
            for (const auto column : columns)
                file << ',' << column;
            file << "\r\n" << std::setprecision(9);
            Statement records(reader.db, record_query);
            records.bind(1, reader.first);
            while (!stop.stop_requested() && records.next()) {
                const auto ns = records.number(0);
                const double t = (ns - reader.first) * 1e-9;
                if (t > to)
                    break;
                const auto count = model.state.total;
                consume(model, parser, records.blob(1), ns);
                if (count == model.state.total || t < from)
                    continue;
                const auto& sample = *model.state.latest;
                file << std::fixed << t << std::defaultfloat << ',' << sample.device_us << ',' << sample.seq
                     << ',' << unsigned(sample.flags) << ',' << unsigned(sample.mask);
                for (const auto value : sample.values) {
                    file << ',';
                    if (std::isfinite(value))
                        file << value;
                }
                file << "\r\n";
                if (++rows % 256 == 0) {
                    std::lock_guard lock(mutex_);
                    state_.rows = rows;
                }
            }
            file.close();
        }
        if (!stop.stop_requested()) {
            std::ofstream file(event_partial, std::ios::binary);
            file.exceptions(std::ios::badbit | std::ios::failbit);
            file << "\xef\xbb\xbfsession_s,level,event\r\n" << std::fixed << std::setprecision(9);
            Statement events(reader.db, event_query);
            events.bind(1, reader.first);
            while (!stop.stop_requested() && events.next()) {
                const auto t = (events.number(0) - reader.first) * 1e-9;
                if (t > to)
                    break;
                if (t >= from)
                    file << t << ',' << quoted_field(events.text(1)) << ',' << quoted_field(events.text(2)) << "\r\n";
            }
            file.close();
        }
        if (stop.stop_requested()) {
            message = L"导出已取消";
        } else {
            if (!MoveFileExW(partial.c_str(), csv.c_str(), MOVEFILE_REPLACE_EXISTING) ||
                !MoveFileExW(event_partial.c_str(), event_csv.c_str(), MOVEFILE_REPLACE_EXISTING))
                win_error("保存 CSV");
            message = L"导出完成";
        }
    } catch (const std::exception& error) {
        message = L"导出失败 · " + wide(error.what());
    }
    DeleteFileW(partial.c_str());
    DeleteFileW(event_partial.c_str());
    std::lock_guard lock(mutex_);
    state_ = {rows, false, std::move(message)};
}
