#include "draw.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

namespace {
constexpr std::array chart_names{L"欧拉角  /  °", L"四元数", L"角速度  /  °/s", L"加速度  /  g"};
constexpr std::array<std::array<int, 6>, 4> chart_channels{{
    {0, 1, 2, -1}, {3, 4, 5, 6, -1}, {7, 8, 9, 13, 14, 15}, {10, 11, 12, 16, 17, 18}
}};
constexpr std::array chart_labels{
    L"Roll", L"Pitch", L"Yaw", L"W", L"X", L"Y", L"Z", L"Gx", L"Gy", L"Gz", L"Ax", L"Ay", L"Az",
    L"Gx′", L"Gy′", L"Gz′", L"Ax′", L"Ay′", L"Az′"
};
constexpr std::array colors{gray(17), gray(94), gray(147), gray(64)};

bool show_value(const Sample& sample, int channel) {
    return (channel >= 7 || (sample.flags & valid_attitude)) && std::isfinite(sample.values[channel]);
}

std::array<float, 9> rotation(const Sample& sample) {
    const auto& v = sample.values;
    if (sample.mask & 2) {
        const double norm = std::hypot(std::hypot(v[3], v[4]), std::hypot(v[5], v[6]));
        if (norm < 1e-8)
            return {};
        const float w = static_cast<float>(v[3] / norm), x = static_cast<float>(v[4] / norm);
        const float y = static_cast<float>(v[5] / norm), z = static_cast<float>(v[6] / norm);
        return {1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w),
            2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w),
            2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)};
    }
    if (!(sample.mask & 1))
        return {};
    constexpr float rad = std::numbers::pi_v<float> / 180;
    const float cr = std::cos(v[0]*rad), sr = std::sin(v[0]*rad), cp = std::cos(v[1]*rad);
    const float sp = std::sin(v[1]*rad), cy = std::cos(v[2]*rad), sy = std::sin(v[2]*rad);
    return {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
            sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, -sp, cp*sr, cp*cr};
}
}

Fonts::~Fonts() {
    for (const auto font : {body, caption, strong, title, number, mono})
        if (font)
            DeleteObject(font);
}

void Fonts::set(int value) {
    if (dpi == value)
        return;
    dpi = value;
    const auto create = [&](HFONT& font, int height, int weight, const wchar_t* face) {
        if (font)
            DeleteObject(font);
        font = CreateFontW(-px(height), 0, 0, 0, weight, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
            OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, face);
    };
    create(body, 14, FW_NORMAL, L"Microsoft YaHei UI");
    create(caption, 12, FW_NORMAL, L"Microsoft YaHei UI");
    create(strong, 15, FW_SEMIBOLD, L"Microsoft YaHei UI");
    create(title, 27, FW_SEMIBOLD, L"Segoe UI");
    create(number, 31, FW_SEMIBOLD, L"Segoe UI");
    create(mono, 13, FW_NORMAL, L"Consolas");
}

void Paint::fill(RECT r, COLORREF color) const {
    SetDCBrushColor(dc, color);
    FillRect(dc, &r, static_cast<HBRUSH>(GetStockObject(DC_BRUSH)));
}

void Paint::border(RECT r, COLORREF color) const {
    SetDCBrushColor(dc, color);
    FrameRect(dc, &r, static_cast<HBRUSH>(GetStockObject(DC_BRUSH)));
}

void Paint::line(int x1, int y1, int x2, int y2, COLORREF color) const {
    SelectObject(dc, GetStockObject(DC_PEN));
    SetDCPenColor(dc, color);
    MoveToEx(dc, x1, y1, nullptr);
    LineTo(dc, x2, y2);
}

void Paint::text(RECT r, std::wstring_view value, HFONT font, COLORREF color, UINT flags) const {
    if (value.empty())
        return;
    SelectObject(dc, font ? font : fonts.body);
    SetTextColor(dc, color);
    SetBkMode(dc, TRANSPARENT);
    DrawTextW(dc, value.data(), static_cast<int>(value.size()), &r, flags | DT_NOPREFIX);
}

Buffer::~Buffer() {
    if (dc_) {
        SelectObject(dc_, original_);
        DeleteObject(bitmap_);
        DeleteDC(dc_);
    }
}

HDC Buffer::get(HDC source, int width, int height) {
    if (width <= 0 || height <= 0)
        return nullptr;
    if (bitmap_ && width == width_ && height == height_)
        return dc_;
    if (!dc_)
        dc_ = CreateCompatibleDC(source);
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = width;
    info.bmiHeader.biHeight = -height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    void* pixels{};
    const auto bitmap = CreateDIBSection(dc_, &info, DIB_RGB_COLORS, &pixels, nullptr, 0);
    if (!dc_ || !bitmap) {
        if (bitmap)
            DeleteObject(bitmap);
        return nullptr;
    }
    const auto old = SelectObject(dc_, bitmap);
    if (bitmap_)
        DeleteObject(bitmap_);
    else
        original_ = old;
    bitmap_ = bitmap;
    pixels_ = static_cast<std::uint32_t*>(pixels);
    width_ = width;
    height_ = height;
    return dc_;
}

RECT inset(RECT rect, int x, int y) {
    InflateRect(&rect, -x, -y);
    return rect;
}

std::wstring fixed(double number, int places) {
    wchar_t buffer[64]{};
    if (!std::isfinite(number) || std::abs(number) >= 1e12)
        return significant(number);
    std::swprintf(buffer, std::size(buffer), L"%.*f", places, number == 0 ? 0.0 : number);
    return buffer;
}

std::wstring significant(double number, int digits) {
    wchar_t buffer[48]{};
    std::swprintf(buffer, std::size(buffer), L"%.*g", digits, number);
    return buffer;
}

void History::append(std::span<const Sample> samples) {
    for (const auto& sample : samples) {
        const auto index = total_++ % Model::capacity;
        samples_[index] = sample;
        auto& block = blocks_[index / block_size];
        const bool fresh = index % block_size == 0;
        if (fresh) {
            block.absent = 0;
            block.gap = false;
        }
        block.gap |= sample.gap;
        for (int i = 0; i < static_cast<int>(columns.size()); ++i) {
            if (!show_value(sample, i)) {
                block.absent |= 1u << i;
                continue;
            }
            const auto value = sample.values[i];
            auto& range = block.values[i];
            range = fresh ? Range{value, value} : Range{std::min(range.low, value), std::max(range.high, value)};
        }
    }
}

std::uint64_t History::search(double time) const {
    auto left = first(), right = total_;
    while (left < right) {
        const auto middle = left + (right - left) / 2;
        if (at(middle).time < time)
            left = middle + 1;
        else
            right = middle;
    }
    return left;
}

const Sample& History::nearest(double time) const {
    auto i = search(time);
    if (i == total_ || (i > first() && time - at(i - 1).time < at(i).time - time))
        --i;
    return at(i);
}

void History::bins(int channel, double from, double to, int width, std::vector<PlotBin>& result) const {
    result.clear();
    const double scale = (width - 1) / (to - from);
    const auto x = [&](double t) { return static_cast<int>((t - from) * scale); };
    bool gap = true;
    for (auto i = search(from), end = search(std::nextafter(to, std::numeric_limits<double>::infinity())); i < end;) {
        const auto& sample = at(i);
        const int pixel = x(sample.time);
        const auto& block = blocks_[(i % Model::capacity) / block_size];
        if (end - i >= block_size && i % block_size == 0 && !block.gap &&
            !(block.absent & (1u << channel)) && x(at(i + block_size - 1).time) == pixel) {
            const auto range = block.values[channel];
            if (gap || result.empty() || pixel != result.back().x)
                result.push_back({pixel, range.low, range.high, gap});
            else {
                result.back().low = std::min(result.back().low, range.low);
                result.back().high = std::max(result.back().high, range.high);
            }
            gap = false;
            i += block_size;
            continue;
        }
        ++i;
        if (!show_value(sample, channel)) {
            gap = true;
            continue;
        }
        gap |= sample.gap;
        const auto value = sample.values[channel];
        if (gap || result.empty() || pixel != result.back().x)
            result.push_back({pixel, value, value, gap});
        else {
            result.back().low = std::min(result.back().low, value);
            result.back().high = std::max(result.back().high, value);
        }
        gap = false;
    }
}

void Charts::draw(const Paint& p, RECT card, int group, const History& history) {
    p.fill(card, paper);
    p.border(card);
    const int margin = p.px(16);
    p.text({card.left + margin, card.top + p.px(8), card.right - margin, card.top + p.px(35)},
           chart_names[group], p.fonts.strong);
    const auto& channels = chart_channels[group];
    const int count = group == 0 ? 3 : group == 1 ? 4 : 6;
    const int cell = (card.right - card.left - margin * 2) / count;
    for (int i = 0; i < count; ++i) {
        const int channel = channels[i], x = card.left + margin + i * cell;
        RECT hit{x, card.top + p.px(36), x + cell, card.top + p.px(59)};
        legends_[channel] = hit;
        const bool on = channels_ & (1u << channel);
        const auto value = history.empty() ? std::numeric_limits<float>::quiet_NaN() : history.latest().values[channel];
        const auto label = std::wstring(chart_labels[channel]) + (std::isfinite(value) ? L" " + significant(value, 4) : L"");
        const auto y = (hit.top + hit.bottom) / 2;
        const auto color = on ? colors[i % (group == 1 ? 4 : 3)] : rule;
        p.line(x, y, x + p.px(channel >= 13 ? 5 : 10), y, color);
        if (channel >= 13)
            p.line(x + p.px(7), y, x + p.px(11), y, color);
        hit.left += p.px(15);
        hit.right -= p.px(3);
        p.text(hit, label, p.fonts.caption, on ? ink : muted);
    }
    const RECT plot{card.left + p.px(58), card.top + p.px(70), card.right - margin, card.bottom - p.px(31)};
    panels_[group] = plot;
    const int width = plot.right - plot.left, height = plot.bottom - plot.top;
    if (width <= 1 || height <= 1)
        return;
    const double right = end.value_or(history.empty() ? seconds : history.latest().time);
    const double left = right - seconds;
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (int i = 0; i < count; ++i) {
        const int ch = channels[i];
        bins_[ch].clear();
        if (!(channels_ & (1u << ch)))
            continue;
        history.bins(ch, left, right, width, bins_[ch]);
        for (const auto& bin : bins_[ch]) {
            low = std::min(low, static_cast<double>(bin.low));
            high = std::max(high, static_cast<double>(bin.high));
        }
    }
    const bool have_values = std::isfinite(low);
    if (!have_values) {
        low = -1;
        high = 1;
    }
    const double center = (high + low) * 0.5;
    const double span = std::max((high - low) * 0.56, std::max(0.05, std::abs(center) * 1e-6)) * y_scale_[group];
    const auto centered = DT_CENTER | DT_VCENTER | DT_SINGLELINE;
    for (int i = 0; i <= 4; ++i) {
        const int y = plot.top + height * i / 4, x = plot.left + width * i / 4;
        const auto c = i == 0 || i == 4 ? rule : gray(237);
        p.line(plot.left, y, plot.right + 1, y, c);
        p.line(x, plot.top, x, plot.bottom + 1, c);
        p.text({card.left + p.px(4), y - p.px(9), plot.left - p.px(7), y + p.px(9)},
            significant(center + span * (1 - i * 0.5), 4), p.fonts.caption, muted,
            DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
        const auto tick_x = std::clamp<LONG>(x, plot.left + p.px(15), plot.right - p.px(18));
        p.text({tick_x - p.px(34), plot.bottom + p.px(5), tick_x + p.px(34), card.bottom},
            fixed(left + seconds * i / 4, 1) + (i == 4 ? L" s" : L""), p.fonts.caption, muted, centered);
    }

    if (PtInRect(&plot, mouse)) {
        hover_ = group;
        p.line(mouse.x, plot.top, mouse.x, plot.bottom, muted);
    }
    if (!have_values) {
        p.text(plot, history.empty() ? L"等待数据" : L"此窗口无可显示数据", p.fonts.body, muted, centered);
        return;
    }

    const int saved = SaveDC(p.dc);
    IntersectClipRect(p.dc, plot.left, plot.top, plot.right, plot.bottom + 1);
    const auto map_y = [&](float value) -> LONG {
        const auto y = std::clamp((center + span - value) / (span * 2), -1.0, 2.0);
        return plot.top + static_cast<LONG>(std::lround(height * y));
    };
    for (int i = 0; i < count; ++i) {
        const int ch = channels[i];
        if (bins_[ch].empty())
            continue;
        const auto color = colors[i % (group == 1 ? 4 : 3)];
        const HPEN dashed = ch >= 13 ? CreatePen(PS_DASH, 1, color) : nullptr;
        SelectObject(p.dc, dashed ? dashed : GetStockObject(DC_PEN));
        SetDCPenColor(p.dc, color);
        points_.clear();
        runs_.clear();
        int start = 0;
        for (const auto& bin : bins_[ch]) {
            if (bin.gap && static_cast<int>(points_.size()) != start) {
                if (points_.size() - start == 1)
                    points_.push_back({points_.back().x + 1, points_.back().y});
                runs_.push_back(static_cast<DWORD>(points_.size() - start));
                start = static_cast<int>(points_.size());
            }
            const LONG x = plot.left + bin.x;
            const LONG bottom = map_y(bin.low), top = map_y(bin.high);
            const bool descends = points_.size() != static_cast<std::size_t>(start) && points_.back().y < (top + bottom) / 2;
            points_.push_back({x, descends ? top : bottom});
            if (bottom != top)
                points_.push_back({x, descends ? bottom : top});
        }
        if (points_.size() - start == 1)
            points_.push_back({points_.back().x + 1, points_.back().y});
        runs_.push_back(static_cast<DWORD>(points_.size() - start));
        PolyPolyline(p.dc, points_.data(), runs_.data(), static_cast<DWORD>(runs_.size()));
        if (dashed) {
            SelectObject(p.dc, GetStockObject(DC_PEN));
            DeleteObject(dashed);
        }
    }
    RestoreDC(p.dc, saved);
}

void Charts::follow() {
    end.reset();
    frozen = false;
    y_scale_.fill(1);
}

int Charts::panel_at(POINT point) const {
    for (int i = 0; i < static_cast<int>(panels_.size()); ++i)
        if (PtInRect(&panels_[i], point))
            return i;
    return -1;
}

bool Charts::toggle(POINT point) {
    for (int i = 0; i < static_cast<int>(legends_.size()); ++i)
        if (PtInRect(&legends_[i], point)) {
            channels_ ^= 1u << i;
            return true;
        }
    return false;
}

void Charts::pan(const History& history, int group, int pixels) {
    if (history.empty())
        return;
    const auto plot = panels_[group];
    const auto last = history.latest().time;
    end = std::clamp(end.value_or(last) - pixels * seconds / std::max(1L, plot.right - plot.left),
        std::min(last, history.nearest(-1e30).time + seconds * 0.1), last);
}

void Charts::zoom(const History& history, int group, POINT point, double factor, bool vertical) {
    if (vertical) {
        y_scale_[group] = std::clamp(y_scale_[group] * factor, 0.05, 30.0);
    } else if (!history.empty()) {
        const auto plot = panels_[group];
        const double portion = (plot.right - point.x) / static_cast<double>(plot.right - plot.left);
        const auto last = history.latest().time;
        const auto anchor = end.value_or(last) - seconds * portion;
        seconds = std::clamp(seconds * factor, 0.2, 60.0);
        end = std::clamp(anchor + seconds * portion, std::min(last, history.nearest(-1e30).time + seconds * 0.1), last);
    }
}

std::wstring Charts::cursor(const History& history) const {
    if (hover_ < 0 || history.empty())
        return {};
    const auto plot = panels_[hover_];
    const double right = end.value_or(history.latest().time);
    const double time = right - seconds * (plot.right - mouse.x) / (plot.right - plot.left);
    const auto& sample = history.nearest(time);
    auto value = fixed(sample.time, 3) + L" s";
    const auto& channels = chart_channels[hover_];
    const int count = hover_ == 0 ? 3 : hover_ == 1 ? 4 : 6;
    for (int i = 0; i < count; ++i) {
        const int ch = channels[i];
        if ((channels_ & (1u << ch)) && show_value(sample, ch))
            value += L"   " + std::wstring(chart_labels[ch]) + L" " + significant(sample.values[ch], 5);
    }
    return value;
}

std::wstring attitude_state(const Snapshot& view, bool stale) {
    const auto local_flags = view.latest ? view.latest->flags : 0;
    const auto flags = view.status ? (*view.status)[Status::Flags] : local_flags;
    if ((flags | local_flags) & sensor_fault)
        return L"模块报告故障";
    if (view.status && !(flags & output_on))
        return L"遥测暂停";
    if (!view.latest)
        return L"等待姿态";
    if (stale)
        return L"数据已过期";
    if (local_flags & warming_up)
        return L"预热中";
    if (!(local_flags & valid_attitude))
        return L"姿态无效";
    if (!(view.latest->mask & 3))
        return L"姿态通道未输出";
    return L"姿态有效";
}

Board::Board(HINSTANCE module) {
    reset();
    const auto res = FindResourceW(module, MAKEINTRESOURCEW(101), RT_RCDATA);
    if (!res)
        return;
    const auto data = static_cast<const Byte*>(LockResource(LoadResource(module, res)));
    const Bytes raw(data, SizeofResource(module, res));
    if (!data || raw.size() < 8)
        return;
    const std::size_t nv = u32(raw, 0), nf = u32(raw, 4);
    static_assert(sizeof(Vec) == 12 && sizeof(Face) == 8);
    if (raw.size() != 8 + nv * sizeof(Vec) + nf * sizeof(Face))
        return;
    vertices_.resize(nv);
    faces_.resize(nf);
    std::memcpy(vertices_.data(), data + 8, nv * sizeof(Vec));
    std::memcpy(faces_.data(), data + 8 + nv * sizeof(Vec), nf * sizeof(Face));
    projected_.resize(nv);
}

void Board::reset() {
    yaw_ = -58 * std::numbers::pi / 180;
    pitch_ = 32 * std::numbers::pi / 180;
    scale_ = 1;
}

void Board::rotate(int x, int y) {
    yaw_ += x * 0.008;
    pitch_ = std::clamp(pitch_ + y * 0.008, -1.5, 1.5);
}

void Board::zoom(double scale) { scale_ = std::clamp(scale_ * scale, 0.4, 2.8); }

void Board::triangle(Point a, Point b, Point c, COLORREF shade, RECT stage) {
    const auto area = (b.xy.x - a.xy.x) * (c.xy.y - a.xy.y) - (b.xy.y - a.xy.y) * (c.xy.x - a.xy.x);
    if (area >= 0)
        return;
    const float dx = ((b.depth - a.depth) * (c.xy.y - a.xy.y) -
                      (c.depth - a.depth) * (b.xy.y - a.xy.y)) / area;
    const float dy = ((c.depth - a.depth) * (b.xy.x - a.xy.x) -
                      (b.depth - a.depth) * (c.xy.x - a.xy.x)) / area;
    const float origin = a.depth - dx * a.xy.x - dy * a.xy.y;
    if (a.xy.y > b.xy.y) std::swap(a, b);
    if (b.xy.y > c.xy.y) std::swap(b, c);
    if (a.xy.y > b.xy.y) std::swap(a, b);
    const int width = stage.right - stage.left;
    const auto edge = [](Point from, Point to, float y) {
        return from.xy.x + (y - from.xy.y) * (to.xy.x - from.xy.x) / (to.xy.y - from.xy.y);
    };
    for (LONG y = std::max(a.xy.y, stage.top); y < std::min(c.xy.y, stage.bottom); ++y) {
        const float middle = y + 0.5f;
        const float x1 = edge(a, c, middle);
        const float x2 = middle < b.xy.y ? edge(a, b, middle) : edge(b, c, middle);
        const int first = std::max<int>(stage.left, static_cast<int>(std::ceil(std::min(x1, x2) - 0.5f)));
        const int last = std::min<int>(stage.right, static_cast<int>(std::ceil(std::max(x1, x2) - 0.5f)));
        if (first >= last)
            continue;
        const auto start = (y - stage.top) * width + first - stage.left;
        auto* pixels = mesh_.pixels() + start;
        auto* depths = depth_.data() + start;
        float depth = origin + dy * middle + dx * (first + 0.5f);
        for (int i = 0; i < last - first; ++i, depth += dx) {
            if (depth > depths[i]) {
                depths[i] = depth;
                pixels[i] = shade;
            }
        }
    }
}

void Board::draw(const Paint& p, RECT card, const Snapshot& snapshot, bool stale) {
    p.fill(card, paper);
    p.border(card);
    const int pad = p.px(18);
    const RECT stage{card.left + pad, card.top + p.px(77), card.right - pad, card.bottom - pad};
    p.text({stage.left, card.top + p.px(12), stage.right, card.top + p.px(39)}, L"三维姿态", p.fonts.strong);
    p.text({stage.left, card.top + p.px(42), stage.right, card.top + p.px(64)}, attitude_state(snapshot, stale), p.fonts.caption, muted);
    if (!ready() || stage.right <= stage.left || stage.bottom <= stage.top)
        return;
    if (!stale && snapshot.latest && (snapshot.latest->flags & valid_attitude)) {
        const auto next = rotation(*snapshot.latest);
        if (next != std::array<float, 9>{})
            orientation_ = next;
    }
    static constexpr std::array<float, 9> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
    const auto& r = orientation_ == std::array<float, 9>{} ? identity : orientation_;
    const double sy = std::sin(yaw_), cy = std::cos(yaw_), sp = std::sin(pitch_), cp = std::cos(pitch_);
    const double scale = scale_ * std::min(stage.right - stage.left, stage.bottom - stage.top) / 38;
    const double cx = (stage.left + stage.right) * 0.5, center_y = (stage.top + stage.bottom) * 0.5;
    const auto view = [&](Vec v) {
        return Vec{static_cast<float>(-sy*v.x + cy*v.y), static_cast<float>(-sp*(cy*v.x + sy*v.y) + cp*v.z),
                   static_cast<float>(cp*(cy*v.x + sy*v.y) + sp*v.z)};
    };
    const auto project = [&](Vec v) {
        const auto w = view(v);
        const float depth = 1 / (64 - w.z);
        const double factor = scale * 64 * depth;
        return Point{{static_cast<LONG>(cx + factor*w.x), static_cast<LONG>(center_y - factor*w.y)}, depth};
    };
    const auto rotate = [&](Vec v) {
        return Vec{r[0]*v.x + r[1]*v.y + r[2]*v.z,
                   r[3]*v.x + r[4]*v.y + r[5]*v.z, r[6]*v.x + r[7]*v.y + r[8]*v.z};
    };

    p.fill(stage, gray(250));
    const int clip = SaveDC(p.dc);
    IntersectClipRect(p.dc, stage.left, stage.top, stage.right, stage.bottom);
    for (int i = -20; i <= 20; i += 5) {
        const float t = static_cast<float>(i);
        const auto x1 = project({t, -20, -6}).xy, x2 = project({t, 20, -6}).xy;
        const auto y1 = project({-20, t, -6}).xy, y2 = project({20, t, -6}).xy;
        p.line(x1.x, x1.y, x2.x, x2.y, gray(229));
        p.line(y1.x, y1.y, y2.x, y2.y, gray(229));
    }
    for (std::size_t i = 0; i < vertices_.size(); ++i)
        projected_[i] = project(rotate(vertices_[i]));
    const int width = stage.right - stage.left, height = stage.bottom - stage.top;
    const auto surface = mesh_.get(p.dc, width, height);
    if (surface) {
        constexpr COLORREF transparent = RGB(255, 0, 255);
        const auto count = static_cast<std::size_t>(width) * height;
        depth_.resize(count);
        std::fill(depth_.begin(), depth_.end(), 0);
        std::fill_n(mesh_.pixels(), count, transparent);
        for (const auto& f : faces_)
            triangle(projected_[f.a], projected_[f.b], projected_[f.c], gray(f.shade), stage);
        TransparentBlt(p.dc, stage.left, stage.top, width, height, surface, 0, 0, width, height, transparent);
    }
    const auto origin = project({0, 0, 0}).xy;
    const POINT reference{stage.left + p.px(39), stage.bottom - p.px(37)};
    constexpr std::array axes{Vec{16, 0, 0}, Vec{0, 16, 0}, Vec{0, 0, 16}};
    for (unsigned i = 0; i < axes.size(); ++i) {
        const auto world = view(axes[i]);
        const auto sensor = project(rotate(axes[i])).xy;
        const int x = reference.x + static_cast<int>(world.x * p.px(24) / 16);
        const int y = reference.y - static_cast<int>(world.y * p.px(24) / 16);
        const auto color = colors[i];
        p.line(origin.x, origin.y, sensor.x, sensor.y, color);
        p.line(reference.x, reference.y, x, y, color);
        const wchar_t label[]{L"XYZ"[i], 0};
        p.text({sensor.x + p.px(3), sensor.y - p.px(8), sensor.x + p.px(19), sensor.y + p.px(8)}, label, p.fonts.caption, ink);
        p.text({x + p.px(3), y - p.px(8), x + p.px(19), y + p.px(8)}, label, p.fonts.caption, muted);
    }
    RestoreDC(p.dc, clip);
}
