#pragma once

#include "model.h"
#include "win.h"

#include <algorithm>

inline constexpr COLORREF gray(int value) { return RGB(value, value, value); }
inline constexpr auto paper = gray(255), background = gray(247), ink = gray(22), muted = gray(94), rule = gray(223);

struct Fonts {
    int dpi{};
    HFONT body{}, caption{}, strong{}, title{}, number{}, mono{};
    ~Fonts();
    void set(int value);
    int px(int value) const { return MulDiv(value, dpi, 96); }
};

struct Paint {
    HDC dc;
    const Fonts& fonts;
    int px(int n) const { return fonts.px(n); }
    void fill(RECT r, COLORREF color) const;
    void border(RECT r, COLORREF color = rule) const;
    void line(int x1, int y1, int x2, int y2, COLORREF color = rule) const;
    void text(RECT r, std::wstring_view value, HFONT font = nullptr, COLORREF color = ink,
              UINT flags = DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS) const;
};

class Buffer {
public:
    ~Buffer();
    HDC get(HDC source, int width, int height);
    std::uint32_t* pixels() const { return pixels_; }

private:
    HDC dc_{};
    HBITMAP bitmap_{};
    HGDIOBJ original_{};
    std::uint32_t* pixels_{};
    int width_{}, height_{};
};

RECT inset(RECT r, int x, int y);
std::wstring fixed(double number, int places = 2);
std::wstring significant(double number, int digits = 6);

struct PlotBin {
    int x;
    float low, high;
    bool gap;
};

class History {
public:
    void append(std::span<const Sample> samples);
    void clear() { total_ = 0; }
    bool empty() const { return total_ == 0; }
    const Sample& latest() const { return at(total_ - 1); }
    const Sample& nearest(double seconds) const;
    void bins(int channel, double from, double to, int width, std::vector<PlotBin>& result) const;

private:
    struct Range { float low{}, high{}; };
    static constexpr std::size_t block_size = 16;
    struct Block {
        std::array<Range, columns.size()> values;
        std::uint32_t absent{};
        bool gap{};
    };
    std::uint64_t first() const { return total_ - std::min<std::uint64_t>(total_, Model::capacity); }
    std::uint64_t search(double time) const;
    const Sample& at(std::uint64_t i) const { return samples_[i % Model::capacity]; }
    std::uint64_t total_{};
    std::vector<Sample> samples_ = std::vector<Sample>(Model::capacity);
    std::vector<Block> blocks_ = std::vector<Block>(Model::capacity / block_size);
};

class Charts {
public:
    void begin() { panels_.fill({}); legends_.fill({}); hover_ = -1; }
    void draw(const Paint& paint, RECT card, int group, const History& history);
    void follow();
    int panel_at(POINT point) const;
    bool toggle(POINT point);
    void pan(const History& history, int group, int pixels);
    void zoom(const History& history, int group, POINT point, double factor, bool vertical);
    std::wstring cursor(const History& history) const;

    double seconds = 10;
    std::optional<double> end;
    POINT mouse{-1, -1};
    bool frozen{};

private:
    std::uint32_t channels_ = (1 << columns.size()) - 1;
    std::array<double, 4> y_scale_{1, 1, 1, 1};
    std::array<RECT, 4> panels_{};
    std::array<RECT, columns.size()> legends_{};
    std::array<std::vector<PlotBin>, columns.size()> bins_;
    std::vector<POINT> points_;
    std::vector<DWORD> runs_;
    int hover_ = -1;
};

class Board {
public:
    explicit Board(HINSTANCE module);
    void draw(const Paint& paint, RECT card, const Snapshot& snapshot, bool stale);
    void reset();
    void clear_pose() { orientation_ = {}; }
    void rotate(int x, int y);
    void zoom(double scale);
    bool ready() const { return !faces_.empty(); }

private:
    struct Vec { float x, y, z; };
    struct Face { std::uint16_t a, b, c; Byte shade, reserved; };
    struct Point { POINT xy; float depth; };
    std::vector<Vec> vertices_;
    std::vector<Face> faces_;
    std::vector<Point> projected_;
    Buffer mesh_;
    std::vector<float> depth_;
    std::array<float, 9> orientation_{};
    double yaw_{}, pitch_{}, scale_{};
    void triangle(Point a, Point b, Point c, COLORREF shade, RECT stage);
};

std::wstring attitude_state(const Snapshot& view, bool stale);
