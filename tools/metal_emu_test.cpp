// Checks the Metal kernels without a Mac: the MSL text from ofx/metal_kernels.h is compiled as C++
// (tools/msl_shim.h maps the few MSL constructs it uses) and run thread by thread, then compared
// with the CPU warp and picture search. Build: see tools/run_metal_emu_test.sh
#include "metal_render.h"
#include "warp.h"
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "msl_shim.h"
namespace emu {
#include "msl_as_cpp.inc"   // generated from kMetalSource
}

static_assert(sizeof(emu::WarpParams) == sizeof(tsm::WarpParams), "WarpParams layout differs");
static_assert(sizeof(emu::DetectParams) == sizeof(tsm::DetectParams), "DetectParams layout differs");

struct Img {
    int x1, y1, x2, y2, rowPx;
    std::vector<float> d;
    Img(int ax1, int ay1, int ax2, int ay2, int pad) : x1(ax1), y1(ay1), x2(ax2), y2(ay2), rowPx(ax2 - ax1 + pad),
        d(size_t(rowPx) * size_t(ay2 - ay1) * 4, 0.f) {}
    ts::ImageView view() { ts::ImageView v; v.base = reinterpret_cast<char*>(d.data()); v.rowBytes = rowPx * 16; v.x1 = x1; v.y1 = y1; v.x2 = x2; v.y2 = y2; return v; }
    tsm::MetalImage metal() { tsm::MetalImage m; m.buffer = d.data(); m.x1 = x1; m.y1 = y1; m.x2 = x2; m.y2 = y2; m.rowBytes = rowPx * 16; return m; }
    float4* f4() { return reinterpret_cast<float4*>(d.data()); }
};

static ts::RectI emuDetect(Img& im)
{
    emu::DetectParams dp{};
    const int W = im.x2 - im.x1, H = im.y2 - im.y1;
    dp.w = W; dp.h = H; dp.row = im.rowPx; dp.eps = 1e-4f;
    std::vector<unsigned> flags(size_t(W + H), 0);
    for (unsigned gy = 0; gy < unsigned((H + 3) / 4); ++gy)
        for (unsigned gx = 0; gx < unsigned(W); ++gx) emu::ts_detect_cols(dp, im.f4(), flags.data(), uint2{gx, gy});
    for (unsigned gy = 0; gy < unsigned(H); ++gy)
        for (unsigned gx = 0; gx < unsigned((W + 3) / 4); ++gx) emu::ts_detect_rows(dp, im.f4(), flags.data(), uint2{gx, gy});
    // same reduction as metal_render.mm
    int a = 0, b = W - 1;
    while (a < W && !flags[size_t(a)]) ++a;
    while (b >= 0 && !flags[size_t(b)]) --b;
    if (a > b) return {};
    ts::RectI r; r.x1 = im.x1 + a; r.x2 = im.x1 + b + 1;
    a = 0; b = H - 1;
    while (a < H && !flags[size_t(W + a)]) ++a;
    while (b >= 0 && !flags[size_t(W + b)]) --b;
    if (a > b) return {};
    r.y1 = im.y1 + a; r.y2 = im.y1 + b + 1;
    return r;
}

static void emuWarp(Img& src, Img& dst, const int win[4], const double H[9], const ts::RectI& valid, bool copy)
{
    const tsm::WarpParams tp = tsm::makeWarpParams(src.metal(), dst.metal(), win, H, valid, copy);
    emu::WarpParams p;
    static_assert(sizeof(p) == sizeof(tp), "");
    std::memcpy(&p, &tp, sizeof(p));
    for (unsigned gy = 0; gy < unsigned(p.wy2 - p.wy1); ++gy)
        for (unsigned gx = 0; gx < unsigned(p.wx2 - p.wx1); ++gx) emu::ts_warp(p, src.f4(), dst.f4(), uint2{gx, gy});
}

int main()
{
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> U(0.f, 1.f);
    int fails = 0;
    for (int trial = 0; trial < 7; ++trial) {
        // frame with an offset origin and padded rows; picture somewhere inside on a near-black background
        const int fx1 = (trial % 2) ? 3 : 0, fy1 = (trial % 3) ? 2 : 0;
        // trial 6: real size (right third of the 3240x1920 comparison timeline) for float precision
        Img src = trial == 6 ? Img(0, 0, 3240, 1920, 0) : Img(fx1, fy1, fx1 + 160 + trial * 7, fy1 + 120 + trial * 5, trial % 2 ? 5 : 0);
        const float bg = (trial == 3) ? 0.01f : 0.f;
        for (size_t i = 0; i < src.d.size(); i += 4) { src.d[i] = src.d[i + 1] = src.d[i + 2] = bg; src.d[i + 3] = 1.f; }
        int px1 = src.x1 + 10 + trial, px2 = src.x2 - 6 - trial, py1 = src.y1 + 4 + 2 * trial, py2 = src.y2 - 9;
        if (trial == 6) { px1 = 2160; px2 = 3240; py1 = 0; py2 = 1920; }
        for (int y = py1; y < py2; ++y)
            for (int x = px1; x < px2; ++x) {
                float* p = src.view().px(x, y);
                p[0] = 0.5f + 0.4f * std::sin(0.31f * x) * std::cos(0.17f * y) ; p[1] = U(rng); p[2] = 0.3f * U(rng) + 0.2f; p[3] = 1.f;
            }
        // picture search
        const ts::RectI rc = ts::findContentRect(src.view());
        const ts::RectI rg = emuDetect(src);
        const bool okDet = rc.x1 == rg.x1 && rc.y1 == rg.y1 && rc.x2 == rg.x2 && rc.y2 == rg.y2;
        if (!okDet) ++fails;
        // warp with a small camera rotation, output into a buffer with another origin/pitch
        const double a = 0.01 * (trial + 1);
        ts::Mat3 R; R.m = {std::cos(a), -std::sin(a), 0.002 * trial, std::sin(a), std::cos(a), -0.003, -0.001 * trial, 0.002, 1};
        double Hm[9];
        ts::cameraHomography(R, trial == 6 ? 1100.0 : 140.0, 1.25, 0.5 * (px1 + px2 - 1), 0.5 * (py1 + py2 - 1), Hm);
        Img dc(src.x1, src.y1, src.x2, src.y2, 3), dg(src.x1, src.y1, src.x2, src.y2, 3);
        const int win[4] = {src.x1, src.y1, src.x2, src.y2};
        ts::warpBicubic(src.view(), dc.view(), win[0], win[1], win[2], win[3], Hm, 1, &rc);
        emuWarp(src, dg, win, Hm, rc, false);
        double maxd = 0; int nz = 0;
        for (int y = src.y1; y < src.y2; ++y)
            for (int x = src.x1; x < src.x2; ++x)
                for (int c = 0; c < 4; ++c) {
                    const double d = std::fabs(double(dc.view().px(x, y)[c]) - double(dg.view().px(x, y)[c]));
                    maxd = std::max(maxd, d);
                    if (dc.view().px(x, y)[c] != 0.f) ++nz;
                }
        // pass-through copy
        Img cg(src.x1, src.y1, src.x2, src.y2, 1);
        emuWarp(src, cg, win, nullptr, ts::RectI{src.x1, src.y1, src.x2, src.y2}, true);
        double maxc = 0;
        for (int y = src.y1; y < src.y2; ++y)
            for (int x = src.x1; x < src.x2; ++x)
                for (int c = 0; c < 4; ++c) maxc = std::max(maxc, double(std::fabs(src.view().px(x, y)[c] - cg.view().px(x, y)[c])));
        const bool okW = maxd < 2e-4 && nz > 1000, okC = maxc == 0;
        if (!okW || !okC) ++fails;
        std::printf("trial %d: search cpu [%d,%d,%d,%d] gpu [%d,%d,%d,%d] %s | warp max diff %.2e (%d values) %s | copy %s\n",
                    trial, rc.x1, rc.y1, rc.x2, rc.y2, rg.x1, rg.y1, rg.x2, rg.y2, okDet ? "OK" : "FAIL", maxd, nz, okW ? "OK" : "FAIL",
                    okC ? "OK" : "FAIL");
    }
    std::printf(fails ? "METAL EMULATION FAILED\n" : "METAL EMULATION OK\n");
    return fails ? 1 : 0;
}
