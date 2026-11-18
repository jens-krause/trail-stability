// Metal render path (macOS). Pure C++ interface; the implementation is metal_render.mm
// (Objective-C++), on other platforms metal_render_stub.cpp reports "not available".
//
// Resolve hands every image as an id<MTLBuffer> (RGBA float, rows of rowBytes, row 0 = bottom
// of the bounds, as on the CPU) together with its own command queue. All work goes onto that
// queue; only the picture search waits for its (tiny) result, the warp is committed and the
// render action returns without waiting, as the OFX spec asks.
#pragma once
#include "metal_kernels.h"
#include "warp.h"
#include <functional>
#include <string>

namespace tsm {

struct MetalImage {
    void* buffer = nullptr;      // id<MTLBuffer>
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    long rowBytes = 0;
};

/// Bounding box of visible content (same result as ts::findContentRect on the CPU). Waits for the GPU.
bool detectContent(void* commandQueue, const MetalImage& img, ts::RectI& out, std::string* err);

/// Warp (or copy, if passThrough) src into dst over the render window. Does not wait.
/// H maps output pixels to input pixels (double, converted to float for the GPU);
/// valid = picture area in source coordinates.
bool warp(void* commandQueue, const MetalImage& src, const MetalImage& dst, const int window[4],
          const double H[9], const ts::RectI& valid, bool passThrough, std::string* err);

/// Warp like warp() and, in the same command buffer, run the picture search on src; its result is
/// handed to onContent when the GPU has finished (other thread, later). Nothing waits.
bool warpAndSearch(void* commandQueue, const MetalImage& src, const MetalImage& dst, const int window[4],
                   const double H[9], const ts::RectI& valid, bool passThrough,
                   std::function<void(const ts::RectI&)> onContent, std::string* err);

/// errors reported asynchronously by finished command buffers since the last call ("" if none)
std::string takeAsyncError();

/// true when this build contains the Metal path
bool available();

/// parameter block for ts_warp (shared with the emulation test)
inline WarpParams makeWarpParams(const MetalImage& src, const MetalImage& dst, const int window[4], const double H[9],
                                 const ts::RectI& valid, bool passThrough)
{
    WarpParams p{};
    p.sx1 = src.x1; p.sy1 = src.y1; p.sw = src.x2 - src.x1; p.sh = src.y2 - src.y1;
    p.srow = int(src.rowBytes > 0 ? src.rowBytes / 16 : p.sw);
    p.dx1 = dst.x1; p.dy1 = dst.y1;
    p.drow = int(dst.rowBytes > 0 ? dst.rowBytes / 16 : dst.x2 - dst.x1);
    // never write outside the destination buffer
    p.wx1 = window[0] > dst.x1 ? window[0] : dst.x1; p.wy1 = window[1] > dst.y1 ? window[1] : dst.y1;
    p.wx2 = window[2] < dst.x2 ? window[2] : dst.x2; p.wy2 = window[3] < dst.y2 ? window[3] : dst.y2;
    // valid area = picture area clipped to the source bounds (as warpBicubic on the CPU)
    p.vx1 = valid.x1 > src.x1 ? valid.x1 : src.x1; p.vy1 = valid.y1 > src.y1 ? valid.y1 : src.y1;
    p.vx2 = valid.x2 < src.x2 ? valid.x2 : src.x2; p.vy2 = valid.y2 < src.y2 ? valid.y2 : src.y2;
    p.mode = passThrough ? 1 : 0;
    // work relative to the centre of the valid area: H' = T^-1 * H * T with T = translate(ox, oy)
    p.ox = (p.vx1 + p.vx2) / 2; p.oy = (p.vy1 + p.vy2) / 2;
    double M[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    if (H) for (int i = 0; i < 9; ++i) M[i] = H[i];
    const double ox = p.ox, oy = p.oy;
    for (int r = 0; r < 3; ++r) M[r * 3 + 2] += M[r * 3 + 0] * ox + M[r * 3 + 1] * oy;   // H * T
    for (int c = 0; c < 3; ++c) { M[c] -= ox * M[6 + c]; M[3 + c] -= oy * M[6 + c]; }    // T^-1 * (H * T)
    for (int i = 0; i < 9; ++i) p.h[i] = float(M[i]);
    return p;
}

} // namespace tsm
