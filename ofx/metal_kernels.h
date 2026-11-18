// Metal compute kernels (Metal Shading Language), compiled at runtime by metal_render.mm.
//
// The kernel bodies use only a small subset of MSL (float4 load/store/add/scale, scalar maths)
// so that tools/metal_emu_test.cpp can compile the very same text as C++ and check it against
// the CPU warp and picture search. Keep it that way when editing.
#pragma once

namespace tsm {

// Shared parameter blocks: only 4-byte scalars and arrays, so C++ and MSL layouts are identical.
struct WarpParams {
    float h[9];                  // homography: output pixel (x, y, 1) -> input pixel (OFX, y up)
    int sx1, sy1, sw, sh, srow;  // source: origin, size, row pitch in pixels
    int dx1, dy1, drow;          // destination: origin, row pitch in pixels
    int wx1, wy1, wx2, wy2;      // render window (destination coordinates)
    int vx1, vy1, vx2, vy2;      // valid picture area (source coordinates)
    int mode;                    // 0 = warp, 1 = copy (pass through)
    int ox, oy;                  // origin of the coordinates h works in (centre of the valid area),
                                 // keeps the float maths precise on large frames
};

struct DetectParams {
    int w, h, row;               // image size, row pitch in pixels
    float eps;
};

static const char* const kMetalSource = R"MSL(
#ifdef __METAL_VERSION__
#include <metal_stdlib>
using namespace metal;
#endif

struct WarpParams {
    float h[9];
    int sx1, sy1, sw, sh, srow;
    int dx1, dy1, drow;
    int wx1, wy1, wx2, wy2;
    int vx1, vy1, vx2, vy2;
    int mode;
    int ox, oy;
};

struct DetectParams {
    int w, h, row;
    float eps;
};

// Catmull-Rom weights (same as the CPU warp)
inline void ts_cubic(float t, thread float* w)
{
    const float t2 = t * t, t3 = t2 * t;
    w[0] = -0.5f * t3 + t2 - 0.5f * t;
    w[1] = 1.5f * t3 - 2.5f * t2 + 1.0f;
    w[2] = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
    w[3] = 0.5f * t3 - 0.5f * t2;
}

inline int ts_clampi(int v, int a, int b) { return v < a ? a : (v > b ? b : v); }

kernel void ts_warp(constant WarpParams& P [[buffer(0)]],
                    device const float4* src [[buffer(1)]],
                    device float4* dst [[buffer(2)]],
                    uint2 gid [[thread_position_in_grid]])
{
    const int x = P.wx1 + int(gid.x), y = P.wy1 + int(gid.y);
    if (x >= P.wx2 || y >= P.wy2) return;
    const int o = (y - P.dy1) * P.drow + (x - P.dx1);
    const float4 zero = float4(0.0f);
    if (P.mode == 1) {
        if (x < P.sx1 || x >= P.sx1 + P.sw || y < P.sy1 || y >= P.sy1 + P.sh) { dst[o] = zero; return; }
        dst[o] = src[(y - P.sy1) * P.srow + (x - P.sx1)];
        return;
    }
    if (y < P.vy1 || y >= P.vy2 || x < P.vx1 || x >= P.vx2) { dst[o] = zero; return; }
    // relative to (ox, oy): small numbers, so float keeps ~1e-4 px precision even at 4K
    const float fx = float(x - P.ox), fy = float(y - P.oy);
    const float zx = P.h[0] * fx + P.h[1] * fy + P.h[2];
    const float zy = P.h[3] * fx + P.h[4] * fy + P.h[5];
    const float zw = P.h[6] * fx + P.h[7] * fy + P.h[8];
    if (zw <= 1e-9f) { dst[o] = zero; return; }
    const float sx = zx / zw, sy = zy / zw;   // relative source position
    if (sx < float(P.vx1 - P.ox) - 0.5f || sy < float(P.vy1 - P.oy) - 0.5f ||
        sx > float(P.vx2 - P.ox) - 0.5f || sy > float(P.vy2 - P.oy) - 0.5f) {
        dst[o] = zero; return;
    }
    const float flx = floor(sx), fly = floor(sy);
    const int ix = int(flx) + P.ox, iy = int(fly) + P.oy;
    float wx[4], wy[4];
    ts_cubic(sx - flx, wx);
    ts_cubic(sy - fly, wy);
    float4 acc = zero;
    for (int j = 0; j < 4; ++j) {
        const int yy = ts_clampi(iy - 1 + j, P.vy1, P.vy2 - 1);
        const int rowo = (yy - P.sy1) * P.srow - P.sx1;
        float4 racc = zero;
        for (int i = 0; i < 4; ++i) {
            const int xx = ts_clampi(ix - 1 + i, P.vx1, P.vx2 - 1);
            racc += src[rowo + xx] * wx[i];
        }
        acc += racc * wy[j];
    }
    dst[o] = acc;
}

// Background colour: the value at least two frame corners share exactly, if it is near black;
// black otherwise (same rule as findContentRect on the CPU).
inline bool ts_same(float4 a, float4 b)
{
    return fabs(a.x - b.x) < 1e-6f && fabs(a.y - b.y) < 1e-6f && fabs(a.z - b.z) < 1e-6f;
}

inline float4 ts_background(constant DetectParams& P, device const float4* img)
{
    float4 c[4];
    c[0] = img[0];
    c[1] = img[P.w - 1];
    c[2] = img[(P.h - 1) * P.row];
    c[3] = img[(P.h - 1) * P.row + P.w - 1];
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            if (ts_same(c[i], c[j]) && max(max(c[i].x, c[i].y), c[i].z) < 0.05f) return c[i];
    return float4(0.0f);
}

inline bool ts_hit(float4 p, float4 bg, float eps)
{
    return fabs(p.x - bg.x) > eps || fabs(p.y - bg.y) > eps || fabs(p.z - bg.z) > eps;
}

// columns with content, sampled on every 4th row -> flags[0 .. w)
kernel void ts_detect_cols(constant DetectParams& P [[buffer(0)]],
                           device const float4* img [[buffer(1)]],
                           device uint* flags [[buffer(2)]],
                           uint2 gid [[thread_position_in_grid]])
{
    const int x = int(gid.x), y = int(gid.y) * 4;
    if (x >= P.w || y >= P.h) return;
    if (ts_hit(img[y * P.row + x], ts_background(P, img), P.eps)) flags[x] = 1u;
}

// rows with content, sampled on every 4th column -> flags[w .. w + h)
kernel void ts_detect_rows(constant DetectParams& P [[buffer(0)]],
                           device const float4* img [[buffer(1)]],
                           device uint* flags [[buffer(2)]],
                           uint2 gid [[thread_position_in_grid]])
{
    const int x = int(gid.x) * 4, y = int(gid.y);
    if (x >= P.w || y >= P.h) return;
    if (ts_hit(img[y * P.row + x], ts_background(P, img), P.eps)) flags[P.w + y] = 1u;
}
)MSL";

} // namespace tsm
