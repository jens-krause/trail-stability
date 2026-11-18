#include "warp.h"
#include "threadpool.h"
#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

namespace ts {

// ------------------------------------------------------------------ picture placement

RectI findContentRect(const ImageView& img, float eps)
{
    // Two passes with a coarse step across the scan direction keep this at ~1/2 of a full
    // scan while the edges are still found to the exact pixel.
    const int W = img.x2 - img.x1, H = img.y2 - img.y1;
    if (W <= 0 || H <= 0) return {};
    const int step = 4;
    // Background colour: the value at least two frame corners share exactly (empty timeline area
    // is uniform, real picture corners practically never are); black if no two corners agree.
    float bg[3] = {0.f, 0.f, 0.f};
    {
        const float* c[4] = {img.px(img.x1, img.y1), img.px(img.x2 - 1, img.y1),
                             img.px(img.x1, img.y2 - 1), img.px(img.x2 - 1, img.y2 - 1)};
        auto same = [](const float* a, const float* b) {
            return std::fabs(a[0] - b[0]) < 1e-6f && std::fabs(a[1] - b[1]) < 1e-6f && std::fabs(a[2] - b[2]) < 1e-6f;
        };
        bool found = false;
        for (int i = 0; i < 4 && !found; ++i)
            for (int j = i + 1; j < 4 && !found; ++j)
                if (same(c[i], c[j]) && std::max({c[i][0], c[i][1], c[i][2]}) < 0.05f) {   // near black only
                    bg[0] = c[i][0]; bg[1] = c[i][1]; bg[2] = c[i][2]; found = true;
                }
    }
    auto hit = [&](int x, int y) {
        const float* p = img.px(x, y);
        return std::fabs(p[0] - bg[0]) > eps || std::fabs(p[1] - bg[1]) > eps || std::fabs(p[2] - bg[2]) > eps;
    };
    std::vector<char> col(size_t(W), 0), row(size_t(H), 0);
    for (int y = img.y1; y < img.y2; y += step)
        for (int x = img.x1; x < img.x2; ++x)
            if (!col[size_t(x - img.x1)] && hit(x, y)) col[size_t(x - img.x1)] = 1;
    for (int y = img.y1; y < img.y2; ++y)
        for (int x = img.x1; x < img.x2; x += step)
            if (hit(x, y)) { row[size_t(y - img.y1)] = 1; break; }
    RectI r;
    int a = 0, b = W - 1;
    while (a < W && !col[size_t(a)]) ++a;
    while (b >= 0 && !col[size_t(b)]) --b;
    if (a > b) return {};
    r.x1 = img.x1 + a; r.x2 = img.x1 + b + 1;
    a = 0; b = H - 1;
    while (a < H && !row[size_t(a)]) ++a;
    while (b >= 0 && !row[size_t(b)]) --b;
    if (a > b) return {};
    r.y1 = img.y1 + a; r.y2 = img.y1 + b + 1;
    return r;
}

Placement placePicture(const RectI& frame, const RectI& content, double aspectHW, double tol)
{
    Placement p;
    // Normal use: timeline has the clip's aspect ratio. Then a content box touching three or four
    // frame edges means "picture fills the frame" – black/clipped rows inside the picture
    // (night sky, shadows) must never shrink or shift it.
    if (!content.empty() && aspectHW > 0 && !frame.empty()) {
        const int touches = (content.x1 <= frame.x1) + (content.x2 >= frame.x2) + (content.y1 <= frame.y1) + (content.y2 >= frame.y2);
        const double fa = double(frame.h()) / double(frame.w());
        if (touches >= 3 && std::fabs(fa / aspectHW - 1.0) <= tol) {
            p.rect = frame;
            p.cx = 0.5 * (frame.x1 + frame.x2 - 1);
            p.cy = 0.5 * (frame.y1 + frame.y2 - 1);
            p.width = 0.5 * (frame.w() + frame.h() / aspectHW);
            p.detected = true;
            return p;
        }
    }
    if (!content.empty() && aspectHW > 0) {
        const double a = double(content.h()) / double(content.w());
        if (std::fabs(a / aspectHW - 1.0) <= tol) {
            p.rect = content;
            p.cx = 0.5 * (content.x1 + content.x2 - 1);
            p.cy = 0.5 * (content.y1 + content.y2 - 1);
            p.width = 0.5 * (content.w() + content.h() / aspectHW);   // average of both edges
            p.detected = true;
            return p;
        }
        // Picture partly outside the frame (Inspector pan/zoom): one dimension is cut by the
        // frame edge, the other one is complete -> reconstruct the full picture from it.
        const bool L = content.x1 <= frame.x1, R = content.x2 >= frame.x2;
        const bool B = content.y1 <= frame.y1, T = content.y2 >= frame.y2;
        const double ccx = 0.5 * (content.x1 + content.x2 - 1), ccy = 0.5 * (content.y1 + content.y2 - 1);
        // A picture fitted to the timeline height touches top and bottom although it is complete
        // vertically; if it is cut on exactly one side horizontally, its height is the real one.
        const bool hComplete = (!B && !T) || (B && T && L != R);
        const bool wComplete = (!L && !R) || (L && R && B != T);
        if (hComplete && (L || R)) {                // full height visible, cut left and/or right
            p.width = content.h() / aspectHW;
            p.cy = ccy;
            p.cx = (L && !R) ? content.x2 - 0.5 - 0.5 * p.width : (R && !L) ? content.x1 - 0.5 + 0.5 * p.width : ccx;
            p.rect = content; p.detected = true;
            return p;
        }
        if (wComplete && (B || T)) {                // full width visible, cut at top and/or bottom
            p.width = content.w();
            const double ph = p.width * aspectHW;
            p.cx = ccx;
            p.cy = (B && !T) ? content.y2 - 0.5 - 0.5 * ph : (T && !B) ? content.y1 - 0.5 + 0.5 * ph : ccy;
            p.rect = content; p.detected = true;
            return p;
        }
    }
    // fallback: picture fitted into the frame, centred (old v0 behaviour)
    const double fw = frame.w(), fh = frame.h();
    p.width = (fh / fw < aspectHW) ? fh / aspectHW : fw;
    const double ph = p.width * aspectHW;
    p.cx = 0.5 * (frame.x1 + frame.x2 - 1);
    p.cy = 0.5 * (frame.y1 + frame.y2 - 1);
    p.rect.x1 = std::max(frame.x1, int(std::floor(p.cx + 0.5 - 0.5 * p.width)));
    p.rect.x2 = std::min(frame.x2, int(std::ceil(p.cx + 0.5 + 0.5 * p.width)));
    p.rect.y1 = std::max(frame.y1, int(std::floor(p.cy + 0.5 - 0.5 * ph)));
    p.rect.y2 = std::min(frame.y2, int(std::ceil(p.cy + 0.5 + 0.5 * ph)));
    p.detected = false;
    return p;
}

// ------------------------------------------------------------------ geometry

void cameraHomography(const Mat3& R, double f, double zoom, double cx, double cy, double H[9])
{
    // OFX (y up) -> camera coordinates centred on the principal point, y down
    Mat3 A;    A.m    = {1, 0, -cx, 0, -1, cy, 0, 0, 1};
    Mat3 Ainv; Ainv.m = {1, 0, cx, 0, -1, cy, 0, 0, 1};
    const double fo = f * zoom;
    Mat3 Ki;   Ki.m   = {f, 0, 0, 0, f, 0, 0, 0, 1};
    Mat3 KoI;  KoI.m  = {1 / fo, 0, 0, 0, 1 / fo, 0, 0, 0, 1};
    Mat3 M = Ainv * Ki * R * KoI * A;
    for (int i = 0; i < 9; ++i) H[i] = M.m[size_t(i)];
}

void frameHomography(const Mat3& R, double focalNorm, double zoom, int W, int Hh, double activeW, double H[9])
{
    cameraHomography(R, focalNorm * activeW, zoom, 0.5 * (W - 1), 0.5 * (Hh - 1), H);
}

// ------------------------------------------------------------------ warp

namespace {

inline void cubicWeights(double t, double w[4])
{   // Catmull-Rom
    const double t2 = t * t, t3 = t2 * t;
    w[0] = -0.5 * t3 + t2 - 0.5 * t;
    w[1] = 1.5 * t3 - 2.5 * t2 + 1.0;
    w[2] = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
    w[3] = 0.5 * t3 - 0.5 * t2;
}

void warpRows(const ImageView& src, const ImageView& dst, int wx1, int wx2, int ya, int yb,
              const double* H, RectI v)   // v = sample/output area, already clipped to src
{
    for (int y = ya; y < yb; ++y) {
        float* out = dst.px(wx1, y);
        const bool rowIn = y >= v.y1 && y < v.y2;
        for (int x = wx1; x < wx2; ++x, out += 4) {
            if (!rowIn || x < v.x1 || x >= v.x2) { out[0] = out[1] = out[2] = out[3] = 0.f; continue; }
            const double zx = H[0] * x + H[1] * y + H[2];
            const double zy = H[3] * x + H[4] * y + H[5];
            const double zw = H[6] * x + H[7] * y + H[8];
            if (zw <= 1e-9) { out[0] = out[1] = out[2] = out[3] = 0.f; continue; }
            const double sx = zx / zw, sy = zy / zw;
            if (sx < v.x1 - 0.5 || sy < v.y1 - 0.5 || sx > v.x2 - 0.5 || sy > v.y2 - 0.5) {
                out[0] = out[1] = out[2] = out[3] = 0.f; continue;
            }
            const int ix = int(std::floor(sx)), iy = int(std::floor(sy));
            double wx[4], wy[4];
            cubicWeights(sx - ix, wx); cubicWeights(sy - iy, wy);
            double acc[4] = {0, 0, 0, 0};
            for (int j = 0; j < 4; ++j) {
                const int yy = std::clamp(iy - 1 + j, v.y1, v.y2 - 1);
                for (int i = 0; i < 4; ++i) {
                    const int xx = std::clamp(ix - 1 + i, v.x1, v.x2 - 1);
                    const float* p = src.px(xx, yy);
                    const double ww = wx[i] * wy[j];
                    acc[0] += ww * p[0]; acc[1] += ww * p[1]; acc[2] += ww * p[2]; acc[3] += ww * p[3];
                }
            }
            out[0] = float(acc[0]); out[1] = float(acc[1]); out[2] = float(acc[2]); out[3] = float(acc[3]);
        }
    }
}

} // namespace

void warpBicubic(const ImageView& src, const ImageView& dst, int wx1, int wy1, int wx2, int wy2,
                 const double H[9], int threads, const RectI* valid)
{
    RectI v{src.x1, src.y1, src.x2, src.y2};
    if (valid) {
        v.x1 = std::max(v.x1, valid->x1); v.y1 = std::max(v.y1, valid->y1);
        v.x2 = std::min(v.x2, valid->x2); v.y2 = std::min(v.y2, valid->y2);
    }
    (void)threads;   // the shared pool decides (performance cores)
    parallelFor(wy2 - wy1, [&](int a, int b) { warpRows(src, dst, wx1, wx2, wy1 + a, wy1 + b, H, v); }, 8);
}

} // namespace ts
