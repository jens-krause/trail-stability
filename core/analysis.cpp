// C++ port of the original Python analysis prototype (not part of the repository).  See analysis.h.
#include "analysis.h"
#include "threadpool.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <random>
#include <thread>
#include <cstdio>
#include <cstdlib>

namespace ts {

namespace {

constexpr double kPi = 3.14159265358979323846;

inline int reflect101(int i, int n)
{
    if (n == 1) return 0;
    while (i < 0 || i >= n) i = i < 0 ? -i : 2 * n - 2 - i;
    return i;
}

// ------------------------------------------------------------------ pyramid

GrayImage pyrDown(const GrayImage& s)
{
    // OpenCV pyrDown: 5-tap binomial [1 4 6 4 1]/16, reflect-101 border, keep even samples
    GrayImage tmp; tmp.w = (s.w + 1) / 2; tmp.h = s.h; tmp.p.resize(size_t(tmp.w) * size_t(tmp.h));
    parallelFor(s.h, [&](int ya, int yb) {
    for (int y = ya; y < yb; ++y)
        for (int x = 0; x < tmp.w; ++x) {
            const int c = 2 * x;
            tmp.at(x, y) = (s.at(reflect101(c - 2, s.w), y) + 4 * s.at(reflect101(c - 1, s.w), y) + 6 * s.at(c, y) +
                            4 * s.at(reflect101(c + 1, s.w), y) + s.at(reflect101(c + 2, s.w), y)) * (1.f / 16.f);
        }
    }, 32);
    GrayImage d; d.w = tmp.w; d.h = (s.h + 1) / 2; d.p.resize(size_t(d.w) * size_t(d.h));
    parallelFor(d.h, [&](int ya, int yb) {
    for (int y = ya; y < yb; ++y) {
        const int c = 2 * y;
        const int r0 = reflect101(c - 2, s.h), r1 = reflect101(c - 1, s.h), r3 = reflect101(c + 1, s.h), r4 = reflect101(c + 2, s.h);
        for (int x = 0; x < d.w; ++x)
            d.at(x, y) = (tmp.at(x, r0) + 4 * tmp.at(x, r1) + 6 * tmp.at(x, c) + 4 * tmp.at(x, r3) + tmp.at(x, r4)) * (1.f / 16.f);
    }
    }, 16);
    return d;
}

struct Level {
    GrayImage I, Ix, Iy;   // Scharr derivatives in grey levels per pixel
};

struct Pyramid {
    std::vector<Level> L;
};

void scharr(const GrayImage& I, GrayImage& Ix, GrayImage& Iy)
{
    Ix.w = Iy.w = I.w; Ix.h = Iy.h = I.h;
    Ix.p.assign(I.p.size(), 0.f); Iy.p.assign(I.p.size(), 0.f);
    parallelFor(I.h, [&](int ya, int yb) {
    for (int y = ya; y < yb; ++y) {
        const int ym = reflect101(y - 1, I.h), yp = reflect101(y + 1, I.h);
        for (int x = 0; x < I.w; ++x) {
            const int xm = reflect101(x - 1, I.w), xp = reflect101(x + 1, I.w);
            Ix.at(x, y) = (3.f * (I.at(xp, ym) - I.at(xm, ym)) + 10.f * (I.at(xp, y) - I.at(xm, y)) +
                           3.f * (I.at(xp, yp) - I.at(xm, yp))) * (1.f / 32.f);
            Iy.at(x, y) = (3.f * (I.at(xm, yp) - I.at(xm, ym)) + 10.f * (I.at(x, yp) - I.at(x, ym)) +
                           3.f * (I.at(xp, yp) - I.at(xp, ym))) * (1.f / 32.f);
        }
    }
    }, 16);
}

std::shared_ptr<Pyramid> buildPyramid(const GrayImage& g, int levels)
{
    auto P = std::make_shared<Pyramid>();
    P->L.resize(size_t(levels) + 1);
    P->L[0].I = g;
    for (int l = 1; l <= levels; ++l) P->L[size_t(l)].I = pyrDown(P->L[size_t(l) - 1].I);
    for (auto& lv : P->L) scharr(lv.I, lv.Ix, lv.Iy);
    return P;
}

inline float sampleClamped(const GrayImage& g, int x, int y)
{
    x = std::min(std::max(x, 0), g.w - 1); y = std::min(std::max(y, 0), g.h - 1);
    return g.at(x, y);
}

// ------------------------------------------------------------------ Lucas-Kanade (Bouguet, as OpenCV)

bool lkPoint(const Pyramid& A, const Pyramid& B, Pt p0, Pt& p1, const AnalysisParams& prm)
{
    const int win = prm.lkWin, half = (win - 1) / 2, levels = int(A.L.size()) - 1;
    const int n = win * win;
    thread_local std::vector<float> Iw, Ixw, Iyw;
    if (int(Iw.size()) < n) { Iw.resize(size_t(n)); Ixw.resize(size_t(n)); Iyw.resize(size_t(n)); }
    float nx = 0, ny = 0;   // next point (window top-left) at the current level
    for (int l = levels; l >= 0; --l) {
        const Level& la = A.L[size_t(l)]; const Level& lb = B.L[size_t(l)];
        const float sc = 1.f / float(1 << l);
        const float px = p0.x * sc - half, py = p0.y * sc - half;
        if (l == levels) { nx = px; ny = py; } else { nx = (nx + half) * 2.f - half; ny = (ny + half) * 2.f - half; }
        const int ix = int(std::floor(px)), iy = int(std::floor(py));
        if (ix < -win || ix >= la.I.w || iy < -win || iy >= la.I.h) { if (l == 0) return false; continue; }
        const float ax = px - ix, ay = py - iy;
        const float w00 = (1 - ax) * (1 - ay), w01 = ax * (1 - ay), w10 = (1 - ax) * ay, w11 = ax * ay;
        double A11 = 0, A12 = 0, A22 = 0;
        const bool inside = ix >= 0 && iy >= 0 && ix + win < la.I.w && iy + win < la.I.h;
        for (int y = 0, k = 0; y < win; ++y)
            for (int x = 0; x < win; ++x, ++k) {
                const int sx = ix + x, sy = iy + y;
                float i, gx, gy;
                if (inside) {
                    const size_t o = size_t(sy) * size_t(la.I.w) + size_t(sx), w = size_t(la.I.w);
                    i = la.I.p[o] * w00 + la.I.p[o + 1] * w01 + la.I.p[o + w] * w10 + la.I.p[o + w + 1] * w11;
                    gx = la.Ix.p[o] * w00 + la.Ix.p[o + 1] * w01 + la.Ix.p[o + w] * w10 + la.Ix.p[o + w + 1] * w11;
                    gy = la.Iy.p[o] * w00 + la.Iy.p[o + 1] * w01 + la.Iy.p[o + w] * w10 + la.Iy.p[o + w + 1] * w11;
                } else {
                    i = sampleClamped(la.I, sx, sy) * w00 + sampleClamped(la.I, sx + 1, sy) * w01 +
                        sampleClamped(la.I, sx, sy + 1) * w10 + sampleClamped(la.I, sx + 1, sy + 1) * w11;
                    gx = sampleClamped(la.Ix, sx, sy) * w00 + sampleClamped(la.Ix, sx + 1, sy) * w01 +
                         sampleClamped(la.Ix, sx, sy + 1) * w10 + sampleClamped(la.Ix, sx + 1, sy + 1) * w11;
                    gy = sampleClamped(la.Iy, sx, sy) * w00 + sampleClamped(la.Iy, sx + 1, sy) * w01 +
                         sampleClamped(la.Iy, sx, sy + 1) * w10 + sampleClamped(la.Iy, sx + 1, sy + 1) * w11;
                }
                Iw[size_t(k)] = i; Ixw[size_t(k)] = gx; Iyw[size_t(k)] = gy;
                A11 += double(gx) * gx; A12 += double(gx) * gy; A22 += double(gy) * gy;
            }
        const double D = A11 * A22 - A12 * A12;
        const double minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) + 4.0 * A12 * A12)) / (2.0 * n);
        if (minEig < 0.1024 || D < 1e-9) { if (l == 0) return false; continue; }   // = OpenCV minEigThreshold 1e-4
        const double iD = 1.0 / D;
        float pdx = 0, pdy = 0;
        for (int it = 0; it < prm.lkIters; ++it) {
            const int jx = int(std::floor(nx)), jy = int(std::floor(ny));
            if (jx < -win || jx >= lb.I.w || jy < -win || jy >= lb.I.h) { if (l == 0) return false; break; }
            const float bx = nx - jx, by = ny - jy;
            const float v00 = (1 - bx) * (1 - by), v01 = bx * (1 - by), v10 = (1 - bx) * by, v11 = bx * by;
            const bool in2 = jx >= 0 && jy >= 0 && jx + win < lb.I.w && jy + win < lb.I.h;
            float b1 = 0, b2 = 0;
            for (int y = 0, k = 0; y < win; ++y)
                for (int x = 0; x < win; ++x, ++k) {
                    const int sx = jx + x, sy = jy + y;
                    float j;
                    if (in2) {
                        const size_t o = size_t(sy) * size_t(lb.I.w) + size_t(sx), w = size_t(lb.I.w);
                        j = lb.I.p[o] * v00 + lb.I.p[o + 1] * v01 + lb.I.p[o + w] * v10 + lb.I.p[o + w + 1] * v11;
                    } else {
                        j = sampleClamped(lb.I, sx, sy) * v00 + sampleClamped(lb.I, sx + 1, sy) * v01 +
                            sampleClamped(lb.I, sx, sy + 1) * v10 + sampleClamped(lb.I, sx + 1, sy + 1) * v11;
                    }
                    const float diff = j - Iw[size_t(k)];
                    b1 += diff * Ixw[size_t(k)]; b2 += diff * Iyw[size_t(k)];
                }
            const float dx = float((A12 * b2 - A22 * b1) * iD), dy = float((A12 * b1 - A11 * b2) * iD);
            nx += dx; ny += dy;
            if (dx * dx + dy * dy <= prm.lkEps * prm.lkEps) break;
            if (it > 0 && std::fabs(dx + pdx) < 0.01f && std::fabs(dy + pdy) < 0.01f) { nx -= dx * 0.5f; ny -= dy * 0.5f; break; }
            pdx = dx; pdy = dy;
        }
    }
    p1 = {nx + half, ny + half};
    return std::isfinite(p1.x) && std::isfinite(p1.y);
}

/// forward + backward LK; keeps points with |p0 - back(p1)| < fbMax (and, if inBounds, p1 inside)
/// nInBounds: only points [0, nInBounds) must land inside the image (all points are tracked in one pass)
void trackFB(const Pyramid& A, const Pyramid& B, const std::vector<Pt>& p0, std::vector<char>& ok, std::vector<Pt>& p1,
             const AnalysisParams& prm, size_t nInBounds)
{
    const int n = int(p0.size());
    ok.assign(size_t(n), 0); p1.assign(size_t(n), Pt{0, 0});
    const int W = A.L[0].I.w, H = A.L[0].I.h;
    parallelFor(n, [&](int a, int b) {
        for (int i = a; i < b; ++i) {
            Pt q, r;
            if (!lkPoint(A, B, p0[size_t(i)], q, prm)) continue;
            if (!lkPoint(B, A, q, r, prm)) continue;
            const float ex = r.x - p0[size_t(i)].x, ey = r.y - p0[size_t(i)].y;
            if (ex * ex + ey * ey >= prm.fbMax * prm.fbMax) continue;
            if (size_t(i) < nInBounds && (q.x < 0 || q.x >= W || q.y < 0 || q.y >= H)) continue;
            p1[size_t(i)] = q; ok[size_t(i)] = 1;
        }
    }, 32);
}

// ------------------------------------------------------------------ small linear algebra

/// eigen decomposition of a symmetric NxN matrix (cyclic Jacobi); returns eigenvalues, V columns
template <int N>
void jacobiEigen(double a[N][N], double ev[N], double V[N][N])
{
    for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) V[i][j] = i == j ? 1.0 : 0.0;
    for (int sweep = 0; sweep < 60; ++sweep) {
        double off = 0;
        for (int i = 0; i < N; ++i) for (int j = i + 1; j < N; ++j) off += a[i][j] * a[i][j];
        if (off < 1e-30) break;
        for (int p = 0; p < N; ++p)
            for (int q = p + 1; q < N; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (th >= 0 ? 1.0 : -1.0) / (std::fabs(th) + std::sqrt(th * th + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < N; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < N; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < N; ++k) {
                    const double vkp = V[k][p], vkq = V[k][q];
                    V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
                }
            }
    }
    for (int i = 0; i < N; ++i) ev[i] = a[i][i];
}

inline Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(const Vec3& a) { const double n = norm(a); return n > 0 ? a * (1.0 / n) : a; }

bool solve3(const double H[3][3], const double g[3], double x[3])
{
    const double a = H[0][0], b = H[0][1], c = H[0][2], d = H[1][0], e = H[1][1], f = H[1][2], gg = H[2][0], h = H[2][1], i = H[2][2];
    const double det = a * (e * i - f * h) - b * (d * i - f * gg) + c * (d * h - e * gg);
    if (std::fabs(det) < 1e-300) return false;
    const double inv[3][3] = {{(e * i - f * h), -(b * i - c * h), (b * f - c * e)},
                              {-(d * i - f * gg), (a * i - c * gg), -(a * f - c * d)},
                              {(d * h - e * gg), -(a * h - b * gg), (a * e - b * d)}};
    for (int r = 0; r < 3; ++r) x[r] = (inv[r][0] * g[0] + inv[r][1] * g[1] + inv[r][2] * g[2]) / det;
    return true;
}

// ------------------------------------------------------------------ rotation estimation

inline double rotResidualPx(const Mat3& R, const Vec3& a, const Vec3& b, double f) { return norm(cross(R * a, b)) * f; }

struct RotEstimate { Mat3 R; double quality = 0; };

RotEstimate robustRotation(const std::vector<Vec3>& a, const std::vector<Vec3>& b, double f, double thr, int iters, std::mt19937& rng)
{
    RotEstimate out;
    const int n = int(a.size());
    if (n < 8) return out;
    std::uniform_int_distribution<int> U(0, n - 1);
    int bestCnt = -1; Mat3 best;
    std::vector<Vec3> pa(2), pb(2);
    for (int it = 0; it < iters; ++it) {
        int i = U(rng), j = U(rng);
        if (i == j) { j = (j + 1) % n; }
        pa[0] = a[size_t(i)]; pa[1] = a[size_t(j)]; pb[0] = b[size_t(i)]; pb[1] = b[size_t(j)];
        const Mat3 R = kabsch(pa, pb);
        int cnt = 0;
        for (int k = 0; k < n; ++k) cnt += rotResidualPx(R, a[size_t(k)], b[size_t(k)], f) < thr;
        if (cnt > bestCnt) { bestCnt = cnt; best = R; }
    }
    Mat3 R = best;
    std::vector<double> w(static_cast<size_t>(n));
    for (int it = 0; it < 5; ++it) {   // IRLS with Cauchy weights
        for (int k = 0; k < n; ++k) { const double r = rotResidualPx(R, a[size_t(k)], b[size_t(k)], f) / thr; w[size_t(k)] = 1.0 / (1.0 + r * r); }
        R = kabsch(a, b, &w);
    }
    int inl = 0;
    for (int k = 0; k < n; ++k) inl += rotResidualPx(R, a[size_t(k)], b[size_t(k)], f) < thr;
    out.R = R; out.quality = double(inl) / double(n);
    return out;
}

/// Relative pose over a chain: rotation R (b ~ R a up to translation parallax) and translation
/// direction t, starting from the integrated frame rotations R0.  Robust (RANSAC on t, then
/// alternating IRLS on the epipolar error).  Returns the number of inliers.
int chainPose(const std::vector<Vec3>& a, const std::vector<Vec3>& b, const Mat3& R0, double thr, Mat3& Rout, std::mt19937& rng)
{
    const int n = int(a.size());
    Mat3 R = R0;
    auto residual = [&](const Mat3& Rm, const Vec3& t, int i, Vec3* cOut = nullptr, double* sOut = nullptr) {
        const Vec3 ra = Rm * a[size_t(i)];
        const Vec3 c = cross(ra, b[size_t(i)]);
        const double s = std::max(1e-9, norm(cross(t, ra)));
        if (cOut) *cOut = c;
        if (sOut) *sOut = s;
        return dot(t, c) / s;    // ~ angular distance of b from the epipolar plane (t, R a)
    };
    // 1) translation direction by RANSAC, rotation fixed at R0
    std::uniform_int_distribution<int> U(0, n - 1);
    Vec3 t{0, 0, 1};
    int bestCnt = -1;
    std::vector<Vec3> c(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) c[size_t(i)] = cross(R * a[size_t(i)], b[size_t(i)]);
    for (int it = 0; it < 100; ++it) {
        const int i = U(rng), j = U(rng);
        if (i == j) continue;
        Vec3 tt = cross(c[size_t(i)], c[size_t(j)]);
        if (norm(tt) < 1e-12) continue;
        tt = normalized(tt);
        int cnt = 0;
        for (int k = 0; k < n; ++k) cnt += std::fabs(residual(R, tt, k)) < thr;
        if (cnt > bestCnt) { bestCnt = cnt; t = tt; }
    }
    // 2) joint robust refinement of (R, t): Levenberg-Marquardt on the epipolar residuals with
    //    Cauchy weights; t stays a unit vector (2 tangent parameters)
    auto cost = [&](const Mat3& Rm, const Vec3& tv) {
        double c2 = 0;
        for (int k = 0; k < n; ++k) { const double r = residual(Rm, tv, k) / thr; c2 += std::log1p(r * r); }
        return c2;
    };
    double lambda = 1e-3, cur = cost(R, t);
    for (int it = 0; it < 40; ++it) {
        const Vec3 u1 = normalized(std::fabs(t.x) < 0.9 ? cross(t, {1, 0, 0}) : cross(t, {0, 1, 0}));
        const Vec3 u2 = cross(t, u1);
        double H[5][5] = {}, g[5] = {};
        for (int k = 0; k < n; ++k) {
            Vec3 ck; double s; const double r = residual(R, t, k, &ck, &s);
            const double rr = r / thr, w = 1.0 / (1.0 + rr * rr);
            const Vec3& bk = b[size_t(k)];
            const Vec3 ra = R * a[size_t(k)];
            // d/d(delta) of t.((R a + delta x R a) x b) = (t.Ra) b - (Ra.b) t ;  d/dt = c (tangent part)
            const Vec3 Jr = (bk * dot(t, ra) - t * dot(ra, bk)) * (1.0 / s);
            const double J[5] = {Jr.x, Jr.y, Jr.z, dot(ck, u1) / s, dot(ck, u2) / s};
            for (int p = 0; p < 5; ++p) { g[p] += w * r * J[p]; for (int q = 0; q < 5; ++q) H[p][q] += w * J[p] * J[q]; }
        }
        bool improved = false;
        for (int tries = 0; tries < 8 && !improved; ++tries) {
            double M[5][6];
            for (int p = 0; p < 5; ++p) { for (int q = 0; q < 5; ++q) M[p][q] = H[p][q] + (p == q ? lambda * (H[p][p] + 1e-12) : 0.0); M[p][5] = -g[p]; }
            bool singular = false;   // Gaussian elimination with partial pivoting
            for (int c = 0; c < 5 && !singular; ++c) {
                int piv = c; for (int r = c + 1; r < 5; ++r) if (std::fabs(M[r][c]) > std::fabs(M[piv][c])) piv = r;
                if (std::fabs(M[piv][c]) < 1e-300) { singular = true; break; }
                if (piv != c) for (int q = 0; q < 6; ++q) std::swap(M[c][q], M[piv][q]);
                for (int r = c + 1; r < 5; ++r) { const double fct = M[r][c] / M[c][c]; for (int q = c; q < 6; ++q) M[r][q] -= fct * M[c][q]; }
            }
            if (singular) break;
            double x[5];
            for (int r = 4; r >= 0; --r) { double v = M[r][5]; for (int q = r + 1; q < 5; ++q) v -= M[r][q] * x[q]; x[r] = v / M[r][r]; }
            const Mat3 Rn = rotmat({x[0], x[1], x[2]}) * R;
            const Vec3 tn = normalized(t + u1 * x[3] + u2 * x[4]);
            const double c2 = cost(Rn, tn);
            if (c2 < cur) {
                const bool small = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4] < 1e-18;
                R = Rn; t = tn; cur = c2; lambda = std::max(1e-9, lambda * 0.3); improved = true;
                if (small) it = 1000;
            } else lambda *= 10;
        }
        if (!improved) break;
    }
    int inl = 0;
    for (int k = 0; k < n; ++k) inl += std::fabs(residual(R, t, k)) < thr;
    Rout = R;
    return inl;
}

// ------------------------------------------------------------------ smoothing helpers

std::vector<double> gaussNearest(const std::vector<double>& x, double sigma)
{
    const int n = int(x.size());
    if (n == 0 || sigma <= 0) return x;
    const int R = std::max(1, int(4.0 * sigma + 0.5));
    std::vector<double> k(size_t(2 * R + 1));
    double s = 0;
    for (int j = -R; j <= R; ++j) { k[size_t(j + R)] = std::exp(-0.5 * j * j / (sigma * sigma)); s += k[size_t(j + R)]; }
    std::vector<double> y(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        double a = 0;
        for (int j = -R; j <= R; ++j) a += k[size_t(j + R)] * x[size_t(std::min(n - 1, std::max(0, i + j)))];
        y[size_t(i)] = a / s;
    }
    return y;
}

// ------------------------------------------------------------------ gravity EKF (2 DoF on the sphere)

struct M2 { double a = 0, b = 0, c = 0, d = 0; };   // [[a b][c d]]

std::vector<Vec3> ekfPass(const std::vector<Mat3>& dRs, Vec3 g0, M2 P, const std::vector<double>& sp,
                          double sigRoll, double pitchPrior, double sigPP)
{
    Vec3 g = normalized(g0);
    Vec3 b1 = normalized(cross({0, 0, 1}, g)), b2 = cross(g, b1);
    std::vector<Vec3> out; out.reserve(dRs.size() + 1); out.push_back(g);
    const double zy = std::cos(pitchPrior), zz = -std::sin(pitchPrior);
    const double meas[3][2] = {{0.0, sigRoll * sigRoll}, {zy, sigPP * sigPP}, {zz, sigPP * sigPP}};
    for (size_t s = 0; s < dRs.size(); ++s) {
        const Mat3& R = dRs[s];
        g = R * g; b1 = R * b1; b2 = R * b2;
        P.a += sp[s] * sp[s]; P.d += sp[s] * sp[s];
        for (int comp = 0; comp < 3; ++comp) {
            const double z = meas[comp][0], r = meas[comp][1];
            const double h0 = b1[comp], h1 = b2[comp];
            const double Ph0 = P.a * h0 + P.b * h1, Ph1 = P.c * h0 + P.d * h1;
            const double S = h0 * Ph0 + h1 * Ph1 + r;
            const double k0 = Ph0 / S, k1 = Ph1 / S;
            const double inn = z - g[comp];
            g = normalized(g + b1 * (k0 * inn) + b2 * (k1 * inn));
            // P = (I - K h^T) P
            const M2 Pn{P.a - k0 * (h0 * P.a + h1 * P.c), P.b - k0 * (h0 * P.b + h1 * P.d),
                        P.c - k1 * (h0 * P.a + h1 * P.c), P.d - k1 * (h0 * P.b + h1 * P.d)};
            P = Pn;
        }
        b1 = normalized(b1 - g * dot(b1, g)); b2 = cross(g, b1);
        out.push_back(g);
    }
    return out;
}

std::vector<Vec3> gravityEkf(const std::vector<Mat3>& dR, const std::vector<double>& quality)
{
    const double deg = kPi / 180.0;
    const double sigProc = 0.05 * deg, pitchPrior = -20.0 * deg;
    const double sigRoll = sigProc * 30, sigPP = sigProc * 150;
    const size_t N = dR.size();
    const std::vector<double> qs = gaussNearest(quality, 30.0);
    std::vector<double> sp(N - 1);
    for (size_t i = 1; i < N; ++i) sp[i - 1] = sigProc * std::min(6.0, std::max(1.0, 0.35 / std::max(qs[i], 1e-3)));
    const Vec3 g0{0, std::cos(pitchPrior), -std::sin(pitchPrior)};
    std::vector<Mat3> back; back.reserve(N - 1);
    for (size_t i = N - 1; i >= 1; --i) back.push_back(dR[i].T());
    std::vector<double> spb(sp.rbegin(), sp.rend());
    std::vector<Mat3> fwd(dR.begin() + 1, dR.end());
    // A warm-up forward pass from the prior settles the state; the backward and forward passes then
    // both start from a converged state with (roughly) steady-state uncertainty. Starting the final
    // passes from the wide prior pulled the first/last ~15 frames by up to 5 deg (false pitch/roll).
    const auto gw = ekfPass(fwd, g0, M2{std::pow(3 * deg, 2), 0, 0, std::pow(20 * deg, 2)}, sp, sigRoll, pitchPrior, sigPP);
    const M2 Pss{std::pow(0.3 * deg, 2), 0, 0, std::pow(0.6 * deg, 2)};
    const auto gb = ekfPass(back, gw.back(), Pss, spb, sigRoll, pitchPrior, sigPP);
    const auto gf = ekfPass(fwd, gb.back(), Pss, sp, sigRoll, pitchPrior, sigPP);
    std::vector<Vec3> g(N);
    for (size_t i = 0; i < N; ++i) g[i] = normalized(gf[i] + gb[N - 1 - i]);
    return g;
}

std::vector<Vec3> fusePath(const std::vector<Mat3>& dR, const std::vector<Vec3>& g)
{
    const size_t N = dR.size();
    std::vector<Vec3> E(N);
    for (size_t i = 0; i < N; ++i) {
        E[i].y = -std::asin(std::max(-1.0, std::min(1.0, g[i].z)));
        E[i].z = std::atan2(g[i].x, g[i].y);
    }
    Mat3 prev = fromEuler({0, E[0].y, E[0].z});
    for (size_t i = 1; i < N; ++i) {
        const Mat3 q = prev * dR[i].T();
        E[i].x = std::atan2(q(0, 2), q(2, 2));   // yaw of Ry*Rx*Rz
        prev = fromEuler(E[i]);
    }
    for (size_t i = 1; i < N; ++i) {               // unwrap yaw
        double d = E[i].x - E[i - 1].x;
        while (d > kPi) { E[i].x -= 2 * kPi; d -= 2 * kPi; }
        while (d < -kPi) { E[i].x += 2 * kPi; d += 2 * kPi; }
    }
    return E;
}

} // namespace

// ------------------------------------------------------------------ public helpers

Mat3 rotmat(const Vec3& w)
{
    const double th = norm(w);
    Mat3 R;
    if (th < 1e-12) { R.m = {1, -w.z, w.y, w.z, 1, -w.x, -w.y, w.x, 1}; return R; }
    const Vec3 k = w * (1.0 / th);
    const double c = std::cos(th), s = std::sin(th), C = 1 - c;
    R.m = {c + k.x * k.x * C, k.x * k.y * C - k.z * s, k.x * k.z * C + k.y * s,
           k.y * k.x * C + k.z * s, c + k.y * k.y * C, k.y * k.z * C - k.x * s,
           k.z * k.x * C - k.y * s, k.z * k.y * C + k.x * s, c + k.z * k.z * C};
    return R;
}

Mat3 kabsch(const std::vector<Vec3>& a, const std::vector<Vec3>& b, const std::vector<double>* w)
{
    // Horn's quaternion method: rotation taking a onto b
    double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (size_t i = 0; i < a.size(); ++i) {
        const double wi = w ? (*w)[i] : 1.0;
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) S[r][c] += wi * a[i][r] * b[i][c];
    }
    const double Sxx = S[0][0], Sxy = S[0][1], Sxz = S[0][2], Syx = S[1][0], Syy = S[1][1], Syz = S[1][2], Szx = S[2][0], Szy = S[2][1], Szz = S[2][2];
    double N[4][4] = {{Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx},
                      {Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz},
                      {Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy},
                      {Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz}};
    double ev[4], V[4][4];
    jacobiEigen<4>(N, ev, V);
    int m = 0; for (int i = 1; i < 4; ++i) if (ev[i] > ev[m]) m = i;
    const double q0 = V[0][m], qx = V[1][m], qy = V[2][m], qz = V[3][m];
    Mat3 R;
    R.m = {q0 * q0 + qx * qx - qy * qy - qz * qz, 2 * (qx * qy - q0 * qz), 2 * (qx * qz + q0 * qy),
           2 * (qy * qx + q0 * qz), q0 * q0 - qx * qx + qy * qy - qz * qz, 2 * (qy * qz - q0 * qx),
           2 * (qz * qx - q0 * qy), 2 * (qz * qy + q0 * qx), q0 * q0 - qx * qx - qy * qy + qz * qz};
    const double n2 = q0 * q0 + qx * qx + qy * qy + qz * qz;
    for (auto& v : R.m) v /= n2;
    return R;
}

std::vector<Pt> detectCorners(const GrayImage& g, const AnalysisParams& p)
{
    // Shi-Tomasi per cell (as cv2.goodFeaturesToTrack on each cell image, blockSize 7, Sobel 3)
    const int ch = g.h / p.cellsY, cw = g.w / p.cellsX;
    const int half = 3;   // blockSize 7
    const int nCells = p.cellsX * p.cellsY;
    std::vector<std::vector<Pt>> perCell(static_cast<size_t>(nCells));
    parallelFor(nCells, [&](int c0, int c1) {
        std::vector<float> dx, dy, eig;
        for (int ci = c0; ci < c1; ++ci) {
            const int cx = ci % p.cellsX, cy = ci / p.cellsX;
            const int x0 = cx * cw, y0 = cy * ch;
            auto I = [&](int x, int y) { return g.at(x0 + reflect101(x, cw), y0 + reflect101(y, ch)); };
            dx.assign(size_t(cw * ch), 0.f); dy.assign(size_t(cw * ch), 0.f);
            for (int y = 0; y < ch; ++y)
                for (int x = 0; x < cw; ++x) {
                    dx[size_t(y * cw + x)] = (I(x + 1, y - 1) + 2 * I(x + 1, y) + I(x + 1, y + 1)) - (I(x - 1, y - 1) + 2 * I(x - 1, y) + I(x - 1, y + 1));
                    dy[size_t(y * cw + x)] = (I(x - 1, y + 1) + 2 * I(x, y + 1) + I(x + 1, y + 1)) - (I(x - 1, y - 1) + 2 * I(x, y - 1) + I(x + 1, y - 1));
                }
            // 7x7 box sums of dx^2, dx*dy, dy^2 (separable, reflect-101 border)
            const size_t M = size_t(cw * ch);
            std::vector<float> xx(M), xy(M), yy(M), hx(M), hxy(M), hy(M);
            for (size_t o = 0; o < M; ++o) { xx[o] = dx[o] * dx[o]; xy[o] = dx[o] * dy[o]; yy[o] = dy[o] * dy[o]; }
            for (int y = 0; y < ch; ++y)
                for (int x = 0; x < cw; ++x) {
                    float a = 0, b = 0, c = 0;
                    for (int u = -half; u <= half; ++u) { const size_t o = size_t(y * cw + reflect101(x + u, cw)); a += xx[o]; b += xy[o]; c += yy[o]; }
                    const size_t o = size_t(y * cw + x); hx[o] = a; hxy[o] = b; hy[o] = c;
                }
            eig.assign(M, 0.f);
            float mx = 0;
            for (int y = 0; y < ch; ++y)
                for (int x = 0; x < cw; ++x) {
                    double a = 0, b = 0, c = 0;
                    for (int v = -half; v <= half; ++v) { const size_t o = size_t(reflect101(y + v, ch) * cw + x); a += hx[o]; b += hxy[o]; c += hy[o]; }
                    const float e = float(((a + c) - std::sqrt((a - c) * (a - c) + 4 * b * b)) * 0.5);
                    eig[size_t(y * cw + x)] = e; mx = std::max(mx, e);
                }
            const float thr = float(mx * p.cornerQuality);
            struct C { float e; int x, y; };
            std::vector<C> cand;
            for (int y = 1; y < ch - 1; ++y)
                for (int x = 1; x < cw - 1; ++x) {
                    const float e = eig[size_t(y * cw + x)];
                    if (e <= thr) continue;
                    bool isMax = true;
                    for (int v = -1; v <= 1 && isMax; ++v)
                        for (int u = -1; u <= 1; ++u)
                            if (eig[size_t((y + v) * cw + x + u)] > e) { isMax = false; break; }
                    if (isMax) cand.push_back({e, x, y});
                }
            std::stable_sort(cand.begin(), cand.end(), [](const C& l, const C& r) { return l.e > r.e; });
            std::vector<Pt> acc;
            const double md2 = p.minDistance * p.minDistance;
            for (const C& c : cand) {
                bool good = true;
                for (const Pt& q : acc) { const double ddx = q.x - c.x, ddy = q.y - c.y; if (ddx * ddx + ddy * ddy < md2) { good = false; break; } }
                if (!good) continue;
                acc.push_back({float(c.x), float(c.y)});
                if (int(acc.size()) >= p.perCell) break;
            }
            for (const Pt& q : acc) perCell[size_t(ci)].push_back({q.x + float(x0), q.y + float(y0)});
        }
    });
    std::vector<Pt> out;
    for (const auto& v : perCell) out.insert(out.end(), v.begin(), v.end());
    return out;
}

void trackPoints(const GrayImage& a, const GrayImage& b, const std::vector<Pt>& p0, std::vector<char>& ok,
                 std::vector<Pt>& p1, const AnalysisParams& p)
{
    auto A = buildPyramid(a, p.lkLevels), B = buildPyramid(b, p.lkLevels);
    trackFB(*A, *B, p0, ok, p1, p, p0.size());
}

GrayImage grayFromU8(const uint8_t* data, int w, int h)
{
    GrayImage g; g.w = w; g.h = h; g.p.resize(size_t(w) * size_t(h));
    for (size_t i = 0; i < g.p.size(); ++i) g.p[i] = float(data[i]);
    return g;
}

GrayImage grayFromRGBA(const ImageView& img, const RectI& rect, int outW, int outH)
{
    // separable area averaging; OFX rows are bottom-up -> output row 0 = top of the picture
    const int W = rect.w(), H = rect.h();
    GrayImage g; g.w = outW; g.h = outH; g.p.assign(size_t(outW) * size_t(outH), 0.f);
    if (W <= 0 || H <= 0) return g;
    std::vector<float> rowTmp(size_t(outW) * size_t(H));   // horizontally resampled luma, top-down
    const double sx = double(W) / outW, sy = double(H) / outH;
    parallelFor(H, [&](int r0, int r1) {
    for (int r = r0; r < r1; ++r) {
        const int y = rect.y2 - 1 - r;
        if (y < img.y1 || y >= img.y2) continue;
        for (int ox = 0; ox < outW; ++ox) {
            const double a = ox * sx, b = (ox + 1) * sx;
            double acc = 0, wsum = 0;
            for (int x = int(std::floor(a)); x < int(std::ceil(b)); ++x) {
                const double wgt = std::min(b, x + 1.0) - std::max(a, double(x));
                const int X = rect.x1 + x;
                if (wgt <= 0 || X < img.x1 || X >= img.x2) continue;
                const float* p = img.px(X, y);
                const double l = std::min(1.0, std::max(0.0, 0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2]));
                acc += wgt * l; wsum += wgt;
            }
            rowTmp[size_t(r) * size_t(outW) + size_t(ox)] = float(wsum > 0 ? acc / wsum * 255.0 : 0.0);
        }
    }
    }, 16);
    parallelFor(outH, [&](int y0, int y1) {
    for (int oy = y0; oy < y1; ++oy) {
        const double a = oy * sy, b = (oy + 1) * sy;
        for (int ox = 0; ox < outW; ++ox) {
            double acc = 0, wsum = 0;
            for (int r = int(std::floor(a)); r < int(std::ceil(b)) && r < H; ++r) {
                const double wgt = std::min(b, r + 1.0) - std::max(a, double(r));
                if (wgt <= 0) continue;
                acc += wgt * rowTmp[size_t(r) * size_t(outW) + size_t(ox)]; wsum += wgt;
            }
            g.at(ox, oy) = float(wsum > 0 ? acc / wsum : 0.0);
        }
    }
    }, 16);
    return g;
}

// ------------------------------------------------------------------ Analyzer

struct Analyzer::Impl {
    AnalysisParams p;
    int W = 0, H = 0;
    double f = 0;
    std::shared_ptr<Pyramid> prev;
    std::vector<Mat3> dR;
    std::vector<double> q;
    struct Chain { int start; std::vector<Pt> p0, p; };
    std::vector<Chain> chains;
    struct ChainResult { int start; Vec3 rvR, rvRc; };
    std::vector<ChainResult> done;
    std::mt19937 rng{0}, rngChain{1};
    AnalysisStats st;
    double ptsSum = 0;

    Vec3 bearing(const Pt& u) const { return normalized({(u.x - W / 2.0) / f, (u.y - H / 2.0) / f, 1.0}); }
};

Analyzer::Analyzer(const AnalysisParams& p) : d_(new Impl) { d_->p = p; }
Analyzer::~Analyzer() = default;
size_t Analyzer::frames() const { return d_->dR.size(); }
const AnalysisStats& Analyzer::stats() const { return d_->st; }

namespace {
using Clk = std::chrono::steady_clock;
inline double msSince(Clk::time_point& t) { const auto n = Clk::now(); const double r = std::chrono::duration<double, std::milli>(n - t).count(); t = n; return r; }
}

void Analyzer::addFrame(const GrayImage& g)
{
    Impl& d = *d_;
    Clk::time_point tk = Clk::now();
    auto cur = buildPyramid(g, d.p.lkLevels);
    d.st.msPyramid += msSince(tk);
    if (!d.prev) {
        d.W = g.w; d.H = g.h; d.f = d.p.focalNorm * d.W;
        d.dR.push_back(Mat3()); d.q.push_back(1.0);
        d.prev = cur;
        return;
    }
    const int i = int(d.dR.size());          // index of the new frame
    const AnalysisParams& p = d.p;
    // frame rotation (i-1 -> i)
    const std::vector<Pt> p0 = detectCorners(d.prev->L[0].I, p);
    d.st.msDetect += msSince(tk);
    // one tracking pass for the frame-to-frame points and all running chains (same result as
    // separate passes, but one split over the cores instead of several small ones)
    std::vector<Pt> all(p0);
    std::vector<size_t> chainOff;
    for (const auto& c : d.chains) { chainOff.push_back(all.size()); all.insert(all.end(), c.p.begin(), c.p.end()); }
    std::vector<char> okAll; std::vector<Pt> p1All;
    trackFB(*d.prev, *cur, all, okAll, p1All, p, p0.size());
    std::vector<char> ok(okAll.begin(), okAll.begin() + std::ptrdiff_t(p0.size()));
    std::vector<Pt> p1(p1All.begin(), p1All.begin() + std::ptrdiff_t(p0.size()));
    d.st.msTrack += msSince(tk);
    std::vector<Pt> A, B;
    std::vector<Vec3> a, b;
    for (size_t k = 0; k < p0.size(); ++k)
        if (ok[k]) { A.push_back(p0[k]); B.push_back(p1[k]); a.push_back(d.bearing(p0[k])); b.push_back(d.bearing(p1[k])); }
    const double scale = double(d.W) / 540.0;
    RotEstimate re = robustRotation(a, b, d.f, p.rotThrPx * scale, p.rotIters, d.rng);
    if (a.size() < 8) { re.R = d.dR.back(); re.quality = 0.0; }   // fog/darkness: keep the last motion instead of "camera stops"
    d.dR.push_back(re.R); d.q.push_back(re.quality);
    d.ptsSum += double(a.size());
    d.st.msRansac += msSince(tk);

    // chains for the bias correction: advance the running ones, start a new one at i-1
    for (size_t ci = 0; ci < d.chains.size(); ++ci) {
        auto& c = d.chains[ci];
        const size_t o = chainOff[ci];
        std::vector<Pt> np0, np;
        for (size_t k = 0; k < c.p.size(); ++k) if (okAll[o + k]) { np0.push_back(c.p0[k]); np.push_back(p1All[o + k]); }
        c.p0.swap(np0); c.p.swap(np);
    }
    if ((i - 1) % p.chainStride == 0) d.chains.push_back({i - 1, A, B});
    for (auto it = d.chains.begin(); it != d.chains.end();) {
        if (i - it->start < p.chainLen) { ++it; continue; }
        ++d.st.chainsTried;
        if (it->p0.size() >= 60) {
            Mat3 Rc;
            for (int j = it->start + 1; j <= it->start + p.chainLen; ++j) Rc = d.dR[size_t(j)] * Rc;
            std::vector<Vec3> ca, cb;
            for (size_t k = 0; k < it->p0.size(); ++k) { ca.push_back(d.bearing(it->p0[k])); cb.push_back(d.bearing(it->p[k])); }
            Mat3 R;
            const int inl = chainPose(ca, cb, Rc, p.chainThrPx * scale / d.f, R, d.rngChain);
            if (std::getenv("TS_DEBUG")) {
                const Vec3 bb = (rotvec(R) - rotvec(Rc)) * (180.0 / kPi / p.chainLen);
                std::fprintf(stderr, "chain %d: n=%zu inl=%d bias deg/frame %.3f %.3f %.3f\n", it->start, ca.size(), inl, bb.x, bb.y, bb.z);
            }
            if (inl >= 40) d.done.push_back({it->start, rotvec(R), rotvec(Rc)});
        }
        it = d.chains.erase(it);
    }
    d.st.msChains += msSince(tk);
    d.prev = cur;
}

CameraTrack Analyzer::finish(double fps, int pictureW, int pictureH)
{
    Impl& d = *d_;
    CameraTrack tr;
    const size_t N = d.dR.size();
    tr.source = "plugin";
    tr.width = pictureW; tr.height = pictureH; tr.focalNorm = d.p.focalNorm; tr.nominalFps = fps;
    if (N < 2) return tr;
    // ---- bias correction (slow drift of the frame rotations: translation parallax)
    const int k = d.p.chainLen;
    std::vector<double> num[3] = {std::vector<double>(N, 0.0), std::vector<double>(N, 0.0), std::vector<double>(N, 0.0)};
    std::vector<double> den(N, 0.0);
    const double maxBias = 0.5 * kPi / 180.0;
    for (const auto& c : d.done) {
        const Vec3 bias = (c.rvR - c.rvRc) * (1.0 / k);
        if (std::fabs(bias.x) > maxBias || std::fabs(bias.y) > maxBias || std::fabs(bias.z) > maxBias) continue;
        ++d.st.chainsUsed;
        for (int j = c.start + 1; j <= c.start + k && size_t(j) < N; ++j) {
            num[0][size_t(j)] += bias.x; num[1][size_t(j)] += bias.y; num[2][size_t(j)] += bias.z; den[size_t(j)] += 1;
        }
    }
    std::vector<size_t> have;
    for (size_t j = 0; j < N; ++j) if (den[j] > 0) have.push_back(j);
    std::vector<Mat3> dR2 = d.dR;
    if (have.size() >= 2) {
        const double sig[3] = {1.0, 2.5, 2.5}, gain[3] = {1.0, 0.5, 0.3};
        std::vector<double> bax[3];
        for (int ax = 0; ax < 3; ++ax) {
            std::vector<double> v(N);
            for (size_t j = 0; j < N; ++j) {
                if (den[j] > 0) { v[j] = num[ax][j] / den[j]; continue; }
                // linear interpolation between known samples, constant outside (np.interp)
                auto it = std::lower_bound(have.begin(), have.end(), j);
                if (it == have.begin()) v[j] = num[ax][have.front()] / den[have.front()];
                else if (it == have.end()) v[j] = num[ax][have.back()] / den[have.back()];
                else {
                    const size_t r = *it, l = *(it - 1);
                    const double vl = num[ax][l] / den[l], vr = num[ax][r] / den[r];
                    v[j] = vl + (vr - vl) * double(j - l) / double(r - l);
                }
            }
            bax[ax] = gaussNearest(v, sig[ax] * fps);
            for (auto& x : bax[ax]) x *= gain[ax];
        }
        for (size_t j = 1; j < N; ++j) dR2[j] = rotmat(rotvec(d.dR[j]) + Vec3{bax[0][j], bax[1][j], bax[2][j]});
    }
    dR2[0] = Mat3();
    // ---- gravity and fusion
    const std::vector<Vec3> g = gravityEkf(dR2, d.q);
    tr.E = fusePath(dR2, g);
    tr.t.resize(N); tr.quality.resize(N);
    for (size_t j = 0; j < N; ++j) { tr.t[j] = double(j) / fps; tr.quality[j] = float(d.q[j]); }
    d.st.frames = N;
    d.st.meanPoints = d.ptsSum / double(N - 1);
    double qs = 0; for (double v : d.q) qs += v;
    d.st.meanQuality = qs / double(N);
    return tr;
}

} // namespace ts
