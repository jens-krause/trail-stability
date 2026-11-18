#include "path_solver.h"
#include "osqp.h"
#include <algorithm>
#include <cmath>
#include <map>

namespace ts {
namespace {

// sparse matrix in triplet form -> OSQP CSC (duplicates summed)
struct Triplets {
    std::map<std::pair<OSQPInt, OSQPInt>, double> v;   // (col,row) ordered
    void add(OSQPInt r, OSQPInt c, double x) { if (x != 0.0) v[{c, r}] += x; }
};

struct Csc {
    std::vector<OSQPFloat> x; std::vector<OSQPInt> i, p;
    OSQPInt rows = 0, cols = 0;
    void build(const Triplets& t, OSQPInt m, OSQPInt n) {
        rows = m; cols = n; x.clear(); i.clear(); p.assign(size_t(n) + 1, 0);
        for (const auto& kv : t.v) { x.push_back(kv.second); i.push_back(kv.first.second); p[size_t(kv.first.first) + 1]++; }
        for (OSQPInt c = 0; c < n; ++c) p[size_t(c) + 1] += p[size_t(c)];
    }
};

// Jacobian: small Euler offsets at E -> body rotation vector
void eulerJacobian(const Vec3& E, double J[3][3])
{
    const double eps = 1e-5;
    Mat3 Qt = fromEuler(E).T();
    for (int k = 0; k < 3; ++k) {
        Vec3 d = E; d[k] += eps;
        Vec3 w = rotvec(Qt * fromEuler(d));
        for (int r = 0; r < 3; ++r) J[r][k] = w[r] / eps;
    }
}

double maxAlpha(const Vec3& Eq, const Vec3& Es, double f, double aspect, double zoom)
{
    Mat3 QT = fromEuler(Eq).T();
    auto ok = [&](double a) { return cornersInside(QT * fromEuler(Eq + (Es - Eq) * a), f, aspect, zoom); };
    if (ok(1.0)) return 1.0;
    double lo = 0, hi = 1;
    for (int it = 0; it < 14; ++it) { double m = 0.5 * (lo + hi); (ok(m) ? lo : hi) = m; }
    return lo;
}

} // namespace

bool cornersInside(const Mat3& R, double f, double aspect, double zoom)
{
    const double w = 1.0, h = aspect, fo = f * zoom;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2) {
            Vec3 b = R * Vec3(sx * w / 2 / fo, sy * h / 2 / fo, 1.0);
            if (b.z <= 1e-6) return false;
            if (std::fabs(b.x / b.z * f) > w / 2 || std::fabs(b.y / b.z * f) > h / 2) return false;
        }
    return true;
}

static SolveResult solveCore(const CameraTrack& tr, const SolveParams& prm)
{
    SolveResult res;
    const OSQPInt N = OSQPInt(tr.size());
    if (N < 4) { res.message = "track too short"; return res; }
    const double fps = double(N - 1) / std::max(1e-6, tr.t.back() - tr.t.front());
    const double lam = std::pow(prm.smoothness * fps, 4.0);
    const double f = tr.focalNorm, aspect = double(tr.height) / double(tr.width);
    const OSQPInt n = 3 * N;

    // ---- objective: sum_axis ||S-E||^2 + lam ||D2 S||^2 (+ level weight on roll)
    Triplets Pt;
    std::vector<OSQPFloat> q(static_cast<size_t>(n));
    for (int ax = 0; ax < 3; ++ax) {
        const OSQPInt o = ax * N;
        const double diag = 2.0 * (1.0 + ((ax == 2 && prm.levelHorizon) ? prm.levelWeight : 0.0));
        for (OSQPInt i = 0; i < N; ++i) { Pt.add(o + i, o + i, diag); q[size_t(o + i)] = -2.0 * tr.E[size_t(i)][ax]; }
        const double c[3] = {1, -2, 1};
        for (OSQPInt r = 0; r + 2 < N; ++r)          // D2 rows -> 2*lam*D2^T D2 (upper triangle)
            for (int a = 0; a < 3; ++a)
                for (int b = a; b < 3; ++b) Pt.add(o + r + a, o + r + b, 2.0 * lam * c[a] * c[b]);
    }

    // ---- corridor constraints (linearised corner positions, 8 rows per frame)
    const double safety = 0.97, fo = f * prm.zoom, w = 1.0, h = aspect;
    double A8[8][3], p0[8], lo[8], hi[8];
    { int k = 0;
      for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2) {
            double vx = sx * w / 2 / fo, vy = sy * h / 2 / fo;
            A8[k][0] = -vx * vy; A8[k][1] = 1 + vx * vx; A8[k][2] = -vy; p0[k] = vx;
            lo[k] = -safety * w / 2 / f; hi[k] = safety * w / 2 / f; ++k;
            A8[k][0] = -(1 + vy * vy); A8[k][1] = vx * vy; A8[k][2] = vx; p0[k] = vy;
            lo[k] = -safety * h / 2 / f; hi[k] = safety * h / 2 / f; ++k;
        } }
    Triplets At; std::vector<OSQPFloat> l, u;
    OSQPInt row = 0;
    for (OSQPInt i = 0; i < N; ++i) {
        double J[3][3]; eulerJacobian(tr.E[size_t(i)], J);
        for (int k = 0; k < 8; ++k) {
            double AJ[3], off = 0;
            for (int ax = 0; ax < 3; ++ax) {
                AJ[ax] = A8[k][0] * J[0][ax] + A8[k][1] * J[1][ax] + A8[k][2] * J[2][ax];
                At.add(row, ax * N + i, AJ[ax]);
                off += AJ[ax] * tr.E[size_t(i)][ax];
            }
            l.push_back(lo[k] - p0[k] + off); u.push_back(hi[k] - p0[k] + off); ++row;
        }
    }
    Csc P, A; P.build(Pt, n, n); A.build(At, row, n);

    OSQPCscMatrix* Pm = OSQPCscMatrix_new(n, n, OSQPInt(P.x.size()), P.x.data(), P.i.data(), P.p.data());
    OSQPCscMatrix* Am = OSQPCscMatrix_new(row, n, OSQPInt(A.x.size()), A.x.data(), A.i.data(), A.p.data());
    OSQPSettings* st = OSQPSettings_new();
    res.S = tr.E;                                  // fallback: no stabilization
    // eps 1e-5 is enough (path differs < 0.02 px from 1e-6) and about 3x faster; the tight tolerance
    // is a developer option (no automatic second run, so the solve time stays predictable)
    {
        const double eps = prm.precise ? 1e-6 : 1e-5;
        osqp_set_default_settings(st);
        st->verbose = 0; st->eps_abs = eps; st->eps_rel = eps; st->max_iter = prm.precise ? 100000 : 40000; st->polishing = 1;
        OSQPSolver* solver = nullptr;
        const OSQPInt err = osqp_setup(&solver, Pm, q.data(), Am, l.data(), u.data(), row, n, st);
        if (err || !solver) res.message = "osqp_setup failed";
        else {
            osqp_solve(solver);
            res.status = int(solver->info->status_val);
            const bool usable = res.status == OSQP_SOLVED || res.status == OSQP_SOLVED_INACCURATE || res.status == OSQP_MAX_ITER_REACHED;
            if (usable && solver->solution && solver->solution->x)
                for (OSQPInt i = 0; i < N; ++i)
                    res.S[size_t(i)] = {solver->solution->x[i], solver->solution->x[N + i], solver->solution->x[2 * N + i]};
        }
        if (solver) osqp_cleanup(solver);
    }
    OSQPSettings_free(st); OSQPCscMatrix_free(Pm); OSQPCscMatrix_free(Am);

    // ---- exact (non-linearised) safety net and warp matrices
    res.warp.resize(size_t(N));
    for (OSQPInt i = 0; i < N; ++i) {
        const Vec3& Eq = tr.E[size_t(i)];
        double a = maxAlpha(Eq, res.S[size_t(i)], f, aspect, prm.zoom);
        if (a < 0.999) { res.clampedFrames++; res.S[size_t(i)] = Eq + (res.S[size_t(i)] - Eq) * a; }
        res.warp[size_t(i)] = fromEuler(Eq).T() * fromEuler(res.S[size_t(i)]);
        Vec3 c = rotvec(res.warp[size_t(i)]);
        res.maxCorrectionDeg = std::max(res.maxCorrectionDeg,
                                        std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z) * 180.0 / M_PI);
    }
    return res;
}

// Gaussian low-pass with odd-reflected edges (sigma in samples)
static std::vector<double> gaussLowpass(const std::vector<double>& x, double sigma)
{
    const int n = int(x.size()), R = std::max(1, int(std::ceil(3.0 * sigma)));
    std::vector<double> k(size_t(2 * R + 1));
    std::vector<double> y(static_cast<size_t>(n));
    double sum = 0;
    for (int j = -R; j <= R; ++j) { k[size_t(j + R)] = std::exp(-0.5 * j * j / (sigma * sigma)); sum += k[size_t(j + R)]; }
    for (int i = 0; i < n; ++i) {
        double a = 0;
        for (int j = -R; j <= R; ++j) {
            const int m = i + j;   // odd reflection about the end points keeps linear drift exact
            const double v = m < 0 ? 2.0 * x[0] - x[size_t(std::min(n - 1, -m))]
                           : m > n - 1 ? 2.0 * x[size_t(n - 1)] - x[size_t(std::max(0, 2 * (n - 1) - m))] : x[size_t(m)];
            a += k[size_t(j + R)] * v;
        }
        y[size_t(i)] = a / sum;
    }
    return y;
}

SolveResult solvePath(const CameraTrack& tr, const SolveParams& prm)
{
    if (tr.size() <= 8) return solveCore(tr, prm);
    CameraTrack adj = tr;
    // Frame-to-frame pitch and roll of the measured camera are dominated by estimation noise
    // (the rotation fit absorbs vertical translation parallax of nearby ground).  Correcting
    // them shook the image more than it calmed it (v0.3 measurements), so the solver works
    // with slightly smoothed pitch/roll.  Yaw is measured well and stays exact.
    if (prm.denoiseFrames > 0.0) {
        std::vector<double> p(tr.size()), r(tr.size());
        for (size_t i = 0; i < tr.size(); ++i) { p[i] = tr.E[i].y; r[i] = tr.E[i].z; }
        p = gaussLowpass(p, prm.denoiseFrames); r = gaussLowpass(r, prm.denoiseFrames);
        for (size_t i = 0; i < tr.size(); ++i) { adj.E[i].y = p[i]; adj.E[i].z = r[i]; }
    }
    // Vertical bounce that pure rotation cannot explain is folded into the pitch of the input
    // camera ("apparent pitch"); the path solver then smooths it like any other rotation.
    // Only the fast part is used, so slow forward-motion flow never accumulates in the angle.
    if (prm.vertical > 0.0 && tr.V.size() == tr.size()) {
        const double fps = double(tr.size() - 1) / std::max(1e-6, tr.t.back() - tr.t.front());
        const std::vector<double> lp = gaussLowpass(tr.V, std::max(1.0, prm.verticalHpSec * fps));
        for (size_t i = 0; i < adj.size(); ++i) adj.E[i].y += prm.vertical * (tr.V[i] - lp[i]);
    }
    return solveCore(adj, prm);
}

} // namespace ts
