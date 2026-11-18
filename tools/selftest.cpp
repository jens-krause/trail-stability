// Self-test of the portable core: path solver vs. Python reference, warp sanity.
#include "analysis.h"
#include "path_solver.h"
#include "sidecar.h"
#include "warp.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

int main(int argc, char** argv)
{
    int fails = 0;
    // ---- 1. warp: identity rotation, zoom 1 -> image unchanged (interior)
    {
        const int W = 64, H = 96;
        std::vector<float> a(W * H * 4), b(W * H * 4, -1.f);
        for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x)
            for (int c = 0; c < 4; ++c) a[(y * W + x) * 4 + c] = float(std::sin(0.3 * x + c) + std::cos(0.2 * y));
        ts::ImageView s{reinterpret_cast<char*>(a.data()), W * 16, 0, 0, W, H}, d{reinterpret_cast<char*>(b.data()), W * 16, 0, 0, W, H};
        double Hm[9]; ts::frameHomography(ts::Mat3(), 1.0, 1.0, W, H, W, Hm);
        ts::warpBicubic(s, d, 0, 0, W, H, Hm, 2);
        double err = 0; for (size_t i = 0; i < a.size(); ++i) err = std::max(err, double(std::fabs(a[i] - b[i])));
        std::printf("warp identity max err %.2e %s\n", err, err < 1e-5 ? "OK" : "FAIL"); fails += err >= 1e-5;
        // small yaw: image content shifts horizontally by about f*tan(angle) pixels
        ts::Mat3 R = ts::rotY(0.02);
        ts::frameHomography(R, 1.0, 1.0, W, H, W, Hm);
        double cx = 0.5 * (W - 1), cy = 0.5 * (H - 1);
        double sx = (Hm[0] * cx + Hm[1] * cy + Hm[2]) / (Hm[6] * cx + Hm[7] * cy + Hm[8]);
        std::printf("yaw 0.02 rad: centre maps to x=%.3f (expected %.3f) %s\n", sx, cx + W * std::tan(0.02),
                    std::fabs(sx - (cx + W * std::tan(0.02))) < 1e-6 ? "OK" : "FAIL");
        fails += std::fabs(sx - (cx + W * std::tan(0.02))) >= 1e-6;
    }
    // ---- 1b. placement: the same picture somewhere inside a larger frame (pillarbox, Inspector
    //          pan, partly off-screen, letterbox) must give the identical stabilized picture,
    //          and nothing may spill outside of the picture area.
    {
        const int PW = 108, PH = 192;                       // picture, aspect 16:9 portrait
        const double aspect = 2816.0 / 1584.0, fN = 1.018519, zoom = 1.25;
        const ts::Mat3 R = ts::rotY(0.03) * ts::rotX(0.02) * ts::rotZ(0.05);
        auto tex = [](int x, int y, int c) { return float(2.0 + std::sin(0.31 * x + c) + std::cos(0.23 * y - c)); };
        // reference: picture fills the frame
        std::vector<float> a(size_t(PW) * PH * 4), ra(a.size());
        for (int y = 0; y < PH; ++y) for (int x = 0; x < PW; ++x) for (int c = 0; c < 4; ++c) a[(size_t(y) * PW + x) * 4 + c] = tex(x, y, c);
        ts::ImageView sa{reinterpret_cast<char*>(a.data()), PW * 16, 0, 0, PW, PH}, da{reinterpret_cast<char*>(ra.data()), PW * 16, 0, 0, PW, PH};
        {
            ts::RectI fr{0, 0, PW, PH};
            ts::Placement p = ts::placePicture(fr, ts::findContentRect(sa), aspect);
            double Hm[9]; ts::cameraHomography(R, fN * p.width, zoom, p.cx, p.cy, Hm);
            ts::warpBicubic(sa, da, 0, 0, PW, PH, Hm, 2, &p.rect);
            double Hold[9]; ts::frameHomography(R, fN, zoom, PW, PH, PW, Hold);
            double d = 0; for (int i = 0; i < 9; ++i) d = std::max(d, std::fabs(Hm[i] - Hold[i]));
            std::printf("placement full frame: detected=%d, homography vs v0 %.1e %s\n", p.detected, d, (p.detected && d < 1e-9) ? "OK" : "FAIL");
            fails += !(p.detected && d < 1e-9);
        }
        {   // full-frame clip whose top rows are pure black (night sky / crushed shadows)
            std::vector<float> c = a;
            for (int y = PH - 25; y < PH; ++y) for (int x = 0; x < PW; ++x) for (int ch = 0; ch < 3; ++ch) c[(size_t(y) * PW + x) * 4 + ch] = 0.f;
            ts::ImageView sc{reinterpret_cast<char*>(c.data()), PW * 16, 0, 0, PW, PH};
            ts::RectI fr{0, 0, PW, PH};
            ts::RectI cr = ts::findContentRect(sc);
            ts::Placement p = ts::placePicture(fr, cr, aspect);
            const bool okc = p.detected && p.rect.x1 == 0 && p.rect.y2 == PH && std::fabs(p.cy - 95.5) < 1e-9 && std::fabs(p.width - PW) < 1e-9;
            std::printf("placement full frame, black top rows: content y2=%d -> centre=(%.1f,%.1f) w=%.2f %s\n", cr.y2, p.cx, p.cy, p.width, okc ? "OK" : "FAIL");
            fails += !okc;
        }
        struct Case { const char* name; int FW, FH, ox, oy; };
        const Case cases[] = {{"pillarbox centred", 3 * PW, PH, PW, 0}, {"pan right third", 3 * PW, PH, 2 * PW, 0},
                              {"partly off right edge", 3 * PW, PH, 3 * PW - 54, 0}, {"partly off left edge", 3 * PW, PH, -40, 0},
                              {"letterbox shifted", PW, 300, 0, 31}};
        for (const Case& cs : cases) {
            std::vector<float> b(size_t(cs.FW) * cs.FH * 4, 0.f), rb(b.size(), -1.f);
            for (int y = 0; y < PH; ++y) for (int x = 0; x < PW; ++x) {
                const int X = x + cs.ox, Y = y + cs.oy;
                if (X < 0 || X >= cs.FW || Y < 0 || Y >= cs.FH) continue;
                for (int c = 0; c < 4; ++c) b[(size_t(Y) * cs.FW + X) * 4 + c] = tex(x, y, c);
            }
            ts::ImageView sb{reinterpret_cast<char*>(b.data()), cs.FW * 16, 0, 0, cs.FW, cs.FH}, db{reinterpret_cast<char*>(rb.data()), cs.FW * 16, 0, 0, cs.FW, cs.FH};
            ts::RectI fr{0, 0, cs.FW, cs.FH};
            ts::Placement p = ts::placePicture(fr, ts::findContentRect(sb), aspect);
            double Hm[9]; ts::cameraHomography(R, fN * p.width, zoom, p.cx, p.cy, Hm);
            ts::warpBicubic(sb, db, 0, 0, cs.FW, cs.FH, Hm, 2, &p.rect);
            double errIn = 0, spill = 0; int nIn = 0;
            for (int Y = 0; Y < cs.FH; ++Y) for (int X = 0; X < cs.FW; ++X) {
                const int x = X - cs.ox, y = Y - cs.oy;
                const float* o = &rb[(size_t(Y) * cs.FW + X) * 4];
                if (x >= 0 && x < PW && y >= 0 && y < PH) {
                    const float* r = &ra[(size_t(y) * PW + x) * 4];
                    // partly visible pictures: compare only where the reference sampled from the visible part
                    const bool fullyVisible = cs.ox >= 0 && cs.ox + PW <= cs.FW;
                    if (!fullyVisible && (o[3] == 0.f)) continue;
                    if (!fullyVisible) {
                        // reference sample position must lie inside the visible part, else skip
                        const double zx = Hm[0] * X + Hm[1] * Y + Hm[2], zw = Hm[6] * X + Hm[7] * Y + Hm[8];
                        const double sx = zx / zw;
                        if (sx < p.rect.x1 + 2 || sx > p.rect.x2 - 3) continue;
                    }
                    for (int c = 0; c < 4; ++c) errIn = std::max(errIn, double(std::fabs(o[c] - r[c])));
                    ++nIn;
                } else for (int c = 0; c < 4; ++c) spill = std::max(spill, double(std::fabs(o[c])));
            }
            const bool okc = p.detected && errIn < 1e-4 && spill == 0 && nIn > 100;
            std::printf("placement %-22s detected=%d centre=(%.1f,%.1f) w=%.2f | max diff to full-frame %.1e (%d px), spill %.1e %s\n",
                        cs.name, p.detected, p.cx, p.cy, p.width, errIn, nIn, spill, okc ? "OK" : "FAIL");
            fails += !okc;
        }
    }
    // ---- 1c. time -> frame as Resolve shows it (nearest timestamp, ties to the later frame)
    if (argc >= 2) {
        ts::CameraTrack tr; std::string err;
        if (ts::loadTrack(argv[1], tr, &err)) {
            int wrongNew = 0, wrongOld = 0;
            for (size_t k = 0; k < tr.size(); ++k) {
                const double sec = double(k) / 60.0;
                wrongNew += ts::CameraTrack(tr).frameIndexAt(sec) != k;
                wrongOld += tr.frameAt(sec + tr.t.front()) != k;
            }
            // synthetic drift like IMG_1854 (one extra 1/600 s tick every 217 frames, up to 0.7 frame):
            // Resolve shows frame k while the drift is <= 1/2 frame and k-1 after it (measured)
            ts::CameraTrack d; d.nominalFps = 60.0; int wrongDrift = 0;
            for (int k = 0; k < 1413; ++k) d.t.push_back(std::round((k / 60.0 + (k / 217) / 600.0) * 1e6) / 1e6);
            for (int k = 0; k < 1413; ++k) {
                const int drift = k / 217;                         // in tenths of a frame
                const size_t expect = size_t(drift <= 5 ? k : k - 1);
                wrongDrift += d.frameIndexAt(k / 60.0) != expect;
            }
            std::printf("frame grid drift test (0.7 frame): wrong %d %s\n", wrongDrift, wrongDrift == 0 ? "OK" : "FAIL");
            fails += wrongDrift != 0;
            const bool okc = std::fabs(tr.nominalFps - 60.0) < 1e-9 && wrongNew == 0;
            std::printf("frame grid: nominal %.3f fps, frames off-grid: %d (timestamp lookup v0: %d) %s\n",
                        tr.nominalFps, wrongNew, wrongOld, okc ? "OK" : "FAIL");
            fails += !okc;
        }
    }
    // ---- 2. path solver against Python reference
    if (argc >= 3) {
        ts::CameraTrack tr; std::string err;
        if (!ts::loadTrack(argv[1], tr, &err)) { std::printf("load failed: %s\n", err.c_str()); return 2; }
        ts::SolveParams p; p.smoothness = 0.6; p.zoom = 1.25; p.levelHorizon = true; p.vertical = 0.0; p.denoiseFrames = 0.0;   // reference: no vertical term, no denoise
        auto t0 = std::chrono::steady_clock::now();
        ts::SolveResult r = ts::solvePath(tr, p);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        FILE* f = std::fopen(argv[2], "r");
        double maxd = 0; size_t n = 0; double y, pp, rr;
        while (f && std::fscanf(f, "%lf %lf %lf", &y, &pp, &rr) == 3 && n < r.S.size()) {
            maxd = std::max({maxd, std::fabs(y - r.S[n].x), std::fabs(pp - r.S[n].y), std::fabs(rr - r.S[n].z)}); ++n;
        }
        if (f) std::fclose(f);
        std::printf("solver: %zu frames, status %d, %.0f ms, clamped %d, max corr %.2f deg, max |S_cpp - S_py| = %.4f deg %s\n",
                    tr.size(), r.status, ms, r.clampedFrames, r.maxCorrectionDeg, maxd * 180 / M_PI,
                    (n == r.S.size() && maxd * 180 / M_PI < 0.05) ? "OK" : "FAIL");
        fails += !(n == r.S.size() && maxd * 180 / M_PI < 0.05);
    }
    // ---- 3. vertical bounce: running bounce in V (3 Hz, 0.4 deg) on top of a steady forward drift
    {
        ts::CameraTrack tr; tr.width = 1584; tr.height = 2816; tr.focalNorm = 1.018519;
        const int N = 360; const double fps = 60.0, amp = 0.4 * M_PI / 180;
        for (int i = 0; i < N; ++i) {
            const double t = i / fps;
            tr.t.push_back(t); tr.E.push_back({0.0, 0.0, 0.0}); tr.quality.push_back(1.f);
            tr.V.push_back(amp * std::sin(2 * M_PI * 3.0 * t) + 0.08 * t);   // 0.08 rad/s steady drift
        }
        ts::SolveParams p0, p1; p0.vertical = 0.0; p1.vertical = 1.0; p0.denoiseFrames = p1.denoiseFrames = 0.0;
        ts::SolveResult r0 = ts::solvePath(tr, p0), r1 = ts::solvePath(tr, p1);
        const double bounce = amp * 180 / M_PI;
        double sMax = 0; for (int i = 30; i < N - 30; ++i) sMax = std::max(sMax, std::fabs(r1.S[size_t(i)].y));
        sMax *= 180 / M_PI;
        const bool smooth = sMax < 0.25 * bounce;                                   // virtual camera stays calm
        const bool corrects = r1.maxCorrectionDeg > 0.6 * bounce && r1.maxCorrectionDeg < 1.6 * bounce;  // removes the bounce, no more
        const bool offIsOff = r0.maxCorrectionDeg < 1e-3;                           // vertical = 0: nothing to correct
        std::printf("vertical: bounce %.2f deg, virtual pitch max %.3f deg, correction %.2f deg (drift 0.08 rad/s must not show), off -> %.4f deg  %s\n",
                    bounce, sMax, r1.maxCorrectionDeg, r0.maxCorrectionDeg, (smooth && corrects && offIsOff) ? "OK" : "FAIL");
        fails += !(smooth && corrects && offIsOff);
    }
    // ---- 4. embedded track: encode/decode round trip (incl. a large yaw jump)
    {
        ts::CameraTrack tr; tr.width = 1080; tr.height = 1920; tr.focalNorm = 550.0 / 540.0;
        for (int i = 0; i < 500; ++i) {
            tr.t.push_back(i / 60.0);
            tr.E.push_back({0.3 * std::sin(i * 0.05) + (i > 300 ? 6.0 : 0.0), -0.3 + 0.01 * std::cos(i * 0.3), 0.02 * std::sin(i * 0.7)});
            tr.quality.push_back(float(0.3 + 0.001 * i));
        }
        ts::EmbeddedInfo in; in.t0 = 12; in.fps = 60; in.rangeMin = 0; in.rangeMax = 1412; in.seconds = 83.5;
        const std::string s = ts::encodeTrack(tr, in);
        ts::CameraTrack back; ts::EmbeddedInfo out; std::string err;
        const bool ok = ts::decodeTrack(s, back, out, &err);
        double e = 0, eq = 0;
        if (ok) for (size_t i = 0; i < tr.size(); ++i) {
            e = std::max({e, std::fabs(back.E[i].x - tr.E[i].x), std::fabs(back.E[i].y - tr.E[i].y), std::fabs(back.E[i].z - tr.E[i].z)});
            eq = std::max(eq, double(std::fabs(back.quality[i] - tr.quality[i])));
        }
        const bool okc = ok && back.size() == tr.size() && e < 1e-5 && eq < 0.003 && out.t0 == 12 && out.rangeMax == 1412 && out.seconds == 83.5 && back.width == 1080;
        std::printf("embedded track: %zu bytes for %zu frames, max angle err %.1e rad %s %s\n", s.size(), tr.size(), e, err.c_str(), okc ? "OK" : "FAIL");
        fails += !okc;
    }
    // ---- 5. analysis: synthetic pure rotations of a textured scene must be recovered
    {
        const int W = 270, H = 480;                // small for speed; focal follows focalNorm
        const double f = 550.0 / 540.0 * W;
        std::mt19937 rng(7); std::uniform_real_distribution<double> U(0, 1);
        // texture on a large "panorama" plane far away: random blobs, sampled through a camera rotation
        const int TW = 1400, TH = 1400;
        std::vector<float> tex(size_t(TW) * TH, 0.f);
        for (int k = 0; k < 2500; ++k) {
            const double cx = U(rng) * TW, cy = U(rng) * TH, r = 3 + 9 * U(rng), v = 255 * U(rng);
            for (int y = std::max(0, int(cy - r)); y < std::min(TH, int(cy + r)); ++y)
                for (int x = std::max(0, int(cx - r)); x < std::min(TW, int(cx + r)); ++x)
                    if ((x - cx) * (x - cx) + (y - cy) * (y - cy) < r * r) tex[size_t(y) * TW + size_t(x)] = float(v);
        }
        auto render = [&](const ts::Mat3& Rcw) {   // camera -> world rotation
            ts::GrayImage g; g.w = W; g.h = H; g.p.resize(size_t(W) * H);
            const double fT = 700;                 // texture "focal"
            for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
                const ts::Vec3 v = Rcw * ts::Vec3((x - W / 2.0) / f, (y - H / 2.0) / f, 1.0);
                const double u = TW / 2.0 + fT * v.x / v.z, w = TH / 2.0 + fT * v.y / v.z;
                const int iu = int(std::floor(u)), iw = int(std::floor(w));
                const double au = u - iu, aw = w - iw;
                auto T = [&](int a, int b) { a = std::min(std::max(a, 0), TW - 1); b = std::min(std::max(b, 0), TH - 1); return tex[size_t(b) * TW + size_t(a)]; };
                g.at(x, y) = float((1 - au) * (1 - aw) * T(iu, iw) + au * (1 - aw) * T(iu + 1, iw) + (1 - au) * aw * T(iu, iw + 1) + au * aw * T(iu + 1, iw + 1));
            }
            return g;
        };
        ts::AnalysisParams ap; ap.width = W; ap.threads = 2;
        ts::Analyzer an(ap);
        const int N = 40;
        std::vector<ts::Vec3> truth;
        for (int i = 0; i < N; ++i) {
            const ts::Vec3 e{0.04 * std::sin(i * 0.31), -0.35 + 0.03 * std::sin(i * 0.47), 0.03 * std::sin(i * 0.23)};
            truth.push_back(e);
            an.addFrame(render(ts::fromEuler(e)));
        }
        const ts::CameraTrack tr = an.finish(60.0, 1080, 1920);
        // compare frame-to-frame relative rotations (absolute pitch/roll come from the gravity prior)
        double err = 0;
        for (int i = 1; i < N; ++i) {
            const ts::Mat3 Rt = ts::fromEuler(truth[size_t(i)]).T() * ts::fromEuler(truth[size_t(i - 1)]);
            const ts::Mat3 Re = ts::fromEuler(tr.E[size_t(i)]).T() * ts::fromEuler(tr.E[size_t(i - 1)]);
            const ts::Vec3 d = ts::rotvec(Re.T() * Rt);
            err = std::max(err, std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z) * f);
        }
        // relative yaw must be exact; pitch/roll pass through the gravity filter, so compare the
        // yaw-only path change separately as well
        double yawErr = 0;
        for (int i = 1; i < N; ++i) yawErr = std::max(yawErr, std::fabs((tr.E[size_t(i)].x - tr.E[0].x) - (truth[size_t(i)].x - truth[0].x)) * f);
        const bool okc = yawErr < 0.3;
        std::printf("analysis (synthetic rotations): yaw path error %.3f px, frame-to-frame rotation error %.2f px, points %.0f, quality %.2f %s\n",
                    yawErr, err, an.stats().meanPoints, an.stats().meanQuality, okc ? "OK" : "FAIL");
        fails += !okc;
    }
    std::printf(fails ? "SELFTEST FAILED (%d)\n" : "SELFTEST OK\n", fails);
    return fails ? 1 : 0;
}
