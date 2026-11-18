#include "sidecar.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <fstream>
#include <sstream>

namespace ts {

size_t CameraTrack::frameAt(double s) const
{
    if (t.empty()) return 0;
    auto it = std::lower_bound(t.begin(), t.end(), s);
    if (it == t.begin()) return 0;
    if (it == t.end()) return t.size() - 1;
    size_t i = size_t(it - t.begin());
    return (s - t[i - 1] < t[i] - s) ? i - 1 : i;
}

size_t CameraTrack::frameIndexAt(double s) const
{
    // Resolve shows, at timeline time s, the source frame whose timestamp is nearest to s;
    // on an exact tie (drift of exactly half a frame) it shows the later frame.  Measured on
    // IMG_1853/IMG_1854 (Resolve 21.1): the stored iPhone timestamps run 59.97 fps, so after
    // the drift passes half a frame Resolve repeats one frame and the index grid is off by one.
    if (t.empty()) return 0;
    const double tol = 1e-4;                                   // << tick of 1/600 s, >> file rounding
    const double q = t.front() + s + 0.5 / nominalFps + tol;
    auto it = std::upper_bound(t.begin(), t.end(), q);         // first frame later than the window
    if (it == t.begin()) return 0;
    return size_t(it - t.begin()) - 1;
}

bool loadTrack(const std::string& path, CameraTrack& tr, std::string* err)
{
    std::ifstream f(path);
    if (!f) { if (err) *err = "cannot open " + path; return false; }
    std::string line, key;
    bool header = false, hasV = false; size_t frames = 0;
    std::vector<double> vv;
    tr = CameraTrack();
    while (std::getline(f, line)) {
        auto hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        std::istringstream ss(line);
        if (!(ss >> key)) continue;
        if (!header) {
            int ver = 0; ss >> ver;
            if (key != "TRAILSTAB" || ver != 1) { if (err) *err = "not a TRAILSTAB v1 file"; return false; }
            header = true; continue;
        }
        if (key == "source") { std::getline(ss >> std::ws, tr.source); }
        else if (key == "width") ss >> tr.width;
        else if (key == "height") ss >> tr.height;
        else if (key == "focal_norm") ss >> tr.focalNorm;
        else if (key == "frames") { ss >> frames; tr.t.reserve(frames); }
        else {
            double tt = std::stod(key), y, p, r, v = 0.0; float q = 1.f;
            ss >> y >> p >> r; if (!(ss >> q)) q = 1.f;
            if (ss >> v) hasV = true;
            tr.t.push_back(tt); tr.E.push_back({y, p, r}); tr.quality.push_back(q); vv.push_back(v);
        }
    }
    if (tr.t.empty() || (frames && frames != tr.t.size())) {
        if (err) *err = "frame count mismatch"; return false;
    }
    if (hasV && vv.size() == tr.t.size()) tr.V = vv;
    if (tr.t.size() > 2) {   // nominal rate: median spacing, snapped to common rates
        std::vector<double> d;
        for (size_t i = 1; i < tr.t.size(); ++i) d.push_back(tr.t[i] - tr.t[i - 1]);
        std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
        const double fps = d[d.size() / 2] > 0 ? 1.0 / d[d.size() / 2] : 60.0;
        const double common[] = {23.976, 24, 25, 29.97, 30, 47.952, 48, 50, 59.94, 60, 100, 119.88, 120, 240};
        tr.nominalFps = fps;
        double bestErr = 0.002;                       // snap to the closest common rate
        for (double c : common) { const double e = std::fabs(fps / c - 1.0); if (e < bestErr) { bestErr = e; tr.nominalFps = c; } }
    }
    return true;
}

bool saveTrack(const std::string& path, const CameraTrack& tr)
{
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    std::fprintf(f, "TRAILSTAB 1\nsource %s\nwidth %d\nheight %d\nfocal_norm %.6f\nframes %zu\n",
                 tr.source.c_str(), tr.width, tr.height, tr.focalNorm, tr.t.size());
    const bool withV = tr.V.size() == tr.t.size();
    for (size_t i = 0; i < tr.t.size(); ++i) {
        std::fprintf(f, "%.6f %.9f %.9f %.9f %.3f", tr.t[i], tr.E[i].x, tr.E[i].y, tr.E[i].z, tr.quality[i]);
        if (withV) std::fprintf(f, " %.9f", tr.V[i]);
        std::fprintf(f, "\n");
    }
    std::fclose(f);
    return true;
}

// ------------------------------------------------------------------ embedded track (project storage)

namespace {
const char* kB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
std::string b64enc(const std::vector<uint8_t>& d)
{
    std::string o; o.reserve((d.size() + 2) / 3 * 4);
    for (size_t i = 0; i < d.size(); i += 3) {
        const uint32_t v = (uint32_t(d[i]) << 16) | (i + 1 < d.size() ? uint32_t(d[i + 1]) << 8 : 0) | (i + 2 < d.size() ? d[i + 2] : 0);
        o += kB64[(v >> 18) & 63]; o += kB64[(v >> 12) & 63];
        o += i + 1 < d.size() ? kB64[(v >> 6) & 63] : '=';
        o += i + 2 < d.size() ? kB64[v & 63] : '=';
    }
    return o;
}
bool b64dec(const std::string& s, std::vector<uint8_t>& d)
{
    int map[256]; for (int& m : map) m = -1;
    for (int i = 0; i < 64; ++i) map[uint8_t(kB64[i])] = i;
    d.clear(); uint32_t v = 0; int bits = 0;
    for (char c : s) {
        if (c == '=') break;
        const int m = map[uint8_t(c)];
        if (m < 0) { if (c == '\n' || c == '\r' || c == ' ') continue; return false; }
        v = (v << 6) | uint32_t(m); bits += 6;
        if (bits >= 8) { bits -= 8; d.push_back(uint8_t((v >> bits) & 0xFF)); }
    }
    return true;
}
void put32(std::vector<uint8_t>& d, int32_t v) { for (int k = 0; k < 4; ++k) d.push_back(uint8_t((uint32_t(v) >> (8 * k)) & 0xFF)); }
void put16(std::vector<uint8_t>& d, int16_t v) { d.push_back(uint8_t(uint16_t(v) & 0xFF)); d.push_back(uint8_t(uint16_t(v) >> 8)); }
constexpr double kQ = 1e-5;   // rad per unit
} // namespace

std::string encodeTrack(const CameraTrack& tr, const EmbeddedInfo& info)
{
    std::vector<uint8_t> d;
    d.reserve(tr.size() * 7 + 16);
    long long prev[3] = {0, 0, 0};
    for (size_t i = 0; i < tr.size(); ++i) {
        const double e[3] = {tr.E[i].x, tr.E[i].y, tr.E[i].z};
        for (int a = 0; a < 3; ++a) {
            const long long q = std::llround(e[a] / kQ);
            const long long dl = q - prev[a];
            if (i == 0 || dl < -32767 || dl > 32767) { put16(d, int16_t(-32768)); put32(d, int32_t(q)); }
            else put16(d, int16_t(dl));
            prev[a] = q;
        }
        const float qq = i < tr.quality.size() ? tr.quality[i] : 1.f;
        d.push_back(uint8_t(std::lround(std::min(1.f, std::max(0.f, qq)) * 255.f)));
    }
    std::ostringstream o;
    o.precision(17);
    o << "TSA1 n=" << tr.size() << " fps=" << info.fps << " t0=" << info.t0 << " rmin=" << info.rangeMin << " rmax=" << info.rangeMax << " dur=" << info.seconds
      << " w=" << tr.width << " h=" << tr.height << " fn=" << tr.focalNorm << " |" << b64enc(d);
    return o.str();
}

bool decodeTrack(const std::string& s, CameraTrack& tr, EmbeddedInfo& info, std::string* err)
{
    auto fail = [&](const char* m) { if (err) *err = m; return false; };
    const size_t bar = s.find('|');
    // format tag: TSA1 = this format; a later incompatible change must use TSA2, TSA3, ...
    if (s.compare(0, 3, "TSA") == 0 && s.compare(0, 5, "TSA1 ") != 0) return fail("stored by a different plugin version");
    if (s.compare(0, 5, "TSA1 ") != 0 || bar == std::string::npos) return fail("no embedded analysis");
    std::istringstream h(s.substr(5, bar - 5));
    std::string kv; size_t n = 0;
    tr = CameraTrack();
    while (h >> kv) {
        const size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        const std::string k = kv.substr(0, eq); const double v = std::atof(kv.c_str() + eq + 1);
        if (k == "n") n = size_t(v); else if (k == "fps") info.fps = v; else if (k == "t0") info.t0 = v;
        else if (k == "rmin") info.rangeMin = v; else if (k == "rmax") info.rangeMax = v; else if (k == "dur") info.seconds = v;
        else if (k == "w") tr.width = int(v); else if (k == "h") tr.height = int(v); else if (k == "fn") tr.focalNorm = v;
    }
    std::vector<uint8_t> d;
    if (!b64dec(s.substr(bar + 1), d)) return fail("analysis data corrupt");
    size_t o = 0; long long prev[3] = {0, 0, 0};
    auto get16 = [&](int16_t& v) { if (o + 2 > d.size()) return false; v = int16_t(uint16_t(d[o]) | (uint16_t(d[o + 1]) << 8)); o += 2; return true; };
    auto get32 = [&](int32_t& v) { if (o + 4 > d.size()) return false; uint32_t u = 0; for (int k = 0; k < 4; ++k) u |= uint32_t(d[o + k]) << (8 * k); v = int32_t(u); o += 4; return true; };
    for (size_t i = 0; i < n; ++i) {
        double e[3];
        for (int a = 0; a < 3; ++a) {
            int16_t v; if (!get16(v)) return fail("analysis data incomplete");
            if (v == -32768) { int32_t q; if (!get32(q)) return fail("analysis data incomplete"); prev[a] = q; }
            else prev[a] += v;
            e[a] = double(prev[a]) * kQ;
        }
        if (o >= d.size()) return fail("analysis data incomplete");
        const float q = float(d[o++]) / 255.f;
        tr.E.push_back({e[0], e[1], e[2]}); tr.quality.push_back(q);
        tr.t.push_back(info.fps > 0 ? double(i) / info.fps : double(i) / 60.0);
    }
    tr.nominalFps = info.fps > 0 ? info.fps : 60.0;
    tr.source = "embedded";
    if (tr.t.empty()) return fail("empty analysis");
    return true;
}

} // namespace ts
