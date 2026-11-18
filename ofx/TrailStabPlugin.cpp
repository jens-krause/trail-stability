// Trail Stability – rotation-based stabilizer for iPhone (Action Mode) footage. OpenFX plugin.
//
// "Run Analysis" fetches the frames of the source clip, analyses the camera rotation
// (core/analysis.cpp) and stores it in a hidden string parameter, i.e. inside the Resolve project.
// Frames are indexed on the effect's own time grid, so retiming/frame repeats of the host are
// followed automatically.  The virtual camera path is re-solved (core/path_solver.cpp) whenever
// Smoothing Window / Zoom / Level Horizon / Precise Solver change; no new analysis is needed.
// Rendering runs on Resolve's Metal queue (ofx/metal_render.mm), CPU path as fallback.
//
// Log: ~/Library/Logs/TrailStab.log (key events only).  Detailed timing/render lines are written
// when the file ~/Library/Logs/TrailStab.debug exists (checked when Resolve loads the plugin).

#include "ofxsImageEffect.h"
#include "ofxsMultiThread.h"
#include "ofxGPURender.h"

#include "analysis.h"
#include "path_solver.h"
#include "sidecar.h"
#include "warp.h"
#include "threadpool.h"
#include "metal_render.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <thread>
#include <functional>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <map>
#include <memory>
#include <tuple>
#include <mutex>
#include <sstream>

#define kPluginName "Trail Stability"
#define kPluginGrouping "Trail Stability"
#define kPluginIdentifier "de.trailstab.Stabilizer"
#ifndef TRAILSTAB_VERSION
#define TRAILSTAB_VERSION "dev"
#endif
#define kTrailStabVersion "v" TRAILSTAB_VERSION   // set in CMakeLists.txt (project VERSION); bump on every build

namespace {

struct Solution {
    std::string data;          // embedded analysis ("" = not analysed)
    ts::EmbeddedInfo info;
    ts::SolveParams params;
    ts::CameraTrack track;
    ts::SolveResult result;
    std::string error;
};

bool sameParams(const ts::SolveParams& a, const ts::SolveParams& b)
{
    return a.smoothness == b.smoothness && a.zoom == b.zoom && a.levelHorizon == b.levelHorizon && a.precise == b.precise;
}

std::string clock(double sec)
{
    if (sec < 0) sec = 0;
    char b[32]; const int m = int(sec / 60); std::snprintf(b, sizeof b, "%d:%05.2f", m, sec - 60 * m);
    return b;
}

constexpr double kUnset = -1e9;
// "Smoothing Window" shown to the user = width of a moving average with the same low-pass
// effect as the solver's time constant (half-gain at 1/(2*pi*tau)); 1.7 s <-> 0.6 s.
constexpr double kWindowPerTau = 1.7 / 0.6;

std::string logDir()
{
    const char* h = std::getenv("HOME");
#ifdef __APPLE__
    return std::string(h ? h : "/tmp") + "/Library/Logs/";
#else
    return "/tmp/";
#endif
}

/// detailed logging only when ~/Library/Logs/TrailStab.debug exists (checked once)
bool debugLog()
{
    static const bool on = [] { FILE* f = std::fopen((logDir() + "TrailStab.debug").c_str(), "r"); if (f) std::fclose(f); return f != nullptr; }();
    return on;
}

// ~/Library/Logs/TrailStab.log; above 1 MB it is moved to TrailStab.old.log (one generation kept)
void logLine(const std::string& s)
{
    static std::mutex m;
    std::lock_guard<std::mutex> l(m);
    const std::string path = logDir() + "TrailStab.log";
    if (FILE* f = std::fopen(path.c_str(), "a")) {
        const long size = std::ftell(f);
        if (size > 1024 * 1024) {
            std::fclose(f);
            std::rename(path.c_str(), (logDir() + "TrailStab.old.log").c_str());
            f = std::fopen(path.c_str(), "a");
            if (!f) return;
        }
        char ts[32]; std::time_t now = std::time(nullptr);
        std::strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        std::fprintf(f, "%s %s\n", ts, s.c_str());
        std::fclose(f);
    }
}

void debugLine(const std::string& s)
{
    if (debugLog()) logLine(s);
}

bool nearlySame(const ts::RectI& a, const ts::RectI& b, int tol)
{
    return std::abs(a.x1 - b.x1) <= tol && std::abs(a.x2 - b.x2) <= tol &&
           std::abs(a.y1 - b.y1) <= tol && std::abs(a.y2 - b.y2) <= tol;
}

} // namespace

/// last picture-search result per frame geometry (shared with GPU completion handlers)
struct ContentCache {
    struct Key {
        int x1, y1, x2, y2; long scale;
        bool operator<(const Key& o) const
        { return std::tie(x1, y1, x2, y2, scale) < std::tie(o.x1, o.y1, o.x2, o.y2, o.scale); }
    };
    bool get(const Key& k, ts::RectI& r)
    {
        std::lock_guard<std::mutex> l(m);
        auto it = map.find(k);
        if (it == map.end()) return false;
        r = it->second; return true;
    }
    void put(const Key& k, const ts::RectI& r)
    {
        std::lock_guard<std::mutex> l(m);
        if (map.size() > 16) { map.clear(); count.clear(); }
        map[k] = r;
    }
    /// run the GPU picture search on this frame? (every 8th render per geometry)
    bool due(const Key& k)
    {
        std::lock_guard<std::mutex> l(m);
        return (count[k]++ % 8) == 0;
    }
    std::mutex m;
    std::map<Key, ts::RectI> map;
    std::map<Key, unsigned> count;
};

class TrailStabPlugin : public OFX::ImageEffect
{
public:
    explicit TrailStabPlugin(OfxImageEffectHandle h) : ImageEffect(h)
    {
        dst_ = fetchClip(kOfxImageEffectOutputClipName);
        src_ = fetchClip(kOfxImageEffectSimpleSourceClipName);
        smooth_ = fetchDoubleParam("smoothWindow");
        zoom_ = fetchDoubleParam("zoom");
        level_ = fetchBooleanParam("levelHorizon");
        precise_ = fetchBooleanParam("preciseSolver");
        status_ = fetchStringParam("status");
        data_ = fetchStringParam("analysisData");
        rangeStart_ = fetchDoubleParam("rangeStart");
        rangeEnd_ = fetchDoubleParam("rangeEnd");
        rangeInfo_ = fetchStringParam("rangeInfo");
        analysisWidth_ = fetchIntParam("analysisWidth");
    }

    // ------------------------------------------------------------------ solution cache
    std::shared_ptr<const Solution> solution(double t)
    {
        std::string data; data_->getValue(data);
        ts::SolveParams p;
        p.smoothness = smooth_->getValueAtTime(t) / kWindowPerTau;
        p.zoom = zoom_->getValueAtTime(t);
        p.levelHorizon = level_->getValueAtTime(t);
        p.vertical = 0.0;
        p.precise = precise_->getValue();

        std::lock_guard<std::mutex> lock(mutex_);
        if (cache_ && cache_->data == data && sameParams(cache_->params, p)) return cache_;

        auto s = std::make_shared<Solution>();
        s->data = data; s->params = p;
        if (!data.empty()) {
            if (cache_ && cache_->data == data && cache_->error.empty()) { s->track = cache_->track; s->info = cache_->info; }
            else ts::decodeTrack(data, s->track, s->info, &s->error);
            if (s->error.empty() && !s->track.empty()) s->result = ts::solvePath(s->track, p);
        }
        cache_ = s;
        return s;
    }

    /// does the stored analysis still fit the clip as the host presents it now?
    bool embeddedFits(const Solution& s) const
    {
        const OfxRangeD fr = src_->getFrameRange();
        const double fps = src_->getFrameRate();
        return std::fabs(fr.min - s.info.rangeMin) < 0.5 && std::fabs(fr.max - s.info.rangeMax) < 0.5 &&
               (s.info.fps <= 0 || fps <= 0 || std::fabs(fps / s.info.fps - 1.0) < 1e-3);
    }

    void updateRangeInfo()
    {
        const double a = rangeStart_->getValue(), b = rangeEnd_->getValue();
        const OfxRangeD fr = src_->getFrameRange();
        const double fps = src_->getFrameRate() > 0 ? src_->getFrameRate() : 60.0;
        std::ostringstream o;
        if (a <= kUnset / 2 && b <= kUnset / 2) o << "whole clip (" << clock((fr.max - fr.min + 1) / fps) << ")";
        else o << (a > kUnset / 2 ? clock((a - fr.min) / fps) : std::string("start")) << " to "
               << (b > kUnset / 2 ? clock((b - fr.min) / fps) : std::string("end")) << " (+1 s margin)";
        rangeInfo_->setValue(o.str());
    }

    static bool solverUsable(int status) { return status == 1 || status == 2 || status == 7; }

    static std::string solverText(int status)
    {
        if (status == 1) return "OK";
        if (status == 2 || status == 7) return "OK (approx.)";   // solved inaccurate / iteration limit: usable path
        return "failed (status " + std::to_string(status) + ")";
    }

    void updateStatus(double t)
    {
        updateRangeInfo();   // clip length may have changed (speed)
        auto s = solution(t);
        std::ostringstream o;
        if (s->data.empty()) o << "Not analysed yet - set an interval (optional), then Run Analysis";
        else if (!s->error.empty()) o << "Stored analysis not readable (" << s->error << ") - please run the analysis again";
        else {
            const double fps = s->info.fps > 0 ? s->info.fps : 60.0;
            const OfxRangeD fr = src_->getFrameRange();
            if (!embeddedFits(*s)) o << "Clip changed (length/speed) - not stabilised, please run the analysis again. ";
            if (!solverUsable(s->result.status))
                o << "Camera path could not be calculated (solver status " << s->result.status
                  << ") - not stabilised. Try a smaller Smoothing Window or a larger Zoom. ";
            o << "Analysed " << clock((s->info.t0 - fr.min) / fps) << "–" << clock((s->info.t0 + double(s->track.size()) - 1 - fr.min) / fps)
              << " (" << s->track.size() << " frames), solver " << solverText(s->result.status)
              << ", max. correction " << std::fixed << std::setprecision(1) << s->result.maxCorrectionDeg << "°"
              << (s->result.clampedFrames ? ", " + std::to_string(s->result.clampedFrames) + " frames limited" : "");
            if (s->info.seconds > 0)
                o << ", analysis took " << clock(s->info.seconds) << " (" << std::setprecision(0)
                  << 1000.0 * s->info.seconds / double(s->track.size()) << " ms/frame)";
        }
        status_->setValue(o.str());
    }

    // ------------------------------------------------------------------ analysis (button)
    static void view(OFX::Image* im, ts::ImageView& v)
    {
        const OfxRectI bb = im->getBounds();
        v.base = static_cast<char*>(im->getPixelAddress(bb.x1, bb.y1)); v.rowBytes = im->getRowBytes();
        v.x1 = bb.x1; v.y1 = bb.y1; v.x2 = bb.x2; v.y2 = bb.y2;
    }

    void analyse(const OFX::InstanceChangedArgs& args)
    {
        const auto tStart = std::chrono::steady_clock::now();
        const OfxRangeD fr = src_->getFrameRange();
        const double fps = src_->getFrameRate() > 0 ? src_->getFrameRate() : 60.0;
        const double margin = std::round(fps);                       // 1 s on each side of the marked range
        const double a = rangeStart_->getValue(), b = rangeEnd_->getValue();
        double t0 = a > kUnset / 2 ? std::max(fr.min, std::round(a) - margin) : fr.min;
        double t1 = b > kUnset / 2 ? std::min(fr.max, std::round(b) + margin) : fr.max;
        if (t1 < t0) std::swap(t0, t1);
        const long N = long(std::floor(t1 - t0)) + 1;
        {
            std::ostringstream lg;
            lg << "analysis start " << kTrailStabVersion << ": clipRange=[" << fr.min << "," << fr.max << "] fps=" << fps << " -> t0=" << t0 << " N=" << N;
            logLine(lg.str());
        }
        if (N < 10) { status_->setValue("Interval too short for an analysis (at least 10 frames) - set a longer interval or Set Whole Clip"); return; }

        // where is the picture inside the frame?  union of the content boxes of 9 frames spread over
        // the interval (a few frames with dark edges must not shrink the picture)
        ts::RectI rect, full; bool haveRect = false;
        for (int si = 0; si < 9; ++si) {
            const double tt = t0 + std::round(double(N - 1) * si / 8.0);
            std::unique_ptr<OFX::Image> im(src_->fetchImage(tt));
            if (!im) continue;
            ts::ImageView v; view(im.get(), v);
            full = {v.x1, v.y1, v.x2, v.y2};
            const ts::RectI c = ts::findContentRect(v);
            if (c.empty()) continue;
            if (!haveRect) { rect = c; haveRect = true; }
            else { rect.x1 = std::min(rect.x1, c.x1); rect.y1 = std::min(rect.y1, c.y1); rect.x2 = std::max(rect.x2, c.x2); rect.y2 = std::max(rect.y2, c.y2); }
        }
        if (!haveRect) rect = full;
        if (rect.empty()) { logLine("analysis: source image not readable"); status_->setValue("Analysis failed: Resolve delivered no source image - please try again"); return; }
        {   // same rule as the render placement: content touching three frame edges of a 9:16 / 16:9
            // frame means "picture fills the frame" (dark rows inside the picture do not shrink it)
            const int touches = (rect.x1 <= full.x1) + (rect.x2 >= full.x2) + (rect.y1 <= full.y1) + (rect.y2 >= full.y2);
            const double fa = full.w() > 0 ? double(full.h()) / double(full.w()) : 0.0;
            const bool videoAspect = std::fabs(fa / (16.0 / 9.0) - 1.0) < 0.03 || std::fabs(fa / (9.0 / 16.0) - 1.0) < 0.03;
            if (touches >= 3 && videoAspect) rect = full;
        }

        ts::AnalysisParams ap;
        ap.width = std::max(240, std::min(1080, analysisWidth_->getValue()));
        // the analysis must never upsample: a reduced playback resolution would silently lower the quality
        if (rect.w() < ap.width) {
            std::ostringstream o; o << "Playback resolution too low for analysis (" << rect.w() << " px). Set Playback Resolution to a higher setting";
            logLine("analysis refused: " + o.str()); status_->setValue(o.str()); return;
        }
        const int W = ap.width, H = std::max(16, int(std::lround(double(W) * rect.h() / rect.w() / 2.0)) * 2);
        ts::Analyzer an(ap);

        // only the picture area is requested (canonical coordinates, 2 px margin)
        const double rsx = args.renderScale.x > 0 ? args.renderScale.x : 1.0, rsy = args.renderScale.y > 0 ? args.renderScale.y : 1.0;
        const OfxRectD region{(rect.x1 - 2) / rsx, (rect.y1 - 2) / rsy, (rect.x2 + 2) / rsx, (rect.y2 + 2) / rsy};
        bool useRegion = true;

        // pipeline: this thread fetches frames from Resolve and makes the grey image, a worker
        // thread runs the tracking at the same time (bounded queue of 3 frames)
        struct Queue {
            std::mutex m; std::condition_variable cv; std::deque<ts::GrayImage> q;
            bool done = false, stop = false; std::string err; size_t processed = 0;
        } Q;
        double addMs = 0;
        std::thread worker([&] {
            ts::setHighQoS();
            for (;;) {
                ts::GrayImage g;
                {
                    std::unique_lock<std::mutex> l(Q.m);
                    Q.cv.wait(l, [&] { return Q.stop || Q.done || !Q.q.empty(); });
                    if (Q.stop || (Q.q.empty() && Q.done)) return;
                    g = std::move(Q.q.front()); Q.q.pop_front();
                }
                Q.cv.notify_all();
                const auto a0 = std::chrono::steady_clock::now();
                try { an.addFrame(g); }
                catch (const std::exception& ex) { std::lock_guard<std::mutex> l(Q.m); Q.err = ex.what(); Q.stop = true; Q.cv.notify_all(); return; }
                addMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0).count();
                { std::lock_guard<std::mutex> l(Q.m); ++Q.processed; }
                Q.cv.notify_all();
            }
        });
        auto stopWorker = [&](bool abort) {
            { std::lock_guard<std::mutex> l(Q.m); if (abort) Q.stop = true; else Q.done = true; }
            Q.cv.notify_all();
            worker.join();
        };

        progressStart("Trail Stability: Analysis", "trailstab.analyse");
        double fetchMs = 0, grayMs = 0, waitMs = 0;
        bool aborted = false; std::string err;
        for (long i = 0; i < N; ++i) {
            const double tt = t0 + double(i);
            const auto q0 = std::chrono::steady_clock::now();
            std::unique_ptr<OFX::Image> im(useRegion ? src_->fetchImage(tt, region) : src_->fetchImage(tt));
            if (im && useRegion) {
                const OfxRectI bb = im->getBounds();
                if (bb.x1 > rect.x1 || bb.y1 > rect.y1 || bb.x2 < rect.x2 || bb.y2 < rect.y2) {   // host gave less than asked
                    debugLine("analysis: region fetch not usable, falling back to full frames");
                    useRegion = false; im.reset(src_->fetchImage(tt));
                }
            }
            const auto q1 = std::chrono::steady_clock::now();
            if (!im) { err = "Resolve delivered no image for frame " + std::to_string(i) + " - please try again"; break; }
            if (im->getPixelDepth() != OFX::eBitDepthFloat || im->getPixelComponents() != OFX::ePixelComponentRGBA) { err = "unexpected image format from Resolve (expected 32-bit float RGBA)"; break; }
            ts::ImageView v; view(im.get(), v);
            ts::GrayImage g = ts::grayFromRGBA(v, rect, W, H);
            im.reset();
            const auto q2 = std::chrono::steady_clock::now();
            {
                std::unique_lock<std::mutex> l(Q.m);
                Q.cv.wait(l, [&] { return Q.stop || Q.q.size() < 3; });
                if (Q.stop) { err = Q.err.empty() ? "analysis stopped unexpectedly" : Q.err; break; }
                Q.q.push_back(std::move(g));
            }
            Q.cv.notify_all();
            const auto q3 = std::chrono::steady_clock::now();
            fetchMs += std::chrono::duration<double, std::milli>(q1 - q0).count();
            grayMs += std::chrono::duration<double, std::milli>(q2 - q1).count();
            waitMs += std::chrono::duration<double, std::milli>(q3 - q2).count();
            size_t doneFrames; { std::lock_guard<std::mutex> l(Q.m); doneFrames = Q.processed; }
            if (!progressUpdate(double(doneFrames) / double(N))) { aborted = true; break; }
            if ((i + 1) % 250 == 0 && debugLog()) {
                std::ostringstream o; o << "analyse: " << (i + 1) << "/" << N << std::fixed << std::setprecision(1) << " fetch "
                                        << fetchMs / double(i + 1) << " ms, grey " << grayMs / double(i + 1) << " ms, waiting for tracking "
                                        << waitMs / double(i + 1) << " ms per frame";
                logLine(o.str());
            }
        }
        if (aborted || !err.empty()) stopWorker(true);
        else {
            // let the worker finish the queued frames (progress keeps moving)
            for (;;) {
                size_t doneFrames; bool stopped;
                { std::unique_lock<std::mutex> l(Q.m);
                  Q.cv.wait_for(l, std::chrono::milliseconds(100), [&] { return Q.stop || Q.processed >= size_t(N); });
                  doneFrames = Q.processed; stopped = Q.stop; if (stopped && !Q.err.empty()) err = Q.err; }
                if (!progressUpdate(double(doneFrames) / double(N))) { aborted = true; break; }
                if (stopped || doneFrames >= size_t(N)) break;
            }
            stopWorker(aborted);
            if (!aborted && err.empty() && an.frames() != size_t(N)) err = "analysis incomplete (" + std::to_string(an.frames()) + " of " + std::to_string(N) + " frames)";
        }
        progressEnd();
        if (aborted) { logLine("analysis cancelled"); status_->setValue("Analysis cancelled - previous analysis kept"); return; }
        if (!err.empty()) { logLine("analysis failed: " + err); status_->setValue("Analysis failed: " + err + " (previous analysis kept)"); return; }
        const double workMs = addMs;

        const auto tFin = std::chrono::steady_clock::now();
        const ts::CameraTrack tr = an.finish(fps, rect.w(), rect.h());
        const double finishMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tFin).count();
        const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
        ts::EmbeddedInfo info; info.t0 = t0; info.fps = fps; info.rangeMin = fr.min; info.rangeMax = fr.max; info.seconds = total;
        const std::string enc = ts::encodeTrack(tr, info);
        data_->setValue(enc);
        std::ostringstream o;
        o << std::fixed << std::setprecision(1) << "analysis done: " << N << " Frames in " << total << " s (fetch " << fetchMs / double(N)
          << " ms, grey " << grayMs / double(N) << " ms, tracking thread " << workMs / double(N) << " ms, main waited " << waitMs / double(N)
          << " ms per frame; region " << (useRegion ? "yes" : "no") << ", workers " << ts::workerCount() << "), picture=[" << rect.x1 << "," << rect.y1 << "," << rect.x2 << "," << rect.y2
          << "] -> " << W << "x" << H << ", points " << std::setprecision(0) << an.stats().meanPoints << ", quality " << std::setprecision(2)
          << an.stats().meanQuality << ", chains " << an.stats().chainsUsed << "/" << an.stats().chainsTried << ", stored " << enc.size() << " bytes";
        logLine(o.str());
        {
            const auto& s = an.stats(); const double n = double(N);
            std::ostringstream q; q << std::fixed << std::setprecision(2) << "analyse split (ms per frame): pyramid " << s.msPyramid / n << ", corners " << s.msDetect / n
                                    << ", tracking " << s.msTrack / n << ", rotation RANSAC " << s.msRansac / n << ", chains " << s.msChains / n
                                    << "; finish() total " << std::setprecision(0) << finishMs << " ms; workers " << ts::workerCount() << " of " << std::thread::hardware_concurrency() << " cores";
            debugLine(q.str());
        }
        std::string back; data_->getValue(back);
        if (back != enc) { logLine("WARNING: stored analysis differs (" + std::to_string(back.size()) + " instead of " + std::to_string(enc.size()) + " characters)");
                             status_->setValue("Analysis could not be stored in the project - please run the analysis again"); return; }
        updateStatus(args.time);
    }

    void changedParam(const OFX::InstanceChangedArgs& args, const std::string& name) override
    {
        if (name == "analyze") { analyse(args); return; }
        if (name == "setStart") { rangeStart_->setValue(args.time); updateRangeInfo(); return; }
        if (name == "setEnd") { rangeEnd_->setValue(args.time); updateRangeInfo(); return; }
        if (name == "clearRange") { rangeStart_->setValue(kUnset); rangeEnd_->setValue(kUnset); updateRangeInfo(); return; }
        if (name == "clearAnalysis") { data_->setValue(""); updateStatus(args.time); return; }
        if (name == "smoothWindow" || name == "zoom" || name == "levelHorizon" || name == "preciseSolver" || name == "analysisData")
            updateStatus(args.time);
    }

    /// the clip's length/speed as the host presents it changed since we last looked?
    /// (Resolve does not send a clip-changed event for a speed change, but asks for the clip
    /// preferences again; the status line is refreshed from there.)
    bool clipRangeChanged(const char* where)
    {
        const OfxRangeD fr = src_->getFrameRange();
        const double fps = src_->getFrameRate();
        std::lock_guard<std::mutex> l(clipMutex_);
        if (haveClip_ && fr.min == lastRange_.min && fr.max == lastRange_.max && fps == lastFps_) return false;
        const bool first = !haveClip_;
        lastRange_ = fr; lastFps_ = fps; haveClip_ = true;
        if (!first) {
            std::ostringstream o; o << "clip changed (" << where << "): range=[" << fr.min << "," << fr.max << "] fps=" << fps;
            debugLine(o.str());
        }
        return true;
    }

    void refreshAfterClipChange(const char* where, double t)
    {
        if (!clipRangeChanged(where)) return;
        try { updateStatus(t); }   // setting a parameter outside instanceChanged: tolerated by Resolve, guarded anyway
        catch (...) { logLine(std::string("status refresh not allowed in ") + where); }
    }

    void changedClip(const OFX::InstanceChangedArgs& args, const std::string& clipName) override
    {
        if (clipName == kOfxImageEffectSimpleSourceClipName) refreshAfterClipChange("clip changed", args.time);
    }

    void getClipPreferences(OFX::ClipPreferencesSetter&) override
    {
        refreshAfterClipChange("clip preferences", 0.0);
    }

    bool isIdentity(const OFX::IsIdentityArguments& args, OFX::Clip*& clip, double& t) override
    {
        return false;
    }

    // The warp may sample anywhere in the source frame -> always ask for all of it.
    void getRegionsOfInterest(const OFX::RegionsOfInterestArguments& args, OFX::RegionOfInterestSetter& rois) override
    {
        rois.setRegionOfInterest(*src_, src_->getRegionOfDefinition(args.time));
    }

    // ------------------------------------------------------------------ render
    static tsm::MetalImage metalImage(OFX::Image* im)
    {
        tsm::MetalImage m;
        const OfxRectI b = im->getBounds();
        m.buffer = im->getPixelData();   // id<MTLBuffer> when Metal is enabled
        m.x1 = b.x1; m.y1 = b.y1; m.x2 = b.x2; m.y2 = b.y2;
        m.rowBytes = im->getRowBytes();
        return m;
    }

    void gpuProblem(const std::string& what)
    {
        if (gpuErrors_++ < 5) logLine("Metal: " + what);
    }

    void render(const OFX::RenderArguments& args) override
    {
        using clk = std::chrono::steady_clock;
        auto ms = [](clk::time_point a, clk::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
        const auto r0 = clk::now();
        std::unique_ptr<OFX::Image> dst(dst_->fetchImage(args.time));
        std::unique_ptr<OFX::Image> src(src_->fetchImage(args.time));
        const auto r1 = clk::now();
        if (!dst || !src) OFX::throwSuiteStatusException(kOfxStatFailed);
        if (dst->getPixelDepth() != OFX::eBitDepthFloat || dst->getPixelComponents() != OFX::ePixelComponentRGBA ||
            src->getPixelDepth() != OFX::eBitDepthFloat || src->getPixelComponents() != OFX::ePixelComponentRGBA)
            OFX::throwSuiteStatusException(kOfxStatErrFormat);

        // Metal: images are GPU buffers, all pixel work goes onto Resolve's command queue
        const bool gpu = args.isEnabledMetalRender && args.pMetalCmdQ != nullptr;
        void* queue = args.pMetalCmdQ;
        ts::ImageView sv, dv;
        tsm::MetalImage msrc, mdst;
        auto view = [](OFX::Image* im, ts::ImageView& v) {
            const OfxRectI b = im->getBounds();
            v.base = static_cast<char*>(im->getPixelAddress(b.x1, b.y1));
            v.rowBytes = im->getRowBytes(); v.x1 = b.x1; v.y1 = b.y1; v.x2 = b.x2; v.y2 = b.y2;
        };
        if (gpu) { msrc = metalImage(src.get()); mdst = metalImage(dst.get()); }
        else { view(src.get(), sv); view(dst.get(), dv); }
        const OfxRectI w = args.renderWindow;
        const int win[4] = {w.x1, w.y1, w.x2, w.y2};

        auto s = solution(args.time);
        bool ok = s->error.empty() && !s->result.warp.empty() && solverUsable(s->result.status);
        size_t k = 0;
        double sec = 0;
        const OfxRangeD fr = src_->getFrameRange();
        const double fps = src_->getFrameRate() > 0 ? src_->getFrameRate() : 60.0;
        if (ok) {
            // embedded analysis: frame i was analysed at effect time t0 + i (the host's own grid,
            // including its frame repeats), so the lookup is direct.
            // outside the analysed interval the correction of its first/last frame is held, so the
            // zoom stays and the picture does not jump at the interval edge
            const long kk = std::lround(args.time - s->info.t0);
            ok = embeddedFits(*s) && !s->track.empty();
            if (ok) mismatchLogged_ = false;
            else if (!mismatchLogged_.exchange(true)) {   // once per change: why the picture is passed through
                std::ostringstream o;
                o << "render: analysis does not fit the clip -> not stabilised. clip range=[" << fr.min << "," << fr.max << "] fps=" << fps
                  << ", analysed range=[" << s->info.rangeMin << "," << s->info.rangeMax << "] fps=" << s->info.fps;
                logLine(o.str());
            }
            k = ok ? size_t(std::max(0L, std::min(long(s->track.size()) - 1, kk))) : 0;
            sec = double(kk) / (s->info.fps > 0 ? s->info.fps : fps);
        }
        if (!ok) {   // pass through
            if (gpu) {
                std::string e;
                if (!tsm::warp(queue, msrc, mdst, win, nullptr, ts::RectI{msrc.x1, msrc.y1, msrc.x2, msrc.y2}, true, &e)) gpuProblem("copy: " + e);
            } else {
                for (int y = w.y1; y < w.y2; ++y) std::memcpy(dv.px(w.x1, y), sv.px(w.x1, y), size_t(w.x2 - w.x1) * 16);
            }
            return;
        }

        // Where is the picture inside the frame? Resolve applies timeline fitting and Inspector
        // pan/zoom BEFORE Edit-page effects, so it is found from the image content.
        const OfxRectD rod = src_->getRegionOfDefinition(args.time);
        ts::RectI frame{int(std::lround(rod.x1 * args.renderScale.x)), int(std::lround(rod.y1 * args.renderScale.y)),
                        int(std::lround(rod.x2 * args.renderScale.x)), int(std::lround(rod.y2 * args.renderScale.y))};
        const double trackAspect = double(s->track.height) / double(s->track.width);
        ts::RectI content;
        // GPU: the picture search runs together with the warp and its result is used from the next
        // frame on (no waiting for the GPU). Only a new frame size/scale waits once for a fresh result.
        ContentCache::Key ckey{msrc.x1, msrc.y1, msrc.x2, msrc.y2, std::lround(args.renderScale.x * 10000.0)};
        const bool searchDue = gpu && contentCache_->due(ckey);
        if (gpu) {
            if (!contentCache_->get(ckey, content)) {
                std::string e;
                if (!tsm::detectContent(queue, msrc, content, &e)) gpuProblem("picture search: " + e);   // -> centred fallback
                else contentCache_->put(ckey, content);
            }
        } else content = ts::findContentRect(sv);
        const auto r2 = clk::now();
        ts::Placement pl;
        bool changed = false;
        {
            std::lock_guard<std::mutex> lock(placeMutex_);
            // keep the previous placement if the edges only flicker by a pixel or two
            // (dark frames, soft edges) so the picture centre does not jitter
            if (havePlace_ && lastScale_ == args.renderScale.x && nearlySame(content, lastContent_, 2)) pl = lastPlace_;
            else {
                pl = ts::placePicture(frame, content, trackAspect);
                changed = !havePlace_ || !nearlySame(content, lastContent_, 2) || lastScale_ != args.renderScale.x;
                lastPlace_ = pl; lastContent_ = content; lastScale_ = args.renderScale.x; havePlace_ = true;
            }
        }

        double Hm[9];
        ts::cameraHomography(s->result.warp[k], s->track.focalNorm * pl.width, s->params.zoom, pl.cx, pl.cy, Hm);
        if (gpu) {
            std::string e;
            std::shared_ptr<ContentCache> cache = contentCache_;
            std::function<void(const ts::RectI&)> onContent;   // picture search only every 8th frame
            if (searchDue) onContent = [cache, ckey](const ts::RectI& r) { cache->put(ckey, r); };
            if (!tsm::warpAndSearch(queue, msrc, mdst, win, Hm, pl.rect, false, onContent, &e)) gpuProblem("warp: " + e);
            const std::string ae = tsm::takeAsyncError();
            if (!ae.empty()) gpuProblem(ae);
        } else {
            ts::warpBicubic(sv, dv, w.x1, w.y1, w.x2, w.y2, Hm, 0, &pl.rect);
        }
        const auto r3 = clk::now();

        const long n = renders_++;
        if (args.renderScale.x >= 0.99 && debugLog()) {   // timing of full-size renders (thumbnails excluded)
            std::lock_guard<std::mutex> lock(statMutex_);
            Stat& st = gpu ? statGpu_ : statCpu_;
            st.fetch += ms(r0, r1); st.search += ms(r1, r2); st.warp += ms(r2, r3); ++st.n;
            if (st.n >= 300) {
                std::ostringstream o;
                o << std::fixed << std::setprecision(1) << "render timing " << (gpu ? "METAL" : "CPU") << " (" << st.n << " frames, "
                  << (w.x2 - w.x1) << "x" << (w.y2 - w.y1) << "): fetch " << st.fetch / st.n << " ms, picture search "
                  << st.search / st.n << " ms" << (gpu ? " (GPU: every 8th frame, waits only on size change)" : "") << ", warp " << st.warp / st.n
                  << " ms" << (gpu ? " (encode only)" : "") << ", plugin total " << (st.fetch + st.search + st.warp) / st.n
                  << " ms per frame";
                logLine(o.str());
                st = Stat();
            }
        }
        if (debugLog() && (n < 20 || changed || n % 250 == 0)) {
            std::ostringstream o;
            o << std::fixed << std::setprecision(3)
              << "render#" << n << " " << (gpu ? "METAL" : "CPU") << " time=" << args.time << " clipRange=[" << fr.min << "," << fr.max << "]"
              << " clipFps=" << fps << " effectFps=" << getFrameRate() << " effectDur=" << getEffectDuration()
              << " -> sec=" << sec << " k=" << k
              << " | scale=" << args.renderScale.x << " rod=[" << rod.x1 << "," << rod.y1 << "," << rod.x2 << "," << rod.y2 << "]"
              << " bounds=[" << (gpu ? msrc.x1 : sv.x1) << "," << (gpu ? msrc.y1 : sv.y1) << "," << (gpu ? msrc.x2 : sv.x2) << ","
              << (gpu ? msrc.y2 : sv.y2) << "] rowBytes=" << (gpu ? msrc.rowBytes : long(sv.rowBytes))
              << " window=[" << w.x1 << "," << w.y1 << "," << w.x2 << "," << w.y2 << "]"
              << " content=[" << content.x1 << "," << content.y1 << "," << content.x2 << "," << content.y2 << "]"
              << " picture=" << (pl.detected ? "found" : "FALLBACK-centred") << " c=(" << pl.cx << "," << pl.cy
              << ") w=" << pl.width << " zoom=" << s->params.zoom << std::setprecision(1)
              << " | ms fetch " << ms(r0, r1) << " search " << ms(r1, r2) << " warp " << ms(r2, r3);
            logLine(o.str());
        }
    }

private:
    OFX::Clip *dst_ = nullptr, *src_ = nullptr;
    OFX::StringParam *status_ = nullptr, *data_ = nullptr, *rangeInfo_ = nullptr;
    OFX::DoubleParam *rangeStart_ = nullptr, *rangeEnd_ = nullptr;
    OFX::DoubleParam *smooth_ = nullptr, *zoom_ = nullptr;
    OFX::BooleanParam *level_ = nullptr;
    OFX::IntParam *analysisWidth_ = nullptr;
    OFX::BooleanParam *precise_ = nullptr;
    std::mutex mutex_;
    std::shared_ptr<const Solution> cache_;
    std::mutex placeMutex_;
    bool havePlace_ = false;
    double lastScale_ = 0;
    ts::RectI lastContent_;
    ts::Placement lastPlace_;
    std::atomic<long> renders_{0};
    std::mutex statMutex_;
    struct Stat { double fetch = 0, search = 0, warp = 0; int n = 0; };
    Stat statCpu_, statGpu_;
    std::atomic<int> gpuErrors_{0};
    std::atomic<bool> mismatchLogged_{false};
    std::mutex clipMutex_;
    bool haveClip_ = false;
    OfxRangeD lastRange_{0, 0};
    double lastFps_ = 0;
    std::shared_ptr<ContentCache> contentCache_ = std::make_shared<ContentCache>();
};

mDeclarePluginFactory(TrailStabFactory, {}, {});

void TrailStabFactory::describe(OFX::ImageEffectDescriptor& d)
{
    d.setLabels(kPluginName, kPluginName, kPluginName);
    d.setPluginGrouping(kPluginGrouping);
    d.setPluginDescription("3D rotation stabilizer for iPhone Action Mode footage (gravity-aware, crop-constrained).");
    d.addSupportedContext(OFX::eContextFilter);
    d.addSupportedBitDepth(OFX::eBitDepthFloat);
    d.setSingleInstance(false);
    d.setHostFrameThreading(false);
    d.setSupportsMultiResolution(true);
    d.setSupportsTiles(false);
    d.setTemporalClipAccess(true);    // the analysis fetches other frames of the source clip
    d.setRenderThreadSafety(OFX::eRenderFullySafe);
#ifdef __APPLE__
    d.setSupportsMetalRender(true);   // images as Metal buffers on Resolve's command queue (metal_render.mm)
#endif
}

void TrailStabFactory::describeInContext(OFX::ImageEffectDescriptor& d, OFX::ContextEnum)
{
    OFX::ClipDescriptor* src = d.defineClip(kOfxImageEffectSimpleSourceClipName);
    src->addSupportedComponent(OFX::ePixelComponentRGBA);
    src->setSupportsTiles(false);
    OFX::ClipDescriptor* dst = d.defineClip(kOfxImageEffectOutputClipName);
    dst->addSupportedComponent(OFX::ePixelComponentRGBA);
    dst->setSupportsTiles(false);

    OFX::PageParamDescriptor* page = d.definePageParam("Controls");
    auto add = [&](OFX::ParamDescriptor* p) { if (page) page->addChild(*p); };

    // ---- interval + analysis
    auto* rs = d.definePushButtonParam("setStart");
    rs->setLabels("Set Playhead as Start", "Set Playhead as Start", "Set Playhead as Start");
    rs->setHint("Optional: start of the part you use (playhead). Saves analysis time on long clips; 1 s margin is added.");
    rs->getPropertySet().propSetInt("OfxParamPropLayoutHint", 2, false);   // next control on the same row
    add(rs);
    auto* re = d.definePushButtonParam("setEnd");
    re->setLabels("Set Playhead as End", "Set Playhead as End", "Set Playhead as End");
    re->setHint("Optional: end of the part you use (playhead)."); add(re);
    auto* rc = d.definePushButtonParam("clearRange");
    rc->setLabels("Set Whole Clip", "Set Whole Clip", "Set Whole Clip");
    rc->setHint("Reset the interval: the whole clip is analysed."); add(rc);
    auto* ri = d.defineStringParam("rangeInfo");
    ri->setLabels("Interval", "Interval", "Interval");
    ri->setStringType(OFX::eStringTypeLabel); ri->setDefault("whole clip");
    ri->setEvaluateOnChange(false); add(ri);
    auto* an = d.definePushButtonParam("analyze");
    an->setLabels("Run Analysis", "Run Analysis", "Run Analysis");
    an->setHint("Analyses the interval and stores the result in the project. "
                "Takes about 30 s per 1000 frames; Resolve does not respond until it is finished. Needs Playback Resolution Half or Full."); add(an);
    for (const char* nm : {"rangeStart", "rangeEnd"}) {
        auto* r = d.defineDoubleParam(nm);
        r->setDefault(kUnset); r->setRange(-1e12, 1e12); r->setAnimates(false);
        r->setIsSecret(true); r->setEvaluateOnChange(false);
    }
    auto* ad = d.defineStringParam("analysisData");
    ad->setLabels("Analysis Data", "Analysis Data", "Analysis Data");
    ad->setStringType(OFX::eStringTypeSingleLine); ad->setDefault(""); ad->setAnimates(false);
    ad->setIsSecret(true);

    // ---- stabilisation
    auto* sm = d.defineDoubleParam("smoothWindow");
    sm->setLabels("Smoothing Window (s)", "Smoothing Window", "Smoothing");
    sm->setDefault(1.7); sm->setRange(0.15, 14.0); sm->setDisplayRange(0.3, 6.0);
    sm->setAnimates(false);   // one camera path per clip: keyframes would re-solve the whole path per frame
    sm->setHint("Camera motion shorter than about this window is smoothed out, longer moves (head turns) are kept. "
                "Larger = calmer, needs more zoom."); add(sm);

    auto* zm = d.defineDoubleParam("zoom");
    zm->setLabels("Zoom (Crop Budget)", "Zoom", "Zoom");
    zm->setDefault(1.25); zm->setRange(1.0, 2.0); zm->setDisplayRange(1.0, 1.5); zm->setAnimates(false);
    zm->setHint("How far the picture is zoomed in. The camera path always stays inside this frame."); add(zm);

    auto* lv = d.defineBooleanParam("levelHorizon");
    lv->setLabels("Level Horizon", "Level Horizon", "Level Horizon");
    lv->setDefault(true); lv->setAnimates(false);
    lv->setHint("Keeps the virtual camera level (gravity estimated from the footage)."); add(lv);

    // ---- developer options (collapsed group)
    auto* dev = d.defineGroupParam("devGroup");
    dev->setLabels("Developer", "Developer", "Developer");
    dev->setOpen(false);
    if (page) page->addChild(*dev);
    auto* aw = d.defineIntParam("analysisWidth");
    aw->setLabels("Analysis Width (px)", "Analysis Width", "Analysis Width");
    aw->setDefault(540); aw->setRange(360, 720); aw->setDisplayRange(360, 720);
    aw->setHint("Width of the grey image the analysis works on. Smaller = faster, less precise. Takes effect on the next Run Analysis.");
    aw->setEvaluateOnChange(false); aw->setParent(*dev);
    auto* ps = d.defineBooleanParam("preciseSolver");
    ps->setLabels("Precise Solver (slower)", "Precise Solver", "Precise Solver");
    ps->setDefault(false); ps->setAnimates(false);
    ps->setHint("Camera path with 10x tighter solver tolerance: about 3x longer path calculation (not the analysis), "
                "path differs by less than 0.02 px. Use if the status shows 'OK (approx.)'.");
    ps->setParent(*dev);
    auto* clr = d.definePushButtonParam("clearAnalysis");
    clr->setLabels("Delete Analysis", "Delete Analysis", "Delete Analysis");
    clr->setHint("Removes the analysis stored in this effect (the clip is shown unstabilised)."); clr->setParent(*dev);

    auto* ver = d.defineStringParam("version");
    ver->setLabels("Version", "Version", "Version");
    ver->setStringType(OFX::eStringTypeLabel);
    ver->setDefault(kTrailStabVersion " | Build " __DATE__ " " __TIME__);
    ver->setEvaluateOnChange(false); ver->setIsSecret(false); add(ver);

    auto* st = d.defineStringParam("status");
    st->setLabels("Status", "Status", "Status");
    st->setStringType(OFX::eStringTypeLabel); st->setDefault("Not analysed yet - set an interval (optional), then Run Analysis");
    st->setEvaluateOnChange(false); add(st);
}

OFX::ImageEffect* TrailStabFactory::createInstance(OfxImageEffectHandle h, OFX::ContextEnum)
{
    return new TrailStabPlugin(h);
}

namespace OFX { namespace Plugin {
void getPluginIDs(OFX::PluginFactoryArray& ids)
{
    static TrailStabFactory p(kPluginIdentifier, 1, 0);   // OFX plugin version 1.0 (major change = new plugin for hosts)
    ids.push_back(&p);
}
}} // namespace OFX::Plugin
