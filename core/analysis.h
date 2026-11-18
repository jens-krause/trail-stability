// Camera-track analysis inside the plugin (C++ port of the original Python prototype, no OpenCV).
//
// Pipeline per frame pair: Shi-Tomasi corners (4x8 cells) -> pyramidal Lucas-Kanade with
// forward/backward check -> robust 3D rotation (2-point RANSAC + IRLS).  Over 10-frame chains:
// relative pose (rotation + translation direction) -> slow bias correction of the rotations.
// Then gravity EKF (forward/backward) and fusion into Euler angles (yaw, pitch, roll) in the
// same conventions as the .tstab files, so the path solver and warp are unchanged.
//
// Streaming: frames are pushed one by one; only the previous frame is kept in memory.
#pragma once
#include "geometry.h"
#include "sidecar.h"
#include "warp.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace ts {

/// 8-bit-range float grey image, top-down rows (y grows downwards), tightly packed.
struct GrayImage {
    int w = 0, h = 0;
    std::vector<float> p;
    float at(int x, int y) const { return p[size_t(y) * size_t(w) + size_t(x)]; }
    float& at(int x, int y) { return p[size_t(y) * size_t(w) + size_t(x)]; }
};

/// Area-averaged grey (luma, 0..255) of the picture rectangle `rect` of an RGBA float OFX image
/// (y up), resampled to outW x outH with top-down rows.
GrayImage grayFromRGBA(const ImageView& img, const RectI& rect, int outW, int outH);

/// Same from an 8-bit grey buffer (tests / command line tool).
GrayImage grayFromU8(const uint8_t* data, int w, int h);

struct AnalysisParams {
    int width = 540;                 // analysis width in pixels (height follows the aspect)
    double focalNorm = 550.0 / 540.0;// focal length / picture width (iPhone 14 Pro Action Mode)
    int cellsX = 4, cellsY = 8, perCell = 30;
    double cornerQuality = 0.005, minDistance = 8;
    int lkWin = 21, lkLevels = 4, lkIters = 30;
    double lkEps = 0.01, fbMax = 0.5;
    double rotThrPx = 1.0;           // RANSAC inlier threshold at width 540
    int rotIters = 150;
    int chainLen = 10, chainStride = 5;
    double chainThrPx = 0.7;
    int threads = 0;                 // 0 = hardware concurrency
};

struct AnalysisStats {
    size_t frames = 0;
    double meanPoints = 0, meanQuality = 0;
    int chainsUsed = 0, chainsTried = 0;
    // accumulated wall times in ms (diagnostics)
    double msPyramid = 0, msDetect = 0, msTrack = 0, msRansac = 0, msChains = 0, msFinish = 0;
};

class Analyzer {
public:
    explicit Analyzer(const AnalysisParams& p = AnalysisParams());
    ~Analyzer();
    /// next frame (analysis size: params.width x height, all frames the same size)
    void addFrame(const GrayImage& g);
    /// finish: E (yaw, pitch, roll), quality, t = i / fps.  width/height = picture aspect.
    CameraTrack finish(double fps, int pictureW, int pictureH);
    size_t frames() const;
    const AnalysisStats& stats() const;
private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

// ---- building blocks (exposed for tests)
struct Pt { float x, y; };
std::vector<Pt> detectCorners(const GrayImage& g, const AnalysisParams& p);
/// pyramidal LK forward+backward (as used in the analysis), for tests
void trackPoints(const GrayImage& a, const GrayImage& b, const std::vector<Pt>& p0, std::vector<char>& ok,
                 std::vector<Pt>& p1, const AnalysisParams& p);
/// Weighted rotation R minimising sum w |b - R a|^2 (a, b unit vectors).
Mat3 kabsch(const std::vector<Vec3>& a, const std::vector<Vec3>& b, const std::vector<double>* w = nullptr);
/// rotation vector -> matrix
Mat3 rotmat(const Vec3& w);

} // namespace ts
