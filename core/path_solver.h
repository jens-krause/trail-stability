// Virtual camera path: smoothest path inside the crop corridor (quadratic program).
#pragma once
#include "geometry.h"
#include "sidecar.h"
#include <string>
#include <vector>

namespace ts {

struct SolveParams {
    double smoothness = 0.6;    // time constant in seconds (higher = calmer)
    double zoom = 1.25;         // crop budget: 1.25 = output shows 80% of width/height
    bool   levelHorizon = true; // virtual camera roll -> 0 (w.r.t. estimated gravity)
    double levelWeight = 1e4;
    double vertical = 0.0;      // 0..1: how much of the vertical bounce (track.V) is stabilised (experimental)
    double denoiseFrames = 1.0; // Gaussian sigma in frames on measured pitch and roll: their frame-to-frame
                                // part is mostly estimation noise (translation parallax), yaw stays exact
    bool   precise = false;     // solver tolerance 1e-6 instead of 1e-5 (about 3x slower, path differs < 0.02 px)
    double verticalHpSec = 1.0; // only motion faster than this time scale is treated as bounce (avoids drift)
};

struct SolveResult {
    std::vector<Vec3> S;        // virtual camera orientation (yaw, pitch, roll)
    std::vector<Mat3> warp;     // per frame: maps virtual-camera bearings to input-camera bearings
    int status = -1;            // OSQP status (1 = solved)
    int clampedFrames = 0;      // frames shrunk by the exact corner check
    double maxCorrectionDeg = 0;
    std::string message;
};

SolveResult solvePath(const CameraTrack& track, const SolveParams& p);

/// exact check: does the output frame (zoomed) stay inside the input frame?
bool cornersInside(const Mat3& warp, double focalNorm, double aspectHW, double zoom);

} // namespace ts
