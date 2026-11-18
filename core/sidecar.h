// Camera track as produced by the analysis (core/analysis.cpp) and its storage formats.
//
// File format (".tstab", used by tools/analyze_cli and tools/dump_*; UTF-8 text, '#' comments):
//   TRAILSTAB 1
//   source IMG_1853.mov
//   width 1584            # display orientation (portrait) of the analysed image
//   height 2816
//   focal_norm 1.0185     # focal length / width
//   frames 1093
//   <t_seconds> <yaw> <pitch> <roll> <quality> [<vert>]   # one line per frame, angles in rad
//   vert (optional): integrated vertical image motion that the rotation does not explain
//        (running bounce, near-field parallax), in rad (= pixels / focal length).
#pragma once
#include "geometry.h"
#include <string>
#include <vector>

namespace ts {

struct CameraTrack {
    std::string source;
    int width = 0, height = 0;      // display orientation
    double focalNorm = 1.0;         // f / width
    std::vector<double> t;          // seconds
    std::vector<Vec3> E;            // estimated input orientation (yaw, pitch, roll)
    std::vector<float> quality;     // 0..1 tracking confidence
    std::vector<double> V;          // optional: integrated vertical residual motion [rad]; empty if absent

    size_t size() const { return t.size(); }
    bool empty() const { return t.empty(); }
    double nominalFps = 60.0;       // frame grid the host uses (from the median frame spacing)

    /// index of the frame closest to a source time in seconds (uses the stored timestamps)
    size_t frameAt(double seconds) const;
    /// source frame Resolve shows at this time (seconds since the clip start): nearest stored
    /// timestamp, ties to the later frame.  iPhone files drift (59.97 vs 60 fps); once the drift
    /// exceeds half a frame Resolve repeats a frame, and so must the plugin.
    size_t frameIndexAt(double secondsFromStart) const;
};

bool loadTrack(const std::string& path, CameraTrack& out, std::string* error = nullptr);
bool saveTrack(const std::string& path, const CameraTrack& tr);

/// Analysis stored inside the Resolve project (hidden string parameter of the effect).
/// Frames are indexed on the effect's own time grid: frame i = effect time t0 + i.
struct EmbeddedInfo {
    double t0 = 0;            // effect time of frame 0
    double fps = 0;           // effect frame rate at analysis time
    double rangeMin = 0, rangeMax = 0;   // clip frame range at analysis time (detects retime changes)
    double seconds = 0;       // how long the analysis took (shown in the status)
};
/// compact text: header + base64 (angles quantised to 1e-5 rad, deltas as int16)
std::string encodeTrack(const CameraTrack& tr, const EmbeddedInfo& info);
bool decodeTrack(const std::string& s, CameraTrack& tr, EmbeddedInfo& info, std::string* err = nullptr);

} // namespace ts
