// CPU image warp (float RGBA) with bicubic sampling. GPU (Metal) version follows the same math.
#pragma once
#include "geometry.h"
#include <cstddef>

namespace ts {

/// Image view in OFX convention: pixel (x,y), y grows upwards, rows may have any stride.
struct ImageView {
    char* base = nullptr;          // address of pixel (x1, y1)
    std::ptrdiff_t rowBytes = 0;
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    float* px(int x, int y) const { return reinterpret_cast<float*>(base + (y - y1) * rowBytes + std::ptrdiff_t(x - x1) * 16); }
};

/// Integer pixel rectangle [x1,x2) x [y1,y2) in OFX pixel coordinates.
struct RectI {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    int w() const { return x2 - x1; }
    int h() const { return y2 - y1; }
    bool empty() const { return x2 <= x1 || y2 <= y1; }
};

/// Where the analysed picture sits inside the frame the host hands to the effect.
/// Needed because Resolve applies Inspector transforms (pan/zoom) and timeline fitting
/// (pillar-/letterbox) BEFORE an Edit-page effect, so the picture is not necessarily
/// centred and not necessarily as wide as the frame.
struct Placement {
    double cx = 0, cy = 0;   // picture centre, OFX pixel coords (y up)
    double width = 0;        // picture width in pixels (focal length scales with it)
    RectI rect;              // picture area; output outside of it is transparent
    bool detected = false;   // true: from image content, false: centred fallback
};

/// Bounding box of pixels with visible content (max(r,g,b) > eps). Empty if nothing found.
RectI findContentRect(const ImageView& img, float eps = 1e-4f);

/// Use the content rectangle if its aspect ratio matches the analysed picture (within tol),
/// otherwise fall back to a picture fitted and centred inside the frame.
Placement placePicture(const RectI& frame, const RectI& content, double pictureAspectHW, double tol = 0.03);

/// Pixel homography for one frame. Maps OUTPUT pixel coords (OFX, y up) to INPUT pixel coords.
///   R        : rotation virtual camera -> input camera (from the path solver)
///   focalPx  : focal length of the input picture in pixels (focalNorm * picture width)
///   zoom     : output focal = focalPx * zoom
///   cx, cy   : principal point (picture centre) in OFX pixel coords
void cameraHomography(const Mat3& R, double focalPx, double zoom, double cx, double cy, double H[9]);

/// Convenience: picture centred in a frame of frameW x frameH pixels (origin 0,0).
void frameHomography(const Mat3& R, double focalNorm, double zoom, int frameW, int frameH,
                     double activeW, double H[9]);

/// Warp src into dst over the window [wx1,wx2) x [wy1,wy2).
/// If `valid` is given, output pixels outside of it are transparent black and samples are only
/// taken from inside of it (so pillarbox black never bleeds into the picture and the zoomed
/// picture never spills over its own area). Outside of src -> transparent black.
void warpBicubic(const ImageView& src, const ImageView& dst, int wx1, int wy1, int wx2, int wy2,
                 const double H[9], int threads = 0, const RectI* valid = nullptr);

} // namespace ts
