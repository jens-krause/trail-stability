// Non-Apple builds: no Metal path (the host never enables it, because it is not declared).
#include "metal_render.h"

namespace tsm {
bool available() { return false; }
std::string takeAsyncError() { return std::string(); }
bool detectContent(void*, const MetalImage&, ts::RectI& out, std::string* err)
{
    out = ts::RectI();
    if (err) *err = "Metal not available in this build";
    return false;
}
bool warp(void*, const MetalImage&, const MetalImage&, const int*, const double*, const ts::RectI&, bool, std::string* err)
{
    if (err) *err = "Metal not available in this build";
    return false;
}
bool warpAndSearch(void*, const MetalImage&, const MetalImage&, const int*, const double*, const ts::RectI&, bool,
                   std::function<void(const ts::RectI&)>, std::string* err)
{
    if (err) *err = "Metal not available in this build";
    return false;
}
} // namespace tsm
