// Metal render path – see metal_render.h. Compiled with ARC (-fobjc-arc).
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include "metal_render.h"
#include <algorithm>
#include <cstring>
#include <map>
#include <mutex>

namespace tsm {
namespace {

struct Pipelines {
    id<MTLComputePipelineState> warp = nil, cols = nil, rows = nil;
    std::string error;
};

std::mutex gMutex;
std::map<void*, Pipelines> gCache;   // per MTLDevice; entries are never removed
std::mutex gErrMutex;
std::string gAsyncError;

std::string nsErr(NSError* e)
{
    if (!e) return "unknown error";
    return std::string([[e localizedDescription] UTF8String] ?: "unknown error");
}

/// pipelines for the device, compiled once from the embedded source
const Pipelines& pipelines(id<MTLDevice> dev)
{
    std::lock_guard<std::mutex> lock(gMutex);
    void* key = (__bridge void*)dev;
    auto it = gCache.find(key);
    if (it != gCache.end()) return it->second;
    Pipelines p;
    @autoreleasepool {
        NSError* err = nil;
        id<MTLLibrary> lib = [dev newLibraryWithSource:[NSString stringWithUTF8String:kMetalSource] options:nil error:&err];
        if (!lib) p.error = "Metal compile: " + nsErr(err);
        else {
            auto make = [&](const char* name) -> id<MTLComputePipelineState> {
                id<MTLFunction> fn = [lib newFunctionWithName:[NSString stringWithUTF8String:name]];
                if (!fn) { p.error = std::string("Metal function missing: ") + name; return nil; }
                NSError* e2 = nil;
                id<MTLComputePipelineState> ps = [dev newComputePipelineStateWithFunction:fn error:&e2];
                if (!ps) p.error = std::string("Metal pipeline ") + name + ": " + nsErr(e2);
                return ps;
            };
            p.warp = make("ts_warp");
            p.cols = make("ts_detect_cols");
            p.rows = make("ts_detect_rows");
        }
    }
    return gCache.emplace(key, p).first->second;
}

MTLSize groupSize(id<MTLComputePipelineState> ps)
{
    const NSUInteger w = ps.threadExecutionWidth;
    const NSUInteger h = w > 0 ? ps.maxTotalThreadsPerThreadgroup / w : 1;
    return MTLSizeMake(w > 0 ? w : 1, h > 0 ? h : 1, 1);
}

void watch(id<MTLCommandBuffer> cb)
{
    [cb addCompletedHandler:^(id<MTLCommandBuffer> done) {
        if (done.status == MTLCommandBufferStatusError) {
            std::lock_guard<std::mutex> l(gErrMutex);
            gAsyncError = "Metal command buffer failed: " + nsErr(done.error);
        }
    }];
}

/// last resort if the kernels are not available: copy the overlapping rows unchanged
void blitCopy(id<MTLCommandQueue> q, const MetalImage& src, const MetalImage& dst)
{
    id<MTLBuffer> s = (__bridge id<MTLBuffer>)src.buffer;
    id<MTLBuffer> d = (__bridge id<MTLBuffer>)dst.buffer;
    const long sr = src.rowBytes > 0 ? src.rowBytes : long(src.x2 - src.x1) * 16;
    const long dr = dst.rowBytes > 0 ? dst.rowBytes : long(dst.x2 - dst.x1) * 16;
    const int x1 = std::max(src.x1, dst.x1), x2 = std::min(src.x2, dst.x2);
    const int y1 = std::max(src.y1, dst.y1), y2 = std::min(src.y2, dst.y2);
    if (x2 <= x1 || y2 <= y1) return;
    id<MTLCommandBuffer> cb = [q commandBuffer];
    cb.label = @"TrailStab copy";
    id<MTLBlitCommandEncoder> b = [cb blitCommandEncoder];
    for (int y = y1; y < y2; ++y)
        [b copyFromBuffer:s sourceOffset:NSUInteger((y - src.y1) * sr + long(x1 - src.x1) * 16)
                 toBuffer:d destinationOffset:NSUInteger((y - dst.y1) * dr + long(x1 - dst.x1) * 16)
                     size:NSUInteger(long(x2 - x1) * 16)];
    [b endEncoding];
    watch(cb);
    [cb commit];
}

/// content rectangle from the column/row flags written by the search kernels
ts::RectI rectFromFlags(const uint32_t* f, const MetalImage& img)
{
    const int W = img.x2 - img.x1, H = img.y2 - img.y1;
    int a = 0, b = W - 1;
    while (a < W && !f[a]) ++a;
    while (b >= 0 && !f[b]) --b;
    if (a > b) return ts::RectI();
    ts::RectI r;
    r.x1 = img.x1 + a; r.x2 = img.x1 + b + 1;
    a = 0; b = H - 1;
    while (a < H && !f[W + a]) ++a;
    while (b >= 0 && !f[W + b]) --b;
    if (a > b) return ts::RectI();
    r.y1 = img.y1 + a; r.y2 = img.y1 + b + 1;
    return r;
}

/// encode the two search passes; returns the flag buffer (nil on failure)
id<MTLBuffer> encodeSearch(id<MTLCommandQueue> q, id<MTLComputeCommandEncoder> enc, const Pipelines& P, id<MTLBuffer> buf,
                           const MetalImage& img)
{
    const int W = img.x2 - img.x1, H = img.y2 - img.y1;
    if (W <= 0 || H <= 0) return nil;
    DetectParams dp{};
    dp.w = W; dp.h = H; dp.row = int(img.rowBytes > 0 ? img.rowBytes / 16 : W); dp.eps = 1e-4f;
    id<MTLBuffer> flags = [q.device newBufferWithLength:NSUInteger(W + H) * 4 options:MTLResourceStorageModeShared];
    if (!flags) return nil;
    std::memset(flags.contents, 0, size_t(W + H) * 4);
    [enc setBytes:&dp length:sizeof(dp) atIndex:0];
    [enc setBuffer:buf offset:0 atIndex:1];
    [enc setBuffer:flags offset:0 atIndex:2];
    [enc setComputePipelineState:P.cols];
    [enc dispatchThreads:MTLSizeMake(NSUInteger(W), NSUInteger((H + 3) / 4), 1) threadsPerThreadgroup:groupSize(P.cols)];
    [enc setComputePipelineState:P.rows];
    [enc dispatchThreads:MTLSizeMake(NSUInteger((W + 3) / 4), NSUInteger(H), 1) threadsPerThreadgroup:groupSize(P.rows)];
    return flags;
}

} // namespace

bool available() { return true; }

std::string takeAsyncError()
{
    std::lock_guard<std::mutex> l(gErrMutex);
    std::string e; e.swap(gAsyncError);
    return e;
}

bool detectContent(void* commandQueue, const MetalImage& img, ts::RectI& out, std::string* err)
{
    out = ts::RectI();
    @autoreleasepool {
        id<MTLCommandQueue> q = (__bridge id<MTLCommandQueue>)commandQueue;
        id<MTLBuffer> buf = (__bridge id<MTLBuffer>)img.buffer;
        if (!q || !buf) { if (err) *err = "no Metal queue/buffer"; return false; }
        const Pipelines& P = pipelines(q.device);
        if (!P.cols || !P.rows) { if (err) *err = P.error; return false; }
        if (img.x2 <= img.x1 || img.y2 <= img.y1) return true;
        id<MTLCommandBuffer> cb = [q commandBuffer];
        cb.label = @"TrailStab picture search";
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        id<MTLBuffer> flags = encodeSearch(q, enc, P, buf, img);
        [enc endEncoding];
        if (!flags) { if (err) *err = "Metal: no flag buffer"; return false; }
        [cb commit];
        [cb waitUntilCompleted];
        if (cb.status == MTLCommandBufferStatusError) { if (err) *err = "Metal picture search: " + nsErr(cb.error); return false; }
        out = rectFromFlags(static_cast<const uint32_t*>(flags.contents), img);
    }
    return true;
}

bool warp(void* commandQueue, const MetalImage& src, const MetalImage& dst, const int window[4],
          const double H[9], const ts::RectI& valid, bool passThrough, std::string* err)
{
    @autoreleasepool {
        id<MTLCommandQueue> q = (__bridge id<MTLCommandQueue>)commandQueue;
        id<MTLBuffer> s = (__bridge id<MTLBuffer>)src.buffer;
        id<MTLBuffer> d = (__bridge id<MTLBuffer>)dst.buffer;
        if (!q || !s || !d) { if (err) *err = "no Metal queue/buffer"; return false; }
        const Pipelines& P = pipelines(q.device);
        if (!P.warp) {
            blitCopy(q, src, dst);
            if (err) *err = P.error;
            return false;
        }
        const WarpParams wp = makeWarpParams(src, dst, window, H, valid, passThrough);
        if (wp.wx2 <= wp.wx1 || wp.wy2 <= wp.wy1) return true;
        id<MTLCommandBuffer> cb = [q commandBuffer];
        cb.label = @"TrailStab warp";
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:P.warp];
        [enc setBytes:&wp length:sizeof(wp) atIndex:0];
        [enc setBuffer:s offset:0 atIndex:1];
        [enc setBuffer:d offset:0 atIndex:2];
        [enc dispatchThreads:MTLSizeMake(NSUInteger(wp.wx2 - wp.wx1), NSUInteger(wp.wy2 - wp.wy1), 1)
       threadsPerThreadgroup:groupSize(P.warp)];
        [enc endEncoding];
        watch(cb);
        [cb commit];
    }
    return true;
}

bool warpAndSearch(void* commandQueue, const MetalImage& src, const MetalImage& dst, const int window[4],
                   const double H[9], const ts::RectI& valid, bool passThrough,
                   std::function<void(const ts::RectI&)> onContent, std::string* err)
{
    @autoreleasepool {
        id<MTLCommandQueue> q = (__bridge id<MTLCommandQueue>)commandQueue;
        id<MTLBuffer> s = (__bridge id<MTLBuffer>)src.buffer;
        id<MTLBuffer> d = (__bridge id<MTLBuffer>)dst.buffer;
        if (!q || !s || !d) { if (err) *err = "no Metal queue/buffer"; return false; }
        const Pipelines& P = pipelines(q.device);
        if (!P.warp || !P.cols || !P.rows) {
            blitCopy(q, src, dst);
            if (err) *err = P.error;
            return false;
        }
        const WarpParams wp = makeWarpParams(src, dst, window, H, valid, passThrough);
        id<MTLCommandBuffer> cb = [q commandBuffer];
        cb.label = @"TrailStab warp + search";
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        id<MTLBuffer> flags = onContent ? encodeSearch(q, enc, P, s, src) : nil;
        if (wp.wx2 > wp.wx1 && wp.wy2 > wp.wy1) {
            [enc setComputePipelineState:P.warp];
            [enc setBytes:&wp length:sizeof(wp) atIndex:0];
            [enc setBuffer:s offset:0 atIndex:1];
            [enc setBuffer:d offset:0 atIndex:2];
            [enc dispatchThreads:MTLSizeMake(NSUInteger(wp.wx2 - wp.wx1), NSUInteger(wp.wy2 - wp.wy1), 1)
           threadsPerThreadgroup:groupSize(P.warp)];
        }
        [enc endEncoding];
        watch(cb);
        if (flags) {
            const MetalImage img = src;
            std::function<void(const ts::RectI&)> cbk = onContent;
            [cb addCompletedHandler:^(id<MTLCommandBuffer> done) {
                if (done.status == MTLCommandBufferStatusCompleted)
                    cbk(rectFromFlags(static_cast<const uint32_t*>(flags.contents), img));
            }];
        }
        [cb commit];
    }
    return true;
}

} // namespace tsm
