// Offline test of the in-plugin analysis: raw 8-bit grey frames on stdin -> .tstab
// usage: ffmpeg -i clip -vf scale=540:-2:flags=area,format=gray -f rawvideo - | analyze_cli 540 960 60 out.tstab
#include "analysis.h"
#include "sidecar.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>
int main(int argc, char** argv)
{
    if (argc < 5) { std::fprintf(stderr, "usage: analyze_cli W H fps out.tstab [maxFrames]\n"); return 1; }
    const int W = std::atoi(argv[1]), H = std::atoi(argv[2]);
    const double fps = std::atof(argv[3]);
    const long maxF = argc > 5 ? std::atol(argv[5]) : 1L << 40;
    ts::AnalysisParams p; p.width = W;
    ts::Analyzer an(p);
    std::vector<uint8_t> buf(size_t(W) * size_t(H));
    auto t0 = std::chrono::steady_clock::now();
    long n = 0;
    while (n < maxF && std::fread(buf.data(), 1, buf.size(), stdin) == buf.size()) {
        an.addFrame(ts::grayFromU8(buf.data(), W, H));
        if (++n % 200 == 0) std::fprintf(stderr, "  %ld frames\n", n);
    }
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    ts::CameraTrack tr = an.finish(fps, W, H);
    tr.source = "analyze_cli";
    ts::saveTrack(argv[4], tr);
    const auto& s = an.stats();
    std::fprintf(stderr, "%ld frames, %.1f ms/frame, points %.0f, quality %.2f, chains %d/%d\n", n, 1000 * sec / std::max(1L, n),
                 s.meanPoints, s.meanQuality, s.chainsUsed, s.chainsTried);
    return 0;
}
