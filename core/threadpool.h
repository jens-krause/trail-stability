// Shared worker pool for analysis and render.
//
// One pool per process, sized to the performance cores (Apple Silicon: hw.perflevel0) and run at
// a high quality-of-service class, so macOS keeps the work on the performance cores.  Avoids
// creating threads on every call (the old code spawned up to 8 threads per frame and step).
#pragma once
#include <functional>

namespace ts {

/// number of worker threads used for parallel loops (performance cores on Apple Silicon)
int workerCount();

/// split [0, n) into chunks and run fn(a, b) on the pool; the caller works on one chunk too.
/// minChunk: do not split below this many items per chunk.  Must not be called from inside fn.
void parallelFor(int n, const std::function<void(int, int)>& fn, int minChunk = 1);

/// raise the calling thread to the high quality-of-service class (no-op off macOS)
void setHighQoS();

} // namespace ts
