#include "threadpool.h"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#if defined(__APPLE__)
#include <pthread.h>
#include <sys/qos.h>
#include <sys/sysctl.h>
#endif

namespace ts {

void setHighQoS()
{
#if defined(__APPLE__)
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INITIATED, 0);
#endif
}

namespace {

int detectWorkers()
{
    int n = 0;
#if defined(__APPLE__)
    int v = 0; size_t len = sizeof(v);
    if (sysctlbyname("hw.perflevel0.logicalcpu", &v, &len, nullptr, 0) == 0 && v > 0) n = v;   // performance cores
#endif
    if (n <= 0) n = int(std::max(1u, std::thread::hardware_concurrency()));
    return std::max(1, std::min(n, 16));
}

class Pool {
public:
    explicit Pool(int n)
    {
        // the caller of parallelFor works too, so n - 1 extra threads use n cores
        for (int i = 0; i + 1 < n; ++i) threads_.emplace_back([this] { loop(); });
    }
    ~Pool()
    {
        { std::lock_guard<std::mutex> l(m_); stop_ = true; }
        cv_.notify_all();
        for (auto& t : threads_) t.join();
    }
    void push(std::function<void()> job)
    {
        { std::lock_guard<std::mutex> l(m_); q_.push_back(std::move(job)); }
        cv_.notify_one();
    }
    /// run queued jobs on the calling thread while waiting (keeps the caller busy, no deadlock)
    bool tryRunOne()
    {
        std::function<void()> job;
        { std::lock_guard<std::mutex> l(m_); if (q_.empty()) return false; job = std::move(q_.front()); q_.pop_front(); }
        job();
        return true;
    }
private:
    void loop()
    {
        setHighQoS();
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> l(m_);
                cv_.wait(l, [this] { return stop_ || !q_.empty(); });
                if (stop_ && q_.empty()) return;
                job = std::move(q_.front()); q_.pop_front();
            }
            job();
        }
    }
    std::vector<std::thread> threads_;
    std::deque<std::function<void()>> q_;
    std::mutex m_;
    std::condition_variable cv_;
    bool stop_ = false;
};

Pool& pool()
{
    static Pool p(detectWorkers());
    return p;
}

} // namespace

int workerCount()
{
    static const int n = detectWorkers();
    return n;
}

void parallelFor(int n, const std::function<void(int, int)>& fn, int minChunk)
{
    if (n <= 0) return;
    const int chunks = std::max(1, std::min(workerCount(), n / std::max(1, minChunk)));
    if (chunks <= 1) { fn(0, n); return; }

    struct Sync { std::mutex m; std::condition_variable cv; int left; std::exception_ptr err; };
    auto sync = std::make_shared<Sync>();
    sync->left = chunks - 1;
    Pool& P = pool();
    for (int c = 1; c < chunks; ++c) {
        const int a = int(int64_t(n) * c / chunks), b = int(int64_t(n) * (c + 1) / chunks);
        P.push([&fn, a, b, sync] {
            std::exception_ptr e;
            try { fn(a, b); } catch (...) { e = std::current_exception(); }
            std::lock_guard<std::mutex> l(sync->m);
            if (e && !sync->err) sync->err = e;
            if (--sync->left == 0) sync->cv.notify_all();
        });
    }
    // the queued jobs reference fn: even if this chunk throws, wait for them before leaving
    std::exception_ptr own;
    try { fn(0, int(int64_t(n) / chunks)); } catch (...) { own = std::current_exception(); }
    // help with queued work instead of idling, then wait for the rest
    for (;;) {
        { std::lock_guard<std::mutex> l(sync->m); if (sync->left == 0) break; }
        if (!P.tryRunOne()) break;
    }
    {
        std::unique_lock<std::mutex> l(sync->m);
        sync->cv.wait(l, [&] { return sync->left == 0; });
        if (!own) own = sync->err;
    }
    if (own) std::rethrow_exception(own);
}

} // namespace ts
