// libfpv - simulation scheduling: the real-time simulation thread (or manual
// stepping), the job queue that keeps all Betaflight access on that thread,
// pause/time scale/auto-pause and snapshots for render interpolation.
// Browser builds have no thread: the same real-time loop is pumped from the
// host's per-frame calls instead (cooperative timing).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include "input/channels.h"
#include "sim/flight.h"
#include "util/clock.h"

namespace fpv {

// Browser (Emscripten) builds step the simulation on the host's thread.
#if defined(__EMSCRIPTEN__)
constexpr bool kCooperativeTiming = true;
#else
constexpr bool kCooperativeTiming = false;
#endif

struct RunnerConfig {
    bool threaded = true;
    float simRateHz = 1000.0f;
    float autoPauseSeconds = 0.5f;  // <= 0 disables
};

class Runner {
public:
    Runner(Flight &flight, ChannelSource &input);
    ~Runner();

    void configure(const RunnerConfig &cfg);
    void start();  // starts the thread (or cooperative timing) in threaded mode
    void stop();   // joins the thread; jobs still queued run inline
    bool threaded() const { return m_cfg.threaded; }
    bool running() const { return m_thread.joinable() || m_cooperative; }
    // Cooperative timing: runs the simulation up to now, as the thread
    // would have. Called from the host's per-frame API calls; no-op otherwise.
    void pump();

    // Runs fn on the simulation thread and waits for it (inline when the
    // thread is not running or when called from the simulation thread).
    template <typename F>
    auto call(F &&fn) -> decltype(fn());
    // Like call() but gives up waiting after timeoutSec (the job still runs).
    bool callWithTimeout(std::function<void()> fn, double timeoutSec);

    // Manual mode: advances the simulation by `seconds` on the caller thread.
    void stepManual(float seconds);

    void setPaused(bool p) { m_paused = p; }
    bool paused() const { return m_paused; }
    bool autoPaused() const { return m_autoPaused; }
    void setTimeScale(float s) { m_timeScale = s; }
    // Host heartbeat, from fpv_drone_tick.
    void heartbeat() { m_lastHeartbeat = wallSeconds(); }

    // Latest two snapshots with the wall time they were produced.
    void snapshots(FlightSnapshot &prev, double &prevWall, FlightSnapshot &cur, double &curWall);
    void takeImpacts(std::vector<ImpactEvent> &out);
    float stepSeconds() const { return 1.0f / m_cfg.simRateHz; }

    // Statistics for the latency benchmark and the overlay.
    double averageStepMicros() const { return m_avgStepUs; }

private:
    void threadMain();
    // One pass of the real-time loop: jobs, then the steps due by now.
    void advance();
    void runJobs();
    void stepOnce(double wall);
    bool shouldSimulate();

    Flight &m_flight;
    ChannelSource &m_input;
    RunnerConfig m_cfg;

    std::thread m_thread;
    std::thread::id m_simThreadId;
    std::atomic<bool> m_stop{false};
    bool m_cooperative = false;
    double m_last = 0, m_acc = 0;  // real-time loop: last pass, unspent time
    double m_manualCarry = 0;      // stepManual: time short of a whole step, for the next call
    std::mutex m_jobMutex;
    std::condition_variable m_jobCv;
    std::deque<std::function<void()>> m_jobs;

    std::atomic<bool> m_paused{false};
    std::atomic<bool> m_autoPaused{false};
    std::atomic<float> m_timeScale{1.0f};
    std::atomic<double> m_lastHeartbeat{0.0};

    std::mutex m_snapMutex;
    FlightSnapshot m_prev, m_cur;
    double m_prevWall = 0, m_curWall = 0;
    std::vector<ImpactEvent> m_impacts;
    double m_avgStepUs = 0;
};

template <typename F>
auto Runner::call(F &&fn) -> decltype(fn()) {
    using R = decltype(fn());
    if (!m_thread.joinable() || std::this_thread::get_id() == m_simThreadId) {
        return fn();
    }
    auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(fn));
    std::future<R> fut = task->get_future();
    {
        std::lock_guard<std::mutex> lock(m_jobMutex);
        m_jobs.push_back([task] { (*task)(); });
    }
    m_jobCv.notify_one();
    return fut.get();
}

}  // namespace fpv
