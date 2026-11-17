// libfpv - simulation scheduling.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sim/runner.h"

#include <algorithm>
#include <chrono>

#include "util/log.h"

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <timeapi.h>
#  ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#    define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#  endif
#endif

namespace fpv {

namespace {
// Sleeps until an absolute wall time with sub-millisecond precision where
// the OS allows it (high-resolution waitable timer on Windows 10+).
class PreciseSleeper {
public:
    PreciseSleeper() {
#if defined(_WIN32)
        m_timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
        if (!m_timer) {
            timeBeginPeriod(1);
            m_period = true;
            m_timer = CreateWaitableTimerW(nullptr, TRUE, nullptr);
        }
#endif
    }
    ~PreciseSleeper() {
#if defined(_WIN32)
        if (m_timer) CloseHandle(m_timer);
        if (m_period) timeEndPeriod(1);
#endif
    }
    void sleepUntil(double target) {
        double now = wallSeconds();
        if (target <= now) return;
#if defined(_WIN32)
        if (m_timer) {
            LARGE_INTEGER due;
            due.QuadPart = -(LONGLONG)((target - now) * 1e7);  // 100 ns units, relative
            if (SetWaitableTimer(m_timer, &due, 0, nullptr, nullptr, FALSE)) {
                WaitForSingleObject(m_timer, 100);
                return;
            }
        }
#endif
        std::this_thread::sleep_for(std::chrono::duration<double>(target - now));
    }

private:
#if defined(_WIN32)
    HANDLE m_timer = nullptr;
    bool m_period = false;
#endif
};
}  // namespace

Runner::Runner(Flight &flight, ChannelSource &input) : m_flight(flight), m_input(input) {}

Runner::~Runner() { stop(); }

void Runner::configure(const RunnerConfig &cfg) {
    m_cfg = cfg;
    m_cfg.simRateHz = std::clamp(m_cfg.simRateHz, 200.0f, 8000.0f);
}

void Runner::start() {
    if (!m_cfg.threaded || running()) return;
    m_stop = false;
    m_lastHeartbeat = wallSeconds();
    m_last = wallSeconds();
    m_acc = 0;
    if (kCooperativeTiming) {
        m_cooperative = true;
        FPV_LOGI("simulation running at %.0f Hz, stepped from the host's frames", m_cfg.simRateHz);
        return;
    }
    m_thread = std::thread([this] { threadMain(); });
}

void Runner::pump() {
    if (m_cooperative) advance();
}

void Runner::stop() {
    m_cooperative = false;
    if (m_thread.joinable()) {
        m_stop = true;
        m_jobCv.notify_all();
        m_thread.join();
    }
    runJobs();
}

bool Runner::callWithTimeout(std::function<void()> fn, double timeoutSec) {
    if (!m_thread.joinable() || std::this_thread::get_id() == m_simThreadId) {
        fn();
        return true;
    }
    auto done = std::make_shared<std::promise<void>>();
    std::future<void> fut = done->get_future();
    {
        std::lock_guard<std::mutex> lock(m_jobMutex);
        m_jobs.push_back([fn = std::move(fn), done] {
            fn();
            done->set_value();
        });
    }
    m_jobCv.notify_one();
    return fut.wait_for(std::chrono::duration<double>(timeoutSec)) == std::future_status::ready;
}

void Runner::runJobs() {
    std::deque<std::function<void()>> jobs;
    {
        std::lock_guard<std::mutex> lock(m_jobMutex);
        jobs.swap(m_jobs);
    }
    for (auto &j : jobs) j();
}

bool Runner::shouldSimulate() {
    bool autoPause = false;
    if (m_cfg.autoPauseSeconds > 0 && m_flight.active()) {
        autoPause = wallSeconds() - m_lastHeartbeat.load() > m_cfg.autoPauseSeconds;
    }
    m_autoPaused = autoPause;
    if (!m_paused && !autoPause) return true;
    // While paused, Betaflight keeps running only if nothing can move: no
    // drone, or a disarmed drone resting on the ground. This keeps the
    // Configurator responsive in a pause menu.
    const FlightSnapshot &s = m_flight.snapshot();
    return !m_flight.active() || (!s.armed && s.onGround);
}

void Runner::stepOnce(double wall) {
    float dt = 1.0f / m_cfg.simRateHz;
    float ch[kChannelCount];
    bool radioLost = false;
    {
        const FlightSnapshot &s = m_flight.snapshot();
        HoverFeedback hf;
        hf.preset = m_flight.preset().fileName;
        hf.climbRate = s.vel.y;
        hf.upright = rotate(s.rot, {0, 1, 0}).y;
        hf.airborne = s.active && !s.onGround;
        m_input.setHoverFeedback(hf);
    }
    m_input.update(wall, dt, ch, radioLost);
    auto t0 = std::chrono::steady_clock::now();
    m_flight.step(dt, ch, radioLost);
    double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    m_avgStepUs = m_avgStepUs * 0.99 + us * 0.01;
    const FlightSnapshot &s = m_flight.snapshot();
    if (radioLost && !s.armed) m_input.resetRadioLatch();

    std::lock_guard<std::mutex> lock(m_snapMutex);
    m_prev = m_cur;
    m_prevWall = m_curWall;
    m_cur = s;
    m_curWall = wall;
    m_flight.takeImpacts(m_impacts);
    if (m_impacts.size() > 256) m_impacts.erase(m_impacts.begin(), m_impacts.end() - 256);
}

void Runner::advance() {
    runJobs();
    const double period = 1.0 / m_cfg.simRateHz;
    double now = wallSeconds();
    double elapsed = now - m_last;
    m_last = now;
    if (shouldSimulate()) {
        double scale = std::clamp(m_timeScale.load(), 0.05f, 4.0f);
        m_acc += elapsed * scale;
        // never try to catch up more than 50 ms (host hitch, debugger)
        m_acc = std::min(m_acc, 0.05);
        while (m_acc >= period && !m_stop) {
            m_acc -= period;
            // each step gets the wall time it represents, for interpolation
            stepOnce(now - m_acc / scale);
        }
    } else {
        m_acc = 0;
    }
}

void Runner::threadMain() {
    m_simThreadId = std::this_thread::get_id();
    PreciseSleeper sleeper;
    const double period = 1.0 / m_cfg.simRateHz;
    double next = m_last + period;
    FPV_LOGI("simulation thread running at %.0f Hz", m_cfg.simRateHz);
    while (!m_stop) {
        advance();
        double now = m_last;
        next += period;
        if (next < now - 0.05) next = now + period;  // fell far behind: resync
        std::unique_lock<std::mutex> lock(m_jobMutex);
        if (m_jobs.empty()) {
            lock.unlock();
            sleeper.sleepUntil(next);
        }
    }
    m_simThreadId = std::thread::id();
}

void Runner::stepManual(float seconds) {
    runJobs();
    float dt = 1.0f / m_cfg.simRateHz;
    int steps = (int)std::lround(seconds / dt);
    double wall = wallSeconds();
    for (int i = 0; i < steps; i++) {
        // manual mode has no auto-pause: the host drives time explicitly
        const FlightSnapshot &s = m_flight.snapshot();
        if (m_paused && m_flight.active() && (s.armed || !s.onGround)) break;
        stepOnce(wall);
    }
}

void Runner::snapshots(FlightSnapshot &prev, double &prevWall, FlightSnapshot &cur, double &curWall) {
    std::lock_guard<std::mutex> lock(m_snapMutex);
    prev = m_prev;
    prevWall = m_prevWall;
    cur = m_cur;
    curWall = m_curWall;
}

void Runner::takeImpacts(std::vector<ImpactEvent> &out) {
    std::lock_guard<std::mutex> lock(m_snapMutex);
    out.insert(out.end(), m_impacts.begin(), m_impacts.end());
    m_impacts.clear();
}

}  // namespace fpv
