// libfpv - the Server of split builds: the simulation runs in libfpv-server,
// a process of its own, and every call goes there as a message
// (simapi/wire.h, docs/DECISIONS.md D37).
//
// The control connection carries the calls in order: those with a result
// wait for the reply, the others are sent and not waited for. The input
// connection goes the other way: each simulation step asks for its
// channels, and a thread here answers from the input manager, so devices
// are started, polled and stopped on that one thread, as the in-process
// server does on its simulation thread. libfpv-server's log lines arrive
// there too.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <random>
#include <thread>

#include "simapi/server.h"
#include "simapi/wire.h"
#include "util/clock.h"
#include "util/fs.h"
#include "util/log.h"
#include "util/platform.h"

#if defined(_WIN32)
#include <windows.h>
#else
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

namespace fpv {

namespace {

#if defined(_WIN32)
const char *const kServerFile = "libfpv-server.exe";
#else
const char *const kServerFile = "libfpv-server";
#endif

// libfpv-server: LIBFPV_SERVER, else next to the library.
std::string serverPath() {
    std::string p = envUtf8("LIBFPV_SERVER");
    if (!p.empty()) return p;
    std::string dir = libraryDirectory();
    return dir.empty() ? std::string(kServerFile) : pathToUtf8(pathFromUtf8(dir) / kServerFile);
}

std::string randomToken() {
    std::random_device rd;
    char buf[33];
    for (int i = 0; i < 4; i++) std::snprintf(buf + i * 8, 9, "%08x", (unsigned)rd());
    return std::string(buf, 32);
}

// The child process, started without inheriting anything but its arguments.
class ChildProcess {
public:
    ~ChildProcess() { release(); }

    bool start(const std::string &path, const std::vector<std::string> &args, std::string &error) {
#if defined(_WIN32)
        auto quote = [](const std::wstring &s) {
            std::wstring q = L"\"";
            for (wchar_t c : s) {
                if (c == L'"') q += L'\\';
                q += c;
            }
            return q + L"\"";
        };
        std::wstring exe = pathFromUtf8(path).wstring();
        std::wstring cmd = quote(exe);
        for (const std::string &a : args) cmd += L" " + quote(pathFromUtf8(a).wstring());
        STARTUPINFOW si;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        PROCESS_INFORMATION pi;
        ZeroMemory(&pi, sizeof(pi));
        // no console window for a game's background process
        if (!CreateProcessW(exe.c_str(), &cmd[0], nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr, nullptr, &si,
                            &pi)) {
            error = "cannot start " + path + " (error " + std::to_string((unsigned long)GetLastError()) + ")";
            return false;
        }
        CloseHandle(pi.hThread);
        m_process = pi.hProcess;
        return true;
#else
        std::vector<char *> argv;
        argv.push_back(const_cast<char *>(path.c_str()));
        for (const std::string &a : args) argv.push_back(const_cast<char *>(a.c_str()));
        argv.push_back(nullptr);
        pid_t pid = 0;
        int rc = posix_spawn(&pid, path.c_str(), nullptr, nullptr, argv.data(), environ);
        if (rc != 0) {
            error = "cannot start " + path + ": " + std::strerror(rc);
            return false;
        }
        m_pid = pid;
        return true;
#endif
    }

    bool running() {
#if defined(_WIN32)
        return m_process && WaitForSingleObject(m_process, 0) == WAIT_TIMEOUT;
#else
        if (m_pid <= 0) return false;
        int status = 0;
        pid_t r = waitpid(m_pid, &status, WNOHANG);
        // ECHILD: the game ignores SIGCHLD, so the system reaped it already
        if (r == m_pid || (r < 0 && errno == ECHILD)) {
            m_pid = 0;
            return false;
        }
        return true;
#endif
    }

    // Waits up to timeoutSec for it to end (it does once its connections close).
    void reap(double timeoutSec) {
        auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSec);
        while (running() && std::chrono::steady_clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void release() {
#if defined(_WIN32)
        if (m_process) CloseHandle(m_process);
        m_process = nullptr;
#else
        running();  // reaps it if it already ended
        m_pid = 0;
#endif
    }

private:
#if defined(_WIN32)
    HANDLE m_process = nullptr;
#else
    pid_t m_pid = 0;
#endif
};

class RemoteServer final : public Server {
public:
    explicit RemoteServer(ChannelSource &input) : m_input(input) {}

    ~RemoteServer() override { disconnect(); }

    // --- lifecycle -------------------------------------------------------------------
    bool start(const ServerConfig &cfg, std::string &error) override {
        if (!connect(error)) return false;
        m_threaded = cfg.threaded;
        m_stepSeconds = 1.0f / (cfg.simRateHz > 0 ? cfg.simRateHz : 1000.0f);
        post(wire::kSetLogLevel, (int)logMinLevel());
        wire::Writer w;
        wire::pack(w, cfg);
        std::string reply;
        if (!call(wire::kStart, w.out, reply)) {
            error = "libfpv-server stopped";
            return false;
        }
        wire::Reader r(reply);
        bool ok = false;
        std::string err;
        wire::unpack(r, ok, err);
        if (!ok) error = err;
        return ok;
    }

    void stop() override {
        std::string reply;
        call(wire::kStop, std::string(), reply);
    }

    void stopForExit() override {
        // never wait long at exit; libfpv-server saves and ends by itself
        // once the connections close
        if (!m_connected) return;
        std::string reply;
        call(wire::kStopForExit, std::string(), reply, 1.5);
        disconnect();
    }

    bool startInput(std::string &error) override {
        std::string reply;
        if (!call(wire::kStartInput, std::string(), reply)) {
            error = "libfpv-server stopped";
            return false;
        }
        wire::Reader r(reply);
        bool ok = false;
        wire::unpack(r, ok, error);
        return ok;
    }

    // --- Betaflight configuration ---------------------------------------------------------
    CliResult cli(const std::string &script, std::string &output) override {
        std::string reply;
        output.clear();
        if (!call(wire::kCli, encode(script), reply)) return CliResult::Error;
        wire::Reader r(reply);
        int result = (int)CliResult::Error;
        wire::unpack(r, result, output);
        return (CliResult)result;
    }
    void setDefaultDiff(const std::string &diff) override { post(wire::kSetDefaultDiff, diff); }
    void setUserDefaults(const std::string &script) override { post(wire::kSetUserDefaults, script); }
    void selectPreset(const DronePreset &preset) override { post(wire::kSelectPreset, preset); }
    void resetEeproms(bool all, const DronePreset &preset) override { post(wire::kResetEeproms, all, preset); }
    void finishImport(const DronePreset &preset) override { post(wire::kFinishImport, preset); }
    void reboot() override { post(wire::kReboot); }
    std::vector<std::string> armingDisableFlagNames() override {
        std::vector<std::string> names;
        std::string reply;
        if (call(wire::kArmingFlagNames, std::string(), reply)) {
            wire::Reader r(reply);
            wire::unpack(r, names);
        }
        return names;
    }
    ConfiguratorStatus configurator() override {
        ConfiguratorStatus s;
        std::string reply;
        if (call(wire::kConfigurator, std::string(), reply)) {
            wire::Reader r(reply);
            wire::unpack(r, s);
        }
        return s;
    }

    // --- the drone ------------------------------------------------------------------------
    void spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilot) override {
        post(wire::kSpawn, preset, pos, rot, pilot);
    }
    void despawn() override { post(wire::kDespawn); }
    void reset(Vec3 pos, Quat rot) override { post(wire::kReset, pos, rot); }
    void repair() override { post(wire::kRepair); }
    void damageProps(int motor, float amount) override { post(wire::kDamageProps, motor, amount); }
    void setPilotPosition(Vec3 pos) override { post(wire::kSetPilotPosition, pos); }
    void push(Vec3 velocity, bool replace) override { post(wire::kPush, velocity, replace); }
    void setCargo(const Cargo &cargo) override { post(wire::kSetCargo, cargo); }
    void setPortals(const std::vector<PortalLink> &links) override { post(wire::kSetPortals, links); }
    void setHome(bool set, Vec3 home) override { post(wire::kSetHome, set, home); }
    void setWind(const WindState &state) override { post(wire::kSetWind, state); }
    void addBlast(Vec3 pos, float radius) override { post(wire::kAddBlast, pos, radius); }
    void setSimOptions(bool batteryDrain, bool damage, const RcLinkConfig &rcLink) override {
        post(wire::kSetSimOptions, batteryDrain, damage, rcLink);
    }
    void setHostObstruction(const HostObstruction &o) override { post(wire::kSetHostObstruction, o); }

    // --- the collision world: libfpv-server keeps its own copy ------------------------------
    void worldSetMaterial(uint8_t id, const Material &m) override { post(wire::kWorldSetMaterial, id, m); }
    void worldLoadStatic(const std::vector<Triangle> &tris) override { post(wire::kWorldLoadStatic, tris); }
    void worldCreateObject(uint32_t id, const std::vector<Triangle> &localTris, const RigidTransform &t,
                           double simTime) override {
        post(wire::kWorldCreateObject, id, localTris, t, simTime);
    }
    void worldMoveObject(uint32_t id, const RigidTransform &t, double simTime, double lead, double leadAlt) override {
        post(wire::kWorldMoveObject, id, t, simTime, lead, leadAlt);
    }
    void worldDeleteObject(uint32_t id) override { post(wire::kWorldDeleteObject, id); }
    void worldSetProxyPlanes(const std::vector<Contact> &planes, const std::vector<ProxyMove> &moves) override {
        post(wire::kWorldSetProxyPlanes, planes, moves);
    }

    // --- time -----------------------------------------------------------------------------------
    bool threaded() override { return m_threaded; }
    float stepSeconds() override { return m_stepSeconds; }
    double simTime() override {
        double t = 0;
        std::string reply;
        if (call(wire::kSimTime, std::string(), reply)) {
            wire::Reader r(reply);
            wire::unpack(r, t);
        }
        return t;
    }
    void setPaused(bool paused) override { post(wire::kSetPaused, paused); }
    bool paused() override {
        fetchPaused();
        return m_paused;
    }
    bool autoPaused() override {
        fetchPaused();
        return m_autoPaused;
    }
    void setTimeScale(float scale) override { post(wire::kSetTimeScale, scale); }
    void heartbeat() override { post(wire::kHeartbeat); }
    void pump() override {}  // the browser's; split builds are native
    void stepManual(float seconds) override { post(wire::kStepManual, seconds); }

    // --- state -----------------------------------------------------------------------------------
    void snapshots(FlightSnapshot &prev, double &prevWall, FlightSnapshot &cur, double &curWall) override {
        std::string reply;
        std::lock_guard<std::mutex> lock(m_snapMutex);
        if (call(wire::kSnapshots, std::string(), reply)) {
            wire::Reader r(reply);
            FlightSnapshot p, c;
            double pw = 0, cw = 0;
            if (wire::unpack(r, p, pw, c, cw)) {
                m_prev = p;
                m_prevWall = pw;
                m_cur = c;
                m_curWall = cw;
            }
        }
        // without libfpv-server, the last state it gave
        prev = m_prev;
        prevWall = m_prevWall;
        cur = m_cur;
        curWall = m_curWall;
    }
    void takeImpacts(std::vector<ImpactEvent> &out) override {
        std::string reply;
        if (!call(wire::kTakeImpacts, std::string(), reply)) return;
        wire::Reader r(reply);
        std::vector<ImpactEvent> got;
        if (wire::unpack(r, got)) out.insert(out.end(), got.begin(), got.end());
    }

#if defined(LIBFPV_KAMIKAZE)
    void setWarhead(const WarheadConfig &config, bool installed, float payloadKg, float payloadDrop) override {
        post(wire::kSetWarhead, config, installed, payloadKg, payloadDrop);
    }
    bool pollDetonation(DetonationEvent &out) override {
        std::string reply;
        if (!call(wire::kPollDetonation, std::string(), reply)) return false;
        wire::Reader r(reply);
        bool found = false;
        DetonationEvent e;
        if (!wire::unpack(r, found, e) || !found) return false;
        out = e;
        return true;
    }
    bool detonate(bool evenIfSafe) override {
        std::string reply;
        if (!call(wire::kDetonate, encode(evenIfSafe), reply)) return false;
        wire::Reader r(reply);
        bool done = false;
        wire::unpack(r, done);
        return done;
    }
#endif

private:
    template <typename... T>
    static std::string encode(const T &...v) {
        wire::Writer w;
        wire::pack(w, v...);
        return w.out;
    }

    // Starts libfpv-server and takes its two connections. Once per RemoteServer:
    // libfpv-server keeps Betaflight booted across stop() and start().
    bool connect(std::string &error) {
        std::lock_guard<std::mutex> lock(m_controlMutex);
        if (m_connected) return true;
        if (m_failed) {
            error = m_failure;
            return false;
        }
        auto fail = [&](const std::string &e) {
            m_failed = true;
            m_failure = e;
            error = e;
            FPV_LOGE("libfpv-server: %s", e.c_str());
            return false;
        };
        LoopbackSocket listener;
        uint16_t port = 0;
        std::string err;
        if (!listener.listen(port, err)) return fail("cannot listen on the loopback interface: " + err);
        std::string token = randomToken();
        std::string path = serverPath();
        std::error_code ec;
        if (!std::filesystem::exists(pathFromUtf8(path), ec))
            return fail(path + " not found (split builds ship libfpv-server next to libfpv; LIBFPV_SERVER overrides)");
        if (!m_child.start(path, {"--port", std::to_string(port), "--token", token}, err)) return fail(err);

        // both connections, in either order, each proving it knows the token
        const std::string id = wire::protocolId();
        for (int got = 0; got < 2;) {
            LoopbackSocket s;
            std::string acceptErr;
            if (!listener.accept(s, 0.25, acceptErr)) {
                if (!m_child.running()) return fail("libfpv-server ended while starting");
                if (++m_waits > 80) return fail("libfpv-server did not connect within 20 s");
                continue;
            }
            uint16_t op = 0;
            std::string payload;
            if (!s.waitReadable(5.0) || !wire::recvMessage(s, op, payload) || op != wire::kHello) continue;
            wire::Reader r(payload);
            std::string theirId, theirToken;
            uint8_t role = 0;
            if (!wire::unpack(r, theirId, theirToken, role) || theirToken != token) continue;  // not ours
            if (theirId != id) {
                wire::send(s, wire::kHelloReply, false, std::string("another libfpv build"));
                return fail("libfpv-server is from another build of libfpv (" + theirId + ", this is " + id + ")");
            }
            LoopbackSocket &slot = role == wire::kInput ? m_inputSocket : m_control;
            if (slot.valid()) continue;
            if (!wire::send(s, wire::kHelloReply, true, std::string(), wallEpoch())) continue;
            slot = std::move(s);
            got++;
        }
        m_connected = true;
        m_inputThread = std::thread([this] { serveInput(); });
        FPV_LOGI("libfpv-server started (%s)", path.c_str());
        return true;
    }

    // Any call starts libfpv-server: the game may load its world before
    // fpv_global_init.
    bool ensureConnected() {
        if (m_connected) return true;
        std::string error;
        return connect(error);
    }

    void disconnect() {
        {
            std::lock_guard<std::mutex> lock(m_controlMutex);
            if (!m_connected) return;
            m_connected = false;
            m_control.shutdown();
            m_inputSocket.shutdown();
        }
        if (m_inputThread.joinable()) m_inputThread.join();
        m_control.close();
        m_inputSocket.close();
        m_child.reap(2.0);
        m_child.release();
    }

    void lost() {
        if (!m_lostLogged.exchange(true))
            FPV_LOGE("libfpv-server stopped: the simulation is gone until the game restarts");
        m_alive = false;
    }

    // A call without a result: sent, not waited for.
    template <typename... T>
    void post(uint16_t op, const T &...v) {
        std::string payload = encode(v...);
        if (!ensureConnected()) return;
        std::lock_guard<std::mutex> lock(m_controlMutex);
        if (!m_connected || !m_alive) return;
        if (!wire::sendMessage(m_control, op, payload)) lost();
    }

    // A call with a result; false when libfpv-server is gone (or did not
    // answer within timeoutSec, > 0).
    bool call(uint16_t op, const std::string &payload, std::string &reply, double timeoutSec = -1) {
        if (!ensureConnected()) return false;
        std::lock_guard<std::mutex> lock(m_controlMutex);
        if (!m_connected || !m_alive) return false;
        uint16_t replyOp = 0;
        if (!wire::sendMessage(m_control, op, payload)) {
            lost();
            return false;
        }
        if (timeoutSec > 0 && !m_control.waitReadable(timeoutSec)) return false;
        if (!wire::recvMessage(m_control, replyOp, reply) || replyOp != op) {
            lost();
            return false;
        }
        return true;
    }

    void fetchPaused() {
        std::string reply;
        if (!call(wire::kPaused, std::string(), reply)) return;
        wire::Reader r(reply);
        bool p = false, a = false;
        if (wire::unpack(r, p, a)) {
            m_paused = p;
            m_autoPaused = a;
        }
    }

    // The input connection's thread: what the simulation asks of the
    // channel source, answered in order.
    void serveInput() {
        uint16_t op = 0;
        std::string payload;
        while (wire::recvMessage(m_inputSocket, op, payload)) {
            wire::Reader r(payload);
            switch (op) {
            case wire::kInputUpdate: {
                HoverFeedback hf;
                double wall = 0;
                float dt = 0;
                float ch[kChannelCount];
                setIdleChannels(ch);
                bool radioLost = false;
                if (wire::unpack(r, hf, wall, dt)) {
                    m_input.setHoverFeedback(hf);
                    m_input.update(wall, dt, ch, radioLost);
                }
                wire::Writer w;
                w.bytes(ch, sizeof(ch));
                wire::put(w, radioLost);
                if (!wire::sendMessage(m_inputSocket, op, w.out)) return;
                break;
            }
            case wire::kInputResetLatch:
                m_input.resetRadioLatch();
                break;
            case wire::kInputStart: {
                std::string error;
                bool ok = m_input.startDevices(error);
                if (!wire::send(m_inputSocket, op, ok, error)) return;
                break;
            }
            case wire::kInputStop:
                m_input.stopDevices();
                if (!wire::sendMessage(m_inputSocket, op, std::string())) return;
                break;
            case wire::kLog: {
                int level = 0;
                std::string line;
                if (wire::unpack(r, level, line)) logWrite((LogLevel)level, "%s", line.c_str());
                break;
            }
            default:
                break;
            }
        }
        // libfpv-server is gone (or we are disconnecting): the devices are
        // this thread's to stop
        m_input.stopDevices();
    }

    ChannelSource &m_input;
    std::mutex m_controlMutex;  // one call at a time, in order
    LoopbackSocket m_control, m_inputSocket;
    std::thread m_inputThread;
    ChildProcess m_child;
    std::atomic<bool> m_connected{false};
    bool m_failed = false;
    std::string m_failure;
    int m_waits = 0;
    std::atomic<bool> m_alive{true}, m_lostLogged{false};

    bool m_threaded = true;
    float m_stepSeconds = 0.001f;
    std::atomic<bool> m_paused{false}, m_autoPaused{false};
    std::mutex m_snapMutex;
    FlightSnapshot m_prev, m_cur;
    double m_prevWall = 0, m_curWall = 0;
};

}  // namespace

std::unique_ptr<Server> createRemoteServer(ChannelSource &input) { return std::make_unique<RemoteServer>(input); }

}  // namespace fpv
