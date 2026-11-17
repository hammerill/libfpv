// libfpv-server - the simulation of split builds in a process of its own:
// Betaflight, SimITL's flight model and the real-time loop (SimServer),
// driven by the game's libfpv over two loopback connections
// (simapi/wire.h, docs/DECISIONS.md D37). The game's libfpv starts it:
//
//   libfpv-server --port <port> --token <token>
//
// It ends when the game's libfpv disconnects or its process ends, after
// saving Betaflight's settings.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

#include "sim/sim_server.h"
#include "simapi/wire.h"
#include "util/clock.h"
#include "util/log.h"

using namespace fpv;

namespace {

// The game's input manager, on the other end of the input connection: each
// call goes there and (most) wait for the answer. The simulation calls it
// from one thread at a time.
class RemoteChannelSource final : public ChannelSource {
public:
    explicit RemoteChannelSource(LoopbackSocket &s) : m_socket(s) {}

    bool startDevices(std::string &error) override {
        std::lock_guard<std::mutex> lock(m_requestMutex);
        std::string reply;
        if (!request(wire::kInputStart, std::string(), reply)) {
            error = "the game's libfpv is gone";
            return false;
        }
        wire::Reader r(reply);
        bool ok = false;
        wire::unpack(r, ok, error);
        return ok;
    }

    void stopDevices() override {
        std::lock_guard<std::mutex> lock(m_requestMutex);
        std::string reply;
        request(wire::kInputStop, std::string(), reply);
    }

    void setHoverFeedback(const HoverFeedback &f) override { m_hover = f; }  // goes with the next update

    void update(double wallTime, float dt, float out[kChannelCount], bool &radioLost) override {
        std::lock_guard<std::mutex> lock(m_requestMutex);
        wire::Writer w;
        wire::pack(w, m_hover, wallTime, dt);
        std::string reply;
        if (request(wire::kInputUpdate, w.out, reply) && reply.size() == sizeof(float) * kChannelCount + 1) {
            wire::Reader r(reply);
            r.bytes(out, sizeof(float) * kChannelCount);
            wire::get(r, radioLost);
            return;
        }
        // no game: sticks centred, every switch off, and the radio lost
        setIdleChannels(out);
        radioLost = true;
    }

    void resetRadioLatch() override { send(wire::kInputResetLatch, std::string()); }

    // libfpv-server's log lines, from any thread
    void log(int level, const char *line) {
        wire::Writer w;
        wire::pack(w, level, std::string(line));
        send(wire::kLog, w.out);
    }

private:
    bool send(uint16_t op, const std::string &payload) {
        std::lock_guard<std::mutex> lock(m_sendMutex);
        if (m_lost) return false;
        if (!wire::sendMessage(m_socket, op, payload)) m_lost = true;
        return !m_lost;
    }

    // Only the requester reads: replies come in the order asked.
    bool request(uint16_t op, const std::string &payload, std::string &reply) {
        if (!send(op, payload)) return false;
        uint16_t replyOp = 0;
        if (!wire::recvMessage(m_socket, replyOp, reply) || replyOp != op) {
            m_lost = true;
            return false;
        }
        return true;
    }

    LoopbackSocket &m_socket;
    std::mutex m_requestMutex, m_sendMutex;
    std::atomic<bool> m_lost{false};
    HoverFeedback m_hover;
};

RemoteChannelSource *g_logTarget = nullptr;

void forwardLog(void *, int level, const char *message) {
    if (g_logTarget) g_logTarget->log(level, message);
}

void printLog(void *, int level, const char *message) {
    std::fprintf(stderr, "[libfpv-server %d] %s\n", level, message);
}

// Connects one of the two connections and shakes hands.
bool connectRole(LoopbackSocket &s, uint16_t port, const std::string &token, wire::Role role) {
    std::string error;
    if (!s.connect(port, error)) {
        FPV_LOGE("cannot connect to the game's libfpv on port %u: %s", (unsigned)port, error.c_str());
        return false;
    }
    if (!wire::send(s, wire::kHello, wire::protocolId(), token, (uint8_t)role)) return false;
    uint16_t op = 0;
    std::string payload;
    if (!wire::recvMessage(s, op, payload) || op != wire::kHelloReply) {
        FPV_LOGE("the game's libfpv refused the connection");
        return false;
    }
    wire::Reader r(payload);
    bool ok = false;
    std::string reason;
    int64_t epoch = 0;
    wire::unpack(r, ok, reason);
    if (!ok) {
        FPV_LOGE("the game's libfpv refused the connection: %s", reason.c_str());
        return false;
    }
    wire::get(r, epoch);
    setWallEpoch(epoch);  // wall times mean the same in both processes
    return r.ok;
}

template <typename... T>
bool reply(LoopbackSocket &s, uint16_t op, const T &...v) {
    return wire::send(s, op, v...);
}

// Runs one control message; false ends the server (a broken connection).
bool dispatch(SimServer &server, LoopbackSocket &s, uint16_t op, const std::string &payload) {
    wire::Reader r(payload);
    switch (op) {
    // lifecycle
    case wire::kStart: {
        ServerConfig cfg;
        if (!wire::unpack(r, cfg)) return reply(s, op, false, std::string("bad start message"));
        std::string error;
        bool ok = server.start(cfg, error);
        return reply(s, op, ok, error);
    }
    case wire::kStop:
        server.stop();
        return reply(s, op);
    case wire::kStopForExit:
        server.stopForExit();
        return reply(s, op);
    case wire::kStartInput: {
        std::string error;
        bool ok = server.startInput(error);
        return reply(s, op, ok, error);
    }
    case wire::kSetLogLevel: {
        int level = 1;
        if (wire::unpack(r, level)) logSetSink(&forwardLog, nullptr, (LogLevel)level);
        return true;
    }
    // Betaflight configuration
    case wire::kCli: {
        std::string script, output;
        wire::unpack(r, script);
        CliResult result = server.cli(script, output);
        return reply(s, op, (int)result, output);
    }
    case wire::kSetDefaultDiff: {
        std::string diff;
        if (wire::unpack(r, diff)) server.setDefaultDiff(diff);
        return true;
    }
    case wire::kSelectPreset: {
        DronePreset p;
        if (wire::unpack(r, p)) server.selectPreset(p);
        return true;
    }
    case wire::kResetEeproms: {
        bool all = false;
        DronePreset p;
        if (wire::unpack(r, all, p)) server.resetEeproms(all, p);
        return true;
    }
    case wire::kFinishImport: {
        DronePreset p;
        if (wire::unpack(r, p)) server.finishImport(p);
        return true;
    }
    case wire::kReboot:
        server.reboot();
        return true;
    case wire::kArmingFlagNames:
        return reply(s, op, server.armingDisableFlagNames());
    case wire::kConfigurator:
        return reply(s, op, server.configurator());
    // the drone
    case wire::kSpawn: {
        DronePreset p;
        Vec3 pos, pilot;
        Quat rot;
        if (wire::unpack(r, p, pos, rot, pilot)) server.spawn(p, pos, rot, pilot);
        return true;
    }
    case wire::kDespawn:
        server.despawn();
        return true;
    case wire::kReset: {
        Vec3 pos;
        Quat rot;
        if (wire::unpack(r, pos, rot)) server.reset(pos, rot);
        return true;
    }
    case wire::kRepair:
        server.repair();
        return true;
    case wire::kDamageProps: {
        int motor = 0;
        float amount = 0;
        if (wire::unpack(r, motor, amount)) server.damageProps(motor, amount);
        return true;
    }
    case wire::kSetPilotPosition: {
        Vec3 pos;
        if (wire::unpack(r, pos)) server.setPilotPosition(pos);
        return true;
    }
    case wire::kPush: {
        Vec3 v;
        bool replace = false;
        if (wire::unpack(r, v, replace)) server.push(v, replace);
        return true;
    }
    case wire::kSetCargo: {
        Cargo c;
        if (wire::unpack(r, c)) server.setCargo(c);
        return true;
    }
    case wire::kSetPortals: {
        std::vector<PortalLink> links;
        if (wire::unpack(r, links)) server.setPortals(links);
        return true;
    }
    case wire::kSetHome: {
        bool set = false;
        Vec3 home;
        if (wire::unpack(r, set, home)) server.setHome(set, home);
        return true;
    }
    case wire::kSetWind: {
        WindState w;
        if (wire::unpack(r, w)) server.setWind(w);
        return true;
    }
    case wire::kAddBlast: {
        Vec3 pos;
        float radius = 0;
        if (wire::unpack(r, pos, radius)) server.addBlast(pos, radius);
        return true;
    }
    case wire::kSetSimOptions: {
        bool drain = false, damage = false;
        RcLinkConfig rc;
        if (wire::unpack(r, drain, damage, rc)) server.setSimOptions(drain, damage, rc);
        return true;
    }
    case wire::kSetHostObstruction: {
        HostObstruction o;
        if (wire::unpack(r, o)) server.setHostObstruction(o);
        return true;
    }
    // the collision world
    case wire::kWorldSetMaterial: {
        uint8_t id = 0;
        Material m;
        if (wire::unpack(r, id, m)) server.worldSetMaterial(id, m);
        return true;
    }
    case wire::kWorldLoadStatic: {
        std::vector<Triangle> tris;
        if (wire::unpack(r, tris)) server.worldLoadStatic(tris);
        return true;
    }
    case wire::kWorldCreateObject: {
        uint32_t id = 0;
        std::vector<Triangle> tris;
        RigidTransform t;
        double simTime = 0;
        if (wire::unpack(r, id, tris, t, simTime)) server.worldCreateObject(id, tris, t, simTime);
        return true;
    }
    case wire::kWorldMoveObject: {
        uint32_t id = 0;
        RigidTransform t;
        double simTime = 0;
        if (wire::unpack(r, id, t, simTime)) server.worldMoveObject(id, t, simTime);
        return true;
    }
    case wire::kWorldDeleteObject: {
        uint32_t id = 0;
        if (wire::unpack(r, id)) server.worldDeleteObject(id);
        return true;
    }
    case wire::kWorldSetProxyPlanes: {
        std::vector<Contact> planes;
        if (wire::unpack(r, planes)) server.worldSetProxyPlanes(planes);
        return true;
    }
    // time
    case wire::kSimTime:
        return reply(s, op, server.simTime());
    case wire::kSetPaused: {
        bool p = false;
        if (wire::unpack(r, p)) server.setPaused(p);
        return true;
    }
    case wire::kPaused:
        return reply(s, op, server.paused(), server.autoPaused());
    case wire::kSetTimeScale: {
        float scale = 1;
        if (wire::unpack(r, scale)) server.setTimeScale(scale);
        return true;
    }
    case wire::kHeartbeat:
        server.heartbeat();
        return true;
    case wire::kStepManual: {
        float seconds = 0;
        if (wire::unpack(r, seconds)) server.stepManual(seconds);
        return true;
    }
    // state
    case wire::kSnapshots: {
        FlightSnapshot prev, cur;
        double prevWall = 0, curWall = 0;
        server.snapshots(prev, prevWall, cur, curWall);
        return reply(s, op, prev, prevWall, cur, curWall);
    }
    case wire::kTakeImpacts: {
        std::vector<ImpactEvent> impacts;
        server.takeImpacts(impacts);
        return reply(s, op, impacts);
    }
#if defined(LIBFPV_KAMIKAZE)
    case wire::kSetWarhead: {
        WarheadConfig c;
        bool installed = false;
        float kg = 0, drop = 0;
        if (wire::unpack(r, c, installed, kg, drop)) server.setWarhead(c, installed, kg, drop);
        return true;
    }
    case wire::kPollDetonation: {
        DetonationEvent e;
        bool found = server.pollDetonation(e);
        return reply(s, op, found, e);
    }
    case wire::kDetonate: {
        bool evenIfSafe = false;
        wire::unpack(r, evenIfSafe);
        return reply(s, op, server.detonate(evenIfSafe));
    }
#endif
    default:
        FPV_LOGW("unknown message %u from the game's libfpv", (unsigned)op);
        return true;
    }
}

}  // namespace

int main(int argc, char **argv) {
    logSetSink(&printLog, nullptr, LogLevel::Info);
    long port = 0;
    std::string token;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--port")) port = std::strtol(argv[i + 1], nullptr, 10);
        else if (!std::strcmp(argv[i], "--token")) token = argv[i + 1];
    }
    if (port <= 0 || port > 65535 || token.empty()) {
        std::fprintf(stderr,
                     "libfpv-server: the simulation of a split libfpv build. The game's libfpv starts it;\n"
                     "it is not meant to be run by hand.\n"
                     "usage: libfpv-server --port <port> --token <token>\n");
        return 2;
    }

    LoopbackSocket control, input;
    if (!connectRole(control, (uint16_t)port, token, wire::kControl) ||
        !connectRole(input, (uint16_t)port, token, wire::kInput))
        return 1;

    RemoteChannelSource channels(input);
    g_logTarget = &channels;
    logSetSink(&forwardLog, nullptr, LogLevel::Info);

    // never destroyed: the process ends with it (Betaflight's statics and
    // threads need no winding down)
    SimServer *server = new SimServer(channels, nullptr);  // its own copy of the game's collision world
    bool running = false;  // started and not stopped: Betaflight's settings to save at the end
    uint16_t op = 0;
    std::string payload;
    while (wire::recvMessage(control, op, payload)) {
        if (!dispatch(*server, control, op, payload)) break;
        if (op == wire::kStart) running = true;
        if (op == wire::kStop) running = false;
        if (op == wire::kStopForExit) {
            running = false;  // saved and stopped
            break;
        }
    }
    // the game's libfpv is gone (or its process ended): save and end
    if (running) server->stopForExit();
    logSetSink(&printLog, nullptr, LogLevel::Info);
    g_logTarget = nullptr;
    std::fflush(stderr);
    std::_Exit(0);
}
