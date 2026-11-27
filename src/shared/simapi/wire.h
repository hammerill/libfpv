// libfpv - the Server calls as messages, for split builds: the game's
// libfpv (RemoteServer) and libfpv-server carry them over two loopback
// connections (docs/DECISIONS.md, D37):
//
//   control  the client's Server calls, in order. A call with a result gets
//            one reply; a call without one (most of them) gets none, so
//            the client never waits for it. The server runs them one by
//            one, so a later call always sees what earlier ones did.
//   input    the other way: the simulation's ChannelSource calls (each
//            step's channels, the devices) and libfpv-server's log lines.
//
// Both ends are built together from this file: the layout of plain structs
// goes as it is in memory, and the handshake refuses another build's peer.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include "config/preset.h"
#include "input/channels.h"
#include "ipc/socket.h"
#include "rc/rc_link.h"
#include "simapi/server.h"
#include "world/collision.h"
#include "world/wind.h"

namespace fpv {
namespace wire {

// Control connection: the client's calls.
enum Op : uint16_t {
    // handshake (both connections)
    kHello = 1,
    kHelloReply,
    // lifecycle
    kStart,        // reply: ok, error
    kStop,         // reply (empty): stop() waits for the EEPROM to be saved
    kStopForExit,  // reply (empty)
    kStartInput,   // reply: ok, error
    kSetLogLevel,
    // Betaflight configuration
    kCli,  // reply: result, output
    kSetDefaultDiff,
    kSetUserDefaults,
    kSelectPreset,
    kResetEeproms,
    kFinishImport,
    kReboot,
    kArmingFlagNames,  // reply: names
    kConfigurator,     // reply: ConfiguratorStatus
    // the drone
    kSpawn,
    kDespawn,
    kReset,
    kRepair,
    kDamageProps,
    kSetPilotPosition,
    kPush,
    kSetCargo,
    kSetPortals,
    kSetHome,
    kSetWind,
    kAddBlast,
    kSetSimOptions,
    kSetHostObstruction,
    // the collision world
    kWorldSetMaterial,
    kWorldLoadStatic,
    kWorldCreateObject,
    kWorldMoveObject,
    kWorldDeleteObject,
    kWorldSetProxyPlanes,
    // time
    kSimTime,  // reply: double
    kSetPaused,
    kPaused,  // reply: paused, autoPaused
    kSetTimeScale,
    kHeartbeat,
    kStepManual,
    // state
    kSnapshots,    // reply: prev, prevWall, cur, curWall
    kTakeImpacts,  // reply: impacts
#if defined(LIBFPV_KAMIKAZE)
    kSetWarhead,
    kPollDetonation,  // reply: found, event
    kDetonate,        // reply: bool
#endif

    // Input connection: libfpv-server asks, the client answers.
    kInputUpdate = 200,  // HoverFeedback, wall, dt; reply: channels, radioLost
    kInputResetLatch,    // no reply
    kInputStart,         // reply: ok, error
    kInputStop,          // reply (empty)
    kLog,                // level, line; no reply
};

enum Role : uint8_t { kControl = 0, kInput = 1 };

// Both ends must match exactly: the version, the build's options and the
// layout of the structs that go as they are.
std::string protocolId();

// --- encoding ----------------------------------------------------------------

struct Writer {
    std::string out;
    void bytes(const void *p, size_t n) { out.append((const char *)p, n); }
};

struct Reader {
    const char *p = nullptr;
    size_t left = 0;
    bool ok = true;
    explicit Reader(const std::string &s) : p(s.data()), left(s.size()) {}
    void bytes(void *d, size_t n) {
        if (!ok || n > left) {
            ok = false;
            std::memset(d, 0, n);
            return;
        }
        std::memcpy(d, p, n);
        p += n;
        left -= n;
    }
};

template <typename T>
using IsPlain = std::enable_if_t<std::is_trivially_copyable<T>::value, int>;

template <typename T, IsPlain<T> = 0>
void put(Writer &w, const T &v) { w.bytes(&v, sizeof(v)); }
template <typename T, IsPlain<T> = 0>
void get(Reader &r, T &v) { r.bytes(&v, sizeof(v)); }

void put(Writer &w, const std::string &s);
void get(Reader &r, std::string &s);

template <typename T>
void put(Writer &w, const std::vector<T> &v) {
    put(w, (uint32_t)v.size());
    if constexpr (std::is_trivially_copyable<T>::value) {
        if (!v.empty()) w.bytes(v.data(), v.size() * sizeof(T));
    } else {
        for (const T &e : v) put(w, e);
    }
}
template <typename T>
void get(Reader &r, std::vector<T> &v) {
    uint32_t n = 0;
    get(r, n);
    v.clear();
    if (!r.ok || n > r.left) {  // every element takes at least a byte
        r.ok = false;
        return;
    }
    v.resize(n);
    if constexpr (std::is_trivially_copyable<T>::value) {
        if (n) r.bytes(v.data(), (size_t)n * sizeof(T));
    } else {
        for (T &e : v) get(r, e);
    }
}

// The structs with strings or vectors inside. A field added to one of them
// must be added to its put/get in wire.cpp too.
void put(Writer &w, const CameraSettings &c);
void get(Reader &r, CameraSettings &c);
void put(Writer &w, const VtxSettings &v);
void get(Reader &r, VtxSettings &v);
void put(Writer &w, const DronePreset &p);
void get(Reader &r, DronePreset &p);
void put(Writer &w, const ServerConfig &c);
void get(Reader &r, ServerConfig &c);
void put(Writer &w, const WindState &s);
void get(Reader &r, WindState &s);
void put(Writer &w, const HoverFeedback &f);
void get(Reader &r, HoverFeedback &f);

// Several values in one go: pack(w, a, b, c) / unpack(r, a, b, c).
template <typename... T>
void pack(Writer &w, const T &...v) { (put(w, v), ...); }
template <typename... T>
bool unpack(Reader &r, T &...v) {
    (get(r, v), ...);
    return r.ok;
}

// --- framing -------------------------------------------------------------------
// A message: payload size (uint32), op (uint16), payload.

bool sendMessage(LoopbackSocket &s, uint16_t op, const std::string &payload);
bool recvMessage(LoopbackSocket &s, uint16_t &op, std::string &payload);

template <typename... T>
bool send(LoopbackSocket &s, uint16_t op, const T &...v) {
    Writer w;
    pack(w, v...);
    return sendMessage(s, op, w.out);
}

}  // namespace wire
}  // namespace fpv
