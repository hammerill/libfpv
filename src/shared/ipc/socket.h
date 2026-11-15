// libfpv - a TCP socket on the loopback interface, for split builds: the
// game's libfpv and libfpv-server talk over two of them (docs/DECISIONS.md,
// D37). Blocking, one reader and one writer at a time.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace fpv {

class LoopbackSocket {
public:
    LoopbackSocket() = default;
    ~LoopbackSocket() { close(); }
    LoopbackSocket(const LoopbackSocket &) = delete;
    LoopbackSocket &operator=(const LoopbackSocket &) = delete;
    LoopbackSocket(LoopbackSocket &&o) noexcept : m_fd(o.m_fd) { o.m_fd = kInvalid; }
    LoopbackSocket &operator=(LoopbackSocket &&o) noexcept;

    // Listens on 127.0.0.1 at a port the system picks.
    bool listen(uint16_t &port, std::string &error);
    // The next connection, or false after timeoutSec.
    bool accept(LoopbackSocket &out, double timeoutSec, std::string &error);
    bool connect(uint16_t port, std::string &error);

    bool sendAll(const void *data, size_t size);
    bool recvAll(void *data, size_t size);  // false on error or when the other side closed
    // Waits for data up to timeoutSec; true when some arrived (or the peer closed).
    bool waitReadable(double timeoutSec);

    bool valid() const { return m_fd != kInvalid; }
    // Ends both directions: a recvAll blocked on another thread returns false.
    void shutdown();
    void close();

private:
    static constexpr intptr_t kInvalid = -1;
    intptr_t m_fd = kInvalid;
};

}  // namespace fpv
