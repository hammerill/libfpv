// libfpv - loopback TCP sockets (Winsock or BSD sockets).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "ipc/socket.h"

#include <cstring>
#include <mutex>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET NativeSocket;
typedef int SockLen;
#define FPV_NATIVE_INVALID INVALID_SOCKET
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int NativeSocket;
typedef socklen_t SockLen;
#define FPV_NATIVE_INVALID (-1)
#endif

namespace fpv {

namespace {

NativeSocket native(intptr_t fd) { return (NativeSocket)fd; }

std::string lastError(const char *what) {
#if defined(_WIN32)
    return std::string(what) + " failed (WSA error " + std::to_string(WSAGetLastError()) + ")";
#else
    return std::string(what) + " failed: " + std::strerror(errno);
#endif
}

bool startup(std::string &error) {
#if defined(_WIN32)
    static std::once_flag once;
    static bool ok = false;
    std::call_once(once, [] {
        WSADATA d;
        ok = WSAStartup(MAKEWORD(2, 2), &d) == 0;
    });
    if (!ok) error = "WSAStartup failed";
    return ok;
#else
    (void)error;
    return true;
#endif
}

NativeSocket makeSocket() {
#if defined(_WIN32)
    // not inherited: libfpv-server is started without handle inheritance anyway
    return WSASocketW(AF_INET, SOCK_STREAM, IPPROTO_TCP, nullptr, 0, WSA_FLAG_NO_HANDLE_INHERIT);
#elif defined(SOCK_CLOEXEC)
    return ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
#else
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd >= 0) fcntl(fd, F_SETFD, FD_CLOEXEC);
    return fd;
#endif
}

void tune(NativeSocket s) {
    // small messages both ways: no Nagle delay
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));
#if defined(SO_NOSIGPIPE)
    setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
}

sockaddr_in loopback(uint16_t port) {
    sockaddr_in a;
    std::memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return a;
}

bool waitFor(NativeSocket s, double timeoutSec) {
    int ms = timeoutSec < 0 ? -1 : (int)(timeoutSec * 1000.0);
#if defined(_WIN32)
    WSAPOLLFD p = {s, POLLRDNORM, 0};
    return WSAPoll(&p, 1, ms) > 0;
#else
    pollfd p = {s, POLLIN, 0};
    int r;
    do {
        r = ::poll(&p, 1, ms);
    } while (r < 0 && errno == EINTR);
    return r > 0;
#endif
}

}  // namespace

LoopbackSocket &LoopbackSocket::operator=(LoopbackSocket &&o) noexcept {
    if (this != &o) {
        close();
        m_fd = o.m_fd;
        o.m_fd = kInvalid;
    }
    return *this;
}

bool LoopbackSocket::listen(uint16_t &port, std::string &error) {
    close();
    if (!startup(error)) return false;
    NativeSocket s = makeSocket();
    if (s == FPV_NATIVE_INVALID) {
        error = lastError("socket");
        return false;
    }
    m_fd = (intptr_t)s;
    sockaddr_in a = loopback(0);
    if (::bind(s, (sockaddr *)&a, sizeof(a)) != 0 || ::listen(s, 4) != 0) {
        error = lastError("bind/listen");
        close();
        return false;
    }
    SockLen len = sizeof(a);
    if (::getsockname(s, (sockaddr *)&a, &len) != 0) {
        error = lastError("getsockname");
        close();
        return false;
    }
    port = ntohs(a.sin_port);
    return true;
}

bool LoopbackSocket::accept(LoopbackSocket &out, double timeoutSec, std::string &error) {
    if (!valid()) {
        error = "not listening";
        return false;
    }
    if (!waitFor(native(m_fd), timeoutSec)) {
        error = "timed out";
        return false;
    }
    NativeSocket c = ::accept(native(m_fd), nullptr, nullptr);
    if (c == FPV_NATIVE_INVALID) {
        error = lastError("accept");
        return false;
    }
#if !defined(_WIN32) && !defined(SOCK_CLOEXEC)
    fcntl(c, F_SETFD, FD_CLOEXEC);
#endif
    tune(c);
    out.close();
    out.m_fd = (intptr_t)c;
    return true;
}

bool LoopbackSocket::connect(uint16_t port, std::string &error) {
    close();
    if (!startup(error)) return false;
    NativeSocket s = makeSocket();
    if (s == FPV_NATIVE_INVALID) {
        error = lastError("socket");
        return false;
    }
    m_fd = (intptr_t)s;
    sockaddr_in a = loopback(port);
    if (::connect(s, (sockaddr *)&a, sizeof(a)) != 0) {
        error = lastError("connect");
        close();
        return false;
    }
    tune(s);
    return true;
}

bool LoopbackSocket::sendAll(const void *data, size_t size) {
    const char *p = (const char *)data;
    while (size > 0 && valid()) {
        int chunk = size > (1u << 30) ? (1 << 30) : (int)size;
#if defined(_WIN32)
        int n = ::send(native(m_fd), p, chunk, 0);
#elif defined(MSG_NOSIGNAL)
        ssize_t n = ::send(native(m_fd), p, (size_t)chunk, MSG_NOSIGNAL);
#else
        ssize_t n = ::send(native(m_fd), p, (size_t)chunk, 0);
#endif
        if (n <= 0) {
#if !defined(_WIN32)
            if (n < 0 && errno == EINTR) continue;
#endif
            return false;
        }
        p += n;
        size -= (size_t)n;
    }
    return size == 0;
}

bool LoopbackSocket::recvAll(void *data, size_t size) {
    char *p = (char *)data;
    while (size > 0 && valid()) {
        int chunk = size > (1u << 30) ? (1 << 30) : (int)size;
#if defined(_WIN32)
        int n = ::recv(native(m_fd), p, chunk, 0);
#else
        ssize_t n = ::recv(native(m_fd), p, (size_t)chunk, 0);
#endif
        if (n <= 0) {
#if !defined(_WIN32)
            if (n < 0 && errno == EINTR) continue;
#endif
            return false;
        }
        p += n;
        size -= (size_t)n;
    }
    return size == 0;
}

bool LoopbackSocket::waitReadable(double timeoutSec) { return valid() && waitFor(native(m_fd), timeoutSec); }

void LoopbackSocket::shutdown() {
    if (!valid()) return;
#if defined(_WIN32)
    ::shutdown(native(m_fd), SD_BOTH);
#else
    ::shutdown(native(m_fd), SHUT_RDWR);
#endif
}

void LoopbackSocket::close() {
    if (!valid()) return;
#if defined(_WIN32)
    ::closesocket(native(m_fd));
#else
    ::close(native(m_fd));
#endif
    m_fd = kInvalid;
}

}  // namespace fpv
