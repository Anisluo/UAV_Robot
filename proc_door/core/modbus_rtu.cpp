#include "modbus_rtu.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr size_t kMaxFrame = 264;   // 256 PDU + slack for CRC
constexpr int    kIdleGapMs = 20;   // "frame finished" silence for raw reads

speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return static_cast<speed_t>(0);
    }
}

int now_ms() {
    using namespace std::chrono;
    return static_cast<int>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace

ModbusRtu::~ModbusRtu() {
    close();
}

uint16_t ModbusRtu::crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int b = 0; b < 8; ++b) {
            if ((crc & 1U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001U);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

bool ModbusRtu::open(const std::string &path, int baud) {
    close();

    if (path.empty()) {
        last_error_ = "uart path is empty";
        return false;
    }
    const speed_t speed = baud_to_speed(baud);
    if (speed == static_cast<speed_t>(0)) {
        last_error_ = "unsupported baudrate " + std::to_string(baud);
        return false;
    }

    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
        last_error_ = "open(" + path + ") failed: " + std::strerror(errno);
        return false;
    }
    // We only needed O_NONBLOCK to dodge a DCD-less port blocking in open().
    // Reads are gated by poll() below, so go back to plain blocking mode.
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    struct termios tio {};
    std::memset(&tio, 0, sizeof(tio));
    tio.c_cflag = static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);  // 8N1
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        last_error_ = std::string("cfsetspeed failed: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        last_error_ = std::string("tcsetattr failed: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }
    (void)tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    last_error_.clear();
    return true;
}

void ModbusRtu::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// These errnos mean the device node is gone, not that the line was noisy:
// the adapter was unplugged, or the board feeding it lost power. Retrying
// on the same fd can only fail, and each retry costs a full deadline — so
// drop it immediately and let the caller's reopen backoff take over.
// Transient failures (timeout, CRC) deliberately do NOT land here; those
// still go through the caller's 5-strike rule.
void ModbusRtu::close_if_device_gone(int err) {
    if (err == EBADF || err == EIO || err == ENODEV || err == ENXIO || err == EPIPE) {
        last_error_ += " (device gone, port closed)";
        close();
    }
}

// Blocking write of the whole buffer.
static bool write_all_fd(int fd, const uint8_t *data, size_t len) {
    size_t done = 0;
    while (done < len) {
        const ssize_t n = ::write(fd, data + done, len - done);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        done += static_cast<size_t>(n);
    }
    return true;
}

int ModbusRtu::transact_raw(const uint8_t *body, size_t body_len,
                            uint8_t *rx, size_t rx_cap, size_t expect) {
    if (fd_ < 0) {
        last_error_ = "port not open";
        return -1;
    }
    if (body == nullptr || body_len == 0 || body_len + 2U > kMaxFrame) {
        last_error_ = "invalid request length";
        return -1;
    }

    uint8_t frame[kMaxFrame];
    std::memcpy(frame, body, body_len);
    const uint16_t crc = crc16(frame, body_len);
    frame[body_len]     = static_cast<uint8_t>(crc & 0xFFU);
    frame[body_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

    (void)tcflush(fd_, TCIFLUSH);
    if (!write_all_fd(fd_, frame, body_len + 2U)) {
        last_error_ = std::string("write failed: ") + std::strerror(errno);
        close_if_device_gone(errno);
        return -1;
    }
    // Deliberately NO tcdrain() here. On a USB-serial adapter that has been
    // unplugged (or whose board lost power) tcdrain can block indefinitely
    // — and this service is a single-threaded loop, so that hangs every RPC
    // including the emergency stop. We don't need it either: the response
    // read below is gated by its own deadline, and the RS485 adapter does
    // its own direction switching in hardware.

    const int deadline = now_ms() + timeout_ms_;
    size_t got = 0;
    while (got < rx_cap) {
        if (expect != 0 && got >= expect) break;

        const int remain = deadline - now_ms();
        if (remain <= 0) break;

        // For "read whatever comes" mode, once bytes have started arriving a
        // short silence means the frame is complete — don't sit out the full
        // deadline on every raw command.
        int wait_ms = remain;
        if (expect == 0 && got > 0 && wait_ms > kIdleGapMs) wait_ms = kIdleGapMs;

        struct pollfd p {};
        p.fd     = fd_;
        p.events = POLLIN;
        const int pr = ::poll(&p, 1, wait_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            last_error_ = std::string("poll failed: ") + std::strerror(errno);
            close_if_device_gone(errno);
            return -1;
        }
        if (pr == 0) {
            if (expect == 0 && got > 0) break;   // idle gap → frame done
            continue;                            // keep waiting until deadline
        }
        const ssize_t n = ::read(fd_, rx + got, rx_cap - got);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            if (n < 0) close_if_device_gone(errno);
            break;
        }
        got += static_cast<size_t>(n);
    }

    if (got == 0) {
        last_error_ = "no response (timeout)";
        return -1;
    }
    last_error_.clear();
    return static_cast<int>(got);
}

bool ModbusRtu::transact(const uint8_t *body, size_t body_len,
                         uint8_t *rx, size_t expect) {
    const int got = transact_raw(body, body_len, rx, expect, expect);
    if (got < 0) return false;

    if (static_cast<size_t>(got) < expect) {
        last_error_ = "short response (" + std::to_string(got) + "/" +
                      std::to_string(expect) + " bytes)";
        return false;
    }
    const uint16_t crc = crc16(rx, expect - 2U);
    if (rx[expect - 2U] != static_cast<uint8_t>(crc & 0xFFU) ||
        rx[expect - 1U] != static_cast<uint8_t>((crc >> 8) & 0xFFU)) {
        last_error_ = "crc mismatch";
        return false;
    }
    if ((rx[1] & 0x80U) != 0U) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "device exception code 0x%02X",
                      static_cast<unsigned>(rx[2]));
        last_error_ = buf;
        return false;
    }
    return true;
}

bool ModbusRtu::read_bits(uint8_t slave, uint8_t func, uint16_t start,
                          uint16_t count, uint8_t *bits_out) {
    if (count == 0U || count > 64U || bits_out == nullptr) {
        last_error_ = "invalid bit count";
        return false;
    }
    const size_t nbytes = (count + 7U) / 8U;
    const uint8_t req[6] = {
        slave, func,
        static_cast<uint8_t>((start >> 8) & 0xFFU), static_cast<uint8_t>(start & 0xFFU),
        static_cast<uint8_t>((count >> 8) & 0xFFU), static_cast<uint8_t>(count & 0xFFU),
    };
    uint8_t rx[kMaxFrame];
    if (!transact(req, sizeof(req), rx, 5U + nbytes)) return false;

    // rx: [slave][func][byte_count][data...][crc_lo][crc_hi]
    if (rx[2] != static_cast<uint8_t>(nbytes)) {
        last_error_ = "unexpected byte count in response";
        return false;
    }
    for (uint16_t i = 0; i < count; ++i) {
        bits_out[i] = static_cast<uint8_t>((rx[3U + (i / 8U)] >> (i % 8U)) & 1U);
    }
    return true;
}

bool ModbusRtu::read_coils(uint8_t slave, uint16_t start, uint16_t count,
                           uint8_t *bits_out) {
    return read_bits(slave, 0x01U, start, count, bits_out);
}

bool ModbusRtu::read_discrete_inputs(uint8_t slave, uint16_t start, uint16_t count,
                                     uint8_t *bits_out) {
    return read_bits(slave, 0x02U, start, count, bits_out);
}

bool ModbusRtu::read_holding(uint8_t slave, uint16_t start, uint16_t count,
                             uint16_t *regs_out) {
    if (count == 0U || count > 32U || regs_out == nullptr) {
        last_error_ = "invalid register count";
        return false;
    }
    const uint8_t req[6] = {
        slave, 0x03U,
        static_cast<uint8_t>((start >> 8) & 0xFFU), static_cast<uint8_t>(start & 0xFFU),
        static_cast<uint8_t>((count >> 8) & 0xFFU), static_cast<uint8_t>(count & 0xFFU),
    };
    uint8_t rx[kMaxFrame];
    if (!transact(req, sizeof(req), rx, 5U + static_cast<size_t>(count) * 2U)) return false;

    for (uint16_t i = 0; i < count; ++i) {
        regs_out[i] = static_cast<uint16_t>((static_cast<uint16_t>(rx[3U + i * 2U]) << 8) |
                                            static_cast<uint16_t>(rx[4U + i * 2U]));
    }
    return true;
}

bool ModbusRtu::write_coil(uint8_t slave, uint16_t coil, bool on) {
    const uint16_t value = on ? 0xFF00U : 0x0000U;
    const uint8_t req[6] = {
        slave, 0x05U,
        static_cast<uint8_t>((coil >> 8) & 0xFFU), static_cast<uint8_t>(coil & 0xFFU),
        static_cast<uint8_t>((value >> 8) & 0xFFU), static_cast<uint8_t>(value & 0xFFU),
    };
    uint8_t rx[kMaxFrame];
    // Echo of the request is the normal 0x05 reply.
    return transact(req, sizeof(req), rx, 8U);
}

bool ModbusRtu::write_coils(uint8_t slave, uint16_t start, uint16_t count, bool on) {
    if (count == 0U || count > 64U) {
        last_error_ = "invalid coil count";
        return false;
    }
    const size_t nbytes = (count + 7U) / 8U;
    uint8_t req[7 + 8] = {};
    req[0] = slave;
    req[1] = 0x0FU;
    req[2] = static_cast<uint8_t>((start >> 8) & 0xFFU);
    req[3] = static_cast<uint8_t>(start & 0xFFU);
    req[4] = static_cast<uint8_t>((count >> 8) & 0xFFU);
    req[5] = static_cast<uint8_t>(count & 0xFFU);
    req[6] = static_cast<uint8_t>(nbytes);
    for (uint16_t i = 0; i < count; ++i) {
        if (on) req[7U + (i / 8U)] |= static_cast<uint8_t>(1U << (i % 8U));
    }
    uint8_t rx[kMaxFrame];
    return transact(req, 7U + nbytes, rx, 8U);
}
