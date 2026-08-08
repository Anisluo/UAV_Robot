#ifndef UAV_PROC_DOOR_MODBUS_RTU_H
#define UAV_PROC_DOOR_MODBUS_RTU_H

#include <cstddef>
#include <cstdint>
#include <string>

// Minimal Modbus-RTU master over a POSIX serial port (8N1).
//
// Scope is exactly what the 中盛 digital-IO module needs — the same set of
// function codes doorMonitor/DioPanel.ps1 exercises:
//
//   0x01 read coils           → relay output state
//   0x02 read discrete inputs → dry contacts / limit switches
//   0x03 read holding regs    → only used as an "are you alive" probe
//                               (register 0x32 holds the station id)
//   0x05 write single coil    → switch one relay
//   0x0F write multiple coils → all-on / all-off
//
// Every call is one synchronous transaction: flush RX, write the framed
// request, then read the expected byte count with a deadline. The module
// answers within a few ms at 38400, so 150 ms is already ~20x margin.
//
// Keep this deadline tight: it is paid in full on every read when the
// module is *not* answering, and proc_door's service loop is single
// threaded. The 400 ms the PowerShell panel uses starved that loop badly
// enough to wedge the whole RPC chain (see proc_door/README 排错).
class ModbusRtu {
 public:
    ModbusRtu() = default;
    ~ModbusRtu();

    ModbusRtu(const ModbusRtu &) = delete;
    ModbusRtu &operator=(const ModbusRtu &) = delete;

    bool open(const std::string &path, int baud);
    void close();
    bool is_open() const { return fd_ >= 0; }

    void set_timeout_ms(int ms) { timeout_ms_ = (ms > 0) ? ms : 1; }

    // bits_out receives `count` bytes, one per bit, value 0 or 1.
    bool read_coils(uint8_t slave, uint16_t start, uint16_t count, uint8_t *bits_out);
    bool read_discrete_inputs(uint8_t slave, uint16_t start, uint16_t count, uint8_t *bits_out);

    bool read_holding(uint8_t slave, uint16_t start, uint16_t count, uint16_t *regs_out);

    bool write_coil(uint8_t slave, uint16_t coil, bool on);
    bool write_coils(uint8_t slave, uint16_t start, uint16_t count, bool on);

    // Raw escape hatch — mirrors the panel's "手动指令(HEX，自动补CRC)" box.
    // `body` is the frame *including* the slave address and *excluding* the
    // CRC, which is appended here. Returns bytes received (CRC included) or
    // -1 on error. `expect` is how many bytes to wait for; pass 0 to grab
    // whatever arrives before the deadline.
    int transact_raw(const uint8_t *body, size_t body_len,
                     uint8_t *rx, size_t rx_cap, size_t expect);

    const std::string &last_error() const { return last_error_; }

    static uint16_t crc16(const uint8_t *data, size_t len);

 private:
    bool transact(const uint8_t *body, size_t body_len, uint8_t *rx, size_t expect);
    // Drop the fd when errno says the device itself vanished (unplugged /
    // powered off), so we stop paying a deadline per doomed retry.
    void close_if_device_gone(int err);
    bool read_bits(uint8_t slave, uint8_t func, uint16_t start, uint16_t count,
                   uint8_t *bits_out);

    int         fd_         = -1;
    int         timeout_ms_ = 150;
    std::string last_error_;
};

#endif  // UAV_PROC_DOOR_MODBUS_RTU_H
