#ifndef WAVESHARE_USB_HAL_H
#define WAVESHARE_USB_HAL_H

// RadioLibHal backend for the Waveshare USB-LoRa "dumb bridge" firmware:
// https://github.com/<user>/waveshare_usb_lora_bridge
//
// Unlike Ch341Hal (USBHal.h), which talks raw SPI/GPIO over USB directly to
// a CH341 bridge chip, this device exposes a custom framed command protocol
// (SPI_XFER / GPIO_READ / GPIO_WRITE) over a CH343 USB-serial virtual COM
// port at 921600 baud. The GD32F103C8T6 MCU on the board is the one that
// actually drives SPI/GPIO against the SX1262; this HAL just speaks the
// firmware's wire protocol to it.
//
// Wire protocol (see the firmware repo's README.md for the authoritative
// spec):
//   Host -> device: [0xAA][CMD][LEN_LO][LEN_HI][PAYLOAD x LEN][CRC8]
//   Device -> host: [0xAA][STATUS][LEN_LO][LEN_HI][PAYLOAD x LEN][CRC8]
//   CRC8 is Dallas/Maxim (poly 0x31, init 0x00) over CMD/STATUS+LEN+PAYLOAD.
//
// GPIO pin IDs (fixed by the firmware, not configurable over the wire):
//   0 NRESET (write)  1 RFSWITCH (write)  2 LED_RX (write)  3 LED_TX (write)
//   4 BUSY (read)     5 DIO1 (read)       6 BUTTON (read)
//
// Chip-select is handled automatically by the firmware's SPI_XFER command
// (asserted low for the transfer's duration, then deasserted) -- there is
// no GPIO pin ID for "CS" on the wire. RadioLib (Module::SPItransfer,
// src/Module.cpp) still calls hal->digitalWrite(csPin, LOW/HIGH) itself
// around every hal->spiTransfer() call, so this HAL must treat whichever
// pin number is configured as "CS" (WAVESHARE_PIN_CS, a sentinel that
// doesn't collide with any real firmware pin ID) as a pure no-op -- sending
// a real GPIO_WRITE for it would get STATUS_ERR_PIN back from the device.
//
// The firmware also has no real interrupt line host-side: DIO1 must be
// polled. This mirrors Ch341Hal/libch341-spi-userspace's own approach
// (a background thread polling all pins and synthesizing edges), just
// over a serial link instead of USB bulk transfers, and against a single
// shared half-duplex byte stream instead of independent USB transactions
// -- so every protocol operation (SPI transfers, GPIO reads/writes, and
// the poll thread's own GPIO reads) is serialized behind one mutex.

#include "platform/portduino/PortduinoGlue.h"
#include <RadioLib.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

#define WAVESHARE_PIN_NRESET (0)
#define WAVESHARE_PIN_RFSWITCH (1)
#define WAVESHARE_PIN_LED_RX (2)
#define WAVESHARE_PIN_LED_TX (3)
#define WAVESHARE_PIN_BUSY (4)
#define WAVESHARE_PIN_DIO1 (5)
#define WAVESHARE_PIN_BUTTON (6)
// Sentinel for "CS" -- not a real firmware GPIO id, always a host-side no-op.
#define WAVESHARE_PIN_CS (255)

#define WAVESHARE_INPUT (0)
#define WAVESHARE_OUTPUT (1)
#define WAVESHARE_LOW (0)
#define WAVESHARE_HIGH (1)
#define WAVESHARE_RISING (1)
#define WAVESHARE_FALLING (2)

// Framed serial transport implementing the wire protocol. Not itself a
// RadioLibHal -- just the bit that talks to the device.
class WaveshareSerialTransport
{
  public:
    explicit WaveshareSerialTransport(const std::string &port, uint32_t baud = 921600) : baud(baud)
    {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            throw std::runtime_error("Could not open serial port " + port);
        }

        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            throw std::runtime_error("tcgetattr failed on " + port);
        }
        cfmakeraw(&tty);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

#ifdef __APPLE__
        // macOS termios only defines standard rates up to B230400; 921600
        // needs the IOSSIOSPEED ioctl instead (same approach pyserial uses
        // on this platform).
        cfsetspeed(&tty, B9600);
#else
        cfsetspeed(&tty, baud);
#endif
        if (!applyLineSettings()) {
            close(fd);
            throw std::runtime_error("failed to set " + std::to_string(baud) + " baud on " + port);
        }
        // Clear O_NONBLOCK now that the port is configured -- reads below
        // use select() for timeouts instead.
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

        tcflush(fd, TCIOFLUSH);
    }

    ~WaveshareSerialTransport()
    {
        if (fd >= 0)
            close(fd);
    }

    // Sends one command frame and reads back the response. Returns true on
    // a well-formed, CRC-valid, STATUS_OK response; false otherwise (leaves
    // in_error set so the HAL can surface it once rather than spamming).
    // inLen is updated to the actual response payload length.
    //
    // attempts > 1 retries after a failure; only pass that for idempotent
    // commands (GPIO_READ, PING) -- a lost *response* to an SPI_XFER or
    // GPIO_WRITE means the device already executed it.
    bool transact(uint8_t cmd, const uint8_t *outPayload, uint16_t outLen, uint8_t *inPayload, uint16_t maxInLen,
                  uint16_t *inLen, int attempts = 1)
    {
        std::lock_guard<std::mutex> lock(ioMutex);

        std::string why;
        for (int attempt = 1; attempt <= attempts; ++attempt) {
            if (transactOnce(cmd, outPayload, outLen, inPayload, maxInLen, inLen, why)) {
                if (hadError)
                    logDeviceState("link recovered");
                hadError = false;
                return true;
            }
            // Whatever went wrong, the byte stream may now be misaligned
            // (unread tail of a response, or a partial command the device
            // is still waiting on). Get both sides back to a frame boundary
            // before anyone sends again.
            resync();
            if (attempt < attempts)
                LOG_WARN("WaveshareUsbHal: cmd 0x%02x: %s, retrying", cmd, why.c_str());
        }
        bool firstError = !hadError;
        markError("cmd " + std::to_string(cmd) + ": " + why);
        if (firstError)
            logDeviceState("after failure");
        return false;
    }

    bool inError() const { return hadError; }

  private:
    bool transactOnce(uint8_t cmd, const uint8_t *outPayload, uint16_t outLen, uint8_t *inPayload, uint16_t maxInLen,
                      uint16_t *inLen, std::string &why)
    {
        uint8_t header[3] = {cmd, (uint8_t)(outLen & 0xFF), (uint8_t)((outLen >> 8) & 0xFF)};
        uint8_t crcBuf[3 + 256];
        std::memcpy(crcBuf, header, 3);
        if (outLen > 0)
            std::memcpy(crcBuf + 3, outPayload, outLen);
        uint8_t crc = crc8(crcBuf, 3 + outLen);

        uint8_t frame[1 + 3 + 256 + 1];
        size_t frameLen = 0;
        frame[frameLen++] = 0xAA;
        std::memcpy(frame + frameLen, header, 3);
        frameLen += 3;
        if (outLen > 0) {
            std::memcpy(frame + frameLen, outPayload, outLen);
            frameLen += outLen;
        }
        frame[frameLen++] = crc;

        if (!writeAll(frame, frameLen)) {
            why = "write failed";
            return false;
        }

        uint8_t sof;
        if (!readAll(&sof, 1)) {
            why = "no response (" + readFailure + ")";
            return false;
        }
        if (sof != 0xAA) {
            char buf[48];
            snprintf(buf, sizeof(buf), "bad SOF 0x%02x in response", sof);
            why = buf;
            return false;
        }
        uint8_t respHeader[3];
        if (!readAll(respHeader, 3)) {
            why = "timed out reading response header";
            return false;
        }
        uint8_t status = respHeader[0];
        uint16_t respLen = respHeader[1] | (respHeader[2] << 8);
        if (respLen > maxInLen) {
            why = "response payload larger than buffer";
            return false;
        }
        if (respLen > 0 && !readAll(inPayload, respLen)) {
            why = "timed out reading response payload";
            return false;
        }
        uint8_t respCrc;
        if (!readAll(&respCrc, 1)) {
            why = "timed out reading response CRC";
            return false;
        }

        uint8_t respCrcBuf[3 + 256];
        std::memcpy(respCrcBuf, respHeader, 3);
        if (respLen > 0)
            std::memcpy(respCrcBuf + 3, inPayload, respLen);
        if (crc8(respCrcBuf, 3 + respLen) != respCrc) {
            why = "CRC mismatch in response";
            return false;
        }
        if (status != 0x00 /* STATUS_OK */) {
            why = "device returned error status " + std::to_string(status);
            return false;
        }

        if (inLen)
            *inLen = respLen;
        return true;
    }

    // PINGs the bridge and logs its uptime and boot reset cause, so a
    // silent spell can be told apart: device rebooted (small uptime; IWDG
    // vs. brownout/POR vs. pin in the flags), device alive but dropping
    // frames (large uptime), or link gone (no reply / read() errors).
    // Caller holds ioMutex; only used after logging is up.
    void logDeviceState(const char *when)
    {
        uint8_t resp[8];
        uint16_t respLen = 0;
        std::string pingWhy;
        if (!transactOnce(0x00 /* PING */, nullptr, 0, resp, sizeof(resp), &respLen, pingWhy)) {
            resync();
            LOG_WARN("WaveshareUsbHal %s: PING failed: %s", when, pingWhy.c_str());
            return;
        }
        if (respLen < 8) {
            LOG_WARN("WaveshareUsbHal %s: PING ok (fw v%d, no uptime/reset info)", when, respLen >= 5 ? resp[4] : -1);
            return;
        }
        uint8_t f = resp[5];
        unsigned upSec = resp[6] | (resp[7] << 8);
        LOG_WARN("WaveshareUsbHal %s: device uptime %us, reset flags 0x%02x%s%s%s%s%s%s", when, upSec, f,
                 (f & 0x80) ? " LPWR" : "", (f & 0x40) ? " WWDG" : "", (f & 0x20) ? " IWDG" : "",
                 (f & 0x10) ? " SFT" : "", (f & 0x08) ? " POR/PDR" : "", (f & 0x04) ? " PIN" : "");
    }

    bool applyLineSettings()
    {
        if (tcsetattr(fd, TCSANOW, &tty) != 0)
            return false;
#ifdef __APPLE__
        speed_t speed = baud;
        if (ioctl(fd, IOSSIOSPEED, &speed) < 0)
            return false;
#endif
        return true;
    }

    // The CH343 can silently lose its baud rate while the kernel still
    // believes it is set (observed: MCU up and fine, no replies at all until
    // the port's speed was changed and changed back). Linux cdc_acm only
    // sends SET_LINE_CODING when the termios speed actually changes, so
    // re-applying the same speed does nothing -- bounce through a different
    // one to force the line coding out to the chip again.
    void resendLineCoding()
    {
        struct termios other = tty;
        cfsetspeed(&other, B115200);
        tcsetattr(fd, TCSANOW, &other);
        if (!applyLineSettings())
            LOG_WARN("WaveshareUsbHal: failed to re-apply %u baud", (unsigned)baud);
    }

    // Discards incoming bytes until the line has been quiet for longer than
    // the bridge firmware's FRAME_TIMEOUT_MS (20ms). After that, the device's
    // parser has abandoned any partial command and any late or half-read
    // response is gone, so the next transact() starts on a frame boundary on
    // both ends. Bounded so a babbling device can't wedge the caller here.
    // Also re-sends the line coding, in case the CH343 lost its baud rate.
    void resync()
    {
        static const int QUIET_MS = 30;
        uint8_t junk[64];
        for (int i = 0; i < 64; ++i) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            struct timeval tv = {0, QUIET_MS * 1000};
            if (select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0)
                break;
            if (read(fd, junk, sizeof(junk)) <= 0)
                break;
        }
        resendLineCoding();
        tcflush(fd, TCIFLUSH);
    }

    static uint8_t crc8(const uint8_t *data, size_t len)
    {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < len; ++i) {
            uint8_t b = data[i];
            for (uint8_t bit = 0; bit < 8; ++bit) {
                uint8_t mix = (crc ^ b) & 0x01;
                crc >>= 1;
                if (mix)
                    crc ^= 0x8C;
                b >>= 1;
            }
        }
        return crc;
    }

    bool writeAll(const uint8_t *buf, size_t len)
    {
        size_t written = 0;
        while (written < len) {
            ssize_t n = write(fd, buf + written, len - written);
            if (n < 0)
                return false;
            written += (size_t)n;
        }
        return true;
    }

    // Reads exactly len bytes, or times out (per-byte timeout, generous
    // since this is a low-throughput control protocol, not the bulk path).
    bool readAll(uint8_t *buf, size_t len, int timeoutMs = 500)
    {
        size_t got = 0;
        while (got < len) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            struct timeval tv;
            tv.tv_sec = timeoutMs / 1000;
            tv.tv_usec = (timeoutMs % 1000) * 1000;
            int ret = select(fd + 1, &fds, nullptr, nullptr, &tv);
            if (ret == 0) {
                readFailure = "timeout after " + std::to_string(got) + "/" + std::to_string(len) + " bytes";
                return false;
            }
            if (ret < 0) {
                readFailure = std::string("select: ") + strerror(errno);
                return false;
            }
            ssize_t n = read(fd, buf + got, len - got);
            if (n == 0) {
                // Readable but EOF: the tty went away (USB re-enumerated?).
                readFailure = "read() returned EOF -- port gone?";
                return false;
            }
            if (n < 0) {
                readFailure = std::string("read: ") + strerror(errno);
                return false;
            }
            got += (size_t)n;
        }
        return true;
    }

    void markError(const std::string &why)
    {
        if (!hadError)
            LOG_ERROR("WaveshareUsbHal: %s", why.c_str());
        hadError = true;
        portduino_status.LoRa_in_error = true;
    }

    int fd = -1;
    uint32_t baud;
    struct termios tty;
    std::mutex ioMutex;
    std::string readFailure;
    bool hadError = false;
};

// the HAL must inherit from the base RadioLibHal class
// and implement all of its virtual methods
class WaveshareUsbHal : public RadioLibHal
{
  public:
    explicit WaveshareUsbHal(const std::string &serialPort)
        : RadioLibHal(WAVESHARE_INPUT, WAVESHARE_OUTPUT, WAVESHARE_LOW, WAVESHARE_HIGH, WAVESHARE_RISING, WAVESHARE_FALLING),
          transport(serialPort)
    {
        uint8_t resp[8];
        uint16_t respLen = 0;
        if (!transport.transact(0x00 /* PING */, nullptr, 0, resp, sizeof(resp), &respLen) || respLen < 4 ||
            std::memcmp(resp, "WSLB", 4) != 0) {
            throw std::runtime_error("Waveshare USB-LoRa bridge did not respond to PING on " + serialPort);
        }
        // Not LOG_INFO: this constructor runs from portduinoSetup(), before
        // the logging singleton is initialized (confirmed by an
        // EXC_BAD_ACCESS crash here during real-hardware testing). Ch341Hal
        // avoids this the same way -- plain std::cout in the constructor,
        // LOG_* only in methods called later during normal operation.
        std::cout << "WaveshareUsbHal: connected on " << serialPort << " (fw version "
                  << (respLen >= 5 ? (int)resp[4] : -1) << ")" << std::endl;
    }

    ~WaveshareUsbHal()
    {
        stopPolling();
    }

    void init() override {}
    void term() override { stopPolling(); }

    void pinMode(uint32_t pin, uint32_t mode) override
    {
        // Pin directions are fixed by the firmware (NRESET/RFSWITCH/LED_*
        // are outputs, BUSY/DIO1/BUTTON are inputs) -- there's no
        // SET_DIRECTION command, so this is a no-op. CS is always a no-op
        // too, handled automatically by the device's SPI_XFER.
        (void)pin;
        (void)mode;
    }

    void digitalWrite(uint32_t pin, uint32_t value) override
    {
        if (pin == WAVESHARE_PIN_CS || pin == RADIOLIB_NC)
            return;
        uint8_t payload[2] = {(uint8_t)pin, (uint8_t)(value ? 1 : 0)};
        uint8_t resp[1];
        uint16_t respLen = 0;
        transport.transact(0x02 /* GPIO_WRITE */, payload, sizeof(payload), resp, sizeof(resp), &respLen);
    }

    uint32_t digitalRead(uint32_t pin) override
    {
        if (pin == WAVESHARE_PIN_CS || pin == RADIOLIB_NC)
            return 0;
        uint8_t payload[1] = {(uint8_t)pin};
        uint8_t resp[1] = {0};
        uint16_t respLen = 0;
        // GPIO_READ is idempotent and makes up nearly all traffic (the
        // poll thread), so a single wire glitch is retried here instead of
        // tripping LoRa_in_error and a full radio re-init.
        if (!transport.transact(0x03 /* GPIO_READ */, payload, sizeof(payload), resp, sizeof(resp), &respLen, 2) ||
            respLen < 1)
            return 0;
        return resp[0];
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override
    {
        if (interruptNum == RADIOLIB_NC)
            return;
        {
            std::lock_guard<std::mutex> lock(pollMutex);
            InterruptEntry &entry = interrupts[interruptNum];
            entry.callback = interruptCb;
            entry.mode = mode;
            entry.previousState = 255; // force a fresh baseline before edge detection
        }
        startPolling();
    }

    // Only edits the map; the poll thread stays alive. Stopping/joining it here
    // deadlocked: RadioLib detaches from both the main thread (setStandby) and
    // from inside the ISR callback running on the poll thread itself.
    void detachInterrupt(uint32_t interruptNum) override
    {
        std::lock_guard<std::mutex> lock(pollMutex);
        interrupts.erase(interruptNum);
    }

    void delay(unsigned long ms) override { delayMicroseconds(ms * 1000); }

    void delayMicroseconds(unsigned long us) override
    {
        if (us == 0) {
            sched_yield();
            return;
        }
        usleep(us);
    }

    void yield() override { sched_yield(); }

    unsigned long millis() override
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return (tv.tv_sec * 1000ULL) + (tv.tv_usec / 1000ULL);
    }

    unsigned long micros() override
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return (tv.tv_sec * 1000000ULL) + tv.tv_usec;
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override
    {
        std::cerr << "pulseIn for pin " << pin << " is not supported!" << std::endl;
        return 0;
    }

    void spiBegin() {}
    void spiBeginTransaction() {}

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in)
    {
        // Our SPI_XFER command caps payload at 256 bytes (see firmware
        // README); RadioLib/SX126x transactions are well under that.
        if (len > 256) {
            LOG_ERROR("WaveshareUsbHal spiTransfer: %zu bytes exceeds the 256-byte SPI_XFER limit", len);
            return;
        }
        uint16_t inLen = 0;
        transport.transact(0x01 /* SPI_XFER */, out, (uint16_t)len, in, (uint16_t)len, &inLen);
    }

    void spiEndTransaction() {}
    void spiEnd() {}

  private:
    struct InterruptEntry {
        void (*callback)(void) = nullptr;
        uint32_t mode = WAVESHARE_RISING;
        uint8_t previousState = 255;
    };

    void startPolling()
    {
        bool expected = false;
        if (!polling.compare_exchange_strong(expected, true))
            return; // already running
        pollThread = std::thread([this]() { pollLoop(); });
    }

    // Only called from term()/destructor, never from the poll thread.
    void stopPolling()
    {
        if (polling.exchange(false) && pollThread.joinable())
            pollThread.join();
    }

    void pollLoop()
    {
        // Tighter than Ch341Hal's ~33ms USB-bulk poll: our per-request cost
        // is a serial round trip, and LoRa RX/TX-done latency benefits from
        // catching the DIO1 edge promptly. Tune against real hardware.
        while (polling.load()) {
            std::vector<uint32_t> pins;
            {
                std::lock_guard<std::mutex> lock(pollMutex);
                for (auto &kv : interrupts)
                    pins.push_back(kv.first);
            }
            for (uint32_t pin : pins) {
                uint32_t state = digitalRead(pin);
                void (*callback)(void) = nullptr;
                {
                    // Re-look-up by pin: the entry may have been erased (or
                    // re-created) while digitalRead() was in flight.
                    std::lock_guard<std::mutex> lock(pollMutex);
                    auto it = interrupts.find(pin);
                    if (it == interrupts.end())
                        continue;
                    InterruptEntry &entry = it->second;
                    if (entry.previousState != 255 && entry.previousState != state) {
                        bool rising = (entry.previousState == 0 && state == 1);
                        bool matches = (rising && entry.mode == WAVESHARE_RISING) ||
                                       (!rising && entry.mode == WAVESHARE_FALLING);
                        if (matches)
                            callback = entry.callback;
                    }
                    entry.previousState = (uint8_t)state;
                }
                // Outside pollMutex: the ISR calls straight back into detachInterrupt().
                if (callback)
                    callback();
            }
            usleep(2000); // ~500Hz poll
        }
    }

    WaveshareSerialTransport transport;
    std::atomic<bool> polling{false};
    std::thread pollThread;
    std::mutex pollMutex;
    std::map<uint32_t, InterruptEntry> interrupts;
};

#endif
