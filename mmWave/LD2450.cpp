#include "LD2450.h"

namespace {
const uint8_t kDataHeader[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t kDataFooter[2] = {0x55, 0xCC};
const uint8_t kCmdHeader[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t kCmdFooter[4] = {0x04, 0x03, 0x02, 0x01};

const uint16_t CMD_ENABLE_CONFIG = 0x00FF;
const uint16_t CMD_END_CONFIG = 0x00FE;
const uint16_t CMD_SINGLE_TARGET = 0x0080;
const uint16_t CMD_MULTI_TARGET = 0x0090;
const uint16_t CMD_READ_FIRMWARE = 0x00A0;
const uint16_t CMD_FACTORY_RESET = 0x00A2;
const uint16_t CMD_RESTART = 0x00A3;
const uint16_t CMD_SET_ZONE_FILTER = 0x00C2;

inline uint16_t le16(const uint8_t* p) { return p[0] | (uint16_t(p[1]) << 8); }
inline uint32_t le32(const uint8_t* p) {
    return p[0] | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
}  // namespace

#if defined(ARDUINO_ARCH_ESP32)
void LD2450::begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin, uint32_t baud) {
    serial.setRxBufferSize(1024);
    serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    begin(static_cast<Stream&>(serial));
}
#endif

void LD2450::begin(Stream& stream) {
    stream_ = &stream;
    idx_ = 0;
    newFrame_ = false;
}

int16_t LD2450::decodeSigned(uint16_t raw) {
    int16_t mag = raw & 0x7FFF;
    return (raw & 0x8000) ? mag : -mag;
}

uint8_t LD2450::targetCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < kMaxTargets; ++i) n += targets_[i].valid;
    return n;
}

bool LD2450::update() {
    if (!stream_) return false;
    while (stream_->available() > 0) {
        feed(static_cast<uint8_t>(stream_->read()));
        ++bytesRx_;
    }
    bool got = newFrame_;
    newFrame_ = false;
    return got;
}

// Byte-wise state machine: sync on the 4-byte header, collect 30 bytes,
// verify footer. Never blocks.
bool LD2450::feed(uint8_t b) {
    if (idx_ < 4) {
        if (b == kDataHeader[idx_]) {
            buf_[idx_++] = b;
        } else {
            idx_ = 0;
            if (b == kDataHeader[0]) buf_[idx_++] = b;
        }
        return false;
    }

    buf_[idx_++] = b;
    if (idx_ < kFrameLen) return false;

    idx_ = 0;
    if (buf_[28] != kDataFooter[0] || buf_[29] != kDataFooter[1]) {
        ++badFrames_;
        return false;
    }
    parseDataFrame();
    return true;
}

void LD2450::parseDataFrame() {
    for (uint8_t i = 0; i < kMaxTargets; ++i) {
        const uint8_t* p = &buf_[4 + i * 8];
        Target& t = targets_[i];
        bool empty = true;
        for (uint8_t k = 0; k < 8; ++k) {
            if (p[k] != 0) { empty = false; break; }
        }
        if (empty) {
            t = Target{};
            continue;
        }
        t.valid = true;
        t.x = decodeSigned(le16(p));
        t.y = decodeSigned(le16(p + 2));
        t.speed = decodeSigned(le16(p + 4));
        t.resolution = le16(p + 6);
    }
    lastFrameMs_ = millis();
    ++frameCount_;
    newFrame_ = true;
}

// ---------------------------------------------------------------------------
// Configuration protocol
//   FD FC FB FA | len(2) | cmd(2) | value(n) | 04 03 02 01
// ACK:
//   FD FC FB FA | len(2) | cmd|0x0100 (2) | status(2) | payload | 04 03 02 01
// ---------------------------------------------------------------------------

bool LD2450::sendCommand(uint16_t cmd, const uint8_t* value, uint8_t valueLen,
                         uint8_t* ackPayload, uint8_t ackCap, uint8_t* ackLen) {
    if (!stream_) return false;

    uint8_t hdr[8];
    memcpy(hdr, kCmdHeader, 4);
    put16(hdr + 4, 2 + valueLen);
    put16(hdr + 6, cmd);
    stream_->write(hdr, sizeof(hdr));
    if (valueLen) stream_->write(value, valueLen);
    stream_->write(kCmdFooter, sizeof(kCmdFooter));
    stream_->flush();

    // Scan incoming bytes for an ACK header; data frames may be interleaved.
    const uint32_t start = millis();
    uint8_t matched = 0;
    uint8_t body[64];
    while (millis() - start < cmdTimeoutMs_) {
        if (!stream_->available()) { delay(1); continue; }
        uint8_t b = stream_->read();
        if (matched < 4) {
            matched = (b == kCmdHeader[matched]) ? matched + 1 : (b == kCmdHeader[0] ? 1 : 0);
            continue;
        }

        // Header matched: read length (2), then body (len), then footer (4).
        uint8_t lenBytes[2] = {b, 0};
        if (stream_->readBytes(&lenBytes[1], 1) != 1) return false;
        uint16_t len = le16(lenBytes);
        if (len < 4 || len > sizeof(body)) { matched = 0; continue; }
        if (stream_->readBytes(body, len) != len) return false;
        uint8_t tail[4];
        if (stream_->readBytes(tail, 4) != 4 || memcmp(tail, kCmdFooter, 4) != 0) {
            matched = 0;
            continue;
        }

        if (le16(body) != (cmd | 0x0100)) { matched = 0; continue; }
        bool ok = le16(body + 2) == 0;
        if (ackPayload && ackLen) {
            uint8_t n = len - 4;
            if (n > ackCap) n = ackCap;
            memcpy(ackPayload, body + 4, n);
            *ackLen = n;
        }
        return ok;
    }
    return false;
}

bool LD2450::enterConfig() {
    const uint8_t v[2] = {0x01, 0x00};
    return sendCommand(CMD_ENABLE_CONFIG, v, 2);
}

bool LD2450::exitConfig() {
    return sendCommand(CMD_END_CONFIG, nullptr, 0);
}

bool LD2450::runConfigCommand(uint16_t cmd, const uint8_t* value, uint8_t valueLen,
                              uint8_t* ackPayload, uint8_t ackCap, uint8_t* ackLen) {
    // The first enable-config after a data stream sometimes gets lost; retry once.
    if (!enterConfig() && !enterConfig()) return false;
    bool ok = sendCommand(cmd, value, valueLen, ackPayload, ackCap, ackLen);
    exitConfig();
    idx_ = 0;
    return ok;
}

bool LD2450::setMultiTargetMode() { return runConfigCommand(CMD_MULTI_TARGET, nullptr, 0); }

bool LD2450::setSingleTargetMode() { return runConfigCommand(CMD_SINGLE_TARGET, nullptr, 0); }

bool LD2450::getFirmwareVersion(FirmwareVersion& out) {
    uint8_t p[16];
    uint8_t n = 0;
    if (!runConfigCommand(CMD_READ_FIRMWARE, nullptr, 0, p, sizeof(p), &n) || n < 8) {
        return false;
    }
    out.type = le16(p);
    out.major = le16(p + 2);
    out.minor = le32(p + 4);
    return true;
}

bool LD2450::restart() {
    if (!enterConfig() && !enterConfig()) return false;
    bool ok = sendCommand(CMD_RESTART, nullptr, 0);
    idx_ = 0;
    return ok;
}

bool LD2450::factoryReset() {
    return runConfigCommand(CMD_FACTORY_RESET, nullptr, 0);
}

bool LD2450::setZoneFilter(uint8_t type, const Zone* zones, uint8_t count) {
    uint8_t v[26] = {0};
    put16(v, type);
    for (uint8_t i = 0; i < 3 && i < count && zones; ++i) {
        uint8_t* p = v + 2 + i * 8;
        put16(p + 0, static_cast<uint16_t>(zones[i].x1));
        put16(p + 2, static_cast<uint16_t>(zones[i].y1));
        put16(p + 4, static_cast<uint16_t>(zones[i].x2));
        put16(p + 6, static_cast<uint16_t>(zones[i].y2));
    }
    return runConfigCommand(CMD_SET_ZONE_FILTER, v, sizeof(v));
}
