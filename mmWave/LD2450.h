#ifndef LD2450_H
#define LD2450_H

#include <Arduino.h>

// Driver for the HLK-LD2450 24 GHz multi-target tracking radar.
//
// Coordinate frame (looking out from the antenna face):
//   +Y = straight out from the sensor (range), always >= 0
//   +X = to the right, -X = to the left
// Units: position in mm, speed in cm/s (positive = moving away from sensor).
//
// Data frame (30 bytes, ~10 Hz):
//   AA FF 03 00 | target1 (8B) | target2 (8B) | target3 (8B) | 55 CC
// Each target: int16 X, int16 Y, int16 speed, uint16 resolution, little endian,
// where X/Y/speed use sign-magnitude with bit 15 SET meaning POSITIVE.
// An all-zero target block means the slot is empty.

class LD2450 {
public:
    static const uint8_t kMaxTargets = 3;
    static const uint32_t kDefaultBaud = 256000;

    struct Target {
        bool valid;
        int16_t x;           // mm
        int16_t y;           // mm
        int16_t speed;       // cm/s
        uint16_t resolution; // mm
    };

    struct FirmwareVersion {
        uint16_t type;
        uint16_t major;
        uint32_t minor;
    };

#if defined(ARDUINO_ARCH_ESP32)
    // Opens the UART on the given pins and attaches the driver to it.
    void begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
               uint32_t baud = kDefaultBaud);
#endif
    // Attaches to an already-opened stream (UART must be 256000 8N1 by default).
    void begin(Stream& stream);

    // Non-blocking. Consumes available UART bytes; returns true when a complete,
    // valid data frame has been decoded since the last call.
    bool update();

    const Target& target(uint8_t i) const { return targets_[i]; }
    uint8_t targetCount() const;
    uint32_t lastFrameMs() const { return lastFrameMs_; }
    uint32_t frameCount() const { return frameCount_; }
    uint32_t badFrameCount() const { return badFrames_; }
    uint32_t bytesReceived() const { return bytesRx_; }

    // --- Configuration commands (block for up to ~timeoutMs each) ---
    bool setMultiTargetMode();
    bool setSingleTargetMode();
    bool getFirmwareVersion(FirmwareVersion& out);
    bool restart();
    bool factoryReset();

    // Zone filtering: type 0 = off, 1 = only detect inside zones,
    // 2 = ignore inside zones. Each zone is an axis-aligned rectangle given by
    // two diagonal corners in mm. Pass nullptr / count 0 for unused zones.
    struct Zone { int16_t x1, y1, x2, y2; };
    bool setZoneFilter(uint8_t type, const Zone* zones, uint8_t count);

    void setCommandTimeout(uint32_t ms) { cmdTimeoutMs_ = ms; }

private:
    static int16_t decodeSigned(uint16_t raw);
    void parseDataFrame();
    bool feed(uint8_t b);

    bool enterConfig();
    bool exitConfig();
    bool sendCommand(uint16_t cmd, const uint8_t* value, uint8_t valueLen,
                     uint8_t* ackPayload = nullptr, uint8_t ackCap = 0,
                     uint8_t* ackLen = nullptr);
    bool runConfigCommand(uint16_t cmd, const uint8_t* value, uint8_t valueLen,
                          uint8_t* ackPayload = nullptr, uint8_t ackCap = 0,
                          uint8_t* ackLen = nullptr);

    Stream* stream_ = nullptr;
    Target targets_[kMaxTargets] = {};
    uint32_t lastFrameMs_ = 0;
    uint32_t frameCount_ = 0;
    uint32_t badFrames_ = 0;
    uint32_t bytesRx_ = 0;
    uint32_t cmdTimeoutMs_ = 300;

    // Data frame parser state
    static const uint8_t kFrameLen = 30;
    uint8_t buf_[kFrameLen];
    uint8_t idx_ = 0;
    bool newFrame_ = false;
};

#endif
