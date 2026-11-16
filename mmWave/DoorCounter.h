#ifndef DOOR_COUNTER_H
#define DOOR_COUNTER_H

#include <Arduino.h>
#include "LD2450.h"

// Counts people crossing a virtual line (the doorway) in the LD2450 frame.
//
// The radar's own target slots can swap between people, so this class keeps
// its own tracks (greedy nearest-neighbour association with a speed-scaled
// gate). Each track remembers which side of the line it was last confirmed on;
// once it is seen on the other side, past the hysteresis band, for
// `confirmFrames` consecutive frames, an entry or exit is counted.
class DoorCounter {
public:
    enum Axis : uint8_t { AXIS_X, AXIS_Y };
    enum Event : int8_t { EXITED = -1, NONE = 0, ENTERED = 1 };

    struct Config {
        Axis axis = AXIS_X;            // axis perpendicular to the doorway
        int16_t lineMm = 0;            // doorway position along `axis`
        int16_t hysteresisMm = 150;    // deadband half-width around the line
        bool insideIsPositive = true;  // which side of the line is the room

        // Region of interest; detections outside are ignored.
        int16_t xMinMm = -3000, xMaxMm = 3000;
        int16_t yMinMm = 1000,  yMaxMm = 6000;

        uint8_t confirmFrames = 2;         // frames on new side before counting
        uint32_t trackTimeoutMs = 1000;    // drop a track unseen for this long
        uint16_t gateMm = 500;             // base association distance
        uint16_t maxSpeedMmPerS = 3000;    // gate grows by this * time unseen
    };

    struct Track {
        bool active;
        uint8_t id;
        int16_t x, y;
        int8_t side;       // -1 outside, +1 inside, 0 not yet established
        int8_t pendingSide;
        uint8_t pendingCount;
        uint32_t lastSeenMs;
    };

    typedef void (*EventCallback)(Event ev, const Track& track, const DoorCounter& counter);

    static const uint8_t kMaxTracks = 6;

    DoorCounter() {}
    explicit DoorCounter(const Config& cfg) : cfg_(cfg) {}

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }
    void onEvent(EventCallback cb) { cb_ = cb; }

    // Call once per new radar frame. Returns the net change (+entries - exits).
    int8_t update(const LD2450& radar, uint32_t nowMs = millis());

    uint32_t entries() const { return entries_; }
    uint32_t exits() const { return exits_; }
    int32_t occupancy() const { return occupancy_; }
    void setOccupancy(int32_t n) { occupancy_ = n < 0 ? 0 : n; }
    void reset();

    const Track& track(uint8_t i) const { return tracks_[i]; }

private:
    int8_t sideOf(int16_t x, int16_t y) const;
    bool inRoi(int16_t x, int16_t y) const;
    Event stepSide(Track& t);

    Config cfg_;
    Track tracks_[kMaxTracks] = {};
    EventCallback cb_ = nullptr;
    uint8_t nextId_ = 1;
    uint32_t entries_ = 0;
    uint32_t exits_ = 0;
    int32_t occupancy_ = 0;
};

#endif
