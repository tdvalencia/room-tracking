#include "DoorCounter.h"

void DoorCounter::reset() {
    for (uint8_t i = 0; i < kMaxTracks; ++i) tracks_[i] = Track{};
    entries_ = exits_ = 0;
    occupancy_ = 0;
}

bool DoorCounter::inRoi(int16_t x, int16_t y) const {
    return x >= cfg_.xMinMm && x <= cfg_.xMaxMm && y >= cfg_.yMinMm && y <= cfg_.yMaxMm;
}

int8_t DoorCounter::sideOf(int16_t x, int16_t y) const {
    int32_t d = int32_t(cfg_.axis == AXIS_X ? x : y) - cfg_.lineMm;
    int8_t s = 0;
    if (d > cfg_.hysteresisMm) s = 1;
    else if (d < -cfg_.hysteresisMm) s = -1;
    return cfg_.insideIsPositive ? s : -s;
}

DoorCounter::Event DoorCounter::stepSide(Track& t) {
    int8_t s = sideOf(t.x, t.y);
    if (s == 0) return NONE;  // inside the deadband: keep pending state

    if (t.side == 0) {
        t.side = s;
        t.pendingCount = 0;
        return NONE;
    }
    if (s == t.side) {
        t.pendingCount = 0;
        return NONE;
    }

    if (t.pendingSide != s) {
        t.pendingSide = s;
        t.pendingCount = 0;
    }
    if (++t.pendingCount < cfg_.confirmFrames) return NONE;

    t.side = s;
    t.pendingCount = 0;
    return s > 0 ? ENTERED : EXITED;
}

int8_t DoorCounter::update(const LD2450& radar, uint32_t now) {
    // Gather in-ROI detections.
    int16_t mx[LD2450::kMaxTargets], my[LD2450::kMaxTargets];
    uint8_t m = 0;
    for (uint8_t i = 0; i < LD2450::kMaxTargets; ++i) {
        const LD2450::Target& t = radar.target(i);
        if (t.valid && inRoi(t.x, t.y)) {
            mx[m] = t.x;
            my[m] = t.y;
            ++m;
        }
    }

    // Greedy nearest-neighbour association: repeatedly take the closest
    // (track, detection) pair that lies within that track's gate.
    bool used[LD2450::kMaxTargets] = {false};
    bool matched[kMaxTracks] = {false};
    int8_t net = 0;

    for (;;) {
        int bestT = -1, bestM = -1;
        uint32_t bestD2 = UINT32_MAX;
        for (uint8_t ti = 0; ti < kMaxTracks; ++ti) {
            const Track& tr = tracks_[ti];
            if (!tr.active || matched[ti]) continue;
            uint32_t gate = cfg_.gateMm +
                            uint32_t(cfg_.maxSpeedMmPerS) * (now - tr.lastSeenMs) / 1000;
            uint32_t gate2 = gate * gate;
            for (uint8_t mi = 0; mi < m; ++mi) {
                if (used[mi]) continue;
                int32_t dx = int32_t(mx[mi]) - tr.x;
                int32_t dy = int32_t(my[mi]) - tr.y;
                uint32_t d2 = uint32_t(dx * dx) + uint32_t(dy * dy);
                if (d2 <= gate2 && d2 < bestD2) {
                    bestD2 = d2;
                    bestT = ti;
                    bestM = mi;
                }
            }
        }
        if (bestT < 0) break;

        Track& tr = tracks_[bestT];
        matched[bestT] = true;
        used[bestM] = true;
        tr.x = mx[bestM];
        tr.y = my[bestM];
        tr.lastSeenMs = now;

        Event ev = stepSide(tr);
        if (ev == ENTERED) {
            ++entries_;
            ++occupancy_;
        } else if (ev == EXITED) {
            ++exits_;
            if (occupancy_ > 0) --occupancy_;
        }
        if (ev != NONE) {
            net += ev;
            if (cb_) cb_(ev, tr, *this);
        }
    }

    // Unmatched detections start new tracks.
    for (uint8_t mi = 0; mi < m; ++mi) {
        if (used[mi]) continue;
        for (uint8_t ti = 0; ti < kMaxTracks; ++ti) {
            Track& tr = tracks_[ti];
            if (tr.active) continue;
            tr = Track{};
            tr.active = true;
            tr.id = nextId_++;
            if (nextId_ == 0) nextId_ = 1;
            tr.x = mx[mi];
            tr.y = my[mi];
            tr.lastSeenMs = now;
            tr.side = sideOf(tr.x, tr.y);
            matched[ti] = true;
            break;
        }
    }

    // Expire stale tracks.
    for (uint8_t ti = 0; ti < kMaxTracks; ++ti) {
        Track& tr = tracks_[ti];
        if (tr.active && now - tr.lastSeenMs > cfg_.trackTimeoutMs) tr.active = false;
    }

    return net;
}
