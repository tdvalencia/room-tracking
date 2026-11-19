/*
    MLX90640 overhead (door-mounted) people detector.

    Pipeline per frame:
      1. Learn / continuously update a per-pixel background (empty doorway)
      2. Subtract background, smooth the difference image
      3. Threshold -> connected regions
      4. Split merged regions into one blob per head/shoulder heat peak
      5. Filter blobs by size, track them across frames
      6. Count IN / OUT when a track crosses the center line

    Memory: ~12 KB of float buffers. Fine on SAMD/ESP32/RP2040/etc,
    too much for an AVR Uno-class board.
*/

#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX9064X_I2C_Driver.h"

#define debug Serial

const byte MLX90640_address = 0x33;
#define TA_SHIFT 8

#define COLS 32
#define ROWS 24
#define NPIX (COLS * ROWS)

// ---------------- Background model ----------------
#define BG_INIT_FRAMES   40     // Frames averaged at startup (keep doorway EMPTY)
#define BG_ALPHA         0.01f  // Background adaptation rate (per frame)
#define BG_UPDATE_DELTA  1.0f   // Only pixels this close to background get updated

// ---------------- Detection tuning ----------------
#define FG_DELTA         2.0f   // °C above background to count as foreground
#define PEAK_MIN_DELTA   3.0f   // A blob must have a peak at least this far above background
#define PEAK_RADIUS      2      // Min spacing between heads (px). Raise if one person splits in two.
#define MIN_BLOB_AREA    5      // px
#define MAX_BLOB_AREA    40     // px, rejects huge warm regions (sun patch, heater, door glare)
#define MAX_BLOBS        12
#define SMOOTH_DELTA     1      // 1 = 3x3 average of the difference image

// ---------------- Tracking / counting ----------------
#define MAX_TRACKS       8
#define MATCH_DIST       3.5f   // Max px a person can move between frames and stay the same track
#define MAX_MISSED       4      // Frames a track may vanish before being dropped
#define TRAVEL_ALONG_COLS 1     // 1 = people walk along the 32-px axis, 0 = along the 24-px axis
#define LINE_HYSTERESIS  1.5f   // Dead zone (px) either side of the center line
#define SWAP_DIRECTION   0      // 1 = flip which direction counts as IN

// Debug output
#define PRINT_FRAME_CSV  0      // 1 = dump full 32x24 temps every frame (slows frame rate a lot at 115200)

static float mlx90640To[NPIX];
static float bg[NPIX];
static float sd[NPIX];          // smoothed (temp - background)
static uint8_t fg[NPIX];
static uint8_t comp[NPIX];
static uint16_t stackBuf[NPIX];
paramsMLX90640 mlx90640;

struct Blob {
    bool valid;
    uint8_t comp;
    uint16_t idx;               // peak pixel
    uint16_t area;
    float sumW, sumX, sumY, sumT;
    float cx, cy;
    float peakDelta;
    float maxTemp;
    float avgTemp;
};

struct Track {
    bool active;
    uint8_t id;
    float x, y;
    int8_t side;                // -1, 0 (unknown / dead zone), +1
    uint8_t missed;
};

static Blob blobs[MAX_BLOBS];
static Track tracks[MAX_TRACKS];
static uint8_t nextTrackId = 1;

static uint16_t bgFrames = 0;
static unsigned long countIn = 0, countOut = 0;

// ------------------------------------------------------------

void setup() {
    Wire.begin();
    Wire.setClock(1000000);

    debug.begin(115200);
    while (!debug);
    debug.println("MLX90640 Overhead People Detector");

    if (!isConnected()) {
        debug.println("MLX90640 not detected. Freezing.");
        while (1);
    }
    debug.println("MLX90640 online!");

    int status;
    uint16_t eeMLX90640[832];
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0) debug.println("Failed to load system parameters");

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0) debug.println("Parameter extraction failed");

    // 0x04 = 8Hz, 0x05 = 16Hz (per subpage)
    MLX90640_SetRefreshRate(MLX90640_address, 0x05);

    memset(tracks, 0, sizeof(tracks));
    debug.println("Learning background - keep the doorway clear...");
}

void loop() {
    int okSubpages = 0;
    for (byte x = 0; x < 2; x++) {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0) continue;

        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        float tr = Ta - TA_SHIFT;
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95f, tr, mlx90640To);
        okSubpages++;
    }
    if (okSubpages < 2) return; // need a complete frame

    // Replace any invalid pixel values
    for (int i = 0; i < NPIX; i++) {
        if (isnan(mlx90640To[i])) mlx90640To[i] = (bgFrames > 0) ? bg[i] : 25.0f;
    }

    // ---- Background learning phase ----
    if (bgFrames < BG_INIT_FRAMES) {
        for (int i = 0; i < NPIX; i++) {
            if (bgFrames == 0) bg[i] = mlx90640To[i];
            else bg[i] += (mlx90640To[i] - bg[i]) / (float)(bgFrames + 1);
        }
        bgFrames++;
        if (bgFrames == BG_INIT_FRAMES) debug.println("Background ready. Detecting.");
        return;
    }

    computeDelta();
    int blobCount = detectBlobs();
    updateTracks(blobCount);
    updateBackground();

    printReport(blobCount);
}

// ------------------------------------------------------------
// Difference image (temp - background), optionally smoothed
// ------------------------------------------------------------
void computeDelta() {
    static float raw[NPIX];
    for (int i = 0; i < NPIX; i++) raw[i] = mlx90640To[i] - bg[i];

#if SMOOTH_DELTA
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            float sum = 0; int n = 0;
            for (int dr = -1; dr <= 1; dr++) {
                int rr = r + dr;
                if (rr < 0 || rr >= ROWS) continue;
                for (int dc = -1; dc <= 1; dc++) {
                    int cc = c + dc;
                    if (cc < 0 || cc >= COLS) continue;
                    sum += raw[rr * COLS + cc];
                    n++;
                }
            }
            sd[r * COLS + c] = sum / n;
        }
    }
#else
    memcpy(sd, raw, sizeof(sd));
#endif
}

// Slowly absorb the environment into the background, but never absorb people
void updateBackground() {
    for (int i = 0; i < NPIX; i++) {
        if (sd[i] < BG_UPDATE_DELTA) {
            bg[i] += BG_ALPHA * (mlx90640To[i] - bg[i]);
        }
    }
}

// ------------------------------------------------------------
// Blob detection: threshold -> components -> peak-based splitting
// ------------------------------------------------------------
int detectBlobs() {
    // 1. Foreground mask
    for (int i = 0; i < NPIX; i++) fg[i] = (sd[i] >= FG_DELTA) ? 1 : 0;

    // 2. Connected components (8-connectivity)
    memset(comp, 0, sizeof(comp));
    uint8_t nComp = 0;
    for (int start = 0; start < NPIX; start++) {
        if (!fg[start] || comp[start]) continue;
        if (nComp == 255) break;
        nComp++;

        int sp = 0;
        stackBuf[sp++] = start;
        comp[start] = nComp;

        while (sp > 0) {
            int idx = stackBuf[--sp];
            int r = idx / COLS, c = idx % COLS;
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    if (dr == 0 && dc == 0) continue;
                    int nr = r + dr, nc = c + dc;
                    if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) continue;
                    int n = nr * COLS + nc;
                    if (!fg[n] || comp[n]) continue;
                    comp[n] = nComp;
                    stackBuf[sp++] = n;
                }
            }
        }
    }

    // 3. Find heat peaks (one per head) inside each component
    int nPeaks = 0;
    memset(blobs, 0, sizeof(blobs));
    for (int r = 0; r < ROWS && nPeaks < MAX_BLOBS; r++) {
        for (int c = 0; c < COLS && nPeaks < MAX_BLOBS; c++) {
            int i = r * COLS + c;
            if (!fg[i] || sd[i] < PEAK_MIN_DELTA) continue;

            bool isPeak = true;
            for (int dr = -PEAK_RADIUS; dr <= PEAK_RADIUS && isPeak; dr++) {
                int nr = r + dr;
                if (nr < 0 || nr >= ROWS) continue;
                for (int dc = -PEAK_RADIUS; dc <= PEAK_RADIUS; dc++) {
                    int nc = c + dc;
                    if (nc < 0 || nc >= COLS) continue;
                    int n = nr * COLS + nc;
                    if (n == i || comp[n] != comp[i]) continue;
                    if (sd[n] > sd[i] || (sd[n] == sd[i] && n < i)) { isPeak = false; break; }
                }
            }
            if (isPeak) {
                Blob &b = blobs[nPeaks++];
                b.valid = true;
                b.comp = comp[i];
                b.idx = i;
                b.peakDelta = sd[i];
                b.maxTemp = -1000.0f;
            }
        }
    }

    // 4. Assign every foreground pixel to its nearest peak in the same component
    for (int i = 0; i < NPIX; i++) {
        if (!fg[i]) continue;
        int r = i / COLS, c = i % COLS;

        int best = -1, bestD2 = 0x7FFF;
        for (int p = 0; p < nPeaks; p++) {
            if (blobs[p].comp != comp[i]) continue;
            int pr = blobs[p].idx / COLS, pc = blobs[p].idx % COLS;
            int d2 = (r - pr) * (r - pr) + (c - pc) * (c - pc);
            if (d2 < bestD2) { bestD2 = d2; best = p; }
        }
        if (best < 0) continue;

        Blob &b = blobs[best];
        float w = sd[i];                  // weight by heat above background
        b.area++;
        b.sumW += w;
        b.sumX += w * c;
        b.sumY += w * r;
        b.sumT += mlx90640To[i];
        if (mlx90640To[i] > b.maxTemp) b.maxTemp = mlx90640To[i];
    }

    // 5. Size filter + finalize + compact
    int count = 0;
    for (int p = 0; p < nPeaks; p++) {
        Blob &b = blobs[p];
        if (b.area < MIN_BLOB_AREA || b.area > MAX_BLOB_AREA || b.sumW <= 0) continue;
        b.cx = b.sumX / b.sumW;
        b.cy = b.sumY / b.sumW;
        b.avgTemp = b.sumT / b.area;
        if (count != p) blobs[count] = b;
        count++;
    }
    return count;
}

// ------------------------------------------------------------
// Tracking + line-crossing counter
// ------------------------------------------------------------
static inline float travelCoord(float x, float y) {
#if TRAVEL_ALONG_COLS
    return x;
#else
    return y;
#endif
}

static inline float lineCenter() {
#if TRAVEL_ALONG_COLS
    return (COLS - 1) / 2.0f;
#else
    return (ROWS - 1) / 2.0f;
#endif
}

static int8_t zoneOf(float pos, int8_t previous) {
    float line = lineCenter();
    if (pos < line - LINE_HYSTERESIS) return -1;
    if (pos > line + LINE_HYSTERESIS) return +1;
    return previous; // inside dead zone: keep last known side
}

void registerCrossing(int8_t from, int8_t to, uint8_t id) {
    bool isIn = (from < 0 && to > 0);
#if SWAP_DIRECTION
    isIn = !isIn;
#endif
    if (isIn) countIn++; else countOut++;
    debug.print("  >>> Track ");
    debug.print(id);
    debug.println(isIn ? " crossed: IN" : " crossed: OUT");
}

void updateTracks(int blobCount) {
    bool claimed[MAX_TRACKS] = {false};

    for (int b = 0; b < blobCount; b++) {
        // Find nearest unclaimed active track
        int best = -1;
        float bestD2 = MATCH_DIST * MATCH_DIST;
        for (int t = 0; t < MAX_TRACKS; t++) {
            if (!tracks[t].active || claimed[t]) continue;
            float dx = blobs[b].cx - tracks[t].x;
            float dy = blobs[b].cy - tracks[t].y;
            float d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; best = t; }
        }

        if (best >= 0) {
            Track &tr = tracks[best];
            tr.x = blobs[b].cx;
            tr.y = blobs[b].cy;
            tr.missed = 0;
            claimed[best] = true;

            int8_t zone = zoneOf(travelCoord(tr.x, tr.y), tr.side);
            if (tr.side != 0 && zone != 0 && zone != tr.side) {
                registerCrossing(tr.side, zone, tr.id);
            }
            tr.side = zone;
        } else {
            // Start a new track
            for (int t = 0; t < MAX_TRACKS; t++) {
                if (tracks[t].active) continue;
                tracks[t].active = true;
                tracks[t].id = nextTrackId++;
                if (nextTrackId == 0) nextTrackId = 1;
                tracks[t].x = blobs[b].cx;
                tracks[t].y = blobs[b].cy;
                tracks[t].missed = 0;
                tracks[t].side = zoneOf(travelCoord(tracks[t].x, tracks[t].y), 0);
                claimed[t] = true;
                break;
            }
        }
    }

    // Age out tracks that weren't matched
    for (int t = 0; t < MAX_TRACKS; t++) {
        if (!tracks[t].active || claimed[t]) continue;
        if (++tracks[t].missed > MAX_MISSED) tracks[t].active = false;
    }
}

// ------------------------------------------------------------
// Output
// ------------------------------------------------------------
void printReport(int blobCount) {
    debug.println("========================================");
    debug.print("Blobs: ");
    debug.print(blobCount);
    debug.print(" | IN: ");
    debug.print(countIn);
    debug.print(" | OUT: ");
    debug.println(countOut);

    for (int i = 0; i < blobCount; i++) {
        Blob &b = blobs[i];
        debug.print("  [#");
        debug.print(i + 1);
        debug.print("] Area: ");
        debug.print(b.area);
        debug.print(" px | Centroid (col,row): ");
        debug.print(b.cx, 1);
        debug.print(",");
        debug.print(b.cy, 1);
        debug.print(" | Peak: ");
        debug.print(b.maxTemp, 1);
        debug.print("C (+");
        debug.print(b.peakDelta, 1);
        debug.print(") | Avg: ");
        debug.print(b.avgTemp, 1);
        debug.println("C");
    }

#if PRINT_FRAME_CSV
    debug.println("--- FULL 32x24 FRAME CSV DATA ---");
    for (int row = 0; row < ROWS; row++) {
        for (int col = 0; col < COLS; col++) {
            debug.print(mlx90640To[row * COLS + col], 1);
            if (col < COLS - 1) debug.print(",");
        }
        debug.println();
    }
#endif
}

boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    return Wire.endTransmission() == 0;
}