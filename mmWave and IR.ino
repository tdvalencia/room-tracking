
#include <Arduino.h>
#include <Wire.h>
#include <LD2450.h>
#include <Adafruit_MLX90640.h>

// ==========================================
// HARDWARE CONFIGURATION
// ==========================================

#define RADAR_RX 16
#define RADAR_TX 17

#define I2C_SDA 21
#define I2C_SCL 22

#define LED_PIN 2

#define MAX_TARGETS 3

// LD2450 normally operates at 256000 baud
#define RADAR_BAUD 256000

// ==========================================
// DOORWAY CONFIGURATION
// ==========================================

// Radar X-axis crossing boundary
// Negative X -> Positive X = ENTERING
// Positive X -> Negative X = EXITING

#define CROSSING_DEADBAND 150

// Minimum time allowed for thermal confirmation
#define CONFIRMATION_TIMEOUT 1500

// Minimum time between crossings for a target slot
#define CROSSING_COOLDOWN 1500

// ==========================================
// THERMAL CAMERA CONFIGURATION
// ==========================================

#define THERMAL_WIDTH 32
#define THERMAL_HEIGHT 24
#define THERMAL_PIXELS 768

// Thermal detection thresholds
#define MIN_HOT_TEMP 30.0
#define MIN_TEMP_DIFFERENCE 3.0

// Minimum connected hot pixels to represent a hotspot
#define MIN_BLOB_SIZE 3

// Thermal camera update interval
#define THERMAL_INTERVAL 250

// ==========================================
// SENSOR INSTANCES
// ==========================================

LD2450 ld2450;
Adafruit_MLX90640 mlx;

float thermalFrame[THERMAL_PIXELS];

// ==========================================
// RADAR TRACKING
// ==========================================

struct TargetTrack {

    int16_t previousX = 0;
    int16_t previousY = 0;

    int8_t previousSide = 0;

    bool initialized = false;

    // Pending crossing direction:
    // +1 = entering
    // -1 = exiting
    //  0 = no pending crossing

    int8_t pendingDirection = 0;

    uint32_t crossingTime = 0;
    uint32_t lastCrossing = 0;

};

TargetTrack tracks[MAX_TARGETS];

// ==========================================
// GLOBAL VARIABLES
// ==========================================

int peopleInside = 0;

bool thermalPresent = false;

uint8_t thermalBlobCount = 0;

uint32_t lastThermalUpdate = 0;

// ==========================================
// RADAR HELPER FUNCTIONS
// ==========================================

// Returns:
// -1 = negative side
//  0 = deadband
// +1 = positive side

int8_t getRadarSide(int16_t x) {

    if (x < -CROSSING_DEADBAND) {
        return -1;
    }

    if (x > CROSSING_DEADBAND) {
        return 1;
    }

    return 0;
}


// ==========================================
// RADAR CROSSING DETECTION
// ==========================================

void updateRadar() {

    int sensorStatus = ld2450.read();

    if (sensorStatus <= 0) {
        return;
    }

    uint32_t now = millis();

    int targetCount = ld2450.getSensorSupportedTargetCount();

    if (targetCount > MAX_TARGETS) {
        targetCount = MAX_TARGETS;
    }

    for (int i = 0; i < targetCount; i++) {

        const LD2450::RadarTarget target =
            ld2450.getTarget(i);

        TargetTrack &track = tracks[i];

        if (!target.valid) {
            continue;
        }

        int16_t currentX = target.x;
        int16_t currentY = target.y;

        int8_t currentSide = getRadarSide(currentX);

        // Initialize track without generating a crossing
        if (!track.initialized) {

            track.previousX = currentX;
            track.previousY = currentY;

            if (currentSide != 0) {
                track.previousSide = currentSide;
            }

            track.initialized = true;

            continue;
        }

        // Ignore positions inside the deadband
        if (currentSide != 0) {

            // Check for crossing
            if (track.previousSide != 0 &&
                currentSide != track.previousSide) {

                // Check cooldown
                if ((uint32_t)(now - track.lastCrossing)
                    > CROSSING_COOLDOWN) {

                    // Negative -> Positive = entering
                    if (track.previousSide == -1 &&
                        currentSide == 1) {

                        track.pendingDirection = 1;

                        track.crossingTime = now;

                    }

                    // Positive -> Negative = exiting
                    else if (track.previousSide == 1 &&
                             currentSide == -1) {

                        track.pendingDirection = -1;

                        track.crossingTime = now;

                    }

                }

            }

            track.previousSide = currentSide;
        }

        track.previousX = currentX;
        track.previousY = currentY;
    }
}


// ==========================================
// THERMAL IMAGE PROCESSING
// ==========================================

// Flood fill connected hot pixels.
// Each connected region is treated as one hotspot.

uint8_t countThermalBlobs(float *frame) {

    bool visited[THERMAL_PIXELS] = {false};

    uint16_t queue[THERMAL_PIXELS];

    bool hot[THERMAL_PIXELS] = {false};

    float averageTemp = 0;

    // Calculate average scene temperature
    for (int i = 0; i < THERMAL_PIXELS; i++) {
        averageTemp += frame[i];
    }

    averageTemp /= THERMAL_PIXELS;

    // Calculate adaptive temperature threshold
    float threshold = averageTemp + MIN_TEMP_DIFFERENCE;

    if (threshold < MIN_HOT_TEMP) {
        threshold = MIN_HOT_TEMP;
    }

    // Identify hot pixels
    for (int i = 0; i < THERMAL_PIXELS; i++) {

        if (frame[i] > threshold) {
            hot[i] = true;
        }

    }

    uint8_t blobCount = 0;

    // Find connected components
    for (int i = 0; i < THERMAL_PIXELS; i++) {

        if (!hot[i] || visited[i]) {
            continue;
        }

        int head = 0;
        int tail = 0;

        int blobSize = 0;

        queue[tail++] = i;
        visited[i] = true;

        while (head < tail) {

            int current = queue[head++];

            blobSize++;

            int x = current % THERMAL_WIDTH;
            int y = current / THERMAL_WIDTH;

            // Check 8 neighboring pixels
            for (int dy = -1; dy <= 1; dy++) {

                for (int dx = -1; dx <= 1; dx++) {

                    if (dx == 0 && dy == 0) {
                        continue;
                    }

                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= THERMAL_WIDTH ||
                        ny < 0 || ny >= THERMAL_HEIGHT) {
                        continue;
                    }

                    int neighbor = ny * THERMAL_WIDTH + nx;

                    if (hot[neighbor] && !visited[neighbor]) {

                        visited[neighbor] = true;

                        queue[tail++] = neighbor;

                    }

                }

            }

        }

        if (blobSize >= MIN_BLOB_SIZE) {
            blobCount++;
        }

    }

    return blobCount;
}


// ==========================================
// THERMAL CAMERA UPDATE
// ==========================================

void updateThermal() {

    uint32_t now = millis();

    if ((uint32_t)(now - lastThermalUpdate)
        < THERMAL_INTERVAL) {
        return;
    }

    lastThermalUpdate = now;

    // Read thermal image
    if (mlx.getFrame(thermalFrame) != 0) {

        Serial.println("Thermal camera read error");

        return;
    }

    thermalBlobCount = countThermalBlobs(thermalFrame);

    thermalPresent = (thermalBlobCount > 0);

}


// ==========================================
// SENSOR FUSION
// ==========================================

void updatePeopleCount() {

    uint32_t now = millis();

    for (int i = 0; i < MAX_TARGETS; i++) {

        TargetTrack &track = tracks[i];

        if (track.pendingDirection == 0) {
            continue;
        }

        // Check for thermal confirmation
        if (thermalPresent) {

            // Only accept thermal confirmation shortly
            // after the radar detects a crossing

            if ((uint32_t)(now - track.crossingTime)
                <= CONFIRMATION_TIMEOUT) {

                if (track.pendingDirection == 1) {

                    peopleInside++;

                    Serial.println("PERSON ENTERED");

                }

                else if (track.pendingDirection == -1) {

                    if (peopleInside > 0) {
                        peopleInside--;
                    }

                    Serial.println("PERSON EXITED");

                }

                track.lastCrossing = now;

                track.pendingDirection = 0;

                Serial.print("People inside: ");
                Serial.println(peopleInside);

            }

        }

        // Expire unconfirmed crossing
        if (track.pendingDirection != 0 &&
            (uint32_t)(now - track.crossingTime)
            > CONFIRMATION_TIMEOUT) {

            track.pendingDirection = 0;

            Serial.println("Crossing rejected: no thermal confirmation");

        }

    }

}


// ==========================================
// SETUP
// ==========================================

void setup() {

    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize radar UART
    Serial2.begin(
        RADAR_BAUD,
        SERIAL_8N1,
        RADAR_RX,
        RADAR_TX
    );

    ld2450.begin(Serial2, false);

    Serial.println("Starting LD2450...");

    if (ld2450.waitForSensorMessage()) {
        Serial.println("Radar connection error!");
    }
    else {
        Serial.println("Radar connected!");
    }

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    Wire.setClock(400000);

    // Initialize MLX90640
    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {

        Serial.println("MLX90640 not detected!");

        while (1) {
            delay(100);
        }

    }

    mlx.setMode(MLX90640_CHESS);

    mlx.setResolution(MLX90640_16BIT);

    mlx.setRefreshRate(MLX90640_4_HZ);

    Serial.println("MLX90640 connected!");

    Serial.println("SYSTEM READY");

}


// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

    // Read radar continuously
    updateRadar();

    // Update thermal image periodically
    updateThermal();

    // Combine sensor information
    updatePeopleCount();

    // LED indicates thermal presence
    digitalWrite(LED_PIN, thermalPresent);

}
