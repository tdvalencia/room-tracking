#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// --- Wi-Fi & Server Configuration ---
const char* ssid = "MWireless";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverUrl = "http://your-central-server.local/api/occupancy";

// --- Hardware UART Configuration (ESP32 UART2) ---
#define RADAR_RX_PIN 16  // Connect to LD2450 TX
#define RADAR_TX_PIN 17  // Connect to LD2450 RX
#define RADAR_BAUD   256000

// --- Spatial Geometry & Hysteresis Constants (in mm) ---
// Per deployment strategy: 1.0m < Y < 6.0m, Hysteresis at +/- 150mm
const int16_t HYSTERESIS_MM = 150; 
const int16_t MIN_Y_MM      = 1000; 
const int16_t MAX_Y_MM      = 6000; 

// --- Tracking State Structures ---
enum ZoneState {
  ZONE_UNKNOWN = 0,
  ZONE_OUTSIDE = -1, // X < -150 mm
  ZONE_INSIDE  = 1   // X > +150 mm
};

struct TargetTracker {
  ZoneState lastZone = ZONE_UNKNOWN;
  uint32_t lastSeenMs = 0;
  bool active = false;
};

TargetTracker targets[3];
int roomOccupancy = 0;
const uint32_t TARGET_TIMEOUT_MS = 1500; // Reset target state if lost for 1.5s

// --- UART Frame Parser Variables ---
// Full frame: [AA FF 03 00] (4B) + [Target 1] (8B) + [Target 2] (8B) + [Target 3] (8B) + [55 CC] (2B) = 30 Bytes
const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
uint8_t payloadBuf[26]; // 24 bytes for 3 targets + 2 footer bytes
int headerIdx = 0;

void sendOccupancyUpdate(int count);

void setup() {
  Serial.begin(115200);
  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  // // Connect to WPA Wi-Fi
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to Wi-Fi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
}

// Helper to parse a single 8-byte target block and update its state machine
void processTarget(int index, uint8_t* buf, uint32_t now) {
  // 1. Parse X-Coordinate (Bytes 0 & 1) - Signed magnitude
  int16_t x_val = ((buf[1] & 0x7F) << 8) | buf[0];
  if ((buf[1] & 0x80) == 0) {
    x_val = -x_val; // Bit 15 == 0 means negative X; Bit 15 == 1 means positive X
  }

  // 2. Parse Y-Coordinate (Bytes 2 & 3) - Strip errant sign bit for positive radial depth
  int16_t y_val = ((buf[3] & 0x7F) << 8) | buf[2];

  // 3. Parse Speed (Bytes 4 & 5) in cm/s
  int16_t speed = ((buf[5] & 0x7F) << 8) | buf[4];
  if ((buf[5] & 0x80) == 0) {
    speed = -speed;
  }

  // Check if target slot is empty (LD2450 pads unused slots with 0x00)
  if (x_val == 0 && y_val == 0) {
    if (targets[index].active && (now - targets[index].lastSeenMs > TARGET_TIMEOUT_MS)) {
      targets[index].active = false;
      targets[index].lastZone = ZONE_UNKNOWN;
    }
    return;
  }

  // 4. Y-Axis Validation Gate (Filter floor/ceiling noise & non-human objects)
  if (y_val < MIN_Y_MM || y_val > MAX_Y_MM) {
    return;
  }

  targets[index].active = true;
  targets[index].lastSeenMs = now;

  // 5. X-Axis Hysteresis & Boundary Crossing State Machine
  if (x_val > HYSTERESIS_MM) {
    // Target is clearly in the INSIDE zone (+X)
    if (targets[index].lastZone == ZONE_OUTSIDE) {
      roomOccupancy++;
      Serial.printf("[Target %d] ENTERED (X: %d mm, Y: %d mm, Speed: %d cm/s) | Total Occupancy: %d\n", 
                    index + 1, x_val, y_val, speed, roomOccupancy);
      // sendOccupancyUpdate(roomOccupancy);
    }
    targets[index].lastZone = ZONE_INSIDE;
  } 
  else if (x_val < -HYSTERESIS_MM) {
    // Target is clearly in the OUTSIDE zone (-X)
    if (targets[index].lastZone == ZONE_INSIDE) {
      roomOccupancy = max(0, roomOccupancy - 1); // Prevent negative room counts
      Serial.printf("[Target %d] EXITED  (X: %d mm, Y: %d mm, Speed: %d cm/s) | Total Occupancy: %d\n", 
                    index + 1, x_val, y_val, speed, roomOccupancy);
      // sendOccupancyUpdate(roomOccupancy);
    }
    targets[index].lastZone = ZONE_OUTSIDE;
  }
  // Note: If -150 <= x_val <= 150, target is inside the deadzone. Do nothing.
}

void loop() {
  // Non-blocking byte-by-byte stream synchronization
  while (Serial2.available() > 0) {
    uint8_t byteIn = Serial2.read();

    if (headerIdx < 4) {
      if (byteIn == HEADER[headerIdx]) {
        headerIdx++;
      } else {
        headerIdx = (byteIn == HEADER[0]) ? 1 : 0;
      }
    } 
    else {
      // Header matched! Read the remaining 26 bytes (24B data + 2B footer)
      size_t bytesRead = Serial2.readBytes(payloadBuf, 26);
      headerIdx = 0; // Reset state machine for next frame

      // Verify frame integrity via the 0x55 0xCC footer
      if (bytesRead == 26 && payloadBuf[24] == 0x55 && payloadBuf[25] == 0xCC) {
        uint32_t now = millis();
        // Loop through all 3 targets (offsets 0, 8, and 16)
        for (int i = 0; i < 3; i++) {
          processTarget(i, &payloadBuf[i * 8], now);
        }
      }
    }
  }
}

// Push real-time count to the central server
void sendOccupancyUpdate(int count) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"team\": 1, \"occupancy\": " + String(count) + "}";
    int httpResponseCode = http.POST(payload);
    http.end();
  }
}