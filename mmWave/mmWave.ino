#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "LD2450.h"
#include "DoorCounter.h"

// --- Wi-Fi & Server Configuration ---
const char* ssid = "MWireless";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverUrl = "http://your-central-server.local/api/occupancy";
const bool ENABLE_WIFI = false;

// --- Radar UART ---
// The LD2450 is wired to the USB-serial pins: LD2450 TX -> GPIO3 (RX0) and
// LD2450 RX -> GPIO1 (TX0). GPIO3 is handed to UART2 at 256000 baud for the
// radar, while UART0 keeps GPIO1 for Serial Monitor output at 115200. As a
// result the ESP32 cannot receive Serial Monitor input or send radar commands
// (the radar ignores the debug text it receives on GPIO1).
// Unpower the radar while uploading: it drives the same line as the USB chip.
#define RADAR_RX_PIN 3
#define RADAR_TX_PIN -1

#define LED_PIN 2   // On-board LED on most ESP32 dev boards
#define BUTTON_PIN 0  // BOOT button, usable as an input after boot

LD2450 radar;
DoorCounter counter;

// Controls (BOOT button, since the Serial Monitor cannot send input):
//   short press        reset counts
//   hold 1-3 s         toggle live target/track printout (use to tune geometry)
//   hold > 3 s         swap which side is inside (also resets counts)
bool debugStream = false;

void sendOccupancyUpdate(int count);

// Non-blocking LED feedback: entry = one long blink, exit = two short blinks.
// Each pattern is a list of on/off durations in ms, starting with "on".
const uint16_t BLINK_ENTER[] = {400};
const uint16_t BLINK_EXIT[] = {120, 120, 120};
const uint16_t* blinkPattern = nullptr;
uint8_t blinkLen = 0;
uint8_t blinkStep = 0;
uint32_t blinkStepStart = 0;

void startBlink(const uint16_t* pattern, uint8_t len) {
  blinkPattern = pattern;
  blinkLen = len;
  blinkStep = 0;
  blinkStepStart = millis();
  digitalWrite(LED_PIN, HIGH);
}

void updateBlink() {
  if (!blinkPattern) return;
  if (millis() - blinkStepStart < blinkPattern[blinkStep]) return;
  blinkStepStart = millis();
  if (++blinkStep >= blinkLen) {
    blinkPattern = nullptr;
    digitalWrite(LED_PIN, LOW);
    return;
  }
  digitalWrite(LED_PIN, (blinkStep % 2 == 0) ? HIGH : LOW);
}

void onDoorEvent(DoorCounter::Event ev, const DoorCounter::Track& t, const DoorCounter& c) {
  if (ev == DoorCounter::ENTERED) startBlink(BLINK_ENTER, sizeof(BLINK_ENTER) / sizeof(BLINK_ENTER[0]));
  else startBlink(BLINK_EXIT, sizeof(BLINK_EXIT) / sizeof(BLINK_EXIT[0]));

  Serial.printf("[Track %u] %s (X: %d mm, Y: %d mm) | In: %lu  Out: %lu  Occupancy: %ld\n",
                t.id, ev == DoorCounter::ENTERED ? "ENTERED" : "EXITED ",
                t.x, t.y, (unsigned long)c.entries(), (unsigned long)c.exits(),
                (long)c.occupancy());
  if (ENABLE_WIFI) sendOccupancyUpdate(c.occupancy());
}

void printStatus() {
  Serial.printf("Frames: %lu (bad: %lu), last frame %lu ms ago | In: %lu  Out: %lu  Occupancy: %ld\n",
                (unsigned long)radar.frameCount(), (unsigned long)radar.badFrameCount(),
                (unsigned long)(millis() - radar.lastFrameMs()),
                (unsigned long)counter.entries(), (unsigned long)counter.exits(),
                (long)counter.occupancy());
}

void printFrame() {
  Serial.print("radar:");
  for (uint8_t i = 0; i < LD2450::kMaxTargets; ++i) {
    const LD2450::Target& t = radar.target(i);
    if (t.valid) Serial.printf(" [%d,%d %dcm/s]", t.x, t.y, t.speed);
  }
  Serial.print("  tracks:");
  for (uint8_t i = 0; i < DoorCounter::kMaxTracks; ++i) {
    const DoorCounter::Track& t = counter.track(i);
    if (t.active) {
      Serial.printf(" #%u(%d,%d %s)", t.id, t.x, t.y,
                    t.side > 0 ? "in" : (t.side < 0 ? "out" : "?"));
    }
  }
  Serial.println();
}

void printConfig() {
  const DoorCounter::Config& c = counter.config();
  Serial.printf("Line %c = %d mm (+/-%d), inside is %c%c, ROI X[%d,%d] Y[%d,%d], confirm %u frames\n",
                c.axis == DoorCounter::AXIS_X ? 'X' : 'Y', c.lineMm, c.hysteresisMm,
                c.insideIsPositive ? '+' : '-', c.axis == DoorCounter::AXIS_X ? 'X' : 'Y',
                c.xMinMm, c.xMaxMm, c.yMinMm, c.yMaxMm, c.confirmFrames);
}

void handleButton() {
  static bool wasDown = false;
  static uint32_t downSince = 0;
  bool down = digitalRead(BUTTON_PIN) == LOW;
  uint32_t now = millis();

  if (down && !wasDown) downSince = now;
  if (!down && wasDown) {
    uint32_t held = now - downSince;
    if (held < 50) {
      // bounce
    } else if (held < 1000) {
      counter.reset();
      Serial.println("Counts reset");
      printStatus();
    } else if (held < 3000) {
      debugStream = !debugStream;
      Serial.printf("Debug stream %s\n", debugStream ? "ON" : "OFF");
    } else {
      DoorCounter::Config c = counter.config();
      c.insideIsPositive = !c.insideIsPositive;
      counter.setConfig(c);
      counter.reset();
      Serial.println("Inside/outside swapped, counts reset");
      printConfig();
    }
  }
  wasDown = down;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(200);

  // Geometry: radar on top of the door frame, antenna facing the floor, rotated
  // so its X axis runs in the walking direction (perpendicular to the door).
  // Y is then the distance straight down to the person, and the doorway is the
  // line X = 0 directly under the sensor. The room is +X unless flipped (hold BOOT > 3 s).
  DoorCounter::Config cfg;
  cfg.axis = DoorCounter::AXIS_X;
  cfg.lineMm = 0;
  cfg.hysteresisMm = 150;
  cfg.insideIsPositive = true;
  cfg.xMinMm = -2500;
  cfg.xMaxMm = 2500;
  cfg.yMinMm = 150;
  cfg.yMaxMm = 2500;
  counter.setConfig(cfg);
  counter.onEvent(onDoorEvent);

  // Must come after Serial.begin(): this moves GPIO3 from UART0 to UART2.
  // The LD2450 defaults to multi-target mode, so no configuration is sent.
  radar.begin(Serial2, RADAR_RX_PIN, RADAR_TX_PIN);

  if (ENABLE_WIFI) {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  }

  printConfig();
  Serial.println("Ready. BOOT button: tap = reset counts, hold 1-3 s = debug stream, hold >3 s = swap in/out");
}

void loop() {
  handleButton();
  updateBlink();

  if (radar.update()) {
    counter.update(radar);
    if (debugStream) printFrame();
  }

  static uint32_t lastWarn = 0;
  if (millis() - radar.lastFrameMs() > 2000 && millis() - lastWarn > 2000) {
    lastWarn = millis();
    if (radar.bytesReceived() == 0) {
      Serial.println("No data from LD2450 - check 5V power, shared GND with ESP32, and LD2450 TX -> GPIO3 (RX0)");
    } else {
      Serial.printf("LD2450 sending data (%lu bytes) but no valid frames - check baud (256000) / signal quality\n",
                    (unsigned long)radar.bytesReceived());
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
    http.POST(payload);
    http.end();
  }
}