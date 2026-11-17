// MLX90640 field-of-view test (BAA 110° vs BAB 55°).
//
// Prints the hottest pixel every frame plus a 32-character bar showing its
// column. Point the camera at a wall ~1 m away and slide a warm object (hand,
// mug of hot water) sideways along the 32-pixel axis until the column reaches
// 0 or 31. Measure the object's distance from the center line at that point:
//   ~0.5 m  -> BAB (55° x 35°)
//   ~1.4 m  -> BAA (110° x 75°)
//
// Wiring (ESP32): SDA -> GPIO 21, SCL -> GPIO 22, 3.3 V, GND. I2C address 0x33.

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
}

#define I2C_SDA 21
#define I2C_SCL 22
const uint8_t MLX_ADDR = 0x33;

paramsMLX90640 params;
uint16_t eeData[832];
uint16_t frameData[834];
float temps[768];

void halt(const char* msg) {
  Serial.println(msg);
  while (true) delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(MLX_ADDR);
  if (Wire.endTransmission() != 0) halt("MLX90640 not found at 0x33 - check SDA/SCL/power");

  if (MLX90640_DumpEE(MLX_ADDR, eeData) != 0) halt("EEPROM read failed");
  if (MLX90640_ExtractParameters(eeData, &params) != 0) halt("Parameter extraction failed");

  MLX90640_SetChessMode(MLX_ADDR);
  MLX90640_SetRefreshRate(MLX_ADDR, 0x04);  // 8 Hz subpages = 4 full frames/s

  Serial.println("FOV test: column 0..31 of the hottest pixel. Slide a warm object along the long axis.");
}

void loop() {
  // Each GetFrameData returns one chess subpage; read both for a full image.
  float ta = 0;
  for (int sp = 0; sp < 2; ++sp) {
    if (MLX90640_GetFrameData(MLX_ADDR, frameData) < 0) {
      Serial.println("Frame read error");
      return;
    }
    ta = MLX90640_GetTa(frameData, &params);
    MLX90640_CalculateTo(frameData, &params, 0.95f, ta - 8.0f, temps);
  }

  int hottest = 0;
  for (int i = 1; i < 768; ++i) {
    if (temps[i] > temps[hottest]) hottest = i;
  }
  int row = hottest / 32;
  int col = hottest % 32;

  char bar[33];
  for (int c = 0; c < 32; ++c) bar[c] = '.';
  bar[col] = '#';
  bar[32] = '\0';

  Serial.printf("|%s|  col %2d  row %2d  %.1f C  (ambient %.1f C)\n",
                bar, col, row, temps[hottest], ta);
}
