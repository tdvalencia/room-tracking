/*
    Read MLX90640 frame, detect radius-based local maximums, 
    print the hotspot count, individual hotspot coordinates/temps, and the full 32x24 grid.
*/

#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX9064X_I2C_Driver.h"

#if defined(ARDUINO_ARCH_AVR)
    #define debug  Serial
#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
    #define debug  Serial
#else
    #define debug  Serial
#endif

const byte MLX90640_address = 0x33; // Default 7-bit unshifted address
#define TA_SHIFT 8 // Default shift for MLX90640 in open air

// Detection tuning parameters
#define TEMP_THRESHOLD 28.0       // Minimum temperature (°C) to qualify as a hotspot
#define HOTSPOT_RADIUS 4          // Radius for local maximum comparison (2 = 5x5 neighborhood window)

static float mlx90640To[768];
paramsMLX90640 mlx90640;

void setup() {
    Wire.begin();
    Wire.setClock(400000); // Increase I2C clock speed to 400kHz

    debug.begin(115200);
    while (!debug); 
    debug.println("MLX90640 Comprehensive Frame & Hotspot Analyzer Initialized");

    if (isConnected() == false) {
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

    // Set refresh rate to 8Hz (0x04)
    MLX90640_SetRefreshRate(MLX90640_address, 0x04);
}

void loop() {
    for (byte x = 0 ; x < 2 ; x++) { // Read both subpages
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0) {
            continue; // Skip if frame read failed (-8 error recovery)
        }

        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

        float tr = Ta - TA_SHIFT;
        float emissivity = 0.95;

        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    }

    // --- SECTION 1: HOTSPOT ANALYSIS & COUNT ---
    debug.println("========================================");
    debug.println("          FRAME HOTSPOT REPORT          ");
    debug.println("========================================");
    
    int hotspotCount = analyzeAndPrintHotspots();
    
    debug.print("-> Total Hotspots Count: ");
    debug.println(hotspotCount);
    debug.println("----------------------------------------");

    // --- SECTION 2: FULL 32x24 FRAME CSV ---
    debug.println("--- FULL 32x24 FRAME CSV DATA ---");
    for (int row = 0; row < 24; row++) {
        for (int col = 0; col < 32; col++) {
            int pixelIndex = row * 32 + col;
            debug.print(mlx90640To[pixelIndex], 1); // 1 decimal place
            if (col < 31) {
                debug.print(",");
            }
        }
        debug.println();
    }
    debug.println("========================================\n");

    delay(1000); // 1 second pacing between full frame evaluations
}

// Scans grid for strict local maximums within HOTSPOT_RADIUS, prints details, and returns count
int analyzeAndPrintHotspots() {
    int numHotspots = 0;
    int R = HOTSPOT_RADIUS;

    for (int row = R; row < (24 - R); row++) {
        for (int col = R; col < (32 - R); col++) {
            int idx = row * 32 + col;
            float currentVal = mlx90640To[idx];

            // Filter out cold background elements
            if (currentVal < TEMP_THRESHOLD) continue;

            // Check if it is strictly the highest pixel within radius R
            bool isLocalMax = true;
            for (int dr = -R; dr <= R; dr++) {
                for (int dc = -R; dc <= R; dc++) {
                    if (dr == 0 && dc == 0) continue;
                    
                    int neighborIdx = (row + dr) * 32 + (col + dc);
                    if (mlx90640To[neighborIdx] >= currentVal) {
                        isLocalMax = false;
                        break;
                    }
                }
                if (!isLocalMax) break;
            }

            if (isLocalMax) {
                numHotspots++;
                debug.print("  [#");
                debug.print(numHotspots);
                debug.print("] Row: ");
                debug.print(row);
                debug.print(" | Col: ");
                debug.print(col);
                debug.print(" | Temp: ");
                debug.print(currentVal, 1);
                debug.println("°C");
            }
        }
    }
    
    if (numHotspots == 0) {
        debug.println("  (No hotspots found above threshold)");
    }
    
    return numHotspots;
}

boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0) return false;
    return true;
}