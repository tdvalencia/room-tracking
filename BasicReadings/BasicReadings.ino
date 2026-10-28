/*
    Read the temperature pixels from the MLX90640 IR array
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


/*
    Read and print all temperature pixels from the MLX90640 IR array (32x24 grid)
*/

#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX9064X_I2C_Driver.h"

const byte MLX90640_address = 0x33; // Default 7-bit unshifted address

#define TA_SHIFT 8 // Default shift for MLX90640 in open air

static float mlx90640To[768];
paramsMLX90640 mlx90640;

void setup() {
    Wire.begin();
    Wire.setClock(400000); // Increase I2C clock speed to 400kHz

    // Increased baud rate to handle streaming 768 values quickly
    debug.begin(115200);
    while (!debug); // Wait for user to open terminal
    debug.println("MLX90640 Full Frame Example");

    if (isConnected() == false) {
        debug.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
    }
    debug.println("MLX90640 online!");

    // Get device parameters - We only have to do this once
    int status;
    uint16_t eeMLX90640[832];
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0) {
        debug.println("Failed to load system parameters");
    }

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0) {
        debug.println("Parameter extraction failed");
    }
}

void loop() {
    for (byte x = 0 ; x < 2 ; x++) { // Read both subpages
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0) {
            debug.print("GetFrame Error: ");
            debug.println(status);
            continue; // Skip if frame read failed
        }

        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

        float tr = Ta - TA_SHIFT; // Reflected temperature
        float emissivity = 0.95;

        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    }

    // Print the full 32x24 grid (24 rows, 32 columns)
    debug.println("--- NEW FRAME ---");
    for (int row = 0; row < 24; row++) {
        for (int col = 0; col < 32; col++) {
            int pixelIndex = row * 32 + col;
            debug.print(mlx90640To[pixelIndex], 1); // 1 decimal place
            if (col < 31) {
                debug.print(","); // Comma-separated values per row
            }
        }
        debug.println(); // Newline after each row
    }

    delay(10); // Wait 1 second before the next frame
}

// Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0) {
        return (false);    // Sensor did not ACK
    }
    return (true);
}