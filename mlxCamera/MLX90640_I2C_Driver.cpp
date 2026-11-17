// Arduino Wire implementation of the I2C functions MLX90640_API.c expects.

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "MLX90640_I2C_Driver.h"
}

// ESP32 Wire buffers 128 bytes; keep each read chunk within that.
static const uint16_t kChunkBytes = 128;

extern "C" void MLX90640_I2CInit(void) {}

extern "C" void MLX90640_I2CFreqSet(int freqKHz) {
    Wire.setClock((uint32_t)freqKHz * 1000);
}

extern "C" int MLX90640_I2CGeneralReset(void) {
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    return Wire.endTransmission() == 0 ? 0 : -1;
}

extern "C" int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                                uint16_t nMemAddressRead, uint16_t *data) {
    uint32_t bytesRemaining = (uint32_t)nMemAddressRead * 2;
    uint16_t addr = startAddress;
    uint16_t idx = 0;

    while (bytesRemaining > 0) {
        uint16_t n = bytesRemaining > kChunkBytes ? kChunkBytes : bytesRemaining;

        Wire.beginTransmission(slaveAddr);
        Wire.write(addr >> 8);
        Wire.write(addr & 0xFF);
        if (Wire.endTransmission(false) != 0) return -1;

        if (Wire.requestFrom(slaveAddr, (uint8_t)n) != n) return -1;
        for (uint16_t k = 0; k < n / 2; ++k) {
            uint16_t hi = Wire.read();
            uint16_t lo = Wire.read();
            data[idx++] = (hi << 8) | lo;
        }

        bytesRemaining -= n;
        addr += n / 2;
    }
    return 0;
}

extern "C" int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(writeAddress >> 8);
    Wire.write(writeAddress & 0xFF);
    Wire.write(data >> 8);
    Wire.write(data & 0xFF);
    if (Wire.endTransmission() != 0) return -1;

    uint16_t check = 0;
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &check) != 0) return -1;
    return check == data ? 0 : -2;
}
