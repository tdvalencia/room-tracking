// MLX I2C base address is 0x33

#include <Wire.h>

void setup() {
  Wire.begin(); // Join the I2C bus as a Master
}

void loop() {
  Wire.beginTransmission(0x08); // Start communicating with slave at address 0x08
  
  Wire.write("Hello");          // Queue bytes to send
  Wire.write(123);              // Queue a single integer byte
  
  Wire.endTransmission();       // Transmit the queued bytes and end communication
  
  delay(500);                   // Wait half a second
}