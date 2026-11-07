#include <Wire.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint32_t SCAN_INTERVAL_MS = 3000;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println("I2C_SCAN_START");
}

void loop() {
  uint8_t found = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("FOUND,0x%02X\n", address);
      ++found;
    }
  }

  Serial.printf("I2C_SCAN_END,count=%u\n", found);
  delay(SCAN_INTERVAL_MS);
}
