#include <MPU9250.h>

#define MPU_SCK  (14)
#define MPU_MISO (12)
#define MPU_MOSI (13)
#define MPU_CS   (15)

// MPU9250 mpu(MPU_CS);
MPU9250 mpu(MPU_SCK, MPU_MISO, MPU_MOSI, MPU_CS);

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing MPU9250 (SPI)...");
  if (!mpu.begin()) {
    Serial.println("Error: MPU9250 not found or invalid CHIP_ID.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("MPU9250 successfully detected!");
}

void loop() {
  // Next steps: configure sensor and read accelerometer/gyroscope/magnetometer data
  delay(1000);
}