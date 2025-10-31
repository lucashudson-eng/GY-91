#include <MPU9250.h>

// Example using I2C with default pins (SDA/SCL default)
// For Arduino Uno/Nano: SDA = A4, SCL = A5
// For ESP32: can specify custom pins
MPU9250 mpu;

// Alternative: use custom I2C pins (for ESP32, etc.)
// #define I2C_SDA  21
// #define I2C_SCL  22
// MPU9250 mpu(I2C_SDA, I2C_SCL);

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing MPU9250 (I2C)...");
  
  // For I2C, you can optionally specify address (default 0x68)
  // Use 0x69 if AD0 pin is HIGH
  if (!mpu.begin()) {
    Serial.println("Error: MPU9250 not found or invalid CHIP_ID.");
    Serial.println("Check I2C connections:");
    Serial.println("  - SDA connected");
    Serial.println("  - SCL connected");
    Serial.println("  - VCC to 3.3V or 5V");
    Serial.println("  - GND to GND");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("MPU9250 successfully detected!");
  Serial.println("Reading accelerometer and gyroscope data...");
  Serial.println();
}

void loop() {
  // Read accelerometer data
  AccelData accel = mpu.readAccel();
  
  // Read gyroscope data
  GyroData gyro = mpu.readGyro();

  // Print accelerometer data (in g)
  Serial.print("Accel (g): ");
  Serial.print("X=");
  Serial.print(accel.x, 3);
  Serial.print("  Y=");
  Serial.print(accel.y, 3);
  Serial.print("  Z=");
  Serial.print(accel.z, 3);

  // Print gyroscope data (in deg/s)
  Serial.print("  |  Gyro (deg/s): ");
  Serial.print("X=");
  Serial.print(gyro.x, 2);
  Serial.print("  Y=");
  Serial.print(gyro.y, 2);
  Serial.print("  Z=");
  Serial.println(gyro.z, 2);

  delay(100);
}

