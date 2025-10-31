#include <MPU9250.h>

// If using SPI

#define MPU_SCK  (14)
#define MPU_MISO (12)
#define MPU_MOSI (13)
#define MPU_CS   (15)

// MPU9250 mpu(MPU_CS); // Default constructor uses SPI with hardware default pins
MPU9250 mpu(MPU_SCK, MPU_MISO, MPU_MOSI, MPU_CS); // Custom SPI pins

// If using I2C

// #define MPU_SDA  (21)
// #define MPU_SCL  (22)

// MPU9250 mpu; // Default constructor uses I2C with default pins
// MPU9250 mpu(MPU_SDA, MPU_SCL); // Custom I2C pins

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing MPU9250...");
  if (!mpu.begin()) {
    Serial.println("Error: MPU9250 not found or invalid CHIP_ID.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("MPU9250 successfully detected!");
  
  // Configure sensor settings
  Serial.println("Configuring sensor...");
  
  // Set accelerometer range to ±4g
  mpu.setAccelerometerRange(MPU9250_ACCEL_FS_4G);
  Serial.println("  Accelerometer range: ±4g");
  
  // Set gyroscope range to ±500°/s
  mpu.setGyroRange(MPU9250_GYRO_FS_500DPS);
  Serial.println("  Gyroscope range: ±500°/s");
  
  // Set gyroscope filter bandwidth to 41Hz
  mpu.setFilterBandwidth(MPU9250_DLPF_41HZ);
  Serial.println("  Gyroscope filter: 41Hz");
  
  // Set accelerometer filter bandwidth to 20Hz
  mpu.setAccelFilterBandwidth(MPU9250_ACCEL_DLPF_20HZ);
  Serial.println("  Accelerometer filter: 20Hz");
  
  // Set sample rate divider (1kHz / (1 + 4) = 200Hz)
  mpu.setSampleRateDivider(4);
  Serial.println("  Sample rate: 200Hz");
  
  Serial.println();
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

  delay(500);  // 2Hz output rate
}