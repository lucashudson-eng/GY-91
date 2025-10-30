#include <BMP280.h>

#define BMP_SCK  (14)
#define BMP_MISO (12)
#define BMP_MOSI (13)
#define BMP_CS   (23)

// BMP280 bmp(BMP_CS);
BMP280 bmp(BMP_SCK, BMP_MISO, BMP_MOSI, BMP_CS);

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing BMP280 (SPI)...");
  if (!bmp.begin()) {
    Serial.println("Error: BMP280 not found or invalid CHIP_ID.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("BMP280 successfully detected!");
}

void loop() {
  // Next steps: configure oversampling and read temperature/pressure
  delay(1000);
}