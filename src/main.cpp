
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// Differential channel pairs
// Lift: A0 (+), A1 (-)
// Drag: A2 (+), A3 (-)

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ESP32 I2C pins: SDA = GPIO 21, SCL = GPIO 22
  Wire.begin(21, 22);

  Serial.println("\nESP32 + ADS1115 Dual Load Cell Test");
  Serial.println("-----------------------------------");

  if (!ads.begin(0x48, &Wire)) {
    Serial.println("ERROR: ADS1115 not detected.");
    Serial.println("Check VIN, GND, SDA, and SCL.");

    while (true) {
      delay(100);
    }
  }


  // +/- 4.096 V full-scale range
  ads.setGain(GAIN_SIXTEEN);

  Serial.println("ADS1115 detected!");
  Serial.println("Lift: A0-A1 | Drag: A2-A3");
  Serial.println();
}

void loop() {
  // Read differential voltage for lift
  int16_t liftRaw = ads.readADC_Differential_0_1();

  // Read differential voltage for drag
  int16_t dragRaw = ads.readADC_Differential_2_3();

  // Convert ADC readings to volts
  float liftVolts = ads.computeVolts(liftRaw);
  float dragVolts = ads.computeVolts(dragRaw);

  Serial.print("Lift raw: ");
  Serial.print(liftRaw);
  Serial.print(" | Lift differential: ");
  Serial.print(liftVolts, 6);
  Serial.print(" V");

  Serial.print(" || Drag raw: ");
  Serial.print(dragRaw);
  Serial.print(" | Drag differential: ");
  Serial.print(dragVolts, 6);
  Serial.println(" V");

  delay(1000);
}

