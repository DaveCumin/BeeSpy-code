#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h> // Library for DS3231 RTC
#include "SparkFun_SCD4x_Arduino_Library.h" // SCD4x library

SCD4x mySensor;
RTC_DS3231 rtc;
const int chipSelect = 10; // SD card CS pin (adjust if different)

void setup()
{
  Serial.begin(115200);
  Serial.println(F("SCD4x Example with SD and RTC"));
  Wire.begin();

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println(F("RTC not detected. Please check wiring. Freezing..."));
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, setting to default time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time
  }

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD card initialization failed! Freezing..."));
    while (1);
  }
  Serial.println(F("SD card initialized."));

  // Initialize SCD4x sensor
  if (mySensor.begin() == false) {
    Serial.println(F("SCD4x not detected. Please check wiring. Freezing..."));
    while (1);
  }

  // Stop periodic measurements to enable low power mode
  if (mySensor.stopPeriodicMeasurement() == true) {
    Serial.println(F("Periodic measurement disabled."));
  }

  // Enable low power periodic measurements
  if (mySensor.startLowPowerPeriodicMeasurement() == true) {
    Serial.println(F("Low power mode enabled."));
  }
}

void loop()
{
  if (mySensor.readMeasurement()) // Check if fresh data is available
  {
    // Get current time from RTC
    DateTime now = rtc.now();
    char timestamp[20];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

    // Prepare data
    float co2 = mySensor.getCO2();
    float temp = mySensor.getTemperature();
    float humidity = mySensor.getHumidity();

    // Print to Serial
    Serial.println();
    Serial.print(F("CO2(ppm):"));
    Serial.print(co2);
    Serial.print(F("\tTemperature(C):"));
    Serial.print(temp, 1);
    Serial.print(F("\tHumidity(%RH):"));
    Serial.print(humidity, 1);
    Serial.print(F("\tTimestamp:"));
    Serial.print(timestamp);
    Serial.println();

    // Write to SD card
    File dataFile = SD.open("measures.csv", FILE_WRITE);
    if (dataFile) {
      // Write header if file is empty
      if (dataFile.size() == 0) {
        dataFile.println("Timestamp,CO2(ppm),Temperature(C),Humidity(%RH)");
      }
      // Write data
      dataFile.print(timestamp);
      dataFile.print(",");
      dataFile.print(co2);
      dataFile.print(",");
      dataFile.print(temp, 1);
      dataFile.print(",");
      dataFile.println(humidity, 1);
      dataFile.close();
      Serial.println(F("Data written to measures.csv"));
    } else {
      Serial.println(F("Error opening measures.csv"));
    }
  }
  else {
    Serial.print(F("."));
  }

  delay(300000); // Wait for next measurement (~300s in low power mode)
}