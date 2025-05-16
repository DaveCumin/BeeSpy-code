#include <Wire.h>
#include <Protocentral_FDC1004.h>
#include "Adafruit_SHT31.h"  // Changed here
#include "ccs811.h"
#include "SdFat.h"

// FDC1004 sensor initialization
#define UPPER_BOUND 0X4000
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0
#define MEASURMENT 0

int capdac = 0;
FDC1004 FDC;

// SHT31 Temperature & Humidity sensor initialization
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // Changed here
float temp, humd;
bool foundTempHum = false;  // so can skip if there is an error with only this sensor


// CCS811 eCO2 and TVOC sensor initialization
CCS811 ccs811;
uint16_t eco2, etvoc, errstat, raw;
bool foundCO2 = false;  // so can skip if there is an error with only this sensor

// SD card initialization
SdFat SD;
File dataFile;
const int chipSelect = 53;

unsigned long lastFileSwitchTime = 0;
const unsigned long fileDuration = 1800000;  // 30 minutes
int fileCount = 1;

unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadDuration = 300000;  // 5 minutes
unsigned long lastCapacitanceReadTime = 0;
const unsigned long capacitanceReadDuration = 250;  // 250ms

long readCapacitance() {
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);
  delay(10);

  uint16_t value[2];
  if (!FDC.readMeasurement(MEASURMENT, value)) {
    int16_t msb = (int16_t)value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb);
    capacitance /= 1000;
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    if (msb > UPPER_BOUND && capdac < FDC1004_CAPDAC_MAX) {
      capdac++;
    } else if (msb < LOWER_BOUND && capdac > 0) {
      capdac--;
    }

    return capacitance;
  }

  return -1;
}

void readTemperatureAndHumidity(float &temp, float &humd) {
  temp = sht31.readTemperature();
  humd = sht31.readHumidity();

  if (isnan(temp) || isnan(humd)) {
    temp = -999.99;
    humd = -999.99;
  }
}

bool readCO2TVOC(uint16_t &eco2, uint16_t &etvoc) {
  ccs811.read(&eco2, &etvoc, &errstat, &raw);

  if (errstat == CCS811_ERRSTAT_OK) {
    return true;
  } else {
    if (errstat == CCS811_ERRSTAT_OK_NODATA) {
      Serial.println("CCS811: waiting for new data");
    } else if (errstat & CCS811_ERRSTAT_I2CFAIL) {
      Serial.println("CCS811: I2C error");
    } else {
      Serial.print("CCS811: errstat=");
      Serial.print(errstat, HEX);
      Serial.print("=");
      Serial.println(ccs811.errstat_str(errstat));
    }
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Sensor Test - Capacitance, Temp/Humidity, eCO2/TVOC");

  Wire.begin();

  // Initialize SHT31
  if (!sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31 sensor");
  } else {
    foundTempHum = true;
  }


  // Initialize CCS811 sensor
  if (!ccs811.begin()) {
    Serial.println("setup: CCS811 begin FAILED");
    foundCO2 = false;
  } else {
    // Start measuring (1 second mode) for CCS811
    if (!ccs811.start(CCS811_MODE_1SEC)) {
      Serial.println("setup: CCS811 start FAILED");
      foundCO2 = false;
    } else {
      foundCO2 = true;
    }
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastCapacitanceReadTime >= capacitanceReadDuration) {
    lastCapacitanceReadTime = currentMillis;
    long capacitance = readCapacitance();

    String capacitanceLine = String(currentMillis) + "," + "Capacitance" + "," + String(capacitance);
    if (dataFile)
      dataFile.println(capacitanceLine);
    Serial.println(capacitanceLine);
  }

  if (currentMillis - lastSensorReadTime >= sensorReadDuration) {
    lastSensorReadTime = currentMillis;

    if (foundTempHum) {
      readTemperatureAndHumidity(temp, humd);

      String tempLine = String(currentMillis) + "," + "Temperature" + "," + String(temp, 1);
      if (dataFile) {
        dataFile.println(tempLine);
      }
      String humidityLine = String(currentMillis) + "," + "Humidity" + "," + String(humd, 1);
      if (dataFile) {
        dataFile.println(humidityLine);
      }
      Serial.println(tempLine);
      Serial.println(humidityLine);
    }

    if (foundCO2) {
      bool success = readCO2TVOC(eco2, etvoc);

      String eco2Line = String(currentMillis) + "," + "eCO2" + "," + String(eco2);
      if (dataFile) {
        dataFile.println(eco2Line);
      }
      String tvocLine = String(currentMillis) + "," + "TVOC" + "," + String(etvoc);
      if (dataFile) {
        dataFile.println(tvocLine);
      }
      Serial.println(eco2Line);
      Serial.println(tvocLine);
    }
  }
}