// VERSION 4.22
// Working on the bench.
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

#include <Wire.h>
#include <Protocentral_FDC1004.h>
#include "SparkFunHTU21D.h"
#include "Adafruit_CCS811.h"
#include "SdFat.h"
#include "RTClib.h"

// RTC
#define USE_RTC 1
// 1 - DS1307
// 2 - DS3231
// 3 - PCF8523
#if USE_RTC == 1
RTC_DS1307 rtc;
#elif USE_RTC == 2
RTC_DS3231 rtc;
#elif USE_RTC == 3
RTC_PCF8523 rtc;
#endif

// FDC1004 sensor initialization
#define UPPER_BOUND 0X4000 // Max readout capacitance
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0    // Channel to be read
#define MEASURMENT 0 // Measurement channel

int capdac = 0; // CapDAC value for adjusting measurement range
FDC1004 FDC;    // FDC1004 sensor object

// HTU21D Temperature & Humidity sensor initialization
HTU21D myHumidity; // Create an instance of the HTU21D object
float temp, humd;
bool foundHTU21D = false; // so can skip if there is an error with only this sensor

// CCS811 eCO2 and TVOC sensor initialization
Adafruit_CCS811 ccs811;
uint16_t eco2, etvoc, ccstemp;
bool foundCO2 = false; // so can skip if there is an error with only this sensor

// SD stuff
//  SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
//  1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
#if SD_FAT_TYPE == 0
SdFat sd;
typedef File file_t;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
SdExFat sd;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
SdFs sd;
typedef FsFile file_t;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE
file_t dataFile;
char filename[30];
const uint8_t SD_CS_PIN = 53;

unsigned long fileStartMillis = 0; // Timer to track the time, when the file starts
// Timer for file switching (30 minutes = 1800 seconds)
unsigned long lastFileSwitchTime = 0;
const unsigned long fileDuration = 1800000; // 1800000;  // 30 minutes in milliseconds

// Timer for measuring temperature, humidity, eCO2, TVOC (5 minutes = 300,000ms)
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadDuration = 300000; // 5 minutes in milliseconds

// Timer for measuring capacitance every 250ms
unsigned long lastCapacitanceReadTime = 0;
const unsigned long capacitanceReadDuration = 250; // 250ms

// Function to read capacitance value from the FDC1004 sensor
long readCapacitance(long &capacitance)
{
    FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac); // Configure the sensor for single measurement on channel 0
    FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);     // Trigger the measurement at 100Hz
    // Wait for the sensor to settle
    delay(15); // Allow enough time for the sensor to complete the measurement
    uint16_t value[2];
    if (!FDC.readMeasurement(MEASURMENT, value))
    {                                                       // Check if the reading was successful
        int16_t msb = (int16_t)value[0];                    // Get the most significant byte from the reading
        capacitance = ((int32_t)457) * ((int32_t)msb);      // Calculate capacitance in aF (femtofarads)
        capacitance /= 1000;                                // Convert to femtofarads (fF)
        capacitance += ((int32_t)3028) * ((int32_t)capdac); // Adjust for CapDAC
        // Adjust CapDAC based on the measured value to stay within the range
        if (msb > UPPER_BOUND)
        {
            if (capdac < FDC1004_CAPDAC_MAX)
            {
                capdac++; // Increase CapDAC if the value is too high
            }
        }
        else if (msb < LOWER_BOUND)
        {
            if (capdac > 0)
            {
                capdac--; // Decrease CapDAC if the value is too low
            }
        }

        return capacitance; // Return the capacitance in femtofarads
    }

    return -1; // Return an error code if measurement fails
}

// Function to read temperature and humidity from the HTU21D sensor
void readTemperatureAndHumidity(float &temp, float &humd)
{
    // Read temperature and humidity
    humd = myHumidity.readHumidity();
    temp = myHumidity.readTemperature();

    // Check if the readings are invalid
    if (humd == -999.99 || temp == -999.99)
    {
        humd = -999.99;
        temp = -999.99;
    }
}

// Function to read eCO2 and TVOC values from the CCS811 sensor
void readCO2TVOC(uint16_t &eco2, uint16_t &etvoc, uint16_t &ccstemp)
{
    if (ccs811.available())
    {
        if (!ccs811.readData())
        {
            ccstemp = ccs811.calculateTemperature();
            etvoc = ccs811.getTVOC();
            eco2 = ccs811.geteCO2();
        }
    }
}

// Function to create a new filenameusing the RTC timestamp
void makeNewFile()
{
    DateTime time = rtc.now(); // Get current date and time from RTC
    sprintf(filename, "%04d_%02d_%02d_%02d_%02d_%02d_cap.txt",
            time.year(), time.month(), time.day(),
            time.hour(), time.minute(), time.second()); // Format the filename based on this timestamp

    Serial.print("Trying to make file: ");
    Serial.println(filename);

    // Open the file on SD card for writing and appending
    if (!dataFile.open(filename, FILE_WRITE))
    {
        Serial.println("Failed to open new file for writing!");
    }
    fileStartMillis = millis(); // Record the start time of the current file being written too
    Serial.print("File made: ");
    Serial.println(filename);
}
// Function to switch to a new file after 30 minutes
void switchFile()
{
    // Close the current file
    dataFile.close();
    makeNewFile();
}

void setup()
{
    // Initialize Serial communication
    Serial.begin(9600);
    Serial.println("Sensor Test - Capacitance, Temp/Humidity, eCO2/TVOC");

    // Initialize I2C
    Wire.begin();

    // Initialize the sensors
    myHumidity.begin();
    if (myHumidity.readTemperature() > 900)
    { // the default when not found is 998
        Serial.println("Check circuit. HTU21D not found!");
        foundHTU21D = false;
    }
    else
    {
        foundHTU21D = true;
    }

    // Initialize CCS811 sensor
    if (!ccs811.begin())
    {
        Serial.println("setup: CCS811 begin FAILED");
        foundCO2 = false;
    }
    else
    {
        // calibrate temp sensor
        float temp = ccs811.calculateTemperature();
        ccs811.setTempOffset(temp - 25.0);
        foundCO2 = true;
    }

    // Initialize SD card
    if (!sd.begin(SD_CS_PIN))
    {
        Serial.println("SD card initialization failed!");
        return;
    }

    Serial.println("SD card initialized.");

    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        while (1)
            delay(10);
    }

    // Open the first file
    makeNewFile(); // Open the first file at startup
}

void loop()
{
    unsigned long currentMillis = millis() - fileStartMillis; // Get elapsed time since the start of the current file

    // Check if it's time to switch to a new file (which is every 30 minutes)
    if (currentMillis >= fileDuration)
    {
        switchFile(); // Switch to a new file
    }

    // Check if it's time to log capacitance (every 250ms)
    if (currentMillis - lastCapacitanceReadTime >= capacitanceReadDuration)
    {
        lastCapacitanceReadTime = currentMillis;

        // Read capacitance value
        long capacitance = 0;
        unsigned long captimems = currentMillis;
        readCapacitance(capacitance);

        // Log capacitance data to SD card
        String capacitanceLine = String(captimems) + "," +
                                 "Capacitance" + "," +
                                 String(capacitance); // Add label and measurement value
        if (dataFile)
        {
            dataFile.println(capacitanceLine); // Write data to the file
        }

        // Print capacitance data to the serial monitor
        Serial.println(capacitanceLine);
    }

    // Check if it's time to log temperature, humidity, eCO2, TVOC (every 5 minutes)
    if (currentMillis - lastSensorReadTime >= sensorReadDuration)
    {
        lastSensorReadTime = currentMillis;

        // Read temperature and humidity
        if (foundHTU21D)
        {
            unsigned long temtimems = currentMillis;
            readTemperatureAndHumidity(temp, humd);

            // Log temperature data to SD card
            String tempLine = String(temtimems) + "," +
                              "Temperature" + "," +
                              String(temp, 1);
            if (dataFile)
            {
                dataFile.println(tempLine); // Write data to the file
            }

            // Log humidity data to SD card
            String humidityLine = String(temtimems) + "," +
                                  "Humidity" + "," +
                                  String(humd, 1);
            if (dataFile)
            {
                dataFile.println(humidityLine); // Write data to the file
            }
            // Print data to the serial monitor
            Serial.println(tempLine);
            Serial.println(humidityLine);
        }

        // Read eCO2 and TVOC values
        if (foundCO2)
        {
            unsigned long co2timems = currentMillis;
            readCO2TVOC(eco2, etvoc, ccstemp); // Read eCO2 and TVOC data

            // Log eCO2 and TVOC data to SD card
            String eco2Line = String(co2timems) + "," +
                              "eCO2" + "," +
                              String(eco2);
            if (dataFile)
            {
                dataFile.println(eco2Line); // Write data to the file
            }

            String tvocLine = String(co2timems) + "," +
                              "TVOC" + "," +
                              String(etvoc);
            if (dataFile)
            {
                dataFile.println(tvocLine); // Write data to the file
            }

            String ctempLine = String(co2timems) + "," +
                               "CTemp" + "," +
                               String(ccstemp);
            if (dataFile)
            {
                dataFile.println(ccstemp); // Write data to the file
            }

            // Print data to the serial monitor
            Serial.println(eco2Line);
            Serial.println(tvocLine);
            Serial.println(ctempLine);
        }
    }
}