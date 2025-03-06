// VERSION 1.2
// To reset the RTC time.
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

#include "RTClib.h";

//----------------------------------------
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
//----------------------------------------

//------------------------------------------------------------------------------
// Function to stop the RTC (set CH bit to 1)
void stopRTC()
{
    Wire.beginTransmission(0x68); // 0x68 is the I2C address of DS1307
    Wire.write(0x00);             // Start at the seconds register (address 0x00)
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);     // Request 1 byte (the seconds register)
    uint8_t seconds = Wire.read(); // Read the seconds register

    // Set the CH (Clock Halt) bit to 1 to stop the clock
    seconds |= 0x80;

    // Write the modified seconds back to stop the clock
    Wire.beginTransmission(0x68);
    Wire.write(0x00);    // Start at the seconds register
    Wire.write(seconds); // Write the modified seconds register (CH bit set)
    Wire.endTransmission();

    Serial.println("RTC stopped!");
}

// Function to start the RTC (clear CH bit to 0)
void startRTC()
{
    Wire.beginTransmission(0x68); // 0x68 is the I2C address of DS1307
    Wire.write(0x00);             // Start at the seconds register (address 0x00)
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);     // Request 1 byte (the seconds register)
    uint8_t seconds = Wire.read(); // Read the seconds register

    // Clear the CH (Clock Halt) bit to 0 to start the clock
    seconds &= 0x7F;

    // Write the modified seconds back to start the clock
    Wire.beginTransmission(0x68);
    Wire.write(0x00);    // Start at the seconds register
    Wire.write(seconds); // Write the modified seconds register (CH bit cleared)
    Wire.endTransmission();

    Serial.println("RTC started!");
}

//------------------------------------------------------------------------------
void printTime(DateTime timeIN)
{
    Serial.print(timeIN.year(), DEC);
    Serial.print('/');
    Serial.print(timeIN.month(), DEC);
    Serial.print('/');
    Serial.print(timeIN.day(), DEC);
    Serial.print(" ");
    Serial.print(timeIN.hour(), DEC);
    Serial.print(':');
    Serial.print(timeIN.minute(), DEC);
    Serial.print(':');
    Serial.print(timeIN.second(), DEC);
    Serial.println();
}

//------------------------------------------------------------------------------

void setup()
{
    Serial.begin(9600);
    Serial.println("Going to reset the RTC...");

    if (!rtc.begin())
    {
        Serial.println("Could not find RTC.");
    }

    stopRTC();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    startRTC();
    Serial.println("Reset RTC time to:");
    printTime(rtc.now());
}

void loop()
{
    printTime(rtc.now());
    delay(1000);
}
