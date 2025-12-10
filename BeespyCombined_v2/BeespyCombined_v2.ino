/**
 * Multi-sensor data logger for Arduino Mega 2560
 * - 6 ADC channels at 5kHz (10 minute binary files)
 * - FDC1004 capacitance at 4Hz (30 minute CSV files)
 * - SCD41 CO2/temp/humidity every 10 minutes
 */

#ifdef __AVR__
#include <SPI.h>
#include <Wire.h>

#include "AvrAdcLogger.h"
#include "BufferedPrint.h"
#include "FreeStack.h"
#include "SdFat.h"
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <Protocentral_FDC1004.h>

// Save SRAM if 328.
#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define Serial MinSerial
#endif  // __AVR_ATmega328P__

//------------------------------------------------------------------------------
// SD Card Configuration
#define SD_FAT_TYPE 3
#define SPI_CLOCK SD_SCK_MHZ(10)

#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif

//------------------------------------------------------------------------------
// RTC Configuration
#define USE_RTC 1
#include "RTClib.h"

//------------------------------------------------------------------------------
// Pin Definitions
const int8_t ERROR_LED_PIN = 16;
const uint8_t SD_CS_PIN = 53;

//------------------------------------------------------------------------------
// ADC Configuration
const uint8_t PIN_LIST[] = {0, 1, 2, 3, 4, 5};
const float SAMPLE_RATE = 5000;  // 5kHz
const float SAMPLE_INTERVAL = 1.0 / SAMPLE_RATE;
#define ROUND_SAMPLE_INTERVAL 1

// Reference voltage
uint8_t const ADC_REF = (1 << REFS0);  // Vcc Reference

// File size: 10 minutes of data
const uint32_t MAX_FILE_SIZE_MiB = 36;  // Adjust based on actual data rate
const uint32_t MAX_FILE_SIZE = MAX_FILE_SIZE_MiB << 20;

#define RECORD_EIGHT_BITS 0

//------------------------------------------------------------------------------
// FDC1004 Configuration
FDC1004 fdc;
File fdcFile;
char fdcFilename[30];
unsigned long lastFdcRead = 0;
unsigned long fdcFileStartTime = 0;
const unsigned long FDC_INTERVAL = 250;  // 250ms = 4Hz
const unsigned long FDC_FILE_DURATION = 30UL * 60UL * 1000UL;  // 30 minutes
bool fdcWorking = false;

//------------------------------------------------------------------------------
// SCD41 Configuration
SCD4x mySensor;
File scdFile;
char scdFilename[30];
unsigned long lastScdRead = 0;
unsigned long scdFileStartTime = 0;
const unsigned long SCD_INTERVAL = 10UL * 60UL * 1000UL;  // 10 minutes
bool scdWorking = false;

//------------------------------------------------------------------------------
// FIFO Configuration
#if RAMEND < 0X8FF
#error SRAM too small
#elif RAMEND < 0X10FF
const size_t FIFO_SIZE_BYTES = 512;
#elif RAMEND < 0X20FF
const size_t FIFO_SIZE_BYTES = 4 * 512;
#elif RAMEND < 0X40FF
const size_t FIFO_SIZE_BYTES = 12 * 512;
#else
const size_t FIFO_SIZE_BYTES = 16 * 512;
#endif

//------------------------------------------------------------------------------
// File System Setup
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
#else
#error Invalid SD_FAT_TYPE
#endif

file_t binFile;
file_t csvFile;

//------------------------------------------------------------------------------
// ADC Data Structures
const uint8_t PIN_COUNT = sizeof(PIN_LIST) / sizeof(PIN_LIST[0]);
const uint16_t MIN_ADC_CYCLES = 15;
const uint16_t ISR_SETUP_ADC = PIN_COUNT > 1 ? 100 : 0;
const uint16_t ISR_TIMER0 = 160;

#if RECORD_EIGHT_BITS
const size_t BLOCK_MAX_COUNT = PIN_COUNT * (DATA_DIM8 / PIN_COUNT);
typedef block8_t block_t;
#else
const size_t BLOCK_MAX_COUNT = PIN_COUNT * (DATA_DIM16 / PIN_COUNT);
typedef block16_t block_t;
#endif

size_t const FIFO_DIM = FIFO_SIZE_BYTES / sizeof(block_t);
block_t* fifoData;
volatile size_t fifoCount = 0;
size_t fifoHead = 0;
size_t fifoTail = 0;

//------------------------------------------------------------------------------
// ISR Variables
volatile bool isrStop = false;
block_t* isrBuf = nullptr;
uint16_t isrOver = 0;
uint8_t adcmux[PIN_COUNT];
uint8_t adcsra[PIN_COUNT];
uint8_t adcsrb[PIN_COUNT];
uint8_t adcindex = 1;
volatile bool timerError = false;
volatile bool timerFlag = false;

//------------------------------------------------------------------------------
// Timing Variables
uint32_t OKtime = millis();

//------------------------------------------------------------------------------
// RTC Setup
#if USE_RTC == 1
RTC_DS1307 rtc;
#elif USE_RTC == 2
RTC_DS3231 rtc;
#elif USE_RTC == 3
RTC_PCF8523 rtc;
#endif

//==============================================================================
// INTERRUPT SERVICE ROUTINES
//==============================================================================

ISR(ADC_vect) {
#if RECORD_EIGHT_BITS
  uint8_t d = ADCH;
#else
  uint16_t d = ADC;
#endif

  if (!isrBuf) {
    if (fifoCount < FIFO_DIM) {
      isrBuf = fifoData + fifoHead;
    } else {
      if (isrOver < 0XFFFF) {
        isrOver++;
      }
      timerFlag = false;
      return;
    }
  }

  if (PIN_COUNT > 1) {
    ADMUX = adcmux[adcindex];
    ADCSRB = adcsrb[adcindex];
    ADCSRA = adcsra[adcindex];
    if (adcindex == 0) {
      timerFlag = false;
    }
    adcindex = adcindex < (PIN_COUNT - 1) ? adcindex + 1 : 0;
  } else {
    timerFlag = false;
  }

  isrBuf->data[isrBuf->count++] = d;

  if (isrBuf->count >= BLOCK_MAX_COUNT) {
    fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
    fifoCount++;
    if (isrStop) {
      adcStop();
      return;
    }
    isrBuf = nullptr;
    isrOver = 0;
  }
}

ISR(TIMER1_COMPB_vect) {
  if (timerFlag) {
    timerError = true;
  }
  timerFlag = true;
}

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

void(* resetFunc) (void) = 0;

#define error(msg) (Serial.println(F(msg)), errorHalt())
#define assert(e) ((e) ? (void)0 : error("assert: " #e))

void fatalBlink() {
  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}

void errorHalt() {
  sd.printSdError(&Serial);
  binFile.close();
  fdcFile.close();
  scdFile.close();
  resetFunc();
}

void printTime(DateTime timeIN) {
  Serial.print(F("RTC: "));
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

void printUnusedStack() {
  Serial.print(F("Unused stack: "));
  Serial.println(UnusedStack());
}

void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}

void blinkOK() {
  // Status blink - can be enabled if needed
}

//------------------------------------------------------------------------------
// RTC Functions
void stopRTC() {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 1);
  uint8_t seconds = Wire.read();
  seconds |= 0x80;

  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.write(seconds);
  Wire.endTransmission();

  Serial.println(F("RTC stopped"));
}

void startRTC() {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 1);
  uint8_t seconds = Wire.read();
  seconds &= 0x7F;

  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.write(seconds);
  Wire.endTransmission();

  Serial.println(F("RTC started"));
}

#if USE_RTC
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();
  *date = FS_DATE(now.year(), now.month(), now.day());
  *time = FS_TIME(now.hour(), now.minute(), now.second());
  *ms10 = now.second() & 1 ? 100 : 0;
}
#endif

//==============================================================================
// SENSOR INITIALIZATION
//==============================================================================

void setupSCD() {
  Wire.begin();
  if (mySensor.begin() == false) {
    Serial.println(F("SCD41 not detected"));
    scdWorking = false;
    
    // Try low power mode
    mySensor.stopPeriodicMeasurement();
    delay(500);
    if (mySensor.startLowPowerPeriodicMeasurement()) {
      Serial.println(F("SCD41 low power mode enabled"));
      scdWorking = true;
    }
  } else {
    Serial.println(F("SCD41 initialized"));
    scdWorking = true;
  }
}

void setupFDC() {
  Wire.begin();
  
  // Try to initialize FDC1004
  fdc.begin();
  
  // Configure measurement on channel 1
  fdc.configureMeasurementSingle(MEAS_1, CH_1, CH_CAPDAC);
  fdc.triggerSingleMeasurement(MEAS_1, FDC1004_100HZ);
  
  delay(100);
  
  // Test read to verify sensor is working
  uint16_t value[2];
  if (fdc.readMeasurement(MEAS_1, value) == 0) {
    Serial.println(F("FDC1004 initialized"));
    fdcWorking = true;
  } else {
    Serial.println(F("FDC1004 not detected"));
    fdcWorking = false;
  }
}

//==============================================================================
// FDC1004 DATA LOGGING
//==============================================================================

void createFdcFile() {
  if (!fdcWorking) return;
  
  fdcFile.close();
  
#if USE_RTC
  DateTime now = rtc.now();
#else
  DateTime now = DateTime(2025, 1, 1, 0, 0, 0);
#endif
  
  snprintf(fdcFilename, sizeof(fdcFilename), "FDC_%04d%02d%02d_%02d%02d%02d.csv",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  
  Serial.print(F("Creating FDC file: "));
  Serial.println(fdcFilename);
  
  fdcFile = sd.open(fdcFilename, FILE_WRITE);
  if (fdcFile) {
    fdcFile.println("Timestamp,Capacitance_pF");
    fdcFile.close();
    fdcFileStartTime = millis();
  } else {
    Serial.println(F("Failed to create FDC file"));
  }
}

void readAndLogFDC() {
  if (!fdcWorking) return;
  
  uint16_t value[2];
  
  if (fdc.readMeasurement(MEAS_1, value) == 0) {
    // Calculate capacitance in pF
    int16_t msb = (int16_t)value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb);
    capacitance /= 1000;
    capacitance += ((int32_t)3028) * ((int32_t)value[1]);
    capacitance /= 1000;
    
    fdcFile = sd.open(fdcFilename, FILE_WRITE);
    if (fdcFile) {
#if USE_RTC
      DateTime now = rtc.now();
      fdcFile.print(now.year());
      fdcFile.print('/');
      fdcFile.print(now.month());
      fdcFile.print('/');
      fdcFile.print(now.day());
      fdcFile.print(' ');
      fdcFile.print(now.hour());
      fdcFile.print(':');
      fdcFile.print(now.minute());
      fdcFile.print(':');
      fdcFile.print(now.second());
#else
      fdcFile.print(millis());
#endif
      fdcFile.print(",");
      fdcFile.println(capacitance);
      fdcFile.close();
    }
    
    // Trigger next measurement
    fdc.triggerSingleMeasurement(MEAS_1, FDC1004_100HZ);
  }
}

//==============================================================================
// SCD41 DATA LOGGING
//==============================================================================

void createScdFile() {
  if (!scdWorking) return;
  
  scdFile.close();
  
#if USE_RTC
  DateTime now = rtc.now();
#else
  DateTime now = DateTime(2025, 1, 1, 0, 0, 0);
#endif
  
  snprintf(scdFilename, sizeof(scdFilename), "SCD_%04d%02d%02d_%02d%02d%02d.csv",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  
  Serial.print(F("Creating SCD file: "));
  Serial.println(scdFilename);
  
  scdFile = sd.open(scdFilename, FILE_WRITE);
  if (scdFile) {
    scdFile.println("Timestamp,CO2_ppm,Humidity_RH,Temperature_C");
    scdFile.close();
    scdFileStartTime = millis();
  } else {
    Serial.println(F("Failed to create SCD file"));
  }
}

void readAndLogSCD() {
  if (!scdWorking) return;
  
  if (mySensor.readMeasurement()) {
    uint16_t co2 = mySensor.getCO2();
    float temp = mySensor.getTemperature();
    float hum = mySensor.getHumidity();
    
    Serial.print(F("CO2: "));
    Serial.print(co2);
    Serial.print(F(" ppm, RH: "));
    Serial.print(hum, 1);
    Serial.print(F(" %, Temp: "));
    Serial.print(temp, 1);
    Serial.println(F(" C"));
    
    scdFile = sd.open(scdFilename, FILE_WRITE);
    if (scdFile) {
#if USE_RTC
      DateTime now = rtc.now();
      scdFile.print(now.year());
      scdFile.print('/');
      scdFile.print(now.month());
      scdFile.print('/');
      scdFile.print(now.day());
      scdFile.print(' ');
      scdFile.print(now.hour());
      scdFile.print(':');
      scdFile.print(now.minute());
      scdFile.print(':');
      scdFile.print(now.second());
#else
      scdFile.print(millis());
#endif
      scdFile.print(",");
      scdFile.print(co2);
      scdFile.print(",");
      scdFile.print(hum, 1);
      scdFile.print(",");
      scdFile.println(temp, 1);
      scdFile.close();
    }
  }
}

//==============================================================================
// ADC FUNCTIONS
//==============================================================================

inline bool adcActive() { 
  return (1 << ADIE) & ADCSRA; 
}

void adcInit(metadata_t* meta) {
  uint8_t adps;
  uint32_t ticks = F_CPU * SAMPLE_INTERVAL + 0.5;

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) {
    error("Invalid ADC reference");
  }

#ifdef ADC_PRESCALER
  adps = ADC_PRESCALER;
#else
  int32_t adcCycles = (ticks - ISR_TIMER0) / PIN_COUNT - ISR_SETUP_ADC;
  for (adps = 7; adps > 0; adps--) {
    if (adcCycles >= (MIN_ADC_CYCLES << adps)) {
      break;
    }
  }
#endif

  meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000)) {
    error("Sample Rate Too High");
  }

#if ROUND_SAMPLE_INTERVAL
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif

  if (PIN_COUNT > BLOCK_MAX_COUNT || PIN_COUNT > PIN_NUM_DIM) {
    error("Too many pins");
  }

  meta->pinCount = PIN_COUNT;
  meta->recordEightBits = RECORD_EIGHT_BITS;

  for (int i = 0; i < PIN_COUNT; i++) {
    uint8_t pin = PIN_LIST[i];
    if (pin >= NUM_ANALOG_INPUTS) {
      error("Invalid Analog pin number");
    }
    meta->pinNumber[i] = pin;

    adcmux[i] = (pin & 7) | ADC_REF;
    if (RECORD_EIGHT_BITS) {
      adcmux[i] |= 1 << ADLAR;
    }

    adcsrb[i] = i == 0 ? (1 << ADTS2) | (1 << ADTS0) : 0;
#ifdef MUX5
    if (pin > 7) {
      adcsrb[i] |= (1 << MUX5);
    }
#endif
    adcsra[i] = (1 << ADEN) | (1 << ADIE) | adps;
    adcsra[i] |= i == 0 ? 1 << ADATE : 1 << ADSC;
  }

  // Setup timer1
  TCCR1A = 0;
  uint8_t tshift;
  if (ticks < 0X10000) {
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    tshift = 0;
  } else if (ticks < 0X10000 * 8) {
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    tshift = 3;
  } else if (ticks < 0X10000 * 64) {
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    tshift = 6;
  } else if (ticks < 0X10000 * 256) {
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
    tshift = 8;
  } else if (ticks < 0X10000 * 1024) {
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
    tshift = 10;
  } else {
    error("Sample Rate Too Slow");
  }

  ticks >>= tshift;
  ICR1 = ticks - 1;
  OCR1B = 0;
  ticks <<= tshift;

  meta->sampleInterval = ticks;
  meta->cpuFrequency = F_CPU;
  
  float sampleRate = (float)meta->cpuFrequency / meta->sampleInterval;
  Serial.print(F("Sample pins:"));
  for (uint8_t i = 0; i < meta->pinCount; i++) {
    Serial.print(' ');
    Serial.print(meta->pinNumber[i], DEC);
  }
  Serial.println();
  Serial.print(F("ADC bits: "));
  Serial.println(meta->recordEightBits ? 8 : 10);
  Serial.print(F("ADC clock kHz: "));
  Serial.println(meta->adcFrequency / 1000);
  Serial.print(F("Sample Rate: "));
  Serial.println(sampleRate);
  Serial.print(F("Sample interval usec: "));
  Serial.println(1000000.0 / sampleRate);
}

void adcStart() {
  adcindex = 1;
  isrBuf = nullptr;
  isrOver = 0;
  isrStop = false;

  ADCSRA |= 1 << ADIF;

  ADMUX = adcmux[0];
  ADCSRB = adcsrb[0];
  ADCSRA = adcsra[0];

  timerError = false;
  timerFlag = false;
  TCNT1 = 0;
  TIFR1 = 1 << OCF1B;
  TIMSK1 = 1 << OCIE1B;
}

inline void adcStop() {
  TIMSK1 = 0;
  ADCSRA = 0;
}

//==============================================================================
// BINARY FILE OPERATIONS
//==============================================================================

void createBinFile() {
  binFile.close();
  
#if USE_RTC
  DateTime now = rtc.now();
#else
  DateTime now = DateTime(2025, 1, 1, 0, 0, 0);
#endif
  
  char binName[30];
  snprintf(binName, sizeof(binName), "ADC_%04d%02d%02d_%02d%02d%02d.bin",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  
  Serial.print(F("Creating ADC file: "));
  Serial.println(binName);
  
  if (!binFile.open(binName, O_RDWR | O_CREAT)) {
    error("open binName failed");
  }
  
  Serial.print(F("Allocating: "));
  Serial.print(MAX_FILE_SIZE_MiB);
  Serial.println(F(" MiB"));
  
  if (!binFile.preAllocate(MAX_FILE_SIZE)) {
    printUnusedStack();
    error("preAllocate failed");
  }
}

//==============================================================================
// MAIN DATA LOGGING FUNCTION
//==============================================================================

void logData() {
  uint32_t t0;
  uint32_t t1;
  uint32_t overruns = 0;
  uint32_t count = 0;
  uint32_t maxLatencyUsec = 0;
  size_t maxFifoUse = 0;
  block_t fifoBuffer[FIFO_DIM];

  // Create all data files
  createBinFile();
  createFdcFile();
  createScdFile();

  adcInit((metadata_t*)fifoBuffer);
  
  if (sizeof(metadata_t) != binFile.write(fifoBuffer, sizeof(metadata_t))) {
    error("Write metadata failed");
  }

  fifoCount = 0;
  fifoHead = 0;
  fifoTail = 0;
  fifoData = fifoBuffer;
  memset(fifoBuffer, 0, sizeof(fifoBuffer));

  Serial.println(F("Logging - type 's' to stop"));
  Serial.flush();
  delay(50);

  t0 = millis();
  t1 = t0;
  lastFdcRead = t0;
  lastScdRead = t0;
  
  adcStart();
  
  while (1) {
    // Status blink every 10 seconds
    if (millis() - OKtime > 10000) {
      blinkOK();
      OKtime = millis();
    }

    // Poll FDC1004 at 4Hz (every 250ms)
    if (fdcWorking && (millis() - lastFdcRead >= FDC_INTERVAL)) {
      readAndLogFDC();
      lastFdcRead = millis();
      
      // Check if need new FDC file (30 minutes)
      if (millis() - fdcFileStartTime >= FDC_FILE_DURATION) {
        createFdcFile();
      }
    }

    // Read SCD41 every 10 minutes
    if (scdWorking && (millis() - lastScdRead >= SCD_INTERVAL)) {
      readAndLogSCD();
      lastScdRead = millis();
      createScdFile();  // New file every 10 minutes
    }

    // Process ADC FIFO
    uint32_t m;
    noInterrupts();
    size_t tmpFifoCount = fifoCount;
    interrupts();
    
    if (tmpFifoCount) {
      block_t* pBlock = fifoData + fifoTail;
      
      m = micros();
      if (sizeof(block_t) != binFile.write(pBlock, sizeof(block_t))) {
        error("write data failed");
      }
      m = micros() - m;
      t1 = millis();
      
      if (m > maxLatencyUsec) {
        maxLatencyUsec = m;
      }
      if (tmpFifoCount > maxFifoUse) {
        maxFifoUse = tmpFifoCount;
      }
      count += pBlock->count;

      if (pBlock->overrun) {
        overruns += pBlock->overrun;
        if (ERROR_LED_PIN >= 0) {
          digitalWrite(ERROR_LED_PIN, HIGH);
        }
      }

      pBlock->count = 0;
      pBlock->overrun = 0;
      fifoTail = fifoTail < (FIFO_DIM - 1) ? fifoTail + 1 : 0;

      noInterrupts();
      fifoCount--;
      interrupts();

      if (binFile.curPosition() >= MAX_FILE_SIZE) {
        adcStop();
        break;
      }
    }

    if (timerError) {
      error("Missed timer event - rate too high");
    }

    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        isrStop = true;
      }
    }

    if (digitalRead(15) == HIGH) {
      isrStop = true;
    }

    if (fifoCount == 0 && !adcActive()) {
      break;
    }
  }

  Serial.println();
  
  if (binFile.curPosition() < MAX_FILE_SIZE) {
    Serial.println(F("Truncating ADC file"));
    Serial.flush();
    if (!binFile.truncate()) {
      error("Can't truncate file");
    }
  }

  binFile.close();
  fdcFile.close();
  scdFile.close();

  Serial.print(F("Max write latency usec: "));
  Serial.println(maxLatencyUsec);
  Serial.print(F("Record time sec: "));
  Serial.println(0.001 * (t1 - t0), 3);
  Serial.print(F("Sample count: "));
  Serial.println(count / PIN_COUNT);
  Serial.print(F("Overruns: "));
  Serial.println(overruns);
  Serial.print(F("FIFO_DIM: "));
  Serial.println(FIFO_DIM);
  Serial.print(F("maxFifoUse: "));
  Serial.println(maxFifoUse + 1);
  Serial.println(F("Done"));
}

//==============================================================================
// CSV CONVERSION
//==============================================================================

bool createCsvFile() {
  char csvName[30];

  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return false;
  }
  
  binFile.getName(csvName, sizeof(csvName));
  char* dot = strchr(csvName, '.');
  if (!dot) {
    error("no dot in binName");
  }
  strcpy(dot + 1, "csv");
  
  if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  
  Serial.print(F("Writing: "));
  Serial.print(csvName);
  Serial.println(F(" - type any character to stop"));
  return true;
}

void binaryToCsv() {
  uint8_t lastPct = 0;
  block_t* pd;
  metadata_t* pm;
  uint32_t t0 = millis();
  
  BufferedPrint<file_t, 64> bp(&csvFile);
  block_t binBuffer[FIFO_DIM];

  assert(sizeof(block_t) == sizeof(metadata_t));
  binFile.rewind();
  uint32_t tPct = millis();
  bool doMeta = true;
  
  while (!Serial.available()) {
    pd = binBuffer;
    int nb = binFile.read(binBuffer, sizeof(binBuffer));
    if (nb < 0) {
      error("read binFile failed");
    }
    size_t nd = nb / sizeof(block_t);
    if (nd < 1) {
      break;
    }
    
    if (doMeta) {
      doMeta = false;
      pm = (metadata_t*)pd++;
      if (PIN_COUNT != pm->pinCount) {
        error("Invalid pinCount");
      }
      bp.print(F("Interval,"));
      float intervalMicros = 1.0e6 * pm->sampleInterval / (float)pm->cpuFrequency;
      bp.print(intervalMicros, 4);
      bp.println(F(",usec"));
      for (uint8_t i = 0; i < PIN_COUNT; i++) {
        if (i) {
          bp.print(',');
        }
        bp.print(F("pin"));
        bp.print(pm->pinNumber[i]);
      }
      bp.println();
      if (nd-- == 1) {
        break;
      }
    }
    
    for (size_t i = 0; i < nd; i++, pd++) {
      if (pd->overrun) {
        bp.print(F("OVERRUN,"));
        bp.println(pd->overrun);
      }
      for (size_t j = 0; j < pd->count; j += PIN_COUNT) {
        for (size_t i = 0; i < PIN_COUNT; i++) {
          if (!bp.printField(pd->data[i + j], i == (PIN_COUNT - 1) ? '\n' : ',')) {
            error("printField failed");
          }
        }
      }
    }
    
    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition() / (binFile.fileSize() / 100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
      }
    }
  }
  
  if (!bp.sync() || !csvFile.close()) {
    error("close csvFile failed");
  }
  
  Serial.print(F("Done: "));
  Serial.print(0.001 * (millis() - t0));
  Serial.println(F(" Seconds"));
}

void openBinFile() {
  char name[30];
  clearSerialInput();
  Serial.println(F("Enter file name"));
  if (!serialReadLine(name, sizeof(name))) {
    return;
  }
  if (!sd.exists(name)) {
    Serial.println(name);
    Serial.println(F("File does not exist"));
    return;
  }
  binFile.close();
  if (!binFile.open(name, O_RDWR)) {
    Serial.println(name);
    Serial.println(F("open failed"));
    return;
  }
  Serial.println(F("File opened"));
}

void printData() {
  block_t buf;
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  if (binFile.read(&buf, sizeof(buf)) != sizeof(buf)) {
    error("Read metadata failed");
  }
  Serial.println(F("Type any character to stop"));
  delay(1000);
  while (!Serial.available() && binFile.read(&buf, sizeof(buf)) == sizeof(buf)) {
    if (buf.count == 0) {
      break;
    }
    if (buf.overrun) {
      Serial.print(F("OVERRUN,"));
      Serial.println(buf.overrun);
    }
    for (size_t i = 0; i < buf.count; i++) {
      Serial.print(buf.data[i], DEC);
      if ((i + 1) % PIN_COUNT) {
        Serial.print(',');
      } else {
        Serial.println();
      }
    }
  }
  Serial.println(F("Done"));
}

bool serialReadLine(char* str, size_t size) {
  size_t n = 0;
  while (!Serial.available()) {
  }
  while (true) {
    int c = Serial.read();
    if (c < ' ') break;
    str[n++] = c;
    if (n >= size) {
      Serial.println(F("input too long"));
      return false;
    }
    uint32_t m = millis();
    while (!Serial.available() && (millis() - m) < 100) {
    }
    if (!Serial.available()) break;
  }
  str[n] = 0;
  return true;
}

//==============================================================================
// SETUP
//==============================================================================

void setup(void) {
  // Initialize pins
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.setTimeout(500);
  
  digitalWrite(17, HIGH);
  
  // Initialize sensors
  setupSCD();
  setupFDC();
  
  // Debug mode indication
  if (digitalRead(14) == LOW) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(17, HIGH);
      delay(200);
      digitalWrite(17, LOW);
      delay(200);
    }
    digitalWrite(17, HIGH);
  }
  
  // Debug mode - print raw ADC values
  while (digitalRead(14) == LOW) {
    Serial.print(analogRead(A0));
    Serial.print(' ');
    Serial.print(analogRead(A1));
    Serial.print(' ');
    Serial.print(analogRead(A2));
    Serial.print(' ');
    Serial.print(analogRead(A3));
    Serial.print(' ');
    Serial.print(analogRead(A4));
    Serial.print(' ');
    Serial.println(analogRead(A5));
  }
  
  digitalWrite(17, HIGH);
  
  FillStack();
  analogRead(PIN_LIST[0]);

#if !ENABLE_DEDICATED_SPI
  Serial.println(F("\nFor best performance edit SdFatConfig.h"));
  Serial.println(F("and set ENABLE_DEDICATED_SPI nonzero"));
#endif

  // Initialize SD card
  if (!sd.begin(SD_CONFIG)) {
    Serial.println(F("sd.begin failed"));
    error("sd.begin failed");
  }

#if USE_RTC
  if (!rtc.begin()) {
    Serial.println(F("RTC begin failed"));
  }
  if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running"));
  } else {
    Serial.println(F("RTC is running"));
    printTime(rtc.now());
  }
  FsDateTime::setCallback(dateTime);
#endif

  Serial.println(F("\n=== Multi-Sensor Data Logger ==="));
  Serial.println(F("ADC: 6 channels @ 5kHz (10 min files)"));
  Serial.println(F("FDC1004: 4Hz (30 min files)"));
  Serial.println(F("SCD41: Every 10 minutes"));
  Serial.println();
}

//==============================================================================
// MAIN LOOP
//==============================================================================

void loop(void) {
  printUnusedStack();
  clearSerialInput();
  digitalWrite(17, HIGH);

  Serial.println();
  Serial.println(F("=== BeeSpy Multi-Sensor Logger ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  r - Record data from all sensors"));
  Serial.println(F("  b - Open existing bin file"));
  Serial.println(F("  c - Convert bin file to CSV"));
  Serial.println(F("  l - List files on SD card"));
  Serial.println(F("  p - Print data to Serial"));
  Serial.println(F("  t - Set RTC time to compile time"));
  Serial.println(F("  s - Show sensor status"));
  Serial.println(F("  x - Reset Arduino"));
  Serial.println();

  char c = 'l';

  // Auto-start recording if switch on pin 15 is active
  if (digitalRead(15) == LOW) {
    c = 'r';
    Serial.println(F("Auto-starting recording..."));
    digitalWrite(17, LOW);
  } else {
    digitalWrite(17, HIGH);

    // Wait for command
    while (!Serial.available()) {
      if (digitalRead(14) == LOW || digitalRead(15) == LOW) {
        resetFunc();
      }
    }
    
    c = tolower(Serial.read());
    Serial.println();
    
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, LOW);
    }
    clearSerialInput();
  }

  // Process command
  switch (c) {
    case 'r':
      logData();
      break;
      
    case 'b':
      openBinFile();
      break;
      
    case 'c':
      if (createCsvFile()) {
        binaryToCsv();
      }
      break;
      
    case 'l':
      Serial.println(F("Files on SD card:"));
      sd.ls(&Serial, LS_DATE | LS_SIZE);
      break;
      
    case 'p':
      printData();
      break;
      
    case 's':
      Serial.println(F("\n=== Sensor Status ==="));
      Serial.print(F("SCD41: "));
      Serial.println(scdWorking ? F("OK") : F("NOT DETECTED"));
      Serial.print(F("FDC1004: "));
      Serial.println(fdcWorking ? F("OK") : F("NOT DETECTED"));
#if USE_RTC
      Serial.print(F("RTC: "));
      if (rtc.isrunning()) {
        Serial.print(F("OK - "));
        printTime(rtc.now());
      } else {
        Serial.println(F("NOT RUNNING"));
      }
#endif
      break;
      
    case 't':
#if USE_RTC
      stopRTC();
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      startRTC();
      Serial.println(F("RTC time set to compile time:"));
      printTime(rtc.now());
#else
      Serial.println(F("RTC not enabled"));
#endif
      break;
      
    case 'x':
      Serial.println(F("Resetting..."));
      delay(100);
      resetFunc();
      break;
      
    default:
      Serial.println(F("Invalid command"));
      break;
  }
  
  digitalWrite(17, HIGH);
}

#else  // __AVR__
#error This program is only for AVR.
#endif  // __AVR__