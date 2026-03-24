/**
 * BeeSpy3 - Combined version H
 * ADC data logger with SCD41 CO2/temp/humidity sensor.
 *
 * If no SD card is detected at startup, the device will still allow
 * live analogue viewing (pin 14 LOW) but will block all recording/file
 * operations and flash the error LED to indicate the problem.
 */
#ifdef __AVR__
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"
#include "AvrAdcLogger.h"
#include "BufferedPrint.h"
#include "FreeStack.h"
#include "SdFat.h"

// Save SRAM if 328.
#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define Serial MinSerial
#endif  // __AVR_ATmega328P__

//------------------------------------------------------------------------------
// VARIABLES THAT CAN BE CHANGED
//------------------------------------------------------------------------------
// Sample rate in samples per second.
const float SAMPLE_RATE = 5000;  // Must be 0.25 or greater.
// Maximum file size in bytes.
const uint32_t MAX_FILE_SIZE_MiB = 18;  // 36MB is ~10 min at 5kHz/6ch
// Unique value per physical device - stored in EEPROM, used in filenames
char sensorID[9];
//------------------------------------------------------------------------------

uint32_t compileHash() {
    const char* d = __DATE__;
    const char* t = __TIME__;
    uint32_t h = 2166136261UL;
    for (int i = 0; d[i]; i++) h = (h ^ d[i]) * 16777619UL;
    for (int i = 0; t[i]; i++) h = (h ^ t[i]) * 16777619UL;
    return h;
}

void loadOrCreateSensorID() {
    uint32_t h = compileHash();
    snprintf(sensorID, sizeof(sensorID), "%08lX", h);
}

//------------------------------------------------------------------------------
// SD_FAT_TYPE = 3 supports FAT16/FAT32 and exFAT (not for Uno)
#define SD_FAT_TYPE 3

// USE_RTC: 0=none, 1=DS1307, 2=DS3231, 3=PCF8523
#define USE_RTC 1

//------------------------------------------------------------------------------
// Pin definitions
const int8_t  ERROR_LED_PIN = 16;   // Blinks on fatal error; solid on overrun
const uint8_t SD_CS_PIN     = 53;   // SD chip select

//------------------------------------------------------------------------------
// Analog pins to sample - may be in any order, may repeat
const uint8_t PIN_LIST[] = {0, 1, 2, 3, 4, 5};

const float SAMPLE_INTERVAL = 1.0 / SAMPLE_RATE;

#define ROUND_SAMPLE_INTERVAL 1

//------------------------------------------------------------------------------
// SCD41 CO2/temp/humidity sensor
#include "SparkFun_SCD4x_Arduino_Library.h"
SCD4x mySensor;
bool  SENSORWORKING  = false;
bool  sdAvailable    = false;   // Set in setup() after sd.begin()
File  logFile;
char  sensorFilename[30];

//------------------------------------------------------------------------------
// ADC reference
uint8_t const ADC_REF = (1 << REFS0);  // Vcc reference

// Log file name template (only used for legacy open-by-name)
#define LOG_FILE_NAME "AvrAdc00.bin"

const size_t NAME_DIM = 40;

#define RECORD_EIGHT_BITS 0

//------------------------------------------------------------------------------
// FIFO sizing
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
// ADC prescaler (auto-calculated unless overridden)
// #define ADC_PRESCALER 7  // F_CPU/128 = 125 kHz on Uno

//------------------------------------------------------------------------------
#define TMP_FILE_NAME "tmp_adc.bin"

const uint8_t  PIN_COUNT      = sizeof(PIN_LIST) / sizeof(PIN_LIST[0]);
const uint16_t MIN_ADC_CYCLES = 15;
const uint16_t ISR_SETUP_ADC  = PIN_COUNT > 1 ? 100 : 0;
const uint16_t ISR_TIMER0     = 160;

const uint32_t MAX_FILE_SIZE = MAX_FILE_SIZE_MiB << 20;

#define SPI_CLOCK SD_SCK_MHZ(10)

#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif

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

unsigned long testms = 0;

#if RECORD_EIGHT_BITS
const size_t BLOCK_MAX_COUNT = PIN_COUNT * (DATA_DIM8  / PIN_COUNT);
typedef block8_t  block_t;
#else
const size_t BLOCK_MAX_COUNT = PIN_COUNT * (DATA_DIM16 / PIN_COUNT);
typedef block16_t block_t;
#endif

const size_t     FIFO_DIM   = FIFO_SIZE_BYTES / sizeof(block_t);
block_t*         fifoData;
volatile size_t  fifoCount  = 0;
size_t           fifoHead   = 0;
size_t           fifoTail   = 0;

//==============================================================================
// ISR state
volatile bool isrStop  = false;
block_t*      isrBuf   = nullptr;
uint16_t      isrOver  = 0;

uint8_t adcmux[PIN_COUNT];
uint8_t adcsra[PIN_COUNT];
uint8_t adcsrb[PIN_COUNT];
uint8_t adcindex = 1;

volatile bool timerError = false;
volatile bool timerFlag  = false;

//------------------------------------------------------------------------------
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
      if (isrOver < 0XFFFF) isrOver++;
      timerFlag = false;
      return;
    }
  }
  if (PIN_COUNT > 1) {
    ADMUX  = adcmux[adcindex];
    ADCSRB = adcsrb[adcindex];
    ADCSRA = adcsra[adcindex];
    if (adcindex == 0) timerFlag = false;
    adcindex = adcindex < (PIN_COUNT - 1) ? adcindex + 1 : 0;
  } else {
    timerFlag = false;
  }
  isrBuf->data[isrBuf->count++] = d;
  if (isrBuf->count >= BLOCK_MAX_COUNT) {
    fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
    fifoCount++;
    if (isrStop) { adcStop(); return; }
    isrBuf  = nullptr;
    isrOver = 0;
  }
}

ISR(TIMER1_COMPB_vect) {
  if (timerFlag) timerError = true;
  timerFlag = true;
}

void(* resetFunc)(void) = 0;

//==============================================================================
// Error macros
#define error(msg) (Serial.println(F(msg)), errorHalt())
#define assert(e)  ((e) ? (void)0 : error("assert: " #e))

//------------------------------------------------------------------------------
void fatalBlink() {
  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH); delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);  delay(200);
    }
  }
}

uint32_t OKtime = millis();
void blinkOK() { /* intentionally empty - was causing ADC noise */ }

void errorHalt() {
  sd.printSdError(&Serial);
  binFile.close();
  resetFunc();
}

//------------------------------------------------------------------------------
// Flash the error LED to signal no SD card (called on boot and on blocked ops)
void flashNoSD() {
  if (ERROR_LED_PIN < 0) return;
  for (int i = 0; i < 5; i++) {
    digitalWrite(ERROR_LED_PIN, HIGH); delay(100);
    digitalWrite(ERROR_LED_PIN, LOW);  delay(100);
  }
}

// Guard function: prints an error and flashes LED if SD is absent.
// Returns true if SD is available, false otherwise.
bool requireSD(const __FlashStringHelper* action) {
  if (sdAvailable) return true;
  Serial.print(F("ERROR: Cannot "));
  Serial.print(action);
  Serial.println(F(" - no SD card detected"));
  flashNoSD();
  return false;
}

//------------------------------------------------------------------------------
void printTime(DateTime timeIN) {
  Serial.print(F("The RTC time is "));
  Serial.print(timeIN.year(),   DEC); Serial.print('/');
  Serial.print(timeIN.month(),  DEC); Serial.print('/');
  Serial.print(timeIN.day(),    DEC); Serial.print(' ');
  Serial.print(timeIN.hour(),   DEC); Serial.print(':');
  Serial.print(timeIN.minute(), DEC); Serial.print(':');
  Serial.print(timeIN.second(), DEC); Serial.println();
}

void printUnusedStack() {
  Serial.print(F("\nUnused stack: "));
  Serial.println(UnusedStack());
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
#if USE_RTC
#if USE_RTC == 1
RTC_DS1307 rtc;
#elif USE_RTC == 2
RTC_DS3231 rtc;
#elif USE_RTC == 3
RTC_PCF8523 rtc;
#else
#error USE_RTC type not implemented.
#endif

void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();
  *date  = FS_DATE(now.year(), now.month(), now.day());
  *time  = FS_TIME(now.hour(), now.minute(), now.second());
  *ms10  = now.second() & 1 ? 100 : 0;
}
#endif  // USE_RTC

//==============================================================================
#if ADPS0 != 0 || ADPS1 != 1 || ADPS2 != 2
#error unexpected ADC prescaler bits
#endif

inline bool adcActive() { return (1 << ADIE) & ADCSRA; }

//------------------------------------------------------------------------------
void adcInit(metadata_t* meta) {
  uint8_t  adps;
  uint32_t ticks = F_CPU * SAMPLE_INTERVAL + 0.5;

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) error("Invalid ADC reference");

#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) error("Invalid ADC prescaler");
  adps = ADC_PRESCALER;
#else
  int32_t adcCycles = (ticks - ISR_TIMER0) / PIN_COUNT - ISR_SETUP_ADC;
  for (adps = 7; adps > 0; adps--) {
    if (adcCycles >= (MIN_ADC_CYCLES << adps)) break;
  }
#endif

  meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000))
    error("Sample Rate Too High");

#if ROUND_SAMPLE_INTERVAL
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif

  if (PIN_COUNT > BLOCK_MAX_COUNT || PIN_COUNT > PIN_NUM_DIM)
    error("Too many pins");

  meta->pinCount        = PIN_COUNT;
  meta->recordEightBits = RECORD_EIGHT_BITS;

  for (int i = 0; i < PIN_COUNT; i++) {
    uint8_t pin = PIN_LIST[i];
    if (pin >= NUM_ANALOG_INPUTS) error("Invalid Analog pin number");
    meta->pinNumber[i] = pin;
    adcmux[i] = (pin & 7) | ADC_REF;
    if (RECORD_EIGHT_BITS) adcmux[i] |= 1 << ADLAR;
    adcsrb[i] = i == 0 ? (1 << ADTS2) | (1 << ADTS0) : 0;
#ifdef MUX5
    if (pin > 7) adcsrb[i] |= (1 << MUX5);
#endif
    adcsra[i]  = (1 << ADEN) | (1 << ADIE) | adps;
    adcsra[i] |= i == 0 ? 1 << ADATE : 1 << ADSC;
  }

  TCCR1A = 0;
  uint8_t tshift;
  if      (ticks < 0X10000)        { TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS10);            tshift=0;  }
  else if (ticks < 0X10000*8)      { TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);            tshift=3;  }
  else if (ticks < 0X10000*64)     { TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); tshift=6;  }
  else if (ticks < 0X10000*256)    { TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12);            tshift=8;  }
  else if (ticks < 0X10000*1024)   { TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12)|(1<<CS10); tshift=10; }
  else error("Sample Rate Too Slow");

  ticks >>= tshift;
  ICR1   = ticks - 1;
  OCR1B  = 0;
  ticks <<= tshift;

  meta->sampleInterval = ticks;
  meta->cpuFrequency   = F_CPU;

  float sampleRate = (float)meta->cpuFrequency / meta->sampleInterval;
  Serial.print(F("Sample pins:"));
  for (uint8_t i = 0; i < meta->pinCount; i++) { Serial.print(' '); Serial.print(meta->pinNumber[i], DEC); }
  Serial.println();
  Serial.print(F("ADC bits: "));          Serial.println(meta->recordEightBits ? 8 : 10);
  Serial.print(F("ADC clock kHz: "));     Serial.println(meta->adcFrequency / 1000);
  Serial.print(F("Sample Rate: "));       Serial.println(sampleRate);
  Serial.print(F("Sample interval usec: ")); Serial.println(1000000.0 / sampleRate);
}

//------------------------------------------------------------------------------
void adcStart() {
  adcindex = 1;
  isrBuf   = nullptr;
  isrOver  = 0;
  isrStop  = false;
  ADCSRA  |= 1 << ADIF;
  ADMUX    = adcmux[0];
  ADCSRB   = adcsrb[0];
  ADCSRA   = adcsra[0];
  timerError = false;
  timerFlag  = false;
  TCNT1  = 0;
  TIFR1  = 1 << OCF1B;
  TIMSK1 = 1 << OCIE1B;
}

inline void adcStop() {
  TIMSK1 = 0;
  ADCSRA = 0;
}

//------------------------------------------------------------------------------
void binaryToCsv() {
  uint8_t    lastPct = 0;
  block_t*   pd;
  metadata_t* pm;
  uint32_t   t0 = millis();
  BufferedPrint<file_t, 64> bp(&csvFile);
  block_t binBuffer[FIFO_DIM];

  assert(sizeof(block_t) == sizeof(metadata_t));
  binFile.rewind();
  uint32_t tPct  = millis();
  bool     doMeta = true;

  while (!Serial.available()) {
    pd = binBuffer;
    int nb = binFile.read(binBuffer, sizeof(binBuffer));
    if (nb < 0) error("read binFile failed");
    size_t nd = nb / sizeof(block_t);
    if (nd < 1) break;

    if (doMeta) {
      doMeta = false;
      pm = (metadata_t*)pd++;
      if (PIN_COUNT != pm->pinCount) error("Invalid pinCount");
      bp.print(F("Interval,"));
      float intervalMicros = 1.0e6 * pm->sampleInterval / (float)pm->cpuFrequency;
      bp.print(intervalMicros, 4);
      bp.println(F(",usec"));
      for (uint8_t i = 0; i < PIN_COUNT; i++) {
        if (i) bp.print(',');
        bp.print(F("pin"));
        bp.print(pm->pinNumber[i]);
      }
      bp.println();
      if (nd-- == 1) break;
    }
    for (size_t i = 0; i < nd; i++, pd++) {
      if (pd->overrun) { bp.print(F("OVERRUN,")); bp.println(pd->overrun); }
      for (size_t j = 0; j < pd->count; j += PIN_COUNT) {
        for (size_t k = 0; k < PIN_COUNT; k++) {
          if (!bp.printField(pd->data[k + j], k == (PIN_COUNT - 1) ? '\n' : ','))
            error("printField failed");
        }
      }
    }
    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition() / (binFile.fileSize() / 100);
      if (pct != lastPct) { tPct = millis(); lastPct = pct; Serial.print(pct, DEC); Serial.println('%'); }
    }
  }
  if (!bp.sync() || !csvFile.close()) error("close csvFile failed");
  Serial.print(F("Done: "));
  Serial.print(0.001 * (millis() - t0));
  Serial.println(F(" Seconds"));
}

//------------------------------------------------------------------------------
void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) m = micros();
  } while (micros() - m < 10000);
}

//------------------------------------------------------------------------------
void takeOtherMeasures() {
  // Read sensor and print to Serial regardless of SD state
  uint16_t theCO2 = 0;
  float    theTemp = 0;
  float    theHum  = 0;

  if (mySensor.measureSingleShot() == false) {
    Serial.println(F("Single shot failed"));
    theCO2 = (uint16_t)-9; theTemp = -9; theHum = -9;
  } else {
    unsigned long startTime = millis();
    while (mySensor.readMeasurement() == false) {
      if (millis() - startTime > 6000) {
        Serial.println(F("Timeout"));
        theCO2 = (uint16_t)-8; theTemp = -8; theHum = -8;
        break;
      }
      delay(100);
    }
    theCO2  = mySensor.getCO2();
    theTemp = mySensor.getTemperature();
    theHum  = mySensor.getHumidity();
  }

  Serial.print(F("CO2(ppm):"));        Serial.print(theCO2);
  Serial.print(F("\tHumidity(%RH):")); Serial.print(theHum,  1);
  Serial.print(F("\tTemperature(C):")); Serial.print(theTemp, 1);
  Serial.println();

  // Only write to file if SD is available
  if (!sdAvailable) {
    Serial.println(F("(SD absent - sensor data not saved)"));
    return;
  }

  logFile = sd.open(sensorFilename, FILE_WRITE);
#if USE_RTC
  printTime(rtc.now());
  logFile.print(rtc.now().year(),   DEC); logFile.print('/');
  logFile.print(rtc.now().month(),  DEC); logFile.print('/');
  logFile.print(rtc.now().day(),    DEC); logFile.print(' ');
  logFile.print(rtc.now().hour(),   DEC); logFile.print(':');
  logFile.print(rtc.now().minute(), DEC); logFile.print(':');
  logFile.print(rtc.now().second(), DEC);
#else
  logFile.print(millis());
#endif
  logFile.print(',');  logFile.print(theCO2);
  logFile.print(',');  logFile.print(theHum,  1);
  logFile.print(',');  logFile.print(theTemp, 1);
  logFile.println();
  logFile.close();
}

//------------------------------------------------------------------------------
void createBinFile() {
  if (!requireSD(F("record"))) return;

  binFile.close();

  if (SENSORWORKING) {
    takeOtherMeasures();
  } else {
    setupSensor();
  }

#if USE_RTC
  DateTime now = rtc.now();
#else
  DateTime now = DateTime(2025, 8, 11, 12, 0, 0);
#endif

  char binName[36] = {0};
  snprintf(binName, sizeof(binName),
           "%04d%02d%02d_%02d%02d%02d_%s.bin",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second(),
           sensorID);

  Serial.print(F("Opening: "));  Serial.println(binName);
  if (!binFile.open(binName, O_RDWR | O_CREAT)) error("open binName failed");

  Serial.print(F("Allocating: ")); Serial.print(MAX_FILE_SIZE_MiB); Serial.println(F(" MiB"));
  if (!binFile.preAllocate(MAX_FILE_SIZE)) {
    printUnusedStack();
    error("preAllocate failed");
  }
}

//------------------------------------------------------------------------------
bool createCsvFile() {
  char csvName[NAME_DIM];
  if (!binFile.isOpen()) { Serial.println(F("No current binary file")); return false; }
  binFile.getName(csvName, sizeof(csvName));
  char* dot = strchr(csvName, '.');
  if (!dot) error("no dot in binName");
  strcpy(dot + 1, "csv");
  if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) error("open csvFile failed");
  Serial.print(F("Writing: ")); Serial.print(csvName); Serial.println(F(" - type any character to stop"));
  return true;
}

//------------------------------------------------------------------------------
void doSyncSignal() {
  Serial.println(F("SYNC BLINK"));
  const int SYNC_LED = 10;
  const int repeats  = 1200;
  for (int i = 0; i < repeats; i++) {
    digitalWrite(SYNC_LED, HIGH);
    delayMicroseconds(1600);
    digitalWrite(SYNC_LED, LOW);
  }
}

//------------------------------------------------------------------------------
void logData() {
  uint32_t t0, t1, overruns = 0, count = 0, maxLatencyUsec = 0;
  uint32_t loggingMillis;
  size_t   maxFifoUse = 0;
  block_t  fifoBuffer[FIFO_DIM];

  adcInit((metadata_t*)fifoBuffer);
  if (sizeof(metadata_t) != binFile.write(fifoBuffer, sizeof(metadata_t)))
    error("Write metadata failed");

  fifoCount = 0; fifoHead = 0; fifoTail = 0;
  fifoData  = fifoBuffer;
  memset(fifoBuffer, 0, sizeof(fifoBuffer));

  Serial.println(F("Logging - type any character to stop"));
  Serial.flush();
  delay(50);

  t0 = loggingMillis = t1 = millis();
  adcStart();

  while (1) {
    uint32_t m;
    noInterrupts();
    size_t tmpFifoCount = fifoCount;
    interrupts();

    if (tmpFifoCount) {
      block_t* pBlock = fifoData + fifoTail;
      m = micros();
      if (sizeof(block_t) != binFile.write(pBlock, sizeof(block_t)))
        error("write data failed");
      m  = micros() - m;
      t1 = millis();
      if (m > maxLatencyUsec)          maxLatencyUsec = m;
      if (tmpFifoCount > maxFifoUse)   maxFifoUse     = tmpFifoCount;
      count += pBlock->count;
      if (pBlock->overrun) {
        overruns += pBlock->overrun;
        if (ERROR_LED_PIN >= 0) digitalWrite(ERROR_LED_PIN, HIGH);
      }
      pBlock->count   = 0;
      pBlock->overrun = 0;
      fifoTail = fifoTail < (FIFO_DIM - 1) ? fifoTail + 1 : 0;
      noInterrupts(); fifoCount--; interrupts();
      if (binFile.curPosition() >= MAX_FILE_SIZE) { adcStop(); break; }
    }

    if (timerError)        error("Missed timer event - rate too high");
    if (Serial.available()) isrStop = true;
    if (digitalRead(15) == HIGH) isrStop = true;
    if (fifoCount == 0 && !adcActive()) break;
  }

  Serial.println();
  if (binFile.curPosition() < MAX_FILE_SIZE) {
    Serial.println(F("Truncating file"));
    Serial.flush();
    if (!binFile.truncate()) error("Can't truncate file");
  }
  Serial.print(F("Max write latency usec: ")); Serial.println(maxLatencyUsec);
  Serial.print(F("Record time sec: "));        Serial.println(0.001 * (t1 - t0), 3);
  Serial.print(F("Sample count: "));           Serial.println(count / PIN_COUNT);
  Serial.print(F("Overruns: "));               Serial.println(overruns);
  Serial.print(F("FIFO_DIM: "));               Serial.println(FIFO_DIM);
  Serial.print(F("maxFifoUse: "));             Serial.println(maxFifoUse + 1);
  Serial.println(F("Done"));
}

//------------------------------------------------------------------------------
void openBinFile() {
  char name[NAME_DIM];
  clearSerialInput();
  Serial.println(F("Enter file name"));
  if (!serialReadLine(name, sizeof(name))) return;
  if (!sd.exists(name)) { Serial.println(name); Serial.println(F("File does not exist")); return; }
  binFile.close();
  if (!binFile.open(name, O_RDWR)) { Serial.println(name); Serial.println(F("open failed")); return; }
  Serial.println(F("File opened"));
}

//------------------------------------------------------------------------------
void printData() {
  block_t buf;
  if (!binFile.isOpen()) { Serial.println(F("No current binary file")); return; }
  binFile.rewind();
  if (binFile.read(&buf, sizeof(buf)) != sizeof(buf)) error("Read metadata failed");
  Serial.println(F("Type any character to stop"));
  delay(1000);
  while (!Serial.available() && binFile.read(&buf, sizeof(buf)) == sizeof(buf)) {
    if (buf.count == 0) break;
    if (buf.overrun) { Serial.print(F("OVERRUN,")); Serial.println(buf.overrun); }
    for (size_t i = 0; i < buf.count; i++) {
      Serial.print(buf.data[i], DEC);
      if ((i + 1) % PIN_COUNT) Serial.print(','); else Serial.println();
    }
  }
  Serial.println(F("Done"));
}

//------------------------------------------------------------------------------
bool serialReadLine(char* str, size_t size) {
  size_t n = 0;
  while (!Serial.available()) {}
  while (true) {
    int c = Serial.read();
    if (c < ' ') break;
    str[n++] = c;
    if (n >= size) { Serial.println(F("input too long")); return false; }
    uint32_t m = millis();
    while (!Serial.available() && (millis() - m) < 100) {}
    if (!Serial.available()) break;
  }
  str[n] = 0;
  return true;
}

//------------------------------------------------------------------------------
void setupSensor() {
  Wire.begin();
  if (mySensor.begin(false, true, false) == false) {
    Serial.println(F("SCD41 sensor not detected - no CO2/temp/humidity data"));
    SENSORWORKING = false;
    // Three blinks on error LED to signal sensor missing
    if (ERROR_LED_PIN >= 0) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(ERROR_LED_PIN, LOW);  delay(200);
        digitalWrite(ERROR_LED_PIN, HIGH); delay(400);
      }
      digitalWrite(ERROR_LED_PIN, LOW);
    }
  } else {
    SENSORWORKING = true;
    Serial.println(F("SCD41 sensor OK"));
    // Write CSV header only if SD is available
    if (sdAvailable) {
      logFile = sd.open(sensorFilename, FILE_WRITE);
      if (logFile) {
        logFile.println(F("Time,CO2,Humidity,Temperature"));
        logFile.close();
      }
    }
  }
}

//==============================================================================
void setup(void) {
  if (ERROR_LED_PIN >= 0) pinMode(ERROR_LED_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.setTimeout(500);

  pinMode(16, OUTPUT);
  pinMode(14, INPUT_PULLUP);
  pinMode(17, OUTPUT);
  pinMode(15, INPUT_PULLUP);
  digitalWrite(17, HIGH);

  loadOrCreateSensorID();
  snprintf(sensorFilename, sizeof(sensorFilename), "otherData%s.csv", sensorID);

  // ── Live view mode (pin 14 held LOW) ─────────────────────────────────────
  // This runs BEFORE SD init so it always works even with no SD card.
  if (digitalRead(14) == LOW) {
    // Signal entry into live view with LED blinks
    for (int i = 0; i < 7; i++) {
      digitalWrite(17, i % 2 == 0 ? HIGH : LOW);
      delay(200);
    }
    digitalWrite(17, HIGH);
  }
  while (digitalRead(14) == LOW) {
    Serial.print(analogRead(A0)); Serial.print(' ');
    Serial.print(analogRead(A1)); Serial.print(' ');
    Serial.print(analogRead(A2)); Serial.print(' ');
    Serial.print(analogRead(A3)); Serial.print(' ');
    Serial.print(analogRead(A4)); Serial.print(' ');
    Serial.println(analogRead(A5));
  }
  digitalWrite(17, HIGH);
  // ─────────────────────────────────────────────────────────────────────────

  FillStack();
  analogRead(PIN_LIST[0]);  // init ADC

#if !ENABLE_DEDICATED_SPI
  Serial.println(F("\nFor best performance edit SdFatConfig.h\nand set ENABLE_DEDICATED_SPI nonzero"));
#endif

  // ── SD initialisation ────────────────────────────────────────────────────
  if (!sd.begin(SD_CONFIG)) {
    Serial.println(F("*** SD card not found - recording disabled ***"));
    Serial.println(F("*** Live view (pin 14) and serial commands still available ***"));
    sdAvailable = false;
    // 3 slow flashes at boot to signal SD missing
    if (ERROR_LED_PIN >= 0) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(ERROR_LED_PIN, HIGH); delay(300);
        digitalWrite(ERROR_LED_PIN, LOW);  delay(300);
      }
    }
  } else {
    sdAvailable = true;
    Serial.println(F("SD card OK"));
  }
  // ─────────────────────────────────────────────────────────────────────────

  // Sensor setup runs after SD init so setupSensor() knows sdAvailable
  setupSensor();

#if USE_RTC
  if (!rtc.begin()) {
    Serial.println(F("RTC begin failed"));
  }
  if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
  } else {
    Serial.println(F("RTC is running"));
    printTime(rtc.now());
  }
  FsDateTime::setCallback(dateTime);
#endif
}

//==============================================================================
void loop(void) {
  printUnusedStack();
  clearSerialInput();
  digitalWrite(17, HIGH);

  Serial.println();
  Serial.println(F("Running BeeSpy3 version Combined_H"));
  Serial.print(F("Sensor ID : ")); Serial.println(sensorID);
  Serial.print(F("SD card   : ")); Serial.println(sdAvailable ? F("OK") : F("NOT DETECTED - recording disabled"));

  Serial.println(F("type:"));
  Serial.println(F("b - open existing bin file"));
  Serial.println(F("c - convert file to csv"));
  Serial.println(F("l - list files"));
  Serial.println(F("p - print data to Serial"));
  Serial.println(F("r - record ADC data"));
  Serial.println(F("t - set the RTC time"));
  Serial.println(F("x - RESET"));

  char c = 'l';

  if (digitalRead(15) == LOW) {
    // Hardware record switch
    if (!sdAvailable) {
      Serial.println(F("Record switch detected but SD card absent - cannot record"));
      flashNoSD();
      // Do NOT set c='r'; fall through to normal menu prompt
    } else {
      c = 'r';
      Serial.println(F("Recording (hardware trigger)"));
      digitalWrite(17, LOW);
    }
  }

  if (c != 'r') {
    // Wait for serial input, but allow live view or record switch to reset
    digitalWrite(17, HIGH);
    while (!Serial.available()) {
      if (digitalRead(14) == LOW || digitalRead(15) == LOW) resetFunc();
    }
    c = tolower(Serial.read());
    Serial.println();
    if (ERROR_LED_PIN >= 0) digitalWrite(ERROR_LED_PIN, LOW);
    clearSerialInput();
  }

  if (c == 'b') {
    if (requireSD(F("open file"))) openBinFile();

  } else if (c == 'c') {
    if (requireSD(F("convert file")) && createCsvFile()) binaryToCsv();

  } else if (c == 'l') {
    if (sdAvailable) {
      Serial.println(F("ls:"));
      sd.ls(&Serial, LS_DATE | LS_SIZE);
    } else {
      Serial.println(F("No SD card - cannot list files"));
      flashNoSD();
    }

  } else if (c == 'p') {
    if (requireSD(F("print data"))) printData();

  } else if (c == 'r') {
    if (requireSD(F("record"))) {
      createBinFile();
      logData();
    }

  } else if (c == 'x') {
    resetFunc();

  } else if (c == 't') {
#if USE_RTC
    stopRTC();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    startRTC();
    Serial.println(F("RTC reset to compile time:"));
    printTime(rtc.now());
#endif

  } else {
    Serial.println(F("Invalid entry"));
  }

  digitalWrite(17, HIGH);
}

#else  // __AVR__
#error This program is only for AVR.
#endif  // __AVR__
