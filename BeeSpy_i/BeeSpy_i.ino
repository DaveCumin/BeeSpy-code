#ifdef __AVR__
#include <SPI.h>
#include <Wire.h>
#include "AvrAdcLogger.h"
#include "BufferedPrint.h"
#include "FreeStack.h"
#include "SdFat.h"
#include "SparkFunHTU21D.h"
#include "ccs811.h"
#include <Protocentral_FDC1004.h>

#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define Serial MinSerial
#endif

#define SD_FAT_TYPE 3
#define USE_RTC 1
#if USE_RTC
#include "RTClib.h"
#endif

// Pin definitions
const int8_t ERROR_LED_PIN = 16;
const uint8_t SD_CS_PIN = 53;

// Analog pins
const uint8_t PIN_LIST[] = { 0, 1, 2, 3, 4, 5 };
const float SAMPLE_RATE = 5000;
const float SAMPLE_INTERVAL = 1.0 / SAMPLE_RATE;
#define ROUND_SAMPLE_INTERVAL 1
uint8_t const ADC_REF = (1 << REFS0);
const uint32_t MAX_FILE_SIZE_MiB = 36;
#define LOG_FILE_NAME "AvrAdc00.bin"
const size_t NAME_DIM = 40;
#define RECORD_EIGHT_BITS 0
#define TMP_FILE_NAME "tmp_adc.bin"

// FIFO size
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

// SPI clock
#define SPI_CLOCK SD_SCK_MHZ(10)
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif

// SD card setup
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

// Sensor objects
HTU21D temphum; 
CCS811 ccs;
uint16_t eco2, etvoc, errstat, raw;
FDC1004 fdc;
int capdac = 0; // CapDAC value for adjusting measurement range
// FDC1004 sensor initialization
#define UPPER_BOUND 0X4000 // Max readout capacitance
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0    // Channel to be read
#define MEASURMENT 0 // Measurement channel

// Sensor data block structure
typedef struct sensor_block {
  uint32_t timestamp;    // RTC timestamp in seconds since epoch
  uint8_t type;          // 1 = sensor block
  float htu_temp;        // HTU21D temperature (°C)
  float htu_hum;         // HTU21D humidity (%)
  uint16_t ccs_co2;      // CCS811 CO2 (ppm)
  float ccs_temp;        // CCS811 temperature (°C)
  int32_t fdc_cap;       // FDC1004 capacitance (raw or converted to pF)
  uint8_t reserved[40];  // Padding to align with 64-byte block
} sensor_block_t;

// ADC block definitions
const uint8_t PIN_COUNT = sizeof(PIN_LIST) / sizeof(PIN_LIST[0]);
const uint16_t MIN_ADC_CYCLES = 15;
const uint16_t ISR_SETUP_ADC = PIN_COUNT > 1 ? 100 : 0;
const uint16_t ISR_TIMER0 = 160;
const uint32_t MAX_FILE_SIZE = MAX_FILE_SIZE_MiB << 20;

inline bool adcActive() { return (1 << ADIE) & ADCSRA; }

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

// ISR variables
volatile bool isrStop = false;
block_t* isrBuf = nullptr;
uint16_t isrOver = 0;
uint8_t adcmux[PIN_COUNT];
uint8_t adcsra[PIN_COUNT];
uint8_t adcsrb[PIN_COUNT];
uint8_t adcindex = 1;
volatile bool timerError = false;
volatile bool timerFlag = false;

// RTC
#if USE_RTC
RTC_DS1307 rtc;
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();
  *date = FS_DATE(now.year(), now.month(), now.day());
  *time = FS_TIME(now.hour(), now.minute(), now.second());
  *ms10 = now.second() & 1 ? 100 : 0;
}
#endif

// ISR for ADC
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

// Timer1 ISR
ISR(TIMER1_COMPB_vect) {
  if (timerFlag) {
    timerError = true;
  }
  timerFlag = true;
}

// Error handling
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
  fatalBlink();
}

void printTime(DateTime timeIN) {
  Serial.print(F("The rtc time is "));
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
  Serial.print(F("\nUnused stack: "));
  Serial.println(UnusedStack());
}

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
  Serial.println("RTC stopped!");
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
  Serial.println("RTC started!");
}

void adcInit(metadata_t* meta) {
  uint8_t adps;
  uint32_t ticks = F_CPU * SAMPLE_INTERVAL + 0.5;

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) {
    error("Invalid ADC reference");
  }
#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) {
    error("Invalid ADC prescaler");
  }
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

void binaryToCsv() {
  uint8_t lastPct = 0;
  block_t* pd;
  sensor_block_t* ps;
  metadata_t* pm;
  uint32_t t0 = millis();
  BufferedPrint<file_t, 64> bp(&csvFile);
  block_t binBuffer[FIFO_DIM];
  sensor_block_t sensorBuffer;

  assert(sizeof(block_t) == sizeof(metadata_t));
  assert(sizeof(sensor_block_t) == 64);  // Ensure sensor block is 64 bytes
  binFile.rewind();
  uint32_t tPct = millis();
  bool doMeta = true;
  bool sensorHeaderWritten = false;

  while (!Serial.available()) {
    int nb = binFile.read(binBuffer, sizeof(binBuffer));
    if (nb < 0) {
      error("read binFile failed");
    }
    size_t nd = nb / sizeof(block_t);
    if (nd < 1 && !binFile.available()) {
      break;
    }

    if (doMeta) {
      doMeta = false;
      pm = (metadata_t*)binBuffer;
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
      continue;  // Process next block
    }

    for (size_t i = 0; i < nd; i++) {
      pd = &binBuffer[i];
      if (pd->count == 0 && pd->overrun == 0) {  // Possible sensor block
        binFile.seekCur(-sizeof(block_t));
        if (binFile.read(&sensorBuffer, sizeof(sensor_block_t)) != sizeof(sensor_block_t)) {
          error("read sensor block failed");
        }
        if (sensorBuffer.type == 1) {
          if (!sensorHeaderWritten) {
            bp.println(F("\nSensor Data:"));
            bp.println(F("Timestamp,HTU_Temp,HTU_Humidity,CCS_CO2,CCS_Temp,FDC_Capacitance"));
            sensorHeaderWritten = true;
          }
          bp.print(sensorBuffer.timestamp);
          bp.print(',');
          bp.print(sensorBuffer.htu_temp, 2);
          bp.print(',');
          bp.print(sensorBuffer.htu_hum, 2);
          bp.print(',');
          bp.print(sensorBuffer.ccs_co2);
          bp.print(',');
          bp.print(sensorBuffer.ccs_temp, 2);
          bp.print(',');
          bp.print(sensorBuffer.fdc_cap);
          bp.println();
          continue;
        } else {
          binFile.seekCur(-sizeof(sensor_block_t) + sizeof(block_t));
        }
      }
      if (pd->overrun) {
        bp.print(F("OVERRUN,"));
        bp.println(pd->overrun);
      }
      for (size_t j = 0; j < pd->count; j += PIN_COUNT) {
        for (size_t k = 0; k < PIN_COUNT; k++) {
          if (!bp.printField(pd->data[k + j], k == (PIN_COUNT - 1) ? '\n' : ',')) {
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

void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}

void createBinFile() {
  binFile.close();
  DateTime now = rtc.now();
  char binName[] = "2024_01_01_23_59_59.bin";
  binName[0] = (now.year() / 1000) % 10 + '0';
  binName[1] = (now.year() / 100) % 10 + '0';
  binName[2] = (now.year() / 10) % 10 + '0';
  binName[3] = now.year() % 10 + '0';
  binName[5] = now.month() / 10 + '0';
  binName[6] = now.month() % 10 + '0';
  binName[8] = now.day() / 10 + '0';
  binName[9] = now.day() % 10 + '0';
  binName[11] = now.hour() / 10 + '0';
  binName[12] = now.hour() % 10 + '0';
  binName[14] = now.minute() / 10 + '0';
  binName[15] = now.minute() % 10 + '0';
  binName[17] = now.second() / 10 + '0';
  binName[18] = now.second() % 10 + '0';

  Serial.print(F("Opening: "));
  Serial.println(binName);
  if (!binFile.open(binName, O_RDWR | O_CREAT)) {
    error("open binName failed");
  }
  Serial.print(F("Allocating: "));
  Serial.print(MAX_FILE_SIZE_MiB);
  Serial.println(F(" MiB"));
  if (!binFile.preAllocate(MAX_FILE_SIZE)) {
    error("preAllocate failed");
  }
}

long readCapacitance()
{
    long capacitance = 0;
    fdc.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac); // Configure the sensor for single measurement on channel 0
    fdc.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);     // Trigger the measurement at 100Hz
    // Wait for the sensor to settle
    delay(15); // Allow enough time for the sensor to complete the measurement
    uint16_t value[2];
    if (!fdc.readMeasurement(MEASURMENT, value))
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

bool createCsvFile() {
  char csvName[NAME_DIM];
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

void logData() {
  uint32_t t0, t1;
  uint32_t overruns = 0;
  uint32_t count = 0;
  uint32_t maxLatencyUsec = 0;
  size_t maxFifoUse = 0;
  block_t fifoBuffer[FIFO_DIM];
  sensor_block_t sensorBuffer;

  // Timers for sensor readings
  uint32_t lastHtuCcsRead = 0;
  uint32_t lastFdcRead = 0;
  const uint32_t HTU_CCS_INTERVAL = 300000;  // 5 minutes
  const uint32_t FDC_INTERVAL = 250;         // 250 ms

  adcInit((metadata_t*)fifoBuffer);
  if (sizeof(metadata_t) != binFile.write(fifoBuffer, sizeof(metadata_t))) {
    error("Write metadata failed");
  }
  fifoCount = 0;
  fifoHead = 0;
  fifoTail = 0;
  fifoData = fifoBuffer;
  memset(fifoBuffer, 0, sizeof(fifoBuffer));
  memset(&sensorBuffer, 0, sizeof(sensor_block_t));
  sensorBuffer.type = 1;  // Sensor block identifier

  Serial.println(F("Logging - type any character to stop"));
  Serial.flush();
  delay(10);

  t0 = millis();
  t1 = t0;
  adcStart();
  while (1) {
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

    // Read sensors at specified intervals
    uint32_t currentMillis = millis();
    if (currentMillis - lastHtuCcsRead >= HTU_CCS_INTERVAL) {
      //time
      sensorBuffer.timestamp = rtc.now().unixtime();
      //temp hum
      sensorBuffer.htu_temp = random(20,24);//temphum.readTemperature();
      sensorBuffer.htu_hum = random(50,60);//temphum.readHumidity();
      //co2
      ccs.read(&eco2, &etvoc, &errstat, &raw);
      sensorBuffer.ccs_co2 = random(400,600);//eco2;
      sensorBuffer.ccs_temp = random(800,900);//etvoc;
      
      lastHtuCcsRead = currentMillis;
      // Write sensor block
      if (sizeof(sensor_block_t) != binFile.write(&sensorBuffer, sizeof(sensor_block_t))) {
        error("write sensor data failed");
      }
    }
    if (currentMillis - lastFdcRead >= FDC_INTERVAL) {
      sensorBuffer.timestamp = rtc.now().unixtime();
      sensorBuffer.fdc_cap = random(0,20);//readCapacitance();  // Read channel 0
      sensorBuffer.htu_temp = 0;
      sensorBuffer.htu_hum = 0;
      sensorBuffer.ccs_co2 = 0;
      sensorBuffer.ccs_temp = 0;
      lastFdcRead = currentMillis;
      if (sizeof(sensor_block_t) != binFile.write(&sensorBuffer, sizeof(sensor_block_t))) {
        error("write sensor data failed");
      }
    }

    if (timerError) {
      error("Missed timer event - rate too high");
    }
    if (Serial.available()) {
      isrStop = true;
    }
    if (digitalRead(15) == LOW) {
      isrStop = true;
    }
    if (fifoCount == 0 && !adcActive()) {
      break;
    }
  }
  Serial.println();
  if (binFile.curPosition() < MAX_FILE_SIZE) {
    Serial.println(F("Truncating file"));
    Serial.flush();
    if (!binFile.truncate()) {
      error("Can't truncate file");
    }
  }
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

void openBinFile() {
  char name[NAME_DIM];
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
  sensor_block_t sensorBuf;
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
  while (!Serial.available()) {
    int n = binFile.read(&buf, sizeof(buf));
    if (n != sizeof(buf)) {
      break;
    }
    if (buf.count == 0 && buf.overrun == 0) {  // Possible sensor block
      binFile.seekCur(-sizeof(block_t));
      if (binFile.read(&sensorBuf, sizeof(sensor_block_t)) != sizeof(sensor_block_t)) {
        break;
      }
      if (sensorBuf.type == 1) {
        Serial.print(F("Sensor: t="));
        Serial.print(sensorBuf.timestamp);
        Serial.print(F(", HTU_T="));
        Serial.print(sensorBuf.htu_temp, 2);
        Serial.print(F(", HTU_H="));
        Serial.print(sensorBuf.htu_hum, 2);
        Serial.print(F(", CCS_CO2="));
        Serial.print(sensorBuf.ccs_co2);
        Serial.print(F(", CCS_T="));
        Serial.print(sensorBuf.ccs_temp, 2);
        Serial.print(F(", FDC_C="));
        Serial.println(sensorBuf.fdc_cap);
        continue;
      } else {
        binFile.seekCur(-sizeof(sensor_block_t) + sizeof(block_t));
      }
    }
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

void setup(void) {
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }
  Serial.begin(9600);
  Serial.setTimeout(500);
  pinMode(16, OUTPUT);
  pinMode(14, INPUT);
  pinMode(17, OUTPUT);
  pinMode(15, INPUT);
  digitalWrite(17, HIGH);

  // Initialize sensors
  temphum.begin();
  if (temphum.readTemperature() > 900){ // the default when not found is 998
      Serial.println("Check circuit. HTU21D not found!");
  }
  if (!ccs.begin()) {
    Serial.println("error CCS811");//error("CCS811 initialization failed");
  }

  Serial.println("just testing");//ccs.setEnvironmentalData(htu.readHumidity(), htu.readTemperature());  // Improve CCS811 accuracy

  if (digitalRead(14) == HIGH) {
    digitalWrite(17, HIGH);
    delay(200);
    digitalWrite(17, LOW);
    delay(200);
    digitalWrite(17, HIGH);
    delay(200);
    digitalWrite(17, LOW);
    delay(200);
    digitalWrite(17, HIGH);
    delay(200);
    digitalWrite(17, LOW);
    delay(200);
    digitalWrite(17, HIGH);
  }
  while (digitalRead(14) == HIGH) {
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
  Serial.println(F("\nFor best performance edit SdFatConfig.h\n"
                   "and set ENABLE_DEDICATED_SPI nonzero"));
#endif
  if (!sd.begin(SD_CONFIG)) {
    error("sd.begin failed");
  }
#if USE_RTC
  if (!rtc.begin()) {
    error("rtc.begin failed");
  }
  if (!rtc.isrunning()) {
    error("RTC is NOT running!");
  } else {
    Serial.println(F("RTC is running"));
    printTime(rtc.now());
  }
  FsDateTime::setCallback(dateTime);
#endif
}

void (*resetFunc)(void) = 0;

void loop(void) {
  printUnusedStack();
  clearSerialInput();
  digitalWrite(17, HIGH);

  Serial.println();
  Serial.println(F("Running BeeSpy3 version I_1"));
  Serial.println(F("type:"));
  Serial.println(F("b - open existing bin file"));
  Serial.println(F("c - convert file to csv"));
  Serial.println(F("l - list files"));
  Serial.println(F("p - print data to Serial"));
  Serial.println(F("r - record ADC data"));
  Serial.println(F("t - set the RTC time"));
  Serial.println(F("x - RESET"));

  char c = 'l';
  
  if (digitalRead(15) == HIGH) {
    c = 'r';
    Serial.println("recording");
    digitalWrite(17, LOW);
  } else {
    digitalWrite(17, HIGH);
    while (!Serial.available()) {
      if (digitalRead(14) == HIGH || digitalRead(15) == HIGH) {
        resetFunc();
      }
    }
    c = tolower(Serial.read());
    Serial.println();
    Serial.println(c);
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, LOW);
    }
    clearSerialInput();
  }
  if (c == 'b') {
    openBinFile();
  } else if (c == 'c') {
    if (createCsvFile()) {
      binaryToCsv();
    }
  } else if (c == 'l') {
    Serial.println(F("ls:"));
    sd.ls(&Serial, LS_DATE | LS_SIZE);
  } else if (c == 'p') {
    printData();
  } else if (c == 'r') {
    createBinFile();
    logData();
  } else if (c == 'x') {
    resetFunc();
  } else if (c == 't') {
    stopRTC();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    startRTC();
    DateTime dtnow = rtc.now();
    Serial.println("Reset RTC time to:");
    printTime(dtnow);
  } else {
    Serial.println(F("Invalid entry"));
  }
  digitalWrite(17, HIGH);
}
#else
#error This program is only for AVR.
#endif