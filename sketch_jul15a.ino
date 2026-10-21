// Build 20261021 1739

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "HX711.h"
#include <RTClib.h>

// ==========================================
// CONFIGURATION SWITCH
// ==========================================
#define SIMULATION_MODE false

// --- I2C Addresses for Atlas Scientific EZO Modules ---
#define PH_I2C_ADDRESS    63    // FIX: Changed from 0x63 to decimal 63 (0x3F) to match exact pH module factory address
#define EC_I2C_ADDRESS    100   // Matches your exact EC I2C address (Decimal 100 = 0x64)

// --- Hardware Pin Allocations ---
#define LOADCELL_DOUT          2
#define LOADCELL_SCK           3
#define ONE_WIRE_WATER_BUS     4
#define ONE_WIRE_GLASS_BUS     7
#define SD_CHIP_SELECT         10

// --- Object Initializations ---
OneWire oneWireWater(ONE_WIRE_WATER_BUS);
DallasTemperature waterSensor(&oneWireWater);

OneWire oneWireGlass(ONE_WIRE_GLASS_BUS);
DallasTemperature glassSensor(&oneWireGlass);

HX711 scale;
File dataFile;

RTC_DS3231 rtc;
bool rtcReady = false;

float weightOffset = 0.0;

// --- Time Synchronization Variable ---
unsigned long startTime = 0;
unsigned long timeOffset = 0;//recording the last-connected time[if disconnected at 13 sec, goes on from that]

// --- Buffers for Physical Atlas Hardware Parsing ---
char ph_data[32];           // 32-byte specification allocated directly from PDF template
char ec_data[32];           // 32-byte specification allocated directly from EC template
byte received_from_sensor;
byte in_char = 0;

void printLeftAlignedWithBorder(String text, int width)
{
  Serial.print(text);
  int spacesToPrint = width - text.length();
  for (int i = 0; i < spacesToPrint; i++)
  {
    Serial.print(' ');
  }
  Serial.println(F("|"));
}

String getTimestamp()
{
  if (!rtcReady)
  {
    return String(millis());
  }

  DateTime now = rtc.now();

  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  return String(buf);
}

unsigned long readLastTimestamp(const char* filename)
{
  unsigned long lastTimestamp = 0;
  File f = SD.open(filename, FILE_READ);
  if (!f) return 0;

  String lastValidLine = "";

  while (f.available())
  {
    String line = f.readStringUntil('\n');
    line.trim();

    if (line.length() > 0 && isDigit(line[0]) && line.indexOf(',') > 0)
    {
      lastValidLine = line;
    }
  }
  f.close();

  if (lastValidLine.length() > 0)
  {
    int commaIdx = lastValidLine.indexOf(',');
    String tsStr = lastValidLine.substring(0, commaIdx);
    lastTimestamp = (unsigned long) tsStr.toInt();
  }

  return lastTimestamp;
}

float getStableWeight(byte samples)
{
  if (samples > 30) samples = 30;
  if (samples < 3) samples = 3;

  float readings[30];
  byte count = 0;

  for (byte i = 0; i < samples; i++)
  {
    if (scale.is_ready())
    {
      readings[count] = scale.get_units(1);
      count++;
    }
    delay(10);
  }

  if (count == 0) return 0.0;

  for (byte i = 0; i < count - 1; i++)
  {
    for (byte j = 0; j < count - i - 1; j++)
    {
      if (readings[j] > readings[j + 1])
      {
        float temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }

  return readings[count / 2];
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Wire.begin();

  Serial.println(F("--- Initializing Distillation Monitoring Rig ---"));

  
  Serial.print(F("Initializing RTC..."));
  if (!rtc.begin())
  {
    Serial.println(F(" FAILED. Check wiring. Falling back to millis()-based timestamps."));
    rtcReady = false;
  }
  else
  {
  rtcReady = true;
  Serial.println(F(" OK."));

  DateTime now = rtc.now();
  DateTime compileTime(F(__DATE__), F(__TIME__));

  Serial.print(F("[DEBUG] Firmware compiled at: "));
  Serial.print(F(__DATE__)); Serial.print(F(" ")); Serial.println(F(__TIME__));

  Serial.print(F("[DEBUG] RTC now: "));
  Serial.println(now.timestamp());   // YYYY-MM-DDThh:mm:ss 형식

  if (rtc.lostPower() || now.unixtime() < compileTime.unixtime())
  {
    Serial.println(F("RTC behind compile time or lost power -> setting to compile time."));
    rtc.adjust(compileTime);
  }
  else
  {
    Serial.println(F("RTC time already valid, keeping current time."));
  }
  }


  // 1. Initialize Dallas OneWire Temperature Sensors
  waterSensor.begin();
  glassSensor.begin();

  // 2. Initialize HX711
  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(422.41);// Current scale value

  // 3. Initialize MicroSD Card Logging Module over SPI
  if (!SD.begin(SD_CHIP_SELECT))
  {
    Serial.println(F("Error: SD initialization failed! Check SPI pins."));
  }
  else
  {
    Serial.println(F("SD card ready."));

    dataFile = SD.open("distill.csv", FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("Timestamp_ms,RealTime,WaterTemp_C,GlassTemp_C,Weight_g,pH,EC_uS");
      dataFile.close();
      Serial.println(F("CSV Header created successfully: distill.csv"));
    }
    else
    {
      Serial.println(F("Error opening distill.csv for header creation."));
    }
  }

  // --- ATLAS EZO SENSORS INITIALIZATION ---
  if (!SIMULATION_MODE)
  {
    Serial.println(F("Configuring Sensors..."));

    Wire.beginTransmission(EC_I2C_ADDRESS);
    Wire.print("K,1.0");
    Wire.endTransmission(); 
    delay(300);

    Wire.beginTransmission(EC_I2C_ADDRESS);
    Wire.print("C,0");
    Wire.endTransmission();
    delay(300);

    Serial.println(F("Sensors initialization commands sent."));
  }

  Serial.println(F("Waiting for load cell to be ready for taring..."));
  unsigned long tareWaitStart = millis();
  while (!scale.is_ready() && (millis() - tareWaitStart < 5000))
  {
    delay(10);
  }

  if (scale.is_ready())
  {
    Serial.println(F("Stabilizing load cell readings..."));
    
    delay(1000); 

    bool stable = false;
    unsigned long stabilizeStart = millis();

    while (!stable && (millis() - stabilizeStart < 5000))
    {
      float minVal = 9999.0;
      float maxVal = -9999.0;
      
      for (int i = 0; i < 10; i++)
      {
        float val = scale.get_units(1);
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        delay(30);
      }

      if ((maxVal - minVal) < 3.0)
      {
        stable = true;
      }
    }

    scale.tare(50);
    
    weightOffset = getStableWeight(10);
    
    Serial.print(F("Auto Zero Offset Set To: "));
    Serial.println(weightOffset);
    Serial.println(F("Tare complete. Baseline forced to 0g."));
  }
  else
  {
    Serial.println(F("[WARN] Load cell not ready after 5s - tare skipped, baseline may be inaccurate!"));
  }

  Serial.print(F("Zero factor (offset): "));
  Serial.println(scale.get_offset());

  //Serial.println(F("--- Initialization Complete. Starting Active Log ---"));
  startTime = millis();
}

void loop()
{
  unsigned long relativeMillis = (millis() - startTime) + timeOffset;
  String realTimestamp = getTimestamp();

  // --- 1. Read Temperatures via OneWire ---
  waterSensor.requestTemperatures();
  float waterTemperature = waterSensor.getTempCByIndex(0);

  glassSensor.requestTemperatures();
  float glassTemperature = glassSensor.getTempCByIndex(0);

  // --- 2. Read Weight via HX711 ---
  float currentWeight = 0.0;
  if (scale.is_ready())
  {
    currentWeight = getStableWeight(15) - weightOffset;

    if (currentWeight > -3.0 && currentWeight < 3.0)
    {
      currentWeight = 0.0;
    }
  }
  else
  {
    Serial.println(F("[WARN] Load cell not ready, skipping weight read this cycle."));
  }

  float phValue = 0.00;
  float ecValue = 0.00;

  // ==========================================
  // DATA POLLING LOGIC
  // ==========================================
  if (SIMULATION_MODE)
  {
    phValue = 7.00;
    ecValue = 150.00;
    delay(2000);
  }
  else
  {
    // --- Step 1: Send pH Reading Command ---
    Wire.beginTransmission(PH_I2C_ADDRESS);
    Wire.write('R');
    Wire.endTransmission(); 

    delay(815); // Matches exact conversion window specified in PDF source code (815ms)

    // ====================================================================
    // ======================= START OF pH SECTION ========================
    // ====================================================================
    Wire.requestFrom(PH_I2C_ADDRESS, 32); // Fix: Removed R4 incompatible third parameter ', 1'
    if (Wire.available() > 0)
    {
      received_from_sensor = Wire.read();    // First byte is the response status code

      byte ph_idx = 0;
      while (Wire.available())
      {
        in_char = Wire.read();
        
        // Break routine on string boundary markers
        if (in_char == 0xFF || in_char == 0 || in_char == '\r' || in_char == '\n')
        {
          break;
        }
        ph_data[ph_idx] = in_char;
        ph_idx++;
      }
      ph_data[ph_idx] = '\0'; // Safe terminal seal

      // Complete string-to-float translation if response returns Success (Status Code = 1)
      if (received_from_sensor == 1 && ph_idx > 0)
      {
        phValue = atof(ph_data); // Successfully updates the dashboard floating parameter
      }
    }
    // ====================================================================
    // ======================== END OF pH SECTION =========================
    // ====================================================================


    // --- Step 2: Send EC Reading Command ---
    // FIX: Send a temporary 'I' (Information) command first to forcefully flush the chip's internal calculation pipeline
    Wire.beginTransmission(EC_I2C_ADDRESS);
    Wire.write('I');
    Wire.endTransmission();
    delay(50);

    // Send the actual active real-time reading command to the calibrated sensor
    Wire.beginTransmission(EC_I2C_ADDRESS);
    Wire.write('R');
    Wire.endTransmission(); 

    delay(900); // Wait 900ms for EC conversion cycle stabilization

    // ====================================================================
    // =================== START OF CONDUCTIVITY SECTION ==================
    // ====================================================================

//고칠 한국어 부분 시작 [1]

    byte ecBytesAvailable = Wire.requestFrom((uint8_t)EC_I2C_ADDRESS, (uint8_t)32, (uint8_t)true);

// EC sensor replying check
//remove annotation if needed
//Serial.print(F("[EC DEBUG] bytes available: "));
//Serial.println(ecBytesAvailable);

if (Wire.available() > 0)
{
  received_from_sensor = Wire.read();

  // status code reply (1=success, 2=failure to receive orders, 254=managing, 255=no reply)
  //remove annotation if needed
  //Serial.print(F("[EC DEBUG] status code: "));
  //Serial.println(received_from_sensor);

  byte ec_idx = 0;
  memset(ec_data, 0, sizeof(ec_data));
  while (Wire.available() && ec_idx < 31)   // ← 버퍼 오버플로우 방지 (31로 제한)
  {
    in_char = Wire.read();
    if (in_char == 0xFF || in_char == 0 || in_char == '\r' || in_char == '\n')
    {
      break;
    }
    ec_data[ec_idx] = in_char;
    ec_idx++;
  }
  ec_data[ec_idx] = '\0';

  // raw string check function: shows the actual EC value
  //remove annotation if needed
  //Serial.print(F("[EC DEBUG] raw string: "));
  //Serial.println(ec_data);

  // Parse isolated pure EC parameter safely whether commas exist or not
  if (received_from_sensor == 1 && ec_idx > 0)
  {
    // Check if the raw string actually contains a comma delimiter
    if (strchr(ec_data, ',') != NULL) 
    {
      char *ec_parsed;
      ec_parsed = strtok(ec_data, ","); 
      if (ec_parsed != NULL)
      {
        ecValue = atof(ec_parsed);       
      }
    }
    else 
    {
      // If no comma exists, directly translate the clean raw buffer string to float
      ecValue = atof(ec_data); 
    }
  }
}
else
{
  Serial.println(F("[EC DEBUG] Wire.available() == 0 -> I2C Reply: Null (Address/Power/Connection check needed)"));
}
//고칠 한국어 부분 끝 [1]
/*[1]의 원본
    Wire.requestFrom((uint8_t)EC_I2C_ADDRESS, (uint8_t)32, (uint8_t)true);
    if (Wire.available() > 0)
    {
      received_from_sensor = Wire.read();    

      byte ec_idx = 0;
      memset(ec_data, 0, sizeof(ec_data));
      while (Wire.available())
      {
        in_char = Wire.read();
        if (in_char == 0xFF || in_char == 0 || in_char == '\r' || in_char == '\n')
        {
          break;
        }
        ec_data[ec_idx] = in_char;
        ec_idx++;
      }
      ec_data[ec_idx] = '\0';

      // Parse isolated pure EC parameter safely whether commas exist or not
      if (received_from_sensor == 1 && ec_idx > 0)
      {
        // Check if the raw string actually contains a comma delimiter
        if (strchr(ec_data, ',') != NULL) 
        {
          char *ec_parsed;
          ec_parsed = strtok(ec_data, ","); 
          if (ec_parsed != NULL)
          {
            ecValue = atof(ec_parsed);       
          }
        }
        else 
        {
          // If no comma exists, directly translate the clean raw buffer string to float
          ecValue = atof(ec_data); 
        }
      }
    }
    
    */

    // ====================================================================
    // ==================== END OF CONDUCTIVITY SECTION ===================
    // ====================================================================

    delay(200);
  }

  // --- Output Full Live Dataset to Serial Monitor ---
  Serial.println(F("+--------------------------------+"));

  Serial.print(F("| Time: "));
  printLeftAlignedWithBorder(String(relativeMillis) + "ms", 25);

  Serial.print(F("| RealTime: "));
  printLeftAlignedWithBorder(realTimestamp, 21);

  unsigned long minutes = relativeMillis / 60000;
  unsigned long seconds = (relativeMillis % 60000) / 1000;
  unsigned long remainderMs = relativeMillis % 1000;
  
  String secondsFormatted = String(relativeMillis / 1000.0, 2) + "s";
  String timeFormatted = String(minutes) + "m " + String(seconds) + "s " + String(remainderMs) + "ms";
  Serial.print(F("| Sec : "));
  printLeftAlignedWithBorder(secondsFormatted, 25);
  
  Serial.print(F("| Min : "));
  printLeftAlignedWithBorder(timeFormatted, 25);

  Serial.print(F("| WaterTemp: "));
  printLeftAlignedWithBorder(String(waterTemperature, 1) + "C", 20);

  Serial.print(F("| GlassTemp: "));
  printLeftAlignedWithBorder(String(glassTemperature, 1) + "C", 20);

  Serial.print(F("| Weight: "));
  printLeftAlignedWithBorder(String(currentWeight, 1) + "g", 23);

  Serial.print(F("| pH: "));
  printLeftAlignedWithBorder(String(phValue, 2) + " pH", 27);

  Serial.print(F("| EC: "));
  printLeftAlignedWithBorder(String(ecValue, 1) + " µS/cm", 28);

  Serial.println(F("+--------------------------------+"));

  // --- Commit All Log Values to the MicroSD Card ---
  dataFile = SD.open("distill.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.print(relativeMillis); dataFile.print(",");
    dataFile.print("=\""); dataFile.print(realTimestamp); dataFile.print("\""); dataFile.print(",");
    dataFile.print(waterTemperature); dataFile.print(",");
    dataFile.print(glassTemperature); dataFile.print(",");
    dataFile.print(currentWeight); dataFile.print(",");
    dataFile.print(phValue); dataFile.print(",");
    dataFile.println(ecValue);
    dataFile.close();
  }
  else
  {
    Serial.println(F("Error writing data matrix row to distill.csv"));
  }
}