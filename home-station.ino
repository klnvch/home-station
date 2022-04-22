#include <Wire.h>
#include <LiquidCrystal_I2C.h>// https://github.com/johnrickman/LiquidCrystal_I2C

#include <Arduino.h>
#include "MHZ19.h"            // https://github.com/WifWaf/MH-Z19
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <BH1750FVI.h>

#include "PMS.h"              // https://github.com/fu-hsi/pms

#include "print_utils.h"

////////////////////
//xxxxppm xxxlx xx//
//xx°C xx% xxxxhPa//
////////////////////
// VCC -> +5V
// GND -> GND
// SDA -> A4
// SCL -> A5
LiquidCrystal_I2C lcd(0x27,20,4);


// Vin -> +5V
// GND -> GND
// Rx  -> D3
// Tx  -> D2
#define PIN_MHZ19_RX 2                                  // Rx pin which the MHZ19 Tx pin is attached to
#define PIN_MHZ19_TX 3                                  // Tx pin which the MHZ19 Rx pin is attached to
#define MHZ19_BAUDRATE 9600                             // Device to MH-Z19 Serial baudrate (should not be changed)
MHZ19 mhz19;                                            // Constructor for library
SoftwareSerial mhz19Serial(PIN_MHZ19_RX, PIN_MHZ19_TX); // (Uno example) create device to MH-Z19 serial

#define PIN_PMS_RX 8
#define PIN_PMS_TX 9
#define PMS_BAUDRATE 9600
SoftwareSerial pmsSerial(PIN_PMS_TX, PIN_PMS_RX);
PMS pms(pmsSerial);
PMS::DATA data;


// VCC -> +5V
// GND -> GND
// SDA -> A4
// SCL -> A5
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// Connecting the sensor to a Arduino UNO:
//  VCC  <-> 3V3
//  GND  <-> GND
//  SDA  <-> A4/SDA 
//  SCL  <-> A5/SCL
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

#define DEBUG 0
#define GLOBAL_DELAY 100

#define PIN_MQ_135 A0
#define PIN_SHAKE A1
#define PIN_SOUND A2
#define PIN_SOUND_DIGITAL 4

#define SOUND_ALERT_LEVEL 1024
#define SHAKE_ALERT_LEVEL 1024

#define MODE_SLEEP 0
#define MODE_ACTIVE 1
int mode = MODE_SLEEP;

#define ACTIVE_MODE_STAGE_AIR_QUALITY 5000
#define ACTIVE_MODE_STAGE_BME 8000
#define ACTIVE_MODE_STAGE_PME 12000
#define ACTIVE_MODE_STAGE_NORMAL 15000
#define ACTIVE_MODE_STAGE_STOP 20000
unsigned long activeModeStage;

#define LCD_UPDATE_INTERVAL_MILLIS 1000
unsigned long lastLcdUpdateTime = 0;

int co2 = 0;
int mhz19Temperature = 0;
int lux = 0;
int mq135Value = 0;
float temperature = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;
int pms_1_0 = 0;
int pms_2_5 = 0;
int pms_10_0 = 0;

char lcd_line[21];

/*
 * To fix:
 * 1. Add shake sensor
 * 2. Screen blinking when no enough power
 * 3. Remove artifacts at the end of the lines, when active mode is on
 * 4. Do not update screen on the last stage
 * 5. Add calibrate button
 * 8. Check initial delay
 */
void setup() {
  if (DEBUG) {
    Serial.begin(9600);
    while(!Serial);
  }
    
  lcd.init();
  lcd.noBacklight();

  mhz19Serial.begin(MHZ19_BAUDRATE);                    // (Uno example) device to MH-Z19 serial start
  mhz19.begin(mhz19Serial);                             // *Serial(Stream) refence must be passed to library begin().
  mhz19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))

  pmsSerial.begin(PMS_BAUDRATE);

  unsigned status = bme.begin(0x76, &Wire);
  if (DEBUG && !status) {
      Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
      Serial.print(F("SensorID was: 0x")); 
      Serial.println(bme.sensorID(),16);
      Serial.println(F("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085"));
      Serial.println(F("ID of 0x56-0x58 represents a BMP 280,"));
      Serial.println(F("ID of 0x60 represents a BME 280."));
      Serial.println(F("ID of 0x61 represents a BME 680."));
      while (1);
  }

  LightSensor.begin();
  
  pinMode(PIN_MQ_135, INPUT);
  pinMode(PIN_SHAKE, INPUT);
  pinMode(PIN_SOUND, INPUT);
  pinMode(PIN_SOUND_DIGITAL, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  if(mode == MODE_SLEEP) {
    if (currentTime - lastLcdUpdateTime > LCD_UPDATE_INTERVAL_MILLIS) {
      lastLcdUpdateTime = currentTime;
      readSensors();
      printSerial();
      printNormal();
    }
  } else {
    if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_STOP) {
      lcd.noBacklight();
      mode = MODE_SLEEP;
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_NORMAL) {
      lcd.clear();
      printNormal();
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_PME) {
      // |PM 1.0: xxxx (ug/m3)|
      // |PM 2.5: xxxx (ug/m3)|
      // |PM 10.: xxxx (ug/m3)|
      lcd.clear();
      lcd.setCursor(0, 0);
      printLinePme_1_0();
      lcd.setCursor(0, 1);
      printLinePme_2_5();
      lcd.setCursor(0, 2);
      printLinePme_10_0();
      clearLcdLine(3);
      
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_BME) {
      // |Temperature: xx.x 'C|
      // |Humidity:    xx.x  %|
      // |Pressure:   xxxx hPa|
      // |Altitude:   xxxx m. |
      
      char str_float[5];
      
      lcd.clear();
      lcd.setCursor(0, 0);
      printFloat(temperature, str_float);
      sprintf_P(lcd_line, PSTR("Temperature: %s \xDF""C"), (int)str_float);
      lcd.print(lcd_line);

      lcd.setCursor(0, 1);
      printFloat(humidity, str_float);
      sprintf_P(lcd_line, PSTR("Humidity:    %s  %%"), (int)str_float);
      lcd.print(lcd_line);
      
      lcd.setCursor(0, 2);
      sprintf_P(lcd_line, PSTR("Pressure:   %4d hPa"), (int)pressure);
      lcd.print(lcd_line);
      
      lcd.setCursor(0, 3);
      sprintf_P(lcd_line, PSTR("Altitude:   %4d m. "), (int)altitude);
      lcd.print(lcd_line);
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_AIR_QUALITY) {
      // |CO2: xxxx ppm       |
      // |Air quality:xxxx    |
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("CO2: "));
      print4digits(co2, ' ');
      lcd.print(F(" ppm"));
      
      lcd.setCursor(0, 1);
      lcd.print(F("Air quality:"));
      print4digits(mq135Value, ' ');

      clearLcdLine(2);
      clearLcdLine(3);
    }
  }
  
  int shakeValue = analogRead(PIN_SHAKE);
  int soundValue = analogRead(PIN_SOUND);
  int soundValueDigital = digitalRead(PIN_SOUND_DIGITAL);
  
  if(DEBUG) {
    Serial.print(F("Shake level: "));
    Serial.print(shakeValue);
    Serial.print(F("; Sound level: "));
    Serial.print(soundValue);
    Serial.print(F(" - "));
    Serial.println(soundValueDigital);
  }
  
  if((shakeValue > SHAKE_ALERT_LEVEL || soundValueDigital) && mode == MODE_SLEEP) {
    lcd.backlight();
    mode = MODE_ACTIVE;
    activeModeStage = millis();
  }

  delay(GLOBAL_DELAY);
}

void readSensors() {
  mhz19Serial.listen();
  co2 = mhz19.getCO2();
  mhz19Temperature = mhz19.getTemperature();
  
  lux = LightSensor.GetLightIntensity();
  mq135Value = analogRead(PIN_MQ_135);
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  pmsSerial.listen();
  if (pms.readUntil(data, 5000)){
    pms_1_0 = (int)data.PM_AE_UG_1_0;
    pms_2_5 = (int)data.PM_AE_UG_2_5;
    pms_10_0 = (int)data.PM_AE_UG_10_0;
  }
}

void printSerial() {
  if(DEBUG) {
    Serial.print(F("CO2: "));
    Serial.print(co2);
    Serial.println(F(" ppm"));
    Serial.print(F("MH-Z19B Temperature: "));
    Serial.print(mhz19Temperature);
    Serial.println(F("'C"));
    Serial.print(F("Light: "));
    Serial.print(lux);
    Serial.println(F(" lux"));
    Serial.print(F("Air quality = "));
    Serial.println(mq135Value);
    
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(F(" *C"));
    Serial.print(F("Humidity = "));
    Serial.print(humidity);
    Serial.println(F(" %"));
    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
    Serial.print(F("Approx. Altitude = "));
    Serial.print(altitude);
    Serial.println(F(" m."));
    Serial.println();
    
    Serial.print(F("PM 1.0 (ug/m3): "));
    Serial.println(pms_1_0);
    Serial.print(F("PM 2.5 (ug/m3): "));
    Serial.println(pms_2_5);
    Serial.print(F("PM 10.0 (ug/m3): "));
    Serial.println(pms_10_0);
  }
}

void printNormal() {
  lcd.setCursor(0, 0);
  printLine1();
  lcd.setCursor(0, 1);
  printLine2();
  lcd.setCursor(0, 2);
  printLinePme_2_5();
  clearLcdLine(3);
}

void printLine1() {
  sprintf_P(lcd_line, PSTR("%4dppm %4dlux %4d"), co2, lux, mq135Value);
  lcd.print(lcd_line);
}

void printLine2() {
  char str_temperature[5];
  char str_humidity[5];
  
  printFloat(temperature, str_temperature);
  printFloat(humidity, str_humidity);

  sprintf_P(lcd_line, PSTR("%s\xDF""C %s%% %4dhPa"), str_temperature, str_humidity, (int)pressure);
  lcd.print(lcd_line);
}

void printLinePme_1_0() {
  sprintf_P(lcd_line, PSTR("PM 1.0: %4d (ug/m3)"), pms_1_0);
  lcd.print(lcd_line);
}

void printLinePme_2_5() {
  sprintf_P(lcd_line, PSTR("PM 2.5: %4d (ug/m3)"), pms_2_5);
  lcd.print(lcd_line);
}

void printLinePme_10_0() {
  sprintf_P(lcd_line, PSTR("PM 10.: %4d (ug/m3)"), pms_10_0);
  lcd.print(lcd_line);
}

void print4digits(int value, char space) {
  if (value < 0) {
    lcd.print(F("----"));
  } else if (value >= 0 && value <= 9) {
    lcd.print(space);
    lcd.print(space);
    lcd.print(space);
    lcd.print(value);
  } else if(value >= 10 && value <= 99) {
    lcd.print(space);
    lcd.print(space);
    lcd.print(value);
  } else if(value >= 100 && value <= 999) {
    lcd.print(space);
    lcd.print(value);
  } else if(value >= 1000 && value <= 9999) {
    lcd.print(value);
  } else if(value >= 10000) {
    lcd.print(F("++++"));
  }
}

void clearLcdLine(int line) {
  lcd.setCursor(0,line);
  for(int n = 0; n < 20; n++){
     lcd.print(F(" "));
  }
}
