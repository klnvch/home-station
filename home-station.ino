#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <Arduino.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <BH1750FVI.h>

////////////////////
//xxxxppm xxxlx xx//
//xx°C xx% xxxxhPa//
////////////////////
// VCC -> +5V
// GND -> GND
// SDA -> A4
// SCL -> A5
LiquidCrystal_I2C lcd(0x27,16,2);


// Vin -> +5V
// GND -> GND
// Rx  -> D3
// Tx  -> D2
#define RX_PIN 2                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 3                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial


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
#define ACTIVE_MODE_STAGE_TEMPERATURE 7000
#define ACTIVE_MODE_STAGE_PRESSURE 9000
#define ACTIVE_MODE_STAGE_NORMAL 11000
#define ACTIVE_MODE_STAGE_STOP 20000
unsigned long activeModeStage;

#define LCD_UPDATE_INTERVAL_MILLIS 10000
unsigned long lastLcdUpdateTime = 0;

int co2 = 0;
int mq135Value = 0;
float temperature = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;

/*
 * To fix:
 * 1. Add shake sensor
 * 2. Screen blinking when no enough power
 * 3. Remove artifacts at the end of the lines, when active mode is on
 * 4. Do not update screen on the last stage
 * 5. Add calibrate button
 * 6. Remove serial
 * 7. Update BME library
 * 8. Check initial delay
 * 9. Exclude Serial in Release mode
 * 10. Solder CO2 sensor
 */
void setup() {
  Serial.begin(9600);
  while(!Serial);
    
  lcd.init();
  lcd.noBacklight();


  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin().
  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))


  unsigned status;
    
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin();  
  if (!status) {
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

/*

*/
void loop() {
  unsigned long currentTime = millis();
  if(mode == MODE_SLEEP) {
    if (currentTime - lastLcdUpdateTime > LCD_UPDATE_INTERVAL_MILLIS) {
      lastLcdUpdateTime = currentTime;
      
      printNormal();
    }
  } else {
    if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_STOP) {
      lcd.noBacklight();
      mode = MODE_SLEEP;
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_NORMAL) {
      printNormal();
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_PRESSURE) {
      // |Pressure:xxxxhPa|
      // |Altitude:xxxx m.|
      
      lcd.setCursor(0, 0);
      lcd.print(F("Pressure:"));
      print4digits((int)pressure, ' ');
      lcd.print(F("hPa"));
      
      lcd.setCursor(0, 1);
      lcd.print(F("Altitude:"));
      print4digits((int)altitude, ' ');
      lcd.print(F(" m."));
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_TEMPERATURE) {
      // |Temp.: xx.xxxx'C|
      // |Hmdt.: xx.xxxx %|
      
      lcd.setCursor(0, 0);
      lcd.print(F("Temp.: "));
      printFloatNumber(temperature);
      lcd.print((char)223);
      lcd.print(F("C"));
      
      lcd.setCursor(0, 1);
      lcd.print(F("Hmdt.: "));
      printFloatNumber(humidity);
      lcd.print(F(" %"));
    } else if (currentTime - activeModeStage > ACTIVE_MODE_STAGE_AIR_QUALITY) {
      // |CO2: xxxx ppm   |
      // |Air quality:xxxx|
      
      lcd.setCursor(0, 0);
      lcd.print(F("CO2: "));
      print4digits(co2, ' ');
      lcd.print(F(" ppm"));
      
      lcd.setCursor(0, 1);
      lcd.print(F("Air quality:"));
      print4digits(mq135Value, ' ');
    }
  }
  
  int shakeValue = analogRead(PIN_SHAKE);
  int soundValue = analogRead(PIN_SOUND);
  int soundValueDigital = digitalRead(PIN_SOUND_DIGITAL);
  
  if(DEBUG) {
    Serial.print(F("Shake level: "));
    Serial.println(shakeValue);
    Serial.print(F("Sound level: "));
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

void printNormal() {
  lcd.setCursor(0, 0);
  int l1 = printMhZ19bValues();
  int l2 = printBh1750Values();
  printMQ135Values(16 - l1 - l2);
  
  lcd.setCursor(0, 1);
  printBmeValues();
}

int printMhZ19bValues() {
  co2 = myMHZ19.getCO2();
  lcd.print(co2);
  lcd.print(F("ppm "));

  int temp = myMHZ19.getTemperature();
  
  if(DEBUG) {
    Serial.print(F("CO2: "));
    Serial.print(co2);
    Serial.println(F(" ppm"));
    Serial.print(F("MH-Z19B Temperature: "));
    Serial.print(temp);
    Serial.println(F("'C"));
  }
  
  return countDigits(co2) + 4;
}

int printBh1750Values() {
  int lux = LightSensor.GetLightIntensity();
  lcd.print(lux);
  lcd.print(F("lx "));
  
  if(DEBUG) {
    Serial.print(F("Light: "));
    Serial.print(lux);
    Serial.println(F(" lux"));
  }
  
  return countDigits(lux) + 3;
}

void printBmeValues() {
  temperature = bme.readTemperature();
  print2digits((int)temperature, ' ');
  lcd.print((char)223);
  lcd.print(F("C "));

  humidity = bme.readHumidity();
  print2digits((int)humidity, ' ');
  lcd.print(F("% "));

  pressure = bme.readPressure() / 100.0F;
  print4digits((int)pressure, ' ');
  lcd.print(F("hPa"));
  
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  if(DEBUG) {
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
  }
}

void printMQ135Values(int remainingSpace) {
  mq135Value = analogRead(PIN_MQ_135);
  
  if(remainingSpace == 1) {
    mq135Value = map(mq135Value, 0, 1023, 0, 9);
    lcd.print(mq135Value);
  } else if(remainingSpace == 2) {
    mq135Value = map(mq135Value, 0, 1023, 0, 99);
    print2digits(mq135Value, '0');
  } else if(remainingSpace == 3) {
    mq135Value = map(mq135Value, 0, 1023, 0, 999);
    print3digits(mq135Value, '0');
  } else if(remainingSpace >= 4) {
    lcd.setCursor(12, 0);
    print4digits(mq135Value, '0');
  }
  
  if(DEBUG) {
    Serial.print(F("Air quality = "));
    Serial.println(mq135Value);
  }
}

void print2digits(int value, char space) {
  if (value <= -10) {
    lcd.print(F("--"));
  } else if(value >= -9 && value <= -1) {
    lcd.print(value);
  } else if (value >= 0 && value <= 9) {
    lcd.print(space);
    lcd.print(value);
  } else if(value >= 10 && value <= 99) {
    lcd.print(value);
  } else if(value >= 100) {
    lcd.print(F("++"));
  }
}

void print3digits(int value, char space) {
  if (value < 0) {
    lcd.print(F("---"));
  } else if (value >= 0 && value <= 9) {
    lcd.print(space);
    lcd.print(space);
    lcd.print(value);
  } else if(value >= 10 && value <= 99) {
    lcd.print(space);
    lcd.print(value);
  } else if(value >= 100 && value <= 999) {
    lcd.print(value);
  } else if(value >= 10000) {
    lcd.print(F("+++"));
  }
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

void printFloatNumber(float value) {
  int i = (int) value;
  int p = (int) ((value - i) * 10000);
  print2digits(value, ' ');
  lcd.print('.');
  lcd.print(p);
}

int countDigits(int value) {
  int count = 0;
  while (value != 0) {
    value /= 10;
    ++count;
  }
  return count;
}
