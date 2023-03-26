#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define pump_pin 27

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int sensor_pin = 26;
int value;

void setup() {
  Serial.begin(9600);
  pinMode(pump_pin, OUTPUT);
  digitalWrite(pump_pin, LOW);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
    String values;
    value = analogRead(sensor_pin);
    value = map(value, 550, 0, 0, 100);
    values += String(bmp.readTemperature());
    values += String(bmp.readPressure());
    values += String(bmp.readAltitude(1013.25));
    values += String(value);
    Serial.println(values);
    delay(1000);
    /* Ralay and water pump control */
    if (abs(value) >= 330){
       digitalWrite(pump_pin, LOW);
       //Serial.println("Very Dry");
    } else if (abs(value) >= 270){
      //digitalWrite(pump_pin, LOW);
      //Serial.println("Dry");
    } else if (abs(value) >= 200){
      digitalWrite(pump_pin, HIGH);
      //Serial.println("Wet");
    } else if (abs(value) < 200){
      //digitalWrite(pump_pin, HIGH);
      //Serial.println("Very Wet");
    }
}
