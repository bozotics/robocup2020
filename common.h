#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <EEPROM.h>

#define whitebot
//#define blackbot

HardwareSerial Serial2(PA3, PA2);

void serialWrite(unsigned char type, char *value);

void serialWrite(unsigned char type, char *value)
{
  char s[20];
  sprintf(s, "%c%s|", type, value);
  Serial1.print(s);
#ifdef DEBUG
  Serial2.println(s);
#endif
}

//compass.h
#define BNO055_SAMPLERATE_DELAY_MS (15)
float offsetYaw = 0;
uint8_t magCalib = 0;

//math.h
float mod(float x, float y) {
  x = fmod(x,y);
  return x < 0 ? x+y : x;
}
