#include <NXPMotionSense_ada.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <EEPROM.h>

NXPMotionSense imu;
Madgwick filter;

HardwareSerial Serial2(PA3, PA2);
void serialWrite(unsigned char type, char *value);