#include <ResponsiveAnalogRead.h>
#include <Bounce2.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#define DEBUG

#define DIP1p 24
#define DIP2p 25
#define DIP3p 26
#define DIP4p 27

#define PWMM1 35
#define PWMM2 36
#define PWMM3 37
#define PWMM4 38
#define INM1 12
#define INM2 11
#define INM3 13
#define INM4 14

#define l3Serial Serial1
#define l1Serial Serial5
#define piSerial Serial4
#define btSerial Serial3
#define l4Serial Serial2

#define lightInt 39
#define battLvl A1

ResponsiveAnalogRead batt(battLvl, false);
Bounce DIP1 = Bounce();
Bounce DIP2 = Bounce();
Bounce DIP3 = Bounce();
Bounce DIP4 = Bounce();

elapsedMillis battTime = 0, DIPTime = 0, sTimeout = 0;
bool light[40] = {0}, motorOn = false;
int lightThres[40], lightMax[40] = {0}, lightMin[40] /* = max */ , lightTemp;

//Serial.h

int fast_atoi(byte *str);
void recv(HardwareSerial &_serial);
void serialWrite(HardwareSerial &_serial, byte type, byte *value);
void serialWrite(HardwareSerial &_serial, char type, char value);
void serialWrite(HardwareSerial &_serial, char type);

byte receivedChars[32];

//Light.h

void processLight(byte *data);
bool recvCalib();