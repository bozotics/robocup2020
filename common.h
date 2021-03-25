#include <ResponsiveAnalogRead.h>
#include <Bounce2.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#define DEBUG
//#define SERIAL_DEBUG

#define DIP1p 24
#define DIP2p 25
#define DIP3p 26
#define DIP4p 27

#define PWMM1 38  //FL
#define PWMM2 35  //FR
#define PWMM3 37  //BL
#define PWMM4 36  //BR
#define INM1 14
#define INM2 12
#define INM3 13
#define INM4 11

#define L3Serial Serial1
#define L1Serial Serial5
#define piSerial Serial4
#define btSerial Serial3
#define L4Serial Serial2

#define lightInt 39
#define battLvl A1

#define ballDist ballPos[0]
#define ballAng  ballPos[1]
#define predDist ballPos[2]
#define predAng  ballPos[3]

#define blueDist goalPos[0]
#define blueAng  goalPos[1]
#define yellowDist  goalPos[2]
#define yellowAng   goalPos[3]

#define pi 3.1415926535897932384626433832795
#define e  2.7182818284590452353602874713527

ResponsiveAnalogRead batt(battLvl, false);
Bounce DIP1 = Bounce();
Bounce DIP2 = Bounce();
Bounce DIP3 = Bounce();
Bounce DIP4 = Bounce();

elapsedMillis battTime = 0, DIPTime = 0, sTimeout = 0;
bool light[40] = {0}, motorOn = false;
int lightThres[40], lightMax[40] = {0}, lightMin[40] /* = max */, lightTemp;

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

//IMU.h
float cmpangle;
void readIMU(byte *data);

//camera.h
int ballPos[4] = {0,0,0,0};
int goalPos[4] = {0,0,0,0};

//movement.h
int FLout=0, FRout=0, BLout=0, BRout=0;
float robotAng=0;