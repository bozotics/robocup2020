#include <ResponsiveAnalogRead.h>
//#include "C:/Program Files (x86)/Arduino/hardware/teensy/avr/libraries/Bounce2/Bounce2.h"   //temp fix for bounce2 conflict between arduino and teensy lib
#include <Bounce2.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#define DEBUG
//#define SERIAL_DEBUG

//#define whitebot
#define blackbot

#define currGoalAng yellowAng
#define currGoalDist yellowDist
#define oppGoalAng blueAng
#define oppGoalDist blueDist

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

#define otherBallDist     otherBotData[0]
#define otherBallAng      otherBotData[1]
#define otherOppGoalDist  otherBotData[2]
#define otherOppGoalAng   otherBotData[3]

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

bool motorOn = false;

elapsedMillis battTime = 0, DIPTime = 0, sTimeout = 0;

//Serial.h
int fast_atoi(byte *str);
void recv(HardwareSerial &_serial, int serNum);
void serialWrite(HardwareSerial &_serial, byte type, byte *value);
void serialWrite(HardwareSerial &_serial, char type, char value);
void serialWrite(HardwareSerial &_serial, char type);
void recvAll();
byte receivedChars[5][50];  //[serial type(0=BT,1=Layer1,2=Pi,3=Layer3, 4=Layer4)][received values]
int ndx[5] = {0,0,0,0,0};

//Light.h
int lightThres[40], lightMax[40] = {0}, lightMin[40] /* = max */, lightTemp;
int clusterStart, clusterEnd;
bool newLightData = false;
float line[40] = {0}, lineAng, prevLineAng, lineLen;
bool onLine=false, prevLine=false;
int outBallAng, outBallDist, lineOut=0, lineStop=0;
unsigned long lastOutTime=0, prevLastOutTime=0, lastInTime=0;

void processLight(byte *data);
bool recvCalib();

//IMU.h
float cmpangle;
float moddedAngle;
float error, prevError;
float cmpCorrection;
unsigned long lastCmpTimer = 0;
void readIMU(byte *data);

//camera.h
int ballPos[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; //EXTRA 6 array slots: temp fix for other data being sent at same time as ball data
int goalPos[12] = {0,0,0,0,0,0,0,0,0,0,0,0};          //^^
unsigned long lastBallTime, lastGoalTime;
bool newBallData = false, newGoalData = false;
float ballError, prevBallError, ballCorrection;

//tof.h
int tof[10]= {0,0,0,0,0,0,0,0,0,0}; //EXTRA array slots because ^^
int prevtof[4] = {0,0,0,0};
int tofFlag[4] = {0,0,0,0}; //flags for when tof detects weird values
unsigned long flagTimer[4] = {0,0,0,0};
int tofOut[4] = {0,0,0,0};  //flags for when tof detects low values when line detected
int tofFlagCnt=0, tofOutCnt = 0;
const int TOF_THRESH[4] = {400,400,400,400};
bool newTOFdata = false;

float tofCentreVectX, tofCentreVectY; //XY vectors needed to reach centre, +X is right, +Y is forwards
float tofCentreAng, tofCentreDist;
#define CENTRE_DIST_X 1000
#define CENTRE_DIST_Y 1000
#define FIELD_X 1600  //varies from 1200 to 1800 on field fking cancer sia
#define FIELD_Y 2000  //varies from 1300 to 2400 on field 

//bluetooth.h
int otherBotData[10] = {0,0,0,0,0,0,0,0,0,0}; //EXTRA array slots because ^^
unsigned long lastBTsendTime, lastBTrecvTime, lastBTswitchTime;

//movement.h
int FLout=0, FRout=0, BLout=0, BRout=0;
int robotSpeed=1200, minSpeed=200, slowSpeed=750; //robotSpeed is normal speed, minSpeed is speed before all motors stall, slowSpeed is slower speed for robot movmement
float goalieCorrection;
float angleDeviation=0, prevAngleDeviation; 
float centreAng, centreDist, ABangle, ABdist;
float robotAng=0;
bool stopMove = false;
//int attackState = 0;
// #ifdef whitebot
//   attackState = 1;
// #endif
// #ifdef blackbot
//   attackState = 0;
// #endif
void angular_drive(float speed, float angle, float angVel, float angSpeed = -1.0);  //must declare function here because of default params

//solenoid.h and dribbler.h
unsigned long kickTimer=-9999, frontDribblerTimer=0, backDribblerTimer=0;
bool frontDribblerOn = false, backDribblerOn = false;

bool backGate=false, frontGate=false, prevFrontGate = false;
unsigned long backTimer=0, lastFrontTime=0, firstFrontTime=0;

//math.h
float mod(float x, float y) {
  x = fmod(x,y);
  return x < 0 ? x+y : x;
}

float angleBetween(float x, float y) {
  return mod((y-x),360);
}

float smallestAngleBetween(float x, float y) {
  float ang = angleBetween(x,y);
  return fmin(ang,360-ang);
}

float midAngleBetween(float x, float y) {
  return mod(x + angleBetween(x,y)/2.0, 360);
}

float degToRad(float angle) {
  return angle*PI/180.0;
}

float radToDeg(float angle) {
  return angle*180.0/PI;
}