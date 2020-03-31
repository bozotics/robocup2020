#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>

#define DEBUG

#define SIG_0 PA_15
#define SIG_1 PB_3
#define SIG_2 PB_4
#define SIG_3 PB_5
#define ALOG_1 PB0
#define ALOG_2 PA1
#define ALOG_3 PA0
#define lightInt PA_11

#define solenoid PA_8
#define motorSw PC_13

#define mouseRst PB_9
#define mouseMot PB_8
#define mouseSS PA4

HardwareSerial Serial2(PA_3, PA_2);

const int sigTable[16][4] = {
	{0, 0, 0, 0}, //0
	{1, 0, 0, 0}, //1
	{0, 1, 0, 0}, //2
	{1, 1, 0, 0}, //3
	{0, 0, 1, 0}, //4
	{1, 0, 1, 0}, //5
	{0, 1, 1, 0}, //6
	{1, 1, 1, 0}, //7
	{0, 0, 0, 1}, //8
	{1, 0, 0, 1}, //9
	{0, 1, 0, 1}, //10
	{1, 1, 0, 1}, //11
	{0, 0, 1, 1}, //12
	{1, 0, 1, 1}, //13
	{0, 1, 1, 1}, //14
	{1, 1, 1, 1}  //15
};

int lightThres[40] = {0};
int lightCnt = 0, lightVals[40], lightDetect[40], thermVals[4], csVals[4];
unsigned long lightTimer, kickTimer, mouseTimer, debugMil;
bool kick = false, mouse = false, cal = false;

//Light.h
void updateLight();
byte *processLight();
void lightCal();
void recvCalib();

//Serial.h
int fast_atoi(unsigned char *str);
void serialRead();
void serialWrite(byte type, byte *value);

//Mouse.h
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65