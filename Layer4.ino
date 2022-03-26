#include "common.h"
#include <Servo.h>
#include <Wire.h>
#include <VL53L1X.h>

Servo frontDrib;
Servo backDrib;

VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;
VL53L1X sensor4;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  Serial2.begin(115200);
#ifdef DEBUG
  Serial2.println("DEBUG");
#endif
  Serial1.begin(250000);
  frontDrib.attach(ESC_1);
  frontDrib.writeMicroseconds(700);
  backDrib.attach(ESC_2);
  backDrib.writeMicroseconds(700);
  delay(1000);

  //init tof
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  pinMode(XSHUT4, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  delay(500);
  Wire.begin();
  Wire.beginTransmission(0x29);

  digitalWrite(XSHUT2, HIGH);
  delay(150);
  sensor2.init();
  Serial2.println("01");
  delay(100);
  sensor2.setAddress(0x31);
  Serial2.println("02");

  digitalWrite(XSHUT3, HIGH);
  delay(150);
  sensor3.init();
  Serial2.println("03");
  delay(100);
  sensor3.setAddress(0x33);
  Serial2.println("04");

  digitalWrite(XSHUT4, HIGH);
  delay(150);
  sensor4.init();
  Serial2.println("05");
  delay(100);
  sensor4.setAddress(0x35);
  Serial2.println("06");

  digitalWrite(XSHUT1, HIGH);
  delay(150);
  Serial2.println("07");
  sensor1.init();
  Serial2.println("08");
  delay(100);

#ifdef whitebot
  sensor1.setDistanceMode(VL53L1X::Short);
  sensor1.setMeasurementTimingBudget(20000);
  sensor1.startContinuous(20);
  sensor1.setTimeout(40);
#endif
#ifdef blackbot
  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(50000);
  sensor1.startContinuous(50);
  sensor1.setTimeout(100);
#endif

  sensor2.setDistanceMode(VL53L1X::Short);
  sensor2.setMeasurementTimingBudget(20000);
  sensor2.startContinuous(20);
  sensor2.setTimeout(40);

  sensor3.setDistanceMode(VL53L1X::Short);
  sensor3.setMeasurementTimingBudget(20000);
  sensor3.startContinuous(20);
  sensor3.setTimeout(40);

  sensor4.setDistanceMode(VL53L1X::Short);
  sensor4.setMeasurementTimingBudget(20000);
  sensor4.startContinuous(20);
  sensor4.setTimeout(40);

  delay(150);
  Serial2.println("addresses set");

  digitalWrite(LED_BUILTIN, 0);
}

char buf;
int cnt = 0, tof[4];

void loop() {
  if (Serial1.available()) {
    buf = Serial.read();
    if (buf == '1') {	//on front
      frontDrib.writeMicroseconds(2000);
      //Serial2.println("bruh");
    }
    else if (buf == '2') { //off front
      frontDrib.writeMicroseconds(700);
    } else if (buf == '3') {  //on back
      backDrib.writeMicroseconds(2000);
    } else if (buf == '4') { //off back
      backDrib.writeMicroseconds(700);
    } 
    else if (buf == '5') {
#ifdef whitebot
      frontDrib.writeMicroseconds(880);
#endif
#ifdef blackbot
      frontDrib.writeMicroseconds(1250);
#endif
    }
    
    else {
      frontDrib.writeMicroseconds(700);
      backDrib.writeMicroseconds(700);
    }
  }


  tof[0] = sensor1.read();
  tof[1] = sensor2.read();
  tof[2] = sensor3.read();
  tof[3] = sensor4.read();

  char s[30];
  sprintf(s, "T%d %d %d %d|", tof[0], tof[1], tof[2], tof[3]);
  Serial1.print(s);
#ifdef DEBUG
  Serial2.println(s);
#endif


  //  frontDrib.writeMicroseconds(875);
  //  delay(5000);
  //  frontDrib.writeMicroseconds(700);
  //  delay(5000);
}
