#include "common.h"
//#define DEBUG

Adafruit_BNO055 bno = Adafruit_BNO055(55,BNO055_ADDRESS_B);

#define BNO055_SAMPLERATE_DELAY_MS 10

unsigned long readTime = millis();

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
  Serial2.begin(115200);
#ifdef DEBUG
	Serial2.println("DEBUG");
#endif
	Serial1.begin(250000);

   if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
     Serial2.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
     while (1);
   }
  delay(1000);
  
  bno.setExtCrystalUse(false);

  digitalWrite(LED_BUILTIN, LOW);
//   while(magCalib < 2) { //when magnetometer not yet calibrated led will be off
//     if (millis()-readTime > BNO055_SAMPLERATE_DELAY_MS) {
//       sensors_event_t event;
//       bno.getEvent(&event);

// #ifdef DEBUG
//       Serial2.print("X: ");
//       Serial2.print(event.orientation.x, 4);
//       Serial2.print("\tY: ");
//       Serial2.print(event.orientation.y, 4);
//       Serial2.print("\tZ: ");
//       Serial2.print(event.orientation.z, 4);
// #endif

//       displayCalStatus();
//       Serial2.println("");
//     }
//   }
  /*int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  bno.getSensor(&sensor);
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);
#ifdef DEBUG  
  displaySensorOffsets(calibrationData);
  Serial2.println("\n\nRestoring Calibration data to the BNO055...");
#endif

  bno.setSensorOffsets(calibrationData);

#ifdef DEBUG
  Serial2.println("\n\nCalibration data loaded into BNO055");
  displaySensorDetails();
  displaySensorStatus();
#endif

  bno.setExtCrystalUse(true);
  sensors_event_t event;
  bno.getEvent(&event);

#ifdef DEBUG
  Serial2.println("Move sensor slightly to calibrate magnetometers");
#endif

  unsigned long timer = millis();
  while (!bno.isFullyCalibrated()) { //wait till magnetometer calibrated OR if 7 seconds have passed
      bno.getEvent(&event);
#ifdef DEBUG
      displayCalStatus();
      Serial2.print("X: ");
      Serial2.print(event.orientation.x, 4);
      Serial2.print("\tY: ");
      Serial2.print(event.orientation.y, 4);
      Serial2.print("\tZ: ");
      Serial2.print(event.orientation.z, 4);
      Serial2.println();
#endif
      delay(BNO055_SAMPLERATE_DELAY_MS);
      if (millis()-timer > 20000) break;
  }

  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
#ifdef DEBUG
  Serial2.println("\nFully calibrated!");
  Serial2.println("--------------------------------");
  Serial2.println("Calibration Results: ");
  displaySensorOffsets(newCalib);
  Serial2.println("\n\nStoring calibration data to EEPROM...");
#endif

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;
  EEPROM.put(eeAddress, bnoID);
  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);

#ifdef DEBUG
  Serial2.println("Data stored to EEPROM.");
  Serial2.println("\n--------------------------------\n");
#endif
*/
  unsigned long blinkTimer = millis();
  while (millis() - blinkTimer < 500) {  //rapid blinking indicator means time to plonk bot flat
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
  //take average of 10 current readings as offset yaw angle
  for (int i=0; i<10; i++) {  
    sensors_event_t event;
    bno.getEvent(&event);
    offsetYaw += event.orientation.x;
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  offsetYaw /= 10.0;
  Serial2.println(offsetYaw);
  digitalWrite(LED_BUILTIN,LOW);
}

void loop() {
  if (millis()-readTime > BNO055_SAMPLERATE_DELAY_MS) {
    sensors_event_t event;
    bno.getEvent(&event);

#ifdef DEBUG
    Serial2.print("X: ");
    Serial2.print(mod(event.orientation.x-offsetYaw,360), 4);
    Serial2.print("\tY: ");
    Serial2.print(event.orientation.y, 4);
    Serial2.print("\tZ: ");
    Serial2.print(event.orientation.z, 4);
    displayCalStatus();
    //adafruit_bno055_offsets_t newCalib;
    //bno.getSensorOffsets(newCalib);
    //displaySensorOffsets(newCalib);
    Serial2.println("");
#endif

    float yaw = mod(event.orientation.x-offsetYaw,360);
    char s[20];
    dtostrf(yaw, 4, 2, s);
    serialWrite('C', s);
    //Serial2.println(s);
    readTime = millis();
  }

}
