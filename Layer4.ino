#include "common.h"
#include <Servo.h>

Servo frontDrib;
Servo backDrib;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    #ifdef DEBUG
	    Serial2.begin(115200);
	    Serial2.println("DEBUG");
    #endif
    Serial1.begin(250000);
    frontDrib.attach(ESC_1);
    frontDrib.writeMicroseconds(700);
    backDrib.attach(ESC_2);
    backDrib.writeMicroseconds(700);
}

char buf;
int cnt = 0;

void loop() {
    if(Serial1.available()) {
        buf = Serial.read();
        if (buf=='1') {	//on front
            frontDrib.writeMicroseconds(2000);
        }
        else if (buf=='2') {  //off front
            frontDrib.writeMicroseconds(700);
        } else if (buf=='3') {    //on back
            backDrib.writeMicroseconds(2000);
        } else if (buf=='4') {  //off back
            backDrib.writeMicroseconds(700);
        } else {
            frontDrib.writeMicroseconds(700);
            backDrib.writeMicroseconds(700);
        }
    }
}