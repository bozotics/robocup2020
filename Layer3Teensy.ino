#include "common.h"

void setup()
{
	L3Serial.setTX(1);
	L3Serial.setRX(0);
#ifdef DEBUG
	Serial.begin(115200);
#endif
	L1Serial.begin(250000);
	L3Serial.begin(250000);
	L4Serial.begin(250000);
	piSerial.begin(1000000);

	DIP1.attach(DIP1p, INPUT);
	DIP2.attach(DIP2p, INPUT);
	DIP3.attach(DIP3p, INPUT);
	DIP4.attach(DIP4p, INPUT);
	DIP1.interval(50);
	DIP2.interval(50);
	DIP3.interval(50);
	DIP4.interval(50);

	pinMode(INM1, OUTPUT);
	pinMode(INM2, OUTPUT);
	pinMode(INM3, OUTPUT);
	pinMode(INM4, OUTPUT);
	digitalWriteFast(INM1, 0);
	digitalWriteFast(INM2, 1);
	digitalWriteFast(INM3, 1);
	digitalWriteFast(INM4, 0);
	analogWriteResolution(12);
	analogWriteFrequency(PWMM1, 14648.437);
	analogWriteFrequency(PWMM2, 14648.437);
	analogWriteFrequency(PWMM3, 14648.437);
	analogWriteFrequency(PWMM4, 14648.437);

	while (L1Serial.available())
		char t = L1Serial.read();
}

unsigned long lastBallTime = millis();

void loop()
{
	recv(L1Serial);
	//recv(L3Serial);;

	recv(piSerial);

	if (millis()-kickTimer < 2000) robotSpeed = 1600;
	else robotSpeed = 1200;
	
	//turn on dribbler if ball within +-60 deg of front AND closer than 40 cm
	if ((ballAng <= 60 || ballAng >= 300) && ballDist < 40 && millis()-dribblerTimer > 500 && millis()-lastBallTime < 1000) {
		if (!dribblerOn) {	//only send to turn on once
			serialWrite(L4Serial, '1');
			dribblerOn = true;
			//Serial.print(ballAng); Serial.print(";");
			//Serial.print(ballDist); Serial.println(";");
			Serial.println("dribbler");
		}
	}
	else {
		if (dribblerOn) {	// only send to turn off once
			serialWrite(L4Serial, '2');
			dribblerOn = false;
		}
		
	}

	//turn off dribbler + turn on kicker if facing blue goal and 50cm away
	//TO DO: CHANGE BALLDIST && BALLANG condition to light gate check
	if((blueAng < 30 || blueAng >= 330) && blueDist < 60 && ballDist < 25 && (ballAng <= 10 || ballAng >= 350) && millis()-lastBallTime < 1000) {
		if (millis()-kickTimer > 1500) {
			serialWrite(L4Serial, '2');
			serialWrite(L1Serial,'K');
			dribblerTimer = millis();
			kickTimer = millis();
			// Serial.print(blueAng); Serial.print(";");
			// Serial.print(blueDist); Serial.print(";");
			Serial.println("kicked");
		}
	}

	calcRobotAngle();
	//angular_drive(robotSpeed, 270, 0);
	lineAvoid();
	move_OUT();

#ifdef DEBUG
	if (battTime >= 1000)
	{
		batt.update();
		if (batt.hasChanged())
		{
			Serial.print("Battery Level: \t");
			Serial.println((float)(batt.getValue() / 77.5));
		}
		battTime = 0;
	}
#endif

	DIP1.update();
	DIP2.update();
	DIP3.update();
	DIP4.update();

	if (DIP1.fell())
	{
//#ifdef DEBUG
		Serial.println("Running LS Cal...");
//#endif
		serialWrite(L1Serial, 'N');
		while (!DIP1.rose())
		{
			recvCalib();
			DIP1.update();
		}
		serialWrite(L1Serial, 'L');
		while (!recvCalib())
			;
	}
}
