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

char buf[255];
int cnt=0;
unsigned long lastBallTime = millis();

void loop()
{
	//recv(L1Serial);
	//recv(L3Serial);
	L4Serial.write('1');
	delay(1000);
	L4Serial.write('2');
	delay(10000);
	//piRecv();
	//calcRobotAngle();
	angular_drive(0, 0, 0);
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
#ifdef DEBUG
		Serial.println("Running LS Cal...");
#endif
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
