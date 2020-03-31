#include "Config.h"

void setup()
{
	l3Serial.setTX(1);
	l3Serial.setRX(0);
#ifdef DEBUG
	Serial.begin(115200);
#endif
	l1Serial.begin(250000);
	l3Serial.begin(250000);
	piSerial.begin(500000);

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

	while(l1Serial.available())
		char t = l1Serial.read();
}

void loop()
{
	recv(l1Serial);

	analogWrite(PWMM1, 0);
	analogWrite(PWMM2, 0);
	analogWrite(PWMM3, 0);
	analogWrite(PWMM4, 0);

#ifdef DEBUG
	if (battTime >= 1000)
	{
		batt.update();
		if (batt.hasChanged())
		{
			Serial.print("Battery Level: \t");
			Serial.println((float)(batt.getValue()/77.5));
		}
		battTime = 0;
	}
#endif
	DIP1.update();
	DIP2.update();
	DIP3.update();
	DIP4.update();

	DIPTime = 0;

	if (DIP1.fell())
	{
#ifdef DEBUG
		Serial.println("Running LS Cal...");
#endif
		serialWrite(l1Serial, 'N');
		for (int i = 0; i < 40; i++)
		{
			lightMin[i] = 1200;
			lightMax[i] = 0;
		}

		while (!DIP1.rose())
		{
			recvCalib();
			DIP1.update();
#ifdef DEBUG
			//Serial.println(sTimeout);
			if (sTimeout > 500)
			{
				for (int i = 0; i < 40; i++)
				{
					Serial.print("No:  ");
					Serial.print(i);
					Serial.print("\tMin:  ");
					Serial.print(lightMin[i]);
					Serial.print("\tMax:  ");
					Serial.println(lightMax[i]);
				}
				sTimeout = 0;
			}
#endif
		}

		serialWrite(l1Serial, 'L');
		sendCalib();
	}
	//delay(100);
}
