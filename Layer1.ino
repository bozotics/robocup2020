#include "Config.h"

void setup()
{
#ifdef DEBUG
	Serial2.begin(115200);
	Serial2.println("DEBUG");
#endif
	Serial1.begin(250000);

	SPI.setMOSI(PA_7);
	SPI.setMISO(PA_6);
	SPI.setSCLK(PA_5);
	SPI.beginTransaction(mouseSS, SPISettings(2000000, MSBFIRST, SPI_MODE3));

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, 1);

	pinMode(pinNametoDigitalPin(solenoid), OUTPUT);
	digitalWriteFast(solenoid, 0);

	pinMode(pinNametoDigitalPin(SIG_0), OUTPUT);
	pinMode(pinNametoDigitalPin(SIG_1), OUTPUT);
	pinMode(pinNametoDigitalPin(SIG_2), OUTPUT);
	pinMode(pinNametoDigitalPin(SIG_3), OUTPUT);
	digitalWriteFast(SIG_0, 0);
	digitalWriteFast(SIG_1, 0);
	digitalWriteFast(SIG_2, 0);
	digitalWriteFast(SIG_3, 0);

	eeprom_buffer_fill();
#ifdef DEBUG
		Serial2.println("Thres loading");
#endif
	for (int i = 1; i <= 40; i++)
	{
		unsigned char Char1; // lower byte
		unsigned char Char2; // upper byte
		Char1 = eeprom_buffered_read_byte(i);
		Char2 = eeprom_buffered_read_byte(i + 40);
		lightThres[i - 1] = (Char2 << 8) | Char1; // merge two char into short
#ifdef DEBUG
		Serial2.print("No:  ");
		Serial2.print(i);
		Serial2.print("\tThres:  ");
		Serial2.println(lightThres[i - 1]);
#endif
	}

	lightTimer = micros();
	debugMil = millis();
}
void loop()
{
	serialRead();
	updateLight();
	if (lightCnt == 0 && (millis() - debugMil) > 1000)
	{
		serialWrite('L', processLight());
		debugMil = millis();
	}
	if (kick && (millis() - kickTimer) > 30)
	{
#ifdef DEBUG
		Serial2.println("Stop Kick");
#endif
		digitalWriteFast(solenoid, 0);
		kick = false;
	}

	/*
	for (int i = 0; i < 16; i++) {
		digitalWriteFast(S0, sigTable[i][0]);
    	digitalWriteFast(S1, sigTable[i][1]);
    	digitalWriteFast(S2, sigTable[i][2]);
    	digitalWriteFast(S3, sigTable[i][3]);
		delayMicroseconds(100);
		readTable[i][0] = analogRead(LSOUT1);
		readTable[i][1] = analogRead(LSOUT2);
		readTable[i][2] = analogRead(LSOUT3);
	}
	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 16; i++) {
			Serial2.print(readTable[i][j]);
			Serial2.print("\t");
		}
		Serial2.println("");
	}
	Serial2.println("");Serial2.println("");
	delay(1000);
	*/
}