#include "common.h"

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

	motorSw.attach(motorSwp, INPUT);
	motorSw.interval(50);

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
	motorSw.update();
	serialRead();
	updateLight();
	if (!lightCnt)
		serialWrite('L', processLight());
	if (motorSw.rose() || digitalReadFast(motorSwp))
	{
		serialWrite('N', 1);
		Serial2.println("motor on");
	}
	if (motorSw.fell() || !digitalReadFast(motorSwp))
	{
		serialWrite('N', (byte)0);
		Serial2.println("motor off");
	}
	if (kick && (millis() - kickTimer) > 30)
	{
#ifdef SERIAL_DEBUG
		Serial2.println("Stop Kick");
#endif
		digitalWriteFast(solenoid, 0);
		kick = false;
	}
}