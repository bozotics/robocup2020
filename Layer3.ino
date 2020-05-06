#include "common.h"
#define DEBUG

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, 1);
#ifdef DEBUG
	Serial2.begin(115200);
	Serial2.println("DEBUG");
#endif
	Serial1.begin(250000);
	imu.begin();
	filter.begin(100); // 100 measurements per second
}
unsigned long time;
void loop()
{
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
	float roll, pitch, heading;

	if (imu.available())
	{
		// Read the motion sensors
		imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

		// Update the Mahony filter
		filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

		heading = filter.getYaw();
#ifdef DEBUG
		Serial2.print(heading);
#endif
		char s[20];
		dtostrf(heading, 4, 2, s);
		serialWrite('C', s);
		Serial2.println(micros() - time);
		time = micros();
	}
}

void serialWrite(unsigned char type, char *value)
{
	char s[20];
	sprintf(s, "%c%s|", type, value);
	Serial1.print(s);
#ifdef DEBUG
	Serial2.println(s);
#endif
}