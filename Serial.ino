int fast_atoi(unsigned char *str)
{
	int val = 0;
	while (*str)
	{
		val = val * 10 + (*str++ - '0');
	}
	return val;
}

void serialRead()
{
	char rc;
	while (Serial1.available() > 0)
	{
		rc = Serial1.read();
		
		switch (rc)
		{
		case 'K':
#ifdef SERIAL_DEBUG
			Serial2.println("Kicking...");
#endif
			digitalWriteFast(solenoid, 1);
			if (!kick)
				kickTimer = millis();
			kick = true;
			break;

		case 'N':
#ifdef SERIAL_DEBUG
			Serial2.println("Starting light calibration...");
#endif
			lightCal();
			break;

		case '|':
#ifdef SERIAL_DEBUG
			Serial2.println("Useless delimeter here, fix code pls");
#endif
			break;

		default:
#ifdef DEBUG
			Serial2.println("Invalid Serial Input!");
#endif
			break;
		}
	}
}

void serialWrite(byte type, byte *value)
{
	//char s[20];
	//sprintf(s, "%c%s|", type, value);
	Serial1.write(type);
	for (int i = 0; i < 5; ++i)
		Serial1.write(value[i]);
	Serial1.write('|');
#ifdef SERIAL_DEBUG
	Serial2.write(type);
	for (int i = 0; i < 5; ++i)
		Serial2.print(value[i], BIN);
	//Serial2.write('|');
	Serial2.println("");
#endif
}

void serialWrite(byte type, byte value)
{
	//char s[20];
	//sprintf(s, "%c%s|", type, value);
	Serial1.write(type);
	Serial1.write(value);
	Serial1.write('|');
#ifdef SERIAL_DEBUG
	Serial2.write(type);
	Serial2.print(value);
	//Serial2.write('|');
	Serial2.println("");
#endif
}