int fast_atoi(byte *str)
{
	int val = 0;
	while (*str)
	{
		val = val * 10 + (*str++ - '0');
	}
	return val;
}

void recv(HardwareSerial &_serial)
{
	static boolean recvInProgress = false;
	static int ndx = 0;
	static byte rc, startPos;
	const char endMarker = '|', separator = ',', separator2 = '/';
	const char startMarker[] = {'L', 'S', 'P', 'N', 'M', 'C'}; // {light, current sense, temp, motor switch, mouse, compass}
	while (_serial.available() > 0)
	{
		rc = _serial.read();
		if (recvInProgress == true)
		{
			if (rc != endMarker || (startPos == 'L' && ndx < 5))
			{
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= sizeof(receivedChars))
				{
#ifdef DEBUG
					Serial.println("Too many chars!");
#endif
					ndx = sizeof(receivedChars) - 1;
				}
			}
			else
			{
				receivedChars[ndx] = '\0'; // terminate the string
#ifdef SERIAL_DEBUG
				char s[32];
				sprintf(s, "%s", receivedChars);
				Serial.println(s);
#endif
				recvInProgress = false;
				ndx = 0;
				switch (startPos)
				{
				case 'L':
					processLight(receivedChars);
#ifdef SERIAL_DEBUG
					Serial.println("Light data received");
#endif
					break;

				case 'S':
					break;

				case 'P':
					break;

				case 'N':
					if (fast_atoi(receivedChars))
					{
						motorOn = true;
#ifdef DEBUG
						Serial.println("Motor switch on");
#endif
					}
					else
					{
						motorOn = false;
#ifdef DEBUG
						Serial.println("Motor switch off");
#endif
					}

					break;

				case 'M':
					break;

				case 'C':
					readIMU(receivedChars);
					break;

				default:
					break;
				}
			}
		}
		else
		{
			for (unsigned int i = 0; i < sizeof(startMarker); i++)
			{
				if (rc == startMarker[i])
				{
					recvInProgress = true;
					startPos = rc;
					memset(receivedChars, 0, sizeof(receivedChars));
				}
			}
		}
	}
}

void serialWrite(HardwareSerial &_serial, byte type, byte *value)
{
	char s[20];
	sprintf(s, "%c%s|", type, value);
	_serial.print(s);
#ifdef DEBUG
	Serial.println(s);
#endif
}

void serialWrite(HardwareSerial &_serial, char type, char value)
{
	char s[20];
	sprintf(s, "%c%c|", type, value);
	_serial.print(s);
#ifdef DEBUG
	Serial.println(s);
#endif
}

void serialWrite(HardwareSerial &_serial, char type)
{
	_serial.print(type);
#ifdef DEBUG
	Serial.println(type);
#endif
}