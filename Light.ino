void processLight(byte *data)
{
#ifdef SERIAL_DEBUG
	Serial.println("Processing light data...");
#endif
	for (int i = 0; i < 40; i++)
	{
		light[i] = (data[i / 8] >> (i % 8) & 0x01);
#ifdef SERIAL_DEBUG
		if (light[i])
		{
			Serial.print(i);
			Serial.print("\t");
		}
#endif
	}
#ifdef SERIAL_DEBUG
	Serial.println("");
#endif
}

bool recvCalib()
{
	static bool recvInProgress = false;
	static int ndx = 0, i = 0;
	const byte endMarker = '|', startMarker = 'L', separator = ',', separator2 = '/'; // format L/0,300/1,200/2,900/.../39,200|
	byte rc;
	while (l1Serial.available())
	{
		rc = l1Serial.read();
		if (recvInProgress == true)
		{
			if (rc == separator2 && !i)
				;
			else if (rc == separator2 && i)
			{
				receivedChars[ndx] = '\0'; // terminate the string
				Serial.print("No. ");
				Serial.print(i);
				Serial.print("\tThres:");
				Serial.println(fast_atoi(receivedChars));
				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
			}
			else if (rc == separator)
			{
				receivedChars[ndx] = '\0'; // terminate the string
				if (fast_atoi(receivedChars) - i > 1 && fast_atoi(receivedChars) != 1)
					Serial.println("Missed data during LS calibration");
				i = fast_atoi(receivedChars);
				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
			}
			else if (rc == endMarker)
			{
				receivedChars[ndx] = '\0'; // terminate the string
				Serial.print("No. ");
				Serial.print(i);
				Serial.print("\tThres:");
				Serial.println(fast_atoi(receivedChars));
				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
				i = 0;
				recvInProgress = false;
				return true;
			}
			else
			{
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= sizeof(receivedChars))
				{
					Serial.println("Too many chars!");
					ndx = sizeof(receivedChars) - 1;
				}
			}
		}
		else if (rc == startMarker)
			recvInProgress = true;
	}
	return false;
}