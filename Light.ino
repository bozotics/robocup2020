void processLight(byte *data)
{
#ifdef DEBUG
	Serial.println("Processing light data...");
#endif
	for (int i = 0; i < 40; i++)
	{
		light[i] = (data[i / 8] >> (i % 8) & 0x01);
#ifdef DEBUG
		if (light[i])
		{
			Serial.print(i);
			Serial.print("\t");
		}
#endif
	}
#ifdef DEBUG
	Serial.println("");
#endif
}

void recvCalib()
{
	static bool recvInProgress = false;
	static int ndx = 0, i = 0;
	const byte endMarker = '|', startMarker = 'N', separator = ',', separator2 = '/'; // format N/0,300/1,200/2,900/.../39,200|
	byte rc;
	while (l1Serial.available())
	{
		rc = l1Serial.read();
		if (recvInProgress == true)
		{
			if (rc == separator2 && !i)
				;
			if (rc == separator2 && i)
			{
				receivedChars[ndx] = '\0'; // terminate the string
				Serial.print("No. ");
				Serial.print(i);
				Serial.print("\tThres:");
				Serial.print(fast_atoi(receivedChars));
				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
			}
			else if (rc == separator)
			{
				receivedChars[ndx] = '\0'; // terminate the string
#ifdef DEBUG
				if (fast_atoi(receivedChars) - i > 1 && fast_atoi(receivedChars) != 1)
					Serial.println("Missed data during LS calibration");
#endif
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
				Serial.print(fast_atoi(receivedChars));
				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
				i = 0;
				recvInProgress = false;
			}
			else
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
		}
		else if (rc == startMarker)
			recvInProgress = true;
	}
}