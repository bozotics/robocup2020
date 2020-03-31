void updateLight()
{
	if (micros() - lightTimer > 70)
	{
		lightVals[20 - lightCnt] = analogRead(ALOG_1);
		lightVals[lightCnt <= 4 ? 4 - lightCnt : 40 - lightCnt] = analogRead(ALOG_2);
		int tempAnalog = analogRead(ALOG_3);
		switch (lightCnt)
		{
		case 0:
			lightVals[39] = tempAnalog;
			break;

		case 5:
			lightVals[37] = tempAnalog;
			break;

		case 6:
			csVals[3] = tempAnalog;
			break;

		case 7:
			csVals[2] = tempAnalog;
			break;

		case 8:
			thermVals[2] = tempAnalog;
			break;

		case 9:
			thermVals[3] = tempAnalog;
			break;

		case 10:
			lightVals[38] = tempAnalog;
			break;

		case 11:
			csVals[0] = tempAnalog;
			break;

		case 12:
			lightVals[36] = tempAnalog;
			break;

		case 13:
			thermVals[0] = tempAnalog;
			break;

		case 14:
			csVals[1] = tempAnalog;
			break;

		case 15:
			thermVals[1] = tempAnalog;
			break;

		default:
			lightVals[lightCnt + 20] = tempAnalog;
			break;
		}

		lightCnt = lightCnt >= 15 ? 0 : lightCnt + 1;

		digitalWriteFast(SIG_0, sigTable[lightCnt][0]);
		digitalWriteFast(SIG_1, sigTable[lightCnt][1]);
		digitalWriteFast(SIG_2, sigTable[lightCnt][2]);
		digitalWriteFast(SIG_3, sigTable[lightCnt][3]);

		lightTimer = micros();
	}
}

byte *processLight()
{
	static byte mask[5] = {0};
	memset(mask, 0, sizeof(mask));
	for (int i = 0; i < 40; i++)
	{
		if (lightVals[i] > lightThres[i])
		{
			mask[i / 8] |= 0x01 << i % 8;
		}
	}
	return mask;
}

void lightCal() // format L/0,300/1,200/2,900/.../39,200|
{
	digitalWriteFast(solenoid, 0);
#ifdef DEBUG
	Serial2.println("Blocking program for Calibration");
#endif
	const unsigned long timeout = 60000;
	unsigned long timeStart = millis();
	while ((millis() - timeStart) < timeout)
	{
		updateLight();
		if (lightCnt == 0)
		{
			Serial1.print('N');
			for (int i = 1; i <= 40; i++)
			{
				Serial1.print('/');
				Serial1.print(i);
				Serial1.print(',');
				Serial1.print(lightVals[i - 1]);
			}
			Serial1.print('|');
		}
		if (Serial1.available())
		{
			if (Serial1.read() == 'L')
			{
#ifdef DEBUG
				Serial2.println("receiving calib");
#endif
				recvCalib();
				break;
			}
		}
	}
}

void recvCalib()
{
	unsigned int ndx = 0;
	int i = -1;
	const byte endMarker = '|', separator = ',', separator2 = '/'; // format L/0,300/1,200/2,900/.../39,200|
	byte rc, receivedChars[32];
	const unsigned long timeout = 1000;
	unsigned long timeStart = millis();
	while ((millis() - timeStart) < timeout)
	{
		while (Serial1.available())
		{
			rc = Serial1.read();
			if (rc == separator2 && i == -1)
			{
			}
			if (rc == separator2 && i != -1)
			{
				receivedChars[ndx] = '\0'; // terminate the string
				lightThres[i] = fast_atoi(receivedChars);

				unsigned char Char1; // lower byte
				unsigned char Char2; // upper byte
				Char1 = lightThres[i] & 0xFF;
				Char2 = lightThres[i] >> 8;
				eeprom_buffered_write_byte(i + 1, Char1);
				eeprom_buffered_write_byte(i + 41, Char2);

				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
			}
			else if (rc == separator)
			{
				receivedChars[ndx] = '\0'; // terminate the string
#ifdef DEBUG
				if (fast_atoi(receivedChars) - i > 1)
					Serial2.println("Missed data during LS calibration");
#endif
				i = fast_atoi(receivedChars) - 1;
				memset(receivedChars, 0, sizeof(receivedChars));
				ndx = 0;
			}
			else if (rc == endMarker)
			{
#ifdef DEBUG
				Serial2.println("Writing calibration data to EEPROM...");
#endif
				eeprom_buffer_flush();
				ndx = 0;
				i = -1;
				break;
			}
			else
			{
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= sizeof(receivedChars))
				{
#ifdef DEBUG
					Serial2.println("Too many chars!");
#endif
					ndx = sizeof(receivedChars) - 1;
				}
			}
		}
	}
}