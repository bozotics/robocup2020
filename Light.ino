void updateLight(bool calib)
{
	if (micros() - lightTimer > 100)
	{
		int tempAnalog = analogRead(ALOG_1);
		if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[20 - lightCnt] = analogRead(ALOG_1);
		tempAnalog = analogRead(ALOG_2);
		if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[lightCnt <= 4 ? 4 - lightCnt : 40 - lightCnt] = analogRead(ALOG_2);
		tempAnalog = analogRead(ALOG_3);
		switch (lightCnt)
		{
		case 0:
			if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[39] = tempAnalog;
			break;

		case 5:
			if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[37] = tempAnalog;
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
			if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[38] = tempAnalog;
			break;

		case 11:
			csVals[0] = tempAnalog;
			break;

		case 12:
			if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[36] = tempAnalog;
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
			if (tempAnalog > LIGHT_MIN_CUTOFF) lightVals[lightCnt + 20] = tempAnalog;
			break;
		}

		lightCnt = lightCnt >= 15 ? 0 : lightCnt + 1;

		digitalWriteFast(SIG_0, sigTable[lightCnt][0]);
		digitalWriteFast(SIG_1, sigTable[lightCnt][1]);
		digitalWriteFast(SIG_2, sigTable[lightCnt][2]);
		digitalWriteFast(SIG_3, sigTable[lightCnt][3]);

		lightTimer = micros();
	}
	if (calib) {
		for (int i = 0; i < 40; i++) {
			if (lightVals[i] > lightMax[i]) lightMax[i] = lightVals[i];
			else if (lightVals[i] < lightMin[i]) lightMin[i] = lightVals[i];
		}
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
	const unsigned long timeout = 60000, sendTime = 1000;
	unsigned long timeStart = millis(), sendMillis = millis();
	for (int i = 0; i < 40; i++)
	{
		lightMin[i] = 1200;
		lightMax[i] = 0;
		lightVals[i] = 100;
	}
	while ((millis() - timeStart) < timeout)
	{
		updateLight(true);
		if (!lightCnt)
		{
			// for (int i = 0; i < 40; i++)
			// {
			// 	if (lightVals[i] > lightMax[i])
			// 		lightMax[i] = lightVals[i];
			// 	else if (lightVals[i] < lightMin[i])
			// 		lightMin[i] = lightVals[i];
			// }
			if ((millis() - sendMillis) > sendTime)
			{
				Serial1.print('L');
				for (int i = 0; i < 40; i++)
				{
					Serial1.print('/');
					Serial1.print(i + 1);
					Serial1.print(',');
					Serial1.print((lightMax[i] + lightMin[i]) / 2);
				}
				Serial1.print('|');
#ifdef DEBUG
				for (int i = 0; i < 40; i++)
				{
					Serial2.print(i + 1);
					Serial2.print(", Thres:  ");
					Serial2.println((lightMax[i] + lightMin[i]) / 2);	//weighted average higher than mid point
				}
#endif
				sendMillis = millis();
			}
		}
		if (Serial1.available())
		{
			if (Serial1.read() == 'L')
			{
#ifdef DEBUG
				Serial2.println("finished calibration");
#endif
				delay(10);
				Serial1.print('L');
				//this for manually calc threshold
				int lightThresManual[40] = {323,276,281,245,212,215,237,271,243,271,253,244,248,228,275,273,241,262,285,282,327,223,211,246,273,321,334,278,227,248,247,217,257,286,299,286,142,159,900,900};
				for (int i = 0; i < 40; i++)
				{
					//lightThres[i] = (lightMax[i] + lightMin[i]) / 2;	//weighted average higher than mid point
					lightThres[i] = lightThresManual[i];	//manually calculated
					unsigned char Char1; // lower byte
					unsigned char Char2; // upper byte
					Char1 = lightThres[i] & 0xFF;
					Char2 = lightThres[i] >> 8;
					eeprom_buffered_write_byte(i + 1, Char1);
					eeprom_buffered_write_byte(i + 41, Char2);

					Serial1.print('/');
					Serial1.print(i + 1);
					Serial1.print(',');
					Serial1.print(lightThres[i]);
#ifdef DEBUG
					Serial2.print(i + 1);
					Serial2.print(", Thres:  ");
					Serial2.println(lightThres[i]);
#endif
				}
				Serial1.print('|');
				eeprom_buffer_flush();
				break;
			}
		}
	}
}