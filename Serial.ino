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

	//for pi serial
	char *token;
	int cnt = 0;
	const char s[7] = " BPbyf";

	const char endMarker = '|', separator = ',', separator2 = '/';
	const char startMarker[] = {'L', 'S', 'T', 'N', 'M', 'C', 'B', 'P', 'b', 'y', 'f'};
	// above is {light, current sense, temp, motor switch, mouse, compass, BALL, PREDICTED, BLUE GOAL, YELLOW GOAL, FIELD}
	while (_serial.available() > 0)
	{	
		Serial.print('+');Serial.print(_serial.available());Serial.print(',');
		rc = _serial.read();
		Serial.write(rc);
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
				int numChars = ndx;
				ndx = 0;
				Serial.println();
				switch (startPos)
				{
				case 'L':
					if (numChars==5) {	//temp fix for overlapping received chars problem
						processLight(receivedChars);
						processLine();
					} else {
						Serial.println("loser");
					}
#ifdef SERIAL_DEBUG
					Serial.println("Light data received");
#endif
					break;

				case 'S':
					break;

				case 'T':
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

				case 'B':	//BALL + PREDICTED
					token = strtok((char *)receivedChars, s);	//get first token
					/* walk through other tokens */
					while( token != NULL ) {
						ballPos[cnt] = atoi(token);
						token = strtok(NULL, s);
						cnt++;
					}
					//fix rotated 90 deg shit
					ballAng = mod(ballAng-90,360);
					predAng = mod(predAng-90,360);
					//Serial.print(ballAng); Serial.print(";");
					//Serial.print(ballDist); Serial.println(";");
					lastBallTime = millis();
					break;

				case 'P':	//PREDICTED only
					break;
				
				case 'b':	//BLUE + YELLOW
					token = strtok((char *)receivedChars, s);	//get first token
					/* walk through other tokens */
					while( token != NULL ) {
						goalPos[cnt] = atoi(token);
						//Serial.print(goalPos[cnt2]); Serial.print(";")
						token = strtok(NULL, s);
						cnt++;
					}
					break;

				case 'y':	//YELLOW only
					break;

				case 'f':	//FIELD
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

// void piRecv() {
// 	if (piSerial.available()) {
// 		buf[cnt] = piSerial.read();
// 		cnt++;
// 		if (buf[cnt-1] == 'e') {	//terminated string
// 			char *token;
// 			int cnt2 = 0;
// 			//Serial.print(buf); Serial.print("gg");
// 			if(buf[0]=='B') {
// 				//Serial.print(buf);
// 				const char s[4] = " BP";
// 				token = strtok(buf, s);	//get first token
// 				/* walk through other tokens */
// 				while( token != NULL ) {
// 					ballPos[cnt2] = atoi(token);
// 					token = strtok(NULL, s);
// 					cnt2++;
// 				}
// 				//fix rotated 90 deg shit
// 				ballAng = mod(ballAng-90,360);
// 				predAng = mod(predAng-90,360);
// 				lastBallTime = millis();
// 				//Serial.println(robotAng);
// 			}
// 			else if(buf[0]=='b') {
// 				const char s[4] = " by";
// 				token = strtok(buf, s);	//get first token
// 				/* walk through other tokens */
// 				while( token != NULL ) {
// 					goalPos[cnt2] = atoi(token);
// 					//Serial.print(goalPos[cnt2]); Serial.print(";")
// 					token = strtok(NULL, s);
// 					cnt2++;
// 				}
// 				//Serial.print(mod(360-blueAng,360)); Serial.print(" "); Serial.println(mod(360-yellowAng,360));
// 			}
// 			memset(buf,0,sizeof(buf));
// 			cnt=0;
// 			//Serial.println();
// 		}
// 	}
// }

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