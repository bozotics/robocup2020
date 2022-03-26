int fast_atoi(byte *str)
{
	int val = 0;
	while (*str)
	{
		val = val * 10 + (*str++ - '0');
	}
	return val;
}

void recv(HardwareSerial &_serial, int serNum)
{
	static boolean recvInProgress = false;
	static byte rc;

	const char endMarker = '|', separator = ',', separator2 = '/';
	const char startMarker[] = {'L', 'S', 'T', 'N', 'M', 'C', 'X', 'B', 'P', 'b', 'y', 'f'};
	// above is {light, current sense, TOF, motor switch, mouse, compass, bluetooth, BALL, PREDICTED, BLUE GOAL, YELLOW GOAL, FIELD}

	while(_serial.available() > 0) {
		rc = _serial.read();
		if(ndx[serNum] > 0) {
			if (rc != endMarker || (receivedChars[serNum][0] == 'L' && ndx[serNum] < 5)) {	//second cond is to prevent '|' light data byte from reading as delimiter		
				receivedChars[serNum][ndx[serNum]] = rc;
				//Serial.print(receivedChars[serNum][ndx[serNum]]); Serial.print(";");Serial.print(ndx[serNum]);
				ndx[serNum]++;
			}
			else {	//rc is endMarker or light received 5 bytes
				receivedChars[serNum][ndx[serNum]] = '\0';
				processRecv(serNum);

			}
		}
		else {	//no data in string yet
			for (unsigned int i = 0; i < sizeof(startMarker); i++) {
				if (rc == startMarker[i]) {
					receivedChars[serNum][0] = rc;
					ndx[serNum]++;
				}
			}
		}
	}
}

void processRecv(int serNum) {
	byte* data = receivedChars[serNum] + 1;	//remove first char
	//for sorting multi object serial strings
	char *token;
	int cnt = 0;
	const char s[7] = " BPbyf"; //space,BALL,PREDICTED,bluegoal,yellowgoal,field

	switch (receivedChars[serNum][0]) {
		case 'L':	//light
			processLight(data);
			processLine();
	#ifdef SERIAL_DEBUG
			Serial.println("Light data received");
	#endif
			break;

		case 'S':	//current sense
			break;

		case 'T':	//tof
			token = strtok((char *)data, " T");	//get first token
			/* walk through other tokens */
			while( token != NULL ) {
				tof[cnt] = atoi(token);
				token = strtok(NULL, " T");
				cnt++;
			}
			processTOF();
			break;

		case 'N':	//motorswitch
			if (fast_atoi(data))
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

		case 'M':	//mouse
			break;

		case 'C':
			readIMU(data);
			break;

		case 'X': //bluetooth data
			/*  Internationals challenge 2 (passing)
			if (data[0] == 'R') {
				if (data[1] == '0') attackState = 0;	//sent from giver, giver in process of delivering ball
				else if(data[1] == '1') attackState = 1;	//sent from giver, receiver should start approach ball
				else if (data[1] == '2') attackState = 2; 	//sent from receiver, ball in receiver catchment, giver should turn off dribbler
				else if (data[1] == '3') attackState = 3;	//sent from giver, giver turned off dribbler, receiver should turn on
				else if (data[1] == '4') attackState = 4; 	//sent from receiver, giver can leave ball (ball in receiver possession)
				Serial.println(attackState);
			}
			*/
#ifdef whitebot
			// char fuck[30];
			// sprintf(fuck, "%s", (char*)data);
			// Serial.print(fuck);Serial.println("bruh");
			if(data[0] == 'Q'){ //other robot's ballDist and ballAng e.g. "XQ12 345|"
				token = strtok((char *)data, " Q");  //get first token
				/* walk through other tokens */
				while( token != NULL ) {
					otherBotData[cnt] = atoi(token);
					token = strtok(NULL, " Q");
					cnt++;
				}
				lastBTrecvTime = millis();
				//Serial.print(otherBallAng); Serial.print(";");Serial.print(otherBallDist); Serial.println(";");
				//Serial.print(otherOppGoalAng); Serial.print(";");Serial.print(otherOppGoalDist); Serial.println(" ");
				//Serial.println(robotAng);
			}
#endif
#ifdef blackbot
			if(data[0] == 'R') {	//other robot wants switch role
				if (data[1] == '0') attackState = 0;
				else if (data[1] == '1') attackState = 1;
				Serial.print("i become "); Serial.println(attackState);
			}
#endif
		break;

		case 'B':	//BALL + PREDICTED
			token = strtok((char *)data, s);	//get first token
			/* walk through other tokens */
			while( token != NULL ) {
				ballPos[cnt] = atoi(token);
				token = strtok(NULL, s);
				cnt++;
			}
			//fix rotated 90 deg shit
			ballAng = mod(ballAng-90,360);
			predAng = mod(predAng-90,360);
			//  Serial.print(ballAng); Serial.print(";");
			//  Serial.print(ballDist); Serial.println(";");
			lastBallTime = millis();
			newBallData = true;
			break;

		case 'P':	//PREDICTED only
			break;

		case 'b':	//BLUE + YELLOW
			token = strtok((char *)data, s);	//get first token
			/* walk through other tokens */
			while( token != NULL ) {
				goalPos[cnt] = atoi(token);
				//Serial.print(goalPos[cnt2]); Serial.print(";")
				token = strtok(NULL, s);
				cnt++;
			}
			currGoalAng = mod(360-currGoalAng,360);
			oppGoalAng = mod(360-oppGoalAng,360);
			newGoalData = true;
			//Serial.print(blueAng);Serial.print(" ");Serial.print(yellowAng);Serial.print(" ");Serial.print(blueDist);Serial.print(" ");Serial.print(yellowDist);Serial.print(" ");Serial.println(midAngleBetween(blueAng, yellowAng));
			break;

		case 'y':	//YELLOW only
			break;

		case 'f':	//FIELD
			break;

		default:
			break;
	}
	memset(receivedChars[serNum],0,sizeof(receivedChars[serNum]));
	ndx[serNum] = 0;
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

void recvAll() {
	recv(btSerial,0);
	recv(L1Serial,1);
	recv(piSerial,2);
	recv(L3Serial,3);
	recv(L4Serial,4);
}