void processLight(byte *data)
{
#ifdef SERIAL_DEBUG
	Serial.println("Processing light data...");
#endif
	newLightData = true;

	//checking light gates here
	frontGate = !(data[4] >> 6 & 0x01);	//pass data through NOT since it sends true when over threshold
	backGate = !(data[4] >> 7 & 0x01);
	if (frontGate) {
	  lastFrontTime = millis();
		if (!prevFrontGate) {
		firstFrontTime = millis();  //if first time frontgate activated, start timer
		//Serial.println(firstFrontTime);
		}
	}
  	prevFrontGate = frontGate;
	if (backGate) backTimer = millis();

	int numDetected = 0;
	for (int i = 0; i < 36; i++)	//NOT checking side 2 light and lightgate here
	{
		bool detected = (data[i / 8] >> (i % 8) & 0x01);	//parse data to see which index is activated
		//Serial.print(detected);
		if (detected)
		{
			line[numDetected] = i;
			numDetected++;
#ifdef SERIAL_DEBUG
			Serial.print(i);
			Serial.print("\t");
#endif
		}
	}

	// //extra side sensors
	// if ((data[4] >> 4 & 0x01) && numDetected==0) {
	// 	line[0] = 9;
	// 	numDetected = 1;
	// }
	// else if ((data[4] >> 5 & 0x01) && numDetected==0) {
	// 	line[0] = 27;
	// 	numDetected = 1;
	// }
	
	//Serial.println("lmao");
	// lineAng: angle of normal from line to centre of robot, taking shorter normal's angle
	// lineLen: relative length of longest chord between detected clusters, i.e. diameter=1
	if (numDetected==0) {	//no light detected
		if (millis() - lastOutTime > 30) {	//must be off line for 30 ms to prevent false negative
			lineAng = -1;
			lineLen = -1;
			prevLine = onLine;
			lastInTime = millis();
			if(onLine) onLine = false;
		}
		return;
	}
	else if (numDetected==1) {	//1 light detected
		lineAng = line[0]*10.0;
		lineLen = 0;
	}
	else {	//2 or more light detected
		float ang=0, tempAng=0;
		clusterStart=-1, clusterEnd=-1;
		for(int i=0; i<numDetected-1; i++) {  //find longest chord
			for(int j=1; j<numDetected; j++) {
				tempAng = smallestAngleBetween(line[i]*10,line[j]*10);
				if(tempAng>ang) {
				ang = tempAng;
				clusterStart=line[i]*10;
				clusterEnd=line[j]*10;
				}
			}
		}
		lineAng = angleBetween(clusterStart, clusterEnd) <= 180 ? midAngleBetween(clusterStart, clusterEnd) : midAngleBetween(clusterEnd, clusterStart);
    lineLen = smallestAngleBetween(clusterStart,clusterEnd)/180;
	}
	prevLine = onLine;
	prevLastOutTime = lastOutTime;
	lastOutTime = millis();
	if(!onLine) onLine = true;
	
#ifdef SERIAL_DEBUG
	Serial.println("");
#endif
}

void processLine() {	
	// this function determines direction of line + degree of passing over line from line
	// lineAng = angle of line relative to centre of robot based on initial entry angle
	// lineLen = relative scale of how far pass the line the robot is
	//	 0-1: 0%-50% OUT field, 
	//	 1-2: 50%-100% OUT field
	//	 3: completely OUT field (past light ring)
	if(prevLine && onLine) {  //was on line, stil on line
		if(smallestAngleBetween(prevLineAng,lineAng)>=90) { //robot on outside of line (because change in angle > 90 i.e. side of acute angle flipped)
			lineAng = mod(lineAng+180,360);	//move opp direction away from line
			lineLen = 2-lineLen;
		}
	}
	else if(!prevLine && onLine) {  //was not on line, now on line

	} 
	else if(prevLine && !onLine) {  //was on line, now not on line
		if(lineLen <= 1) {  //last seen inside of line, now in field
			lineLen=0;
			lineAng=0;
		} else {  //last seen outside of line, now out of field
			lineLen=3;
		}
	}
	prevLineAng = lineAng;
}

void lineAvoid() {
	float tofAng = processTOFout();	//returns angle to move back into field
	
	if(onLine) {
		if (tofAng >= 0) {
			angular_drive(1200,tofAng,cmpCorrection,300);
			//Serial.print("tof");Serial.println(tofAng);
		}
		else {
			if(lineLen > 0.5) {	//changing this value changes when robot starts rebounding (refer to processLine)
				if(lineLen==3) angular_drive(2.0*robotSpeed,mod(lineAng+180,360),cmpCorrection,100);
				else {
					angular_drive(1.0*lineLen*robotSpeed,mod(lineAng+180,360),cmpCorrection,300);
					//Serial.print(1.5*lineLen*robotSpeed);
				}
			}
			//Serial.print("line");Serial.println(mod(lineAng+180,360));
		}
		// else if(lineLen > 1.0 && isOutsideLine(ballAng)) {
		// 	//angular_drive(0,0,0);
		// }
	} else if(millis()-lastOutTime < 200 && tofAng>=0) {	//time off line less than 0.2s, continue rejecting line based on tof for safety
		//angular_drive(900,tofAng,cmpCorrection,300);
		//Serial.print("toffffuck");
		//Serial.println(tofAng);
	}
}

bool recvCalib()
{
	static bool recvInProgress = false;
	static int ndx = 0, i = 0;
	const byte endMarker = '|', startMarker = 'L', separator = ',', separator2 = '/'; // format L/0,300/1,200/2,900/.../39,200|
	byte rc;
	while (L1Serial.available())
	{
		rc = L1Serial.read();
		if (recvInProgress == true)
		{
			if (rc == separator2 && !i)
				;
			else if (rc == separator2 && i)
			{
				receivedChars[1][ndx] = '\0'; // terminate the string
				Serial.print("No. ");
				Serial.print(i);
				Serial.print("\tThres:");
				Serial.println(fast_atoi(receivedChars[1]));
				memset(receivedChars[1], 0, sizeof(receivedChars[1]));
				ndx = 0;
			}
			else if (rc == separator)
			{
				receivedChars[1][ndx] = '\0'; // terminate the string
				if (fast_atoi(receivedChars[1]) - i > 1 && fast_atoi(receivedChars[1]) != 1)
					Serial.println("Missed data during LS calibration");
				i = fast_atoi(receivedChars[1]);
				memset(receivedChars[1], 0, sizeof(receivedChars[1]));
				ndx = 0;
			}
			else if (rc == endMarker)
			{
				receivedChars[1][ndx] = '\0'; // terminate the string
				Serial.print("No. ");
				Serial.print(i);
				Serial.print("\tThres:");
				Serial.println(fast_atoi(receivedChars[1]));
				memset(receivedChars[1], 0, sizeof(receivedChars[1]));
				ndx = 0;
				i = 0;
				recvInProgress = false;
				return true;
			}
			else
			{
				receivedChars[1][ndx] = rc;
				ndx++;
				if (ndx >= sizeof(receivedChars[1]))
				{
					Serial.println("Too many chars!");
					ndx = sizeof(receivedChars[1]) - 1;
				}
			}
		}
		else if (rc == startMarker)
			recvInProgress = true;
	}
	return false;
}

void lineTrack() {
	if (onLine) {
		float angle, angleDeviation, dist;
		if (smallestAngleBetween(clusterStart,ballAng) < smallestAngleBetween(clusterEnd,ballAng)) {
			angle = clusterStart;
			angleDeviation = abs(90-smallestAngleBetween(clusterStart,ballAng));
		}
		else {
			angle = clusterEnd;
			angleDeviation = abs(90-smallestAngleBetween(clusterEnd,ballAng));
		}
		dist = lineLen > 1 ? lineLen-1 : 1-lineLen;
		//Serial.print(angle); Serial.print(" "); Serial.println(min(1100,450*sqrt(angleDeviation/2))+dist*200.0);
		angular_drive(min(900,450*sqrt(angleDeviation/2))+dist*100.0,angle,cmpCorrection,300);
	} else {
		angular_drive(0,0,cmpCorrection,300);
	}
}