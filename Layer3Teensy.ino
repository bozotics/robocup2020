#include "common.h"

void setup()
{
	L3Serial.setTX(1);
	L3Serial.setRX(0);
#ifdef DEBUG
	Serial.begin(115200);
#endif
	L1Serial.begin(250000);
	L3Serial.begin(250000);
	L4Serial.begin(250000);
  	btSerial.begin(115200);
	piSerial.begin(1000000);

	DIP1.attach(DIP1p, INPUT);
	DIP2.attach(DIP2p, INPUT);
	DIP3.attach(DIP3p, INPUT);
	DIP4.attach(DIP4p, INPUT);
	DIP1.interval(50);
	DIP2.interval(50);
	DIP3.interval(50);
	DIP4.interval(50);

	pinMode(INM1, OUTPUT);
	pinMode(INM2, OUTPUT);
	pinMode(INM3, OUTPUT);
	pinMode(INM4, OUTPUT);
	digitalWriteFast(INM1, 0);
	digitalWriteFast(INM2, 1);
	digitalWriteFast(INM3, 1);
	digitalWriteFast(INM4, 0);
	analogWriteResolution(12);
	analogWriteFrequency(PWMM1, 14648.437);
	analogWriteFrequency(PWMM2, 14648.437);
	analogWriteFrequency(PWMM3, 14648.437);
	analogWriteFrequency(PWMM4, 14648.437);

	while (L1Serial.available())
		char t = L1Serial.read();
	delay(1000);
}

unsigned long lastSwitchTime = millis();
unsigned long printTimer = millis();
//PRECISION MOVEMENT challenge
unsigned long lastMoveTime = millis();
int points[] = {  9 , 10 ,  8 ,  1 ,  4 ,  2 ,  5 ,  3 , 11 ,  6 ,  7};
bool nextPoint = false;
int cnt=0;

bool pointOnLine(int point) {
	if (point == 3 || point==4 || point==6 || point==8 || point==9 || point < 0) return false;
	else return true;
}

int moveToPoint(int point) {
	recv(piSerial,2);
	recv(L1Serial,1);
	float angle, dist, lineAngleRef, lineDistRef;
	float centreAng = mod((yellowAng*PI/180+atan2(yellowDist*sin(yellowAng*PI/180-yellowAng*PI/180), yellowDist + yellowDist*cos(yellowAng*PI/180-yellowAng*PI/180)))*180/PI , 360);
  	float centreDist = sqrt(pow(yellowDist,2) + pow(yellowDist,2) + 2*yellowDist*yellowDist*cos(yellowAng*PI/180-yellowAng*PI/180)) / 2;
  	
	 //angle from centre to yellow, relative to robot
	float tempAng = centreAng + 180;	//negative vector
	float ABangle = mod((tempAng*PI/180+atan2(yellowDist*sin(yellowAng*PI/180-tempAng*PI/180), centreDist + yellowDist*cos(yellowAng*PI/180-tempAng*PI/180)))*180/PI , 360);
	float ABdist = sqrt(pow(centreDist,2) + pow(yellowDist,2) + 2*centreDist*yellowDist*cos(yellowAng*PI/180-tempAng*PI/180));

	float tempAng2, tempDist2;
	switch(point) {
		case -1:	//same as case 1, but without line tracking
			tempAng2 = mod(ABangle-60,360);
			tempDist2 = 115;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 1:
			tempAng2 = mod(ABangle-60,360);
			tempDist2 = 115;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			break;
		case -2:	//same as case 2, but without line tracking
			tempAng2 = mod(ABangle+65,360);
			tempDist2 = 115;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 2:
			tempAng2 = mod(ABangle+65,360);
			tempDist2 = 115;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			
			break;
		case 3:	//top left neutral point
			tempAng2 = mod(ABangle-60,360);
			tempDist2 = 30;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 4:	//top right neutral point
			tempAng2 = mod(ABangle+62,360);
			tempDist2 = 35;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 5:	//centre left line point
			tempAng2 = mod(ABangle-95,360);
			tempDist2 = 63;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			break;
		case 6:	//central netural point
			tempAng2 = mod(ABangle+190,360);
			tempDist2 =7;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 7: //centre right line point
			tempAng2 = mod(ABangle+100,360);
			tempDist2 = 63;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			break;
		case 8:	//bottom left neutral point
			tempAng2 = mod(ABangle-135,360);
			tempDist2 = 49;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 9: //bottom right neutral point
			tempAng2 = mod(ABangle+135,360);
			tempDist2 = 49;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 10:
			tempAng2 = mod(ABangle-144,360);
			tempDist2 = 150;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			break;
		case 11:
			tempAng2 = mod(ABangle+144,360);
			tempDist2 = 145;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			break;
		case -12:
			tempAng2 = mod(ABangle,360);
			tempDist2 =40;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			break;
		case 12: 	//same as 12 but wants to track line version
			tempAng2 = mod(ABangle,360);
			tempDist2 =40;
			angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));
			if (onLine) {
				lineAngleRef = smallestAngleBetween(clusterStart,angle) < smallestAngleBetween(clusterEnd,angle) ? clusterStart : clusterEnd;
				lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
			}
			break;
	}
	if (!pointOnLine(point)) {
		if (dist >= 10 || (dist >= 25 && point==9) || dist >= 20 && point==8) {
			//Serial.print(dist);Serial.print("here");Serial.print(point);
			//note use of lesser speed for negative points (anti line point)
			angular_drive(min(dist*70.0,point < 0 ? 1100 : 1300),angle,ABangle <= 180 ? ABangle/30 : (ABangle-360)/30);
			lastMoveTime = millis();
		}
		else {
			angular_drive(0,0,0);
			if(millis()-lastMoveTime > 500) {
				unsigned long tmpTimer = millis();
				while (millis()-tmpTimer < 2600) {
					recv(L1Serial,1);
					recv(piSerial,2);
					angular_drive(0,0,0);move_OUT();
				}
				lastMoveTime = millis();
				nextPoint = true;
			}
		}
	}
	else {	//point on line
		if (onLine) {	
			if (dist >= 25) {
				angular_drive(min(dist*50.0,1000)+lineDistRef*200.0,lineAngleRef,ABangle <= 180 ? ABangle/30 : (ABangle-360)/30);
				lastMoveTime = millis();
				//Serial.println(lineAngleRef);
			} 
			else {
				angular_drive(0,0,0);
				if(millis()-lastMoveTime > 350) {
					unsigned long tmpTimer2 = millis();
					while (millis()-tmpTimer2 < 2800) {
						recv(L1Serial,1);
						recv(piSerial,2);
						angular_drive(0,0,0);move_OUT();
					}
					lastMoveTime = millis();
					nextPoint = true;
				}
			}
			
		} else {	//robot not on line yet
			angular_drive(min(dist*60.0,900),angle,ABangle <= 180 ? ABangle/30 : (ABangle-360)/30);
			//Serial.println("here");
			lastMoveTime = millis();
		}
	} 	
	move_OUT();
	return dist;
}

int counter = 0;
//challegne stuff
int pointToOrbit = 1;
unsigned long lmaoTimer = millis();
int attackState = 0, prevAttackState = 0;

void loop()

/*
{	//PRECISION MOVEMENT challenge
	//recv(L4Serial);
	
	// if (millis() - lastBallTime > 100) {
	// 	Serial.printf("%f %f;\t%d %d;\t%f %f;\t%f %f;\n", centreAng, centreDist, yellowAng, yellowDist, ABangle, ABdist, angle, dist);
	// 	lastBallTime = millis();
		
	// }

	
	nextPoint = true;
	for (int i=0; i<11; i++) {
		while(!nextPoint) {
			moveToPoint(points[i]);
		} 
		nextPoint = false;
		if (pointOnLine(points[i] && pointOnLine(points[i+1]))) {	//currently on line, need move to another point on line
			int transitionPoint;
			switch (points[i+1]) {
				case 1:
					transitionPoint = 3; break;
				case 2:
					transitionPoint = 4; break;
				case 5:
					transitionPoint = 6; break;
				case 7:
					transitionPoint = 6; break;
				case 10:
					transitionPoint = 8; break;
				case 11:
					transitionPoint = 9; break;
			}
			while(moveToPoint(transitionPoint) > 20);	//move until within 30 dist of transitionPoint
			
		}
	}
	nextPoint=false;
	if (pointOnLine(points[10] && pointOnLine(points[0]))) {
		int transitionPoint;
		switch (points[0]) {
			case 1:
				transitionPoint = 3; break;
			case 2:
				transitionPoint = 4; break;
			case 5:
				transitionPoint = 6; break;
			case 7:
				transitionPoint = 6; break;
			case 10:
				transitionPoint = 8; break;
			case 11:
				transitionPoint = 9; break;
		}
		while(moveToPoint(transitionPoint) > 20) ;	//move until within 30 dist of transitionPoint
	}
	while(!nextPoint) {
		moveToPoint(points[0]);
	}
	while(1) {
		angular_drive(0,0,0);
		move_OUT();
	}
}
*/

/*
{	//robustness in presence of other agents challenge
	while(!motorOn) {
		recv(L1Serial);
		recv(piSerial);
	}
	delay(1000);
	cnt++;
	unsigned long timer = millis();
	while (millis()-timer < 1000) {
		recv(piSerial);
	}
	bool endRight;
	if (smallestAngleBetween(90,blueAng) < smallestAngleBetween(270,blueAng)) {
		while(moveToPoint(1)>40);
		endRight = true;
	} else {
		while(moveToPoint(2)>40);
		endRight = false;
	}

	while(onLine) moveToPoint(-12);
	nextPoint = false;
	while (!nextPoint) moveToPoint(12);

	if(endRight) {
		while (moveToPoint(-2)>30);
		nextPoint = false;
		while (!nextPoint) moveToPoint(11);
	}
	else {
		while(moveToPoint(-1)>30);
		nextPoint=false;
		while (!nextPoint) moveToPoint(10);
	}
	
	delay(10000);
	if (counter ==3) {
		while(1) {
			angular_drive(0,0,0);
			move_OUT();
		}
	}
}
*/

/*{	//precision shooter (not done)
	recv(L3Serial,3);
	recv(L1Serial,1);
	//recv(btSerial,0);
	recv(piSerial,2);

		//turn on front dribbler if ball within +-50 deg of front AND closer than 50 cm
	if ((((ballAng <= 50 || ballAng >= 310) && ballDist < 50 && millis()-lastBallTime < 1000) || (frontGate || millis()-lastFrontTime < 100)) && millis()-frontDribblerTimer > 500 ) {
		if (!frontDribblerOn) {	//only send to turn on once
			serialWrite(L4Serial, '1');
			frontDribblerOn = true;
			//Serial.print(ballAng); Serial.print(";");
			//Serial.print(ballDist); Serial.println(";");
			Serial.println("dribbler");
		}
	}
	else {
		if (frontDribblerOn) {	// only send to turn off once
			serialWrite(L4Serial, '2');
			frontDribblerOn = false;
		}
	}
	//turn off front dribbler + turn on kicker if facing blue goal and 50cm away
	//if((currGoalAng <= 30 || currGoalAng >= 330) && currGoalDist < 50 && frontGate && millis()-lastBallTime < 1000 && (robotAng < 60 || robotAng > 300)) {
	if(frontGate && millis()-firstFrontTime > 3000){
		if (millis()-kickTimer > 1500) {
			serialWrite(L4Serial, '2');
			delay(10);
			serialWrite(L1Serial,'K');
			frontDribblerOn = false;
			frontDribblerTimer = millis();
			kickTimer = millis();
			//  Serial.print(currGoalAng); Serial.print(";");
			//  Serial.print(currGoalDist); Serial.print(";");
			Serial.println("kicked");
		}
	}

	robotSpeed = 800;
	calcRobotAngle();

	if (millis()-lastCmpTimer > 500) stopMove = true;
	else stopMove = false;

  	if(stopMove) {
		//angular_drive(0,0,cmpCorrection,900);
		angular_drive(0,0,0);
	}

	move_OUT();

	if (millis()-printTimer > 100) {

		printTimer = millis();
	}
	// if (millis() - orbitTimer > 8000) {
	// 	if(pointToOrbit == 4) pointToOrbit = 1;
	// 	else pointToOrbit++;
	// 	orbitTimer = millis();
	// }
}*/

/*{		//Challenges codes
	recvAll();


//Internationals Challenge 3
#ifdef whitebot
	if ((((ballAng <= 50 || ballAng >= 310) && ballDist < 50 && millis()-lastBallTime < 1000) || (frontGate || millis()-lastFrontTime < 100)) && millis()-frontDribblerTimer > 500 ) {
		if (!frontDribblerOn) {	//only send to turn on once
			serialWrite(L4Serial, '1');
			frontDribblerOn = true;
			//Serial.print(ballAng); Serial.print(";");
			//Serial.print(ballDist); Serial.println(";");
			Serial.println("dribbler");
		}
	}
	else {
		if (frontDribblerOn) {	// only send to turn off once
			serialWrite(L4Serial, '2');
			frontDribblerOn = false;
		}
	}
	if (attackState == 0) {	//waiting for bt "XR1"
		angular_drive(0,0,0);
	}
	else if (attackState == 1) {
		float *vect = calcToTOFpoint(130,300,2,3);
		angular_drive(min(*vect*7.0,800),*(vect+1),cmpCorrection,300);
		if (*vect < 90) {
			attackState = 2;
		}
	}
	else if (attackState == 2) {
		float *vect = calcToTOFpoint(130,1000,2,3);
		angular_drive(min(*vect*6.5,800),*(vect+1),cmpCorrection,300);
		if (*vect < 50) {
			attackState = 3;
		}
	}
	else if (attackState==3) {
		float *vect = calcToTOFpoint(400,400,0,1);
		angular_drive(min(*vect*6.0,800),*(vect+1),cmpCorrection,300);
		if (*vect < 90) {
			attackState = 4;
		}
	}
	else if (attackState == 4) {
		if (ballAng > 180) angular_drive(0,0,-10,400);
		else angular_drive(0,0,10,400);
		if (ballAng < 5 || ballAng >355) attackState = 5;
	}
	else if (attackState == 5) {
		angular_drive(700,ballAng,ballAng < 180 ? ballAng/20 : (ballAng-360)/20);
		if (frontGate && millis()-firstFrontTime > 200) attackState = 6;
	}
	else if (attackState == 6) {
		if (cmpangle > 180) angular_drive(0,0,10,350);
		else angular_drive(0,0,-10,350);
		if (cmpangle < 5 || cmpangle >355) attackState = 7;
	}
	else if (attackState==7) {
		float *vect = calcToTOFpoint(900,350,0,1);
		angular_drive(min(*vect*6.0,750),*(vect+1),cmpCorrection,300);
		if (*vect < 50) {
			attackState = 8;
		}
	}
	else if (attackState == 8) {
		//angular_drive(0,0,0); move_OUT(); delay(500);
		lmaoTimer = millis();
		while (millis()-lmaoTimer < 500) {
			FLout=-1000;BLout=-1000;FRout=1000;BRout=1000;
			move_OUT();
		}
		attackState = 9;
	}
	else if (attackState == 9) {
		serialWrite(L4Serial, '2');
		frontDribblerOn = false;
		angular_drive(0,0,0);
		move_OUT();
		attackState = 0;
	}
#endif 

// #ifdef blackbot
// 	if(frontGate && millis()-firstFrontTime > 50){
// 		if (millis()-kickTimer > 1500) {
// 			serialWrite(L4Serial, '2');
// 			serialWrite(L1Serial,'K');
// 			frontDribblerOn = false;
// 			frontDribblerTimer = millis();
// 			kickTimer = millis();
// 			//  Serial.print(currGoalAng); Serial.print(";");
// 			//  Serial.print(currGoalDist); Serial.print(";");
// 			Serial.println("kicked");
// 			attackState = 1;
// 		}
// 	}
// 	if (attackState == -1) {
// 		delay(5000); //let time to reset
// 		attackState = 0;
// 	}
// 	if (attackState == 0) {
// 		robotSpeed = 650;
// 		if (millis()-lastBallTime > 500) {	//no ball
// 			angular_drive(0,0,0);
// 		}
// 		else {
// 			double ballOffset;
// 			if(ballAng<180) ballOffset = fmin(ballAng*1.0,90);
// 			else ballOffset = fmax((ballAng-360)*1.0,-90);
// 			angular_drive(robotSpeed,ballAng+ballOffset*1.0, cmpCorrection,300);
// 			//Serial.println(ballAng+ballOffset*0.95);
// 		}
		
// 	}
// 	else if (attackState == 1) {
// 		float *vect = calcToTOFpoint(300,800,2,1);
// 		if (*vect < 80) {
// 			angular_drive(0,0,0);
// 		}
// 		else {
// 			angular_drive(min(*vect*7.0,1100),*(vect+1),cmpCorrection,300);
// 			lastMoveTime = millis();
// 		}

// 		if (millis()-lastMoveTime > 800) {
// 			angular_drive(800,180,cmpCorrection,300);
// 			move_OUT(); delay(800);
// 			attackState =2;
// 			btSerial.print("XR1|");
// 		}		
// 	}
// 	else if (attackState == 2) {
// 		Serial.println("stopped!");
// 		angular_drive(0,0,0);
// 		if (millis()-lastCmpTimer > 500) {	//ball shot in goal
// 			attackState = 0;
// 		}
// 	}
// #endif

	//some challenge???
	// angular_drive(900,mod(90-cmpangle,360),max(min(mod(90-cmpangle,360)<=180 ? mod(90-cmpangle,360)/20 : (mod(90-cmpangle,360)-360)/20,1),-1));
	// Serial.println(attackState);
	// if(attackState ==0) {
	// 	if(millis()-lastBallTime < 1000) {
	// 		robotSpeed = 700;
	// 		double ballOffset;
	// 		if(ballAng<180) ballOffset = fmin(ballAng*1.0,90);
	// 		else ballOffset = fmax((ballAng-360)*1.0,-90);
	// 		angular_drive(robotSpeed,ballAng, cmpCorrection, 300);
	// 	} else {
	// 		angular_drive(0,0,0);
	// 	}
	// 	if (ballDist < 30 && millis()-lastBallTime < 1000) attackState = 1;
	// }
	// else if (attackState == 1) {
	// 	robotSpeed = 650;
	// 	double ballOffset;
	// 	if(ballAng<180) ballOffset = 90;
	// 	else ballOffset = -90;
	// 	angular_drive(robotSpeed,mod(ballAng+ballOffset*0.9,360), blueAng < 180 ? blueAng/14 : (blueAng-360)/14);
	// 	if (ballAng < 5 || ballAng > 355) attackState = 2;
	// }
	// else if (attackState == 2) {
	// 	if (blueAng > 180) angular_drive(0,0,-10,400);
	// 	else angular_drive(0,0,10,400);
	// 	if (blueAng < 5 || blueAng >355) attackState = 3;
	// }
	// else if (attackState == 3) {
	// 	angular_drive(0,0,0);move_OUT();delay(500);attackState=4;
	// }
	// else if (attackState == 4) {
	// 	attackState = 5;
	// }
	// else if (attackState == 5) {
	// 	angular_drive(500,mod(0-cmpangle,360),cmpCorrection,300);
	// 	move_OUT();
	// 	if(frontGate && millis()-firstFrontTime > 5) attackState = 6;
	// }
	// else if (attackState ==6) {
	// 	serialWrite(L1Serial,'K');
	// 	angular_drive(0,0,0);
	// 	attackState = 7;
	// }
	// else if (attackState == 7) {
	// 	if (millis()-lastCmpTimer > 500) {	//ball shot in goal
	// 		attackState = 0;
	// 	}
	// }

	// if(frontGate && millis()-firstFrontTime > 100){
	// 	if (millis()-kickTimer > 1500) {
	// 		serialWrite(L4Serial, '2');
	// 		serialWrite(L1Serial,'K');
	// 		frontDribblerOn = false;
	// 		frontDribblerTimer = millis();
	// 		kickTimer = millis();
	// 		//  Serial.print(currGoalAng); Serial.print(";");
	// 		//  Serial.print(currGoalDist); Serial.print(";");
	// 		Serial.println("kicked");
	// 		attackState = 2;
	// 	}
	// }
	

	if(frontGate && millis()-firstFrontTime > 5) {
		serialWrite(L1Serial,'K');
	}

	if (millis()-lastCmpTimer > 500) stopMove = true;
	else stopMove = false;

  	if(stopMove) {
		//angular_drive(0,0,cmpCorrection,900);
		angular_drive(0,0,0);
	}
	//angular_drive(0,0,0);
	move_OUT();

	if (millis()-printTimer > 100) {
		
		printTimer = millis();
	}
	prevAttackState = attackState;
} */

{
	newLightData = false;
	newBallData = false;
	newGoalData = false;

	recv(L3Serial,3);
	recv(L1Serial,1);
	
	recv(L4Serial,4);
	recv(piSerial,2);
  	recv(btSerial,0);
	
	//turn on front dribbler if ball within +-50 deg of front AND closer than 50 cm
	if ((((ballAng <= 50 || ballAng >= 310) && ballDist < 50 && millis()-lastBallTime < 1000) || (frontGate || millis()-lastFrontTime < 100)) && millis()-frontDribblerTimer > 500 ) {
		if (!frontDribblerOn) {	//only send to turn on once
			serialWrite(L4Serial, '1');
			frontDribblerOn = true;
			//Serial.print(ballAng); Serial.print(";");
			//Serial.print(ballDist); Serial.println(";");
			Serial.println("dribbler");
		}
	}
	else {
		if (frontDribblerOn) {	// only send to turn off once
			serialWrite(L4Serial, '2');
			frontDribblerOn = false;
		}
	}

	//turn off front dribbler + turn on kicker if facing front goal and 50cm away
	if((currGoalAng <= 30 || currGoalAng >= 330) && currGoalDist < 50 && frontGate && millis()-lastBallTime < 1000 && (robotAng < 60 || robotAng > 300)) {
	//if(frontGate && millis()-firstFrontTime > 2500){
		if (millis()-kickTimer > 1500) {
			serialWrite(L4Serial, '2');
			serialWrite(L1Serial,'K');
			frontDribblerOn = false;
			frontDribblerTimer = millis();
			kickTimer = millis();
			//  Serial.print(currGoalAng); Serial.print(";");
			//  Serial.print(currGoalDist); Serial.print(";");
			Serial.println("kicked");
		}
	}
  
	//turn on back dribbler if ball within +-50 deg of back (130 to 230 deg) AND close enough to goal
	if ((((ballAng >= 130 && ballAng <= 230) && ballDist < 70 && millis()-lastBallTime < 1000) || (backGate || millis()-backTimer < 100)) && attackState) {
	//if (backGate) {
		if (!backDribblerOn) {	//only send to turn on once
			serialWrite(L4Serial, '3');
			backDribblerOn = true;
			//Serial.print(ballAng); Serial.print(";");
			//Serial.print(ballDist); Serial.println(";");
			Serial.println("dribbler");
		}
	} else {
		if (backDribblerOn) {	// only send to turn off once
			serialWrite(L4Serial, '4');
			backDribblerOn = false;
		}
	}

	calcRobotAngle();
	//angular_drive(0,0,cmpCorrection,1000);
	//robotSpeed=1200;

	if (attackState) {	//attack program
		//line avoidance
		lineAvoid();
		/*	//line avoid using line tracking
		if (receivedLight && onLine) {
			if(millis()-prevLastOutTime>200 && millis()-lastInTime<500) {
				if(lineOut<=3) lineOut++;
			}
			if (lineOut > 3) {
				if(millis()-lastInTime>500) {	//after stopping on line for 0.5 sec, ...
					if(lineStop==1) {
						lineStop=2;
						outBallAng = ballAng;
						outBallDist = ballDist;
						Serial.println(outBallAng);
					}
					//stop line tracking if: >40 deg change in ballAng, outBallAng in front but current ballAng in back, >30cm change in ballDist, stopped on line for >5secs, don't detect ball for >1 sec
					if(smallestAngleBetween(outBallAng, ballAng)>30 || millis()-lastInTime>5000 || millis()-lastBallTime>1000) {
					//if (millis() - lastInTime > 4000) {
						lineOut=0;
						lineStop=0;
					} 
					else {
						lineStop = 2;
						Serial.println("track!");
					}
				} 
				else {	//stop on line for 0.5secs
					lineStop = 1;
					Serial.println("stop!");
				}
			}
		}
		*/
	} else {	//goalie program
		float angle=0, angle2=0, dist=0, rotation=0;
		if ((millis()-lastBallTime > 1000 || ballDist>100) && !frontGate && !backGate) {  //no ball, align to centre of goal
			angle = smallestAngleBetween(clusterStart,oppGoalAng) < smallestAngleBetween(clusterEnd,oppGoalAng) ? clusterStart : clusterEnd;  //find line angle closer to goal
			
			//line based method (i.e. make ball 90 deg from line)
			angle2 = midAngleBetween(clusterStart,clusterEnd);
			angle2 = smallestAngleBetween(mod(angle2+90,360),angle) < smallestAngleBetween(mod(angle2-90,360),angle) ? mod(angle2+90,360) : mod(angle2-90,360);
			// angleDeviation = abs(90-smallestAngleBetween(angle2,oppGoalAng));
			
			//compass based method (i.e. assume 180 as desired goal angle)
			angleDeviation = mod(oppGoalAng+cmpangle,360)-180;	//negative means move towards 0-180deg range
			angleDeviation = abs(angleDeviation);
			if(newGoalData) {
				float P, I, D;
				P = angleDeviation * 10.0;
				I = (I * 0.8 + angleDeviation)*10.0;
				D = (angleDeviation - prevAngleDeviation)*30.0;
				goalieCorrection = P+I+D;
				prevAngleDeviation = angleDeviation;
				//Serial.print(angle); Serial.print(" "); Serial.println(min(1100,450*sqrt(angleDeviation/2))+dist*200.0);Serial.print(ballAng);Serial.print(" ");Serial.print(angle);Serial.print(" ");Serial.print(angleDeviation);Serial.print(" ");Serial.print(P);Serial.print(" ");Serial.print(I);Serial.print(" ");Serial.print(D);Serial.print(" ");Serial.println(goalieCorrection);
			}
		} else {  //have ball, align to ball
			angle = smallestAngleBetween(clusterStart,ballAng) < smallestAngleBetween(clusterEnd,ballAng) ? clusterStart : clusterEnd;  //find line angle closer to ball
			
			//line based method (i.e. make ball 90 deg from line)
			angle2 = midAngleBetween(clusterStart,clusterEnd);
			angle2 = smallestAngleBetween(mod(angle2+90,360),angle) < smallestAngleBetween(mod(angle2-90,360),angle) ? mod(angle2+90,360) : mod(angle2-90,360);
			//angleDeviation = abs(90-smallestAngleBetween(angle2,ballAng));
			
			//compass based emthod (i.e. assume 0 as desired ball angle)
			angleDeviation = mod(ballAng+cmpangle,360) <= 180 ? mod(ballAng+cmpangle,360) : mod(ballAng+cmpangle,360)-360;	//neagtive means move towards 180 to 360 deg range
			angleDeviation = abs(angleDeviation);
			if(newBallData) {
				float P, I, D;
				P = angleDeviation * 30.0;
				I = (I * 0.8 + angleDeviation)*70.0;
				D = (angleDeviation - prevAngleDeviation)*90.0;
				goalieCorrection = P+I+D;
				prevAngleDeviation = angleDeviation;
				//Serial.print(angle); Serial.print(" "); Serial.println(min(1100,450*sqrt(angleDeviation/2))+dist*200.0);Serial.print(ballAng);Serial.print(" ");Serial.print(angle);Serial.print(" ");Serial.print(angleDeviation);Serial.print(" ");Serial.print(P);Serial.print(" ");Serial.print(I);Serial.print(" ");Serial.print(D);Serial.print(" ");Serial.println(goalieCorrection);
			}
		}
		//Serial.print(oppGoalDist); Serial.print(" ");Serial.print(angle2);Serial.print(" ");Serial.println(angleDeviation);
		dist = lineLen > 1 ? lineLen-1 : 1-lineLen;
		if (!onLine) {	//return to goal
			// float tempAng2 = mod(ABangle+180,360);
			// float tempDist2 =55;
			// float angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
			// float dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));

			// if (dist>=10) angular_drive(min(dist*70.0,1300),angle,cmpCorrection,300);
			// else angular_drive(0,0,0);
			float goalMult, goalOffset;
			int tempAng = oppGoalAng - 180;
			if (tempAng >= 0) {
				goalOffset = fmin(tempAng*1.0,90);
			} else {
				goalOffset = fmax(tempAng*1.0,-90);
			}
			if (oppGoalAng < 100 || oppGoalAng > 260) goalMult = 0.3;
			else goalMult = 0;
			robotAng = mod(oppGoalAng + goalOffset*goalMult,360);
			if(tof[0]<500) angular_drive(800,robotAng,cmpCorrection);
			else angular_drive(1200,robotAng,cmpCorrection,300);
		} 
		else if(oppGoalDist > 75) {
			float goalMult, goalOffset;
			int tempAng = oppGoalAng - 180;
			if (tempAng >= 0) {
				goalOffset = fmin(tempAng*1.0,90);
			} else {
				goalOffset = fmax(tempAng*1.0,-90);
			}
			if (oppGoalAng < 100 || oppGoalAng > 260) goalMult = 0.3;
			else goalMult = 0;
			robotAng = mod(oppGoalAng + goalOffset*goalMult,360);
			angular_drive(900,robotAng,cmpCorrection,300);
		}
		else {	//track ball or goal
			if (angle2 > 115 && angle2 < 245) angular_drive(0,0,cmpCorrection, 900);
			else angular_drive(min(goalieCorrection,1700)+dist*100.0,angle, cmpCorrection,200);
			//Serial.print(min(1100,450*sqrt(angleDeviation/6.0)));Serial.print(" ");Serial.print(oppGoalAng);Serial.print(" ");Serial.print(ballAng);Serial.print(" ");Serial.println(angleDeviation);
		}
	}
  
	
  	//angular_drive(800,180,0);
	if (millis()-lastCmpTimer > 500) stopMove = true;
	else stopMove = false;

  	if(stopMove) {
		//angular_drive(0,0,cmpCorrection,900);
		angular_drive(0,0,0);
	}
	else {
		//calcToPoint(200,300);
		// float *vect;
		// vect = calcPos();
		// //Serial.print(*vect);Serial.print(" ");Serial.print(*(vect+1));Serial.print(" ");Serial.print(*(vect+2));Serial.print(" ");Serial.print(*(vect+3));Serial.println(" ");
		// float angToCentre = mod(atan2(-(*vect), -(*(vect+1)))*180/PI,360);
    	// float distToCentre = sqrt(pow(*vect,2) + pow(*(vect+1),2));
		// //Serial.print(angToCentre);Serial.print(" ");Serial.println(distToCentre);
		// // vect = calcToTOFpoint(300,300, 0,1);
		// if (abs(*vect) < *(vect+2)/1.5 && abs(*(vec t+1)) < *(vect+3)/1.5) {	//if x and y coord of robot within bounding box
		// 	angular_drive(0,0,cmpCorrection,900);
		// 	//Serial.println("stop!");
		// } else {
		// 	angular_drive(min(distToCentre*8.0,1300),angToCentre,cmpCorrection,300);
		// }
	}

	// if (lineStop) {
	// 	serialWrite(L4Serial, '2');
	// 	serialWrite(L4Serial, '4');
	// 	angular_drive(0,0,cmpCorrection,800);
	// }
	//else if (lineStop == 2) lineTrack();

	move_OUT();

	//state switching stuff
#ifdef whitebot
	//if ball further away (weighted to favour whitebot), last switch time was more than 0.2s ago, currently attack mode
	if (ballDist > (float)otherBallDist*0.9 && millis() - lastBTswitchTime > 200 && attackState) {
		btSerial.print("XR1|");	//'X':bluetooth, 'R':rotate roles, '1':Be attack
		Serial.println("im defence");
		attackState = 0;
		lastBTswitchTime = millis();
	}
	//if ball further away (weighted to favour whitebot), last switch time was more than 0.2s ago, currently attack mode
	else if ((float)ballDist*0.9 < otherBallDist && millis() - lastBTswitchTime > 200 && !attackState) {
		btSerial.print("XR0|");	//'X':bluetooth, 'R':rotate roles, '0':Be defence
		Serial.println("im attack");
		attackState = 1;
		lastBTswitchTime = millis();
	}
#endif
#ifdef blackbot
	if(millis()-lastBTsendTime > 5) {	//cap bt send data rate to 200hz
		char x[40];
		sprintf(x,"XQ%d %.0f %d %.0f|", ballDist, mod(ballAng+cmpangle,360), oppGoalDist, mod(oppGoalAng+cmpangle,360));
		btSerial.print(x);
		//Serial.println(x);
		lastBTsendTime = millis();
	}
	
#endif
	//switch goalie to attack if ball in either catchment
	if(((frontGate || millis()-lastFrontTime < 200) || (backGate || millis()-backTimer < 200)) && !attackState) {
		attackState = 1;
	}

#ifdef DEBUG
	if (battTime >= 1000)
	{
		batt.update();
		if (batt.hasChanged())
		{
			Serial.print("Battery Level: \t");
			Serial.println((float)(batt.getValue() / 77.5));
		}
		battTime = 0;
	}
#endif

	//DIP switches
	DIP1.update();
	DIP2.update();
	DIP3.update();
	DIP4.update();

	//LS calibration program
	if (DIP1.fell())
	{
//#ifdef DEBUG
		Serial.println("Running LS Cal...");
//#endif
		serialWrite(L1Serial, 'N');
		while (!DIP1.rose())
		{
			recvCalib();
			DIP1.update();
		}
		serialWrite(L1Serial, 'L');
		while (!recvCalib())
			;
	}
}
