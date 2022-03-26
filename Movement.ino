void calcRobotAngle() {
  centreAng = mod((yellowAng*PI/180+atan2(blueDist*sin(blueAng*PI/180-yellowAng*PI/180), yellowDist + blueDist*cos(blueAng*PI/180-yellowAng*PI/180)))*180/PI , 360);
  centreDist = sqrt(pow(yellowDist,2) + pow(blueDist,2) + 2*yellowDist*blueDist*cos(blueAng*PI/180-yellowAng*PI/180)) / 2;
	 //angle from centre to yellow, relative to robot
	float tempAng = centreAng + 180;	//negative vector
	ABangle = mod((tempAng*PI/180+atan2(currGoalDist*sin(currGoalAng*PI/180-tempAng*PI/180), centreDist + currGoalDist*cos(currGoalAng*PI/180-tempAng*PI/180)))*180/PI , 360);
	ABdist = sqrt(pow(centreDist,2) + pow(currGoalDist,2) + 2*centreDist*currGoalDist*cos(currGoalAng*PI/180-tempAng*PI/180));

  //robotSpeed = 1200;
  //robotSpeed = min(1200,1.15*pow(e,0.127*smallestAngleBetween(0,robotAng))+800);
  //Serial.print(robotAng); Serial.print(" ");Serial.print(min(smallestAngleBetween(0,robotAng),smallestAngleBetween(180,robotAng)));Serial.print(" ");Serial.println(robotSpeed);
  if (millis()-lastBallTime > 500 && !frontGate && !backGate) { //no ball
    //if (millis()-lastBTrecvTime > 500) { //no info from other robot too, move to between both goals
      // float tempAng2 = mod(ABangle+180,360);
      // float tempDist2 =20;
      // float angle = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
      // float dist = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));

      // //if (dist>=10) angular_drive(min(dist*70.0,1300),angle,ABangle <= 180 ? ABangle/30 : (ABangle-360)/30);
      // if (dist>=10) angular_drive(min(dist*70.0,1300),angle,cmpangle <= 180 ? -cmpangle/30 : -(cmpangle-360)/30);
      // else angular_drive(0,0,0);
      float *vect;
      vect = calcToPoint(-100,-200);
      if (abs(*(vect+2)) < *(vect+4)/1.5 && abs(*(vect+3)) < *(vect+5)/1.5) {	//if x and y coord of robot within bounding box
        angular_drive(0,0,cmpCorrection,900);
        //Serial.println("stop!");
      } else {
        angular_drive(min(*vect*8.0,1300),*(vect+1),cmpCorrection,300);
      }
    //}
    // else {  //have other ball data
    //   float *vect;
    //   vect = calcPos();
    //   float absBallX = otherOppGoalDist * sin(degToRad(otherOppGoalAng-180)) + otherBallDist * sin(degToRad(otherBallAng));
		// 	float absBallY = otherOppGoalDist * cos(degToRad(otherOppGoalAng-180))-60 + otherBallDist * cos(degToRad(otherBallAng));
    //   float relBallX = absBallX-((*vect*0.1)+10);
		// 	float relBallY = absBallY-(*(vect+1)*0.1);
    //   ballAng = mod(radToDeg(atan2(relBallX,relBallY)),360);
    //   ballDist = sqrt(pow(relBallX,2)+pow(relBallY,2));

    //   float ballOffset;
    //   if(ballAng<180) ballOffset = fmin(ballAng*1.0,90);
    //   else ballOffset = fmax((ballAng-360)*1.0,-90);
    //   robotAng = ballAng + ballOffset * 0.7;
    //   // angular_drive(robotSpeed,robotAng, ABangle<=180 ? ABangle/20 : (ABangle-360)/20);
    //   angular_drive(900,robotAng, cmpCorrection,300);
    // }
  
	} else {
    if (backGate || millis()-backTimer < 100) { //if ball in back: chase curr goal and flick when close
      bool left;
      float angle,dist;
      
      // float tempAng2 = mod(ABangle-60,360);
			// float tempDist2 = 30;
      // float angle2 = mod((centreAng*PI/180+atan2(tempDist2*sin(tempAng2*PI/180-centreAng*PI/180), centreDist + tempDist2*cos(tempAng2*PI/180-centreAng*PI/180)))*180/PI , 360);
      // float dist2 = sqrt(pow(centreDist,2) + pow(tempDist2,2) + 2*centreDist*tempDist2*cos(tempAng2*PI/180-centreAng*PI/180));

      // float tempAng3 = mod(ABangle+60,360);
			// float tempDist3 = 30;
      // float angle3 = mod((centreAng*PI/180+atan2(tempDist3*sin(tempAng3*PI/180-centreAng*PI/180), centreDist + tempDist3*cos(tempAng3*PI/180-centreAng*PI/180)))*180/PI , 360);
      // float dist3 = sqrt(pow(centreDist,2) + pow(tempDist3,2) + 2*centreDist*tempDist3*cos(tempAng3*PI/180-centreAng*PI/180));
      
      // if (dist2 < dist3) {  //move to closest neutral point
      //   angle = angle2;
      //   dist = dist2;
      //   left = true;
      // } else {
      //   angle = angle3;
      //   dist = dist3;
      //   left = false;
      // }

      float * polarVect1;
      float * polarVect2;
      polarVect1 = calcToPoint(-200,300);
      polarVect2 = calcToPoint(200,300);
      
      if (*polarVect1 < *polarVect2) {
        angle = *(polarVect1+1);
        dist = *polarVect1;
        left = true;
      } else {
        angle = *(polarVect2+1);
        dist = *polarVect2;
        left = false;
      }
      //if (dist>=10) angular_drive(min(dist*50.0,800),angle,ABangle <= 180 ? ABangle/30 : (ABangle-360)/30);
      if (dist>=80) angular_drive(min(dist*50.0,750),angle,cmpCorrection,200);
      else {  //at point alrdy
        if(left) { 
            //spin anti clock
            FLout = -1000; FRout = 1000; BLout = -1000; BRout = 1000;
        } else {
            //spin clock
            FLout = 1000; FRout = -1000; BLout = 1000; BRout = -1000;
        }
        unsigned long now = millis();
        while (millis()-now < 200) {
          recv(L3Serial,3);
	        recv(L1Serial,1);
          recv(L4Serial,4);
          recv(piSerial,2);
          //Serial.println(millis());
          //backGate = false;
          move_OUT();
        }
      }
      /*if (oppGoalAng >= 180) angular_drive(800, mod(currGoalAng+10,360), currGoalAng<=180 ? currGoalAng/16 : (currGoalAng-360)/16);
      else angular_drive(800, mod(currGoalAng-10,360), currGoalAng<=180 ? currGoalAng/16 : (currGoalAng-360)/16);
      //Serial.println("lmfao");
      if (currGoalDist < 55) {
        if (oppGoalAng >= 180) { //yellow goal to the left of robot
          FLout=900;FRout=-900;BLout=900;BRout=-900;
          unsigned long now = millis();
          while (millis()-now < 200) {
            Serial.println(millis());
            move_OUT();
          }
        } else {
          FLout=-900;FRout=900;BLout=-900;BRout=900;
          unsigned long now = millis();
          while (millis()-now < 200) {
            Serial.println(millis());
            move_OUT();
          }
        }
      }*/
    }
    else if ((frontGate || millis()-lastFrontTime < 500) && millis() - firstFrontTime > 100) {  //ball in front, aim for goal
      // float goalOffset;
      // if(currGoalAng<180) goalOffset = fmin(currGoalAng*1.0,90);
      // else goalOffset = fmax((currGoalAng-360)*1.0,-90);
      
      // robotAng = currGoalAng + goalOffset*0.9;
      // // angular_drive(robotSpeed,robotAng, ABangle<=180 ? ABangle/20 : (ABangle-360)/20);
      // angular_drive(robotSpeed,robotAng, cmpCorrection,300);
      angular_drive(min(centreDist*50.0,1000), centreAng,cmpCorrection,300);
    }
    else {  //ball not in either catchment: chase ball
      double ballFactor = 1.0-ballDist/90.0;	//convert ball distance to a percentage of closeness
      //float ballFactor = ballDist<22 ? 1.0 : 1.0-ballDist/43.0
      double ballMult = fmin(0.0118*pow(e,5.6*ballFactor),1);   //to lessen curve amount: decrease a,b,c
      double ballOffset;

      if (ballAng <= 180 || ballAng >= 180) { //was 135,225
        //chase with front
        if(ballAng<180) {
            //ballOffset = fmax(fmin(pow(e,(0.05*ballAng))-1,90),0);
            //ballOffset = fmax(fmin(185*(1/(1+pow(e,(-ballAng/20)))-0.5),90),-90);
            ballOffset = fmin(ballAng*1.0,90);
        } else {
            //ballOffset = fmax(fmin(pow(e,(0.05*(ballAng-360)))-1,90),0);
            //ballOffset = fmax(fmin(185*(1/(1+pow(e,(-(ballAng-360)/20)))-0.5),90),-90);
            ballOffset = fmax((ballAng-360)*1.0,-90);
        }
        
        //robotAng = ballAng + ballOffset*ballMult;
        if (ballDist > 35) robotAng = ballAng + ballOffset * 0.35;
        else if (ballDist > 20) robotAng = ballAng + ballOffset * 0.7;
        else robotAng = ballAng + ballOffset * 0.95;  //ball directly next to robot
        // angular_drive(robotSpeed,robotAng, ABangle<=180 ? ABangle/20 : (ABangle-360)/20);
        angular_drive(robotSpeed,robotAng, cmpCorrection,300);
      }
      else {
        //chase with back
        int tempAng = ballAng - 180;
        if (tempAng >= 0) {
          ballOffset = fmin(tempAng*1.0,90);
        } else {
          ballOffset = fmax(tempAng*1.0,-90);
        }
        robotAng = mod(ballAng + ballOffset*0.9,360);
        if (ballDist < 40) robotSpeed = 800;
        angular_drive(robotSpeed,robotAng, cmpCorrection,300);
      }
    }
	}
}

void angular_drive(float speed, float angle, float angVel, float angSpeed = -1.0) {
  if (angSpeed == -1.0) angSpeed = speed; //no special angular speed set

  float radAng = angle*pi/180;

  float x_co = sinf(radAng)*0.707;  //all robot vectors are +-(1/sqrt2)
  float y_co = cosf(radAng)*0.707;

  float fl= (x_co+y_co)*speed + (0.1 * angVel)*angSpeed;
  float bl= (-x_co+y_co)*speed + (0.1 * angVel)*angSpeed;
  float fr= (-x_co+y_co)*speed - (0.1 * angVel)*angSpeed;
  float br= (x_co+y_co)*speed - (0.1 * angVel)*angSpeed;

  FLout = round(fl);
  FRout = round(fr);
  BLout = round(bl);
  BRout = round(br);
  //Serial.printf("%d %d %d %d \n", FLout,FRout,BLout,BRout);  
}


void move_OUT() {
  analogWrite(PWMM1,abs(FLout) > minSpeed ? abs(FLout) : 0);
  analogWrite(PWMM2,abs(FRout) > minSpeed ? abs(FRout) : 0);
  analogWrite(PWMM3,abs(BLout) > minSpeed ? abs(BLout) : 0);
  analogWrite(PWMM4,abs(BRout) > minSpeed ? abs(BRout) : 0);

#ifdef whitebot
  digitalWriteFast(INM1, (FLout>0 ? HIGH : LOW));
  digitalWriteFast(INM2, (FRout>0 ? LOW : HIGH));
  digitalWriteFast(INM3, (BLout >0 ? HIGH : LOW));
  digitalWriteFast(INM4, (BRout>0 ? HIGH : LOW));
#endif
#ifdef blackbot
  digitalWriteFast(INM1, (FLout>0 ? LOW : HIGH));
  digitalWriteFast(INM2, (FRout>0 ? LOW : HIGH));
  digitalWriteFast(INM3, (BLout>0 ? LOW : HIGH));
  digitalWriteFast(INM4, (BRout>0 ? HIGH : LOW));
#endif
}
