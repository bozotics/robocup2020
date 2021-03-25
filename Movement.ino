void calcRobotAngle() {
    if (millis()-lastBallTime > 1000) {
		angular_drive(0, 0, blueAng<=180 ? -blueAng/18 : -(blueAng-360)/18);  	
	} else {
		float ballFactor = 1.0-ballDist/61.4;	//convert ball distance to a percentage of closeness
        //float ballFactor = ballDist<22 ? 1.0 : 1.0-ballDist/43.0
        float ballMult = fmin(0.024*pow(e,4.7*ballFactor),1);   //to lessen curve amount: decrease a,b,c
        float ballOffset;
        /*if(ballAng<180) ballOffset = fmin(ballAng,90);
        else ballOffset = fmax(ballAng-360,-90);*/
        if(ballAng<180) {
            //ballOffset = fmax(fmin(pow(e,(0.05*ballAng))-1,90),0);
            //ballOffset = fmax(fmin(185*(1/(1+pow(e,(-ballAng/20)))-0.5),90),-90);
            ballOffset = fmin(ballAng*0.95,90);
        } else {
            //ballOffset = fmax(fmin(pow(e,(0.05*(ballAng-360)))-1,90),0);
            //ballOffset = fmax(fmin(185*(1/(1+pow(e,(-(ballAng-360)/20)))-0.5),90),-90);
            ballOffset = fmax((ballAng-360)*0.95,-90);
        }

        robotAng = ballAng + ballOffset*ballMult;
        Serial.print(ballAng); Serial.print(" "); Serial.println(ballOffset);
        angular_drive(1200,robotAng, blueAng<=180 ? -blueAng/18 : -(blueAng-360)/18);
	}
}

void angular_drive(int speed, float angle, float angVel) {
  float radAng = angle*pi/180;

  float x_co = sinf(radAng)*0.707;  //all robot vectors are +-(1/sqrt2)
  float y_co = cosf(radAng)*0.707;

  float fl=(x_co+y_co + 0.1 * angVel);
  float bl=(-x_co+y_co + 0.1 * angVel);
  float fr=-(x_co-y_co + 0.1 * angVel);
  float br=-(-x_co-y_co + 0.1 * angVel);

  FLout = round(fl*speed);
  FRout = round(fr*speed);
  BLout = round(bl*speed);
  BRout = round(br*speed);
}

void move_OUT() {
analogWrite(PWMM1,abs(FLout));
  digitalWriteFast(INM1, (FLout>0 ? LOW : HIGH));

  analogWrite(PWMM2,abs(FRout));
  digitalWriteFast(INM2, (FRout>0 ? HIGH : LOW));

  analogWrite(PWMM3,abs(BLout));
  digitalWriteFast(INM3, (BLout>0 ? HIGH : LOW));

  analogWrite(PWMM4,abs(BRout));
  digitalWriteFast(INM4, (BRout>0 ? LOW : HIGH));
}

float mod(float x, float y) {
  x = fmod(x,y);
  return x < 0 ? x+y : x;
}