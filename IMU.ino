void readIMU(byte *data)
{
	cmpangle = atof((char *)data);
	//Serial.print((char *) data); Serial.print(" ");
#ifdef DEBUG
	//Serial.println(cmpangle);
#endif
	processIMU();
}
void processIMU()
{
	 //moddedAngle = cmpangle;
#ifdef blackbot
	if (attackState==4) {
		moddedAngle = cmpangle;
	}

	if (attackState==5) {
		error = mod(cmpangle-moddedAngle,360) <= 180 ? -mod(cmpangle-moddedAngle,360) : -(mod(cmpangle-moddedAngle,360)-360);
	} else {
		error = cmpangle <= 180 ? -cmpangle : -(cmpangle-360);
	}
	
#endif
	
	float P, I, D;
	P = error*0.04;
	I = (I * 0.95 + error)*0.18;
	D = (error - prevError)*1.0;
	cmpCorrection = P+I+D;
	prevError = error;
	//Serial.println(millis()-lastCmpTimer);
	lastCmpTimer = millis();
	
}
