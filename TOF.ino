void processTOF() {
    //for (int i=0; i<4; i++) tof[i] = cos(degToRad(cmpangle)) * tof[i];  //correct for angle

    //flagging for bad data
    tofFlagCnt = 0;
    for(int i=0; i<4; i++) {
        if(tof[i] < 150) {      //if value less than 150, hard flag it
            tofFlag[i] = 1;
            flagTimer[i] = millis();
            prevtof[i] = tof[i];
            tofFlagCnt++;
        } 
        else if (tof[i] < prevtof[i]*0.7) {   //if value dropped by more than 0.7 of prev value, soft flag it
            if (!tofFlag[i]) flagTimer[i] = millis();
            tofFlag[i] = 2;
            tofFlagCnt++;
        }
        else {
            tofFlag[i] = 0;
            prevtof[i] = tof[i];
        }

        if (millis() - flagTimer[i] > 3000) {   //if timer called for longer than 3 secs, remove flag
            tofFlag[i] = 0;
            prevtof[i] = tof[i];
        }
    }

    if (tof[0] < 400 || tof[1] < 400 || tof[2] < 400 || tof[3] < 400) robotSpeed = 1000;
    else robotSpeed = 1300;

    newTOFdata = true;
    // for (int i=0; i<4; i++) {
    //     Serial.print(tof[i]);
    //     if(tofFlag[i]) Serial.print("[x]");
    //     Serial.print(" ");
    // }
    // Serial.println();
    //calcToCentre();
}

float processTOFout() {     //returns angle to move at to go back in field
    //flagging for out detection
    tofOutCnt = 0;
    memset(tofOut,0,sizeof(tofOut));
    for(int i=0; i<4; i++) {
        if (tofFlag[i]) continue;   //ignore value if its flagged
        if (tof[i] < TOF_THRESH[i]) {
            tofOut[tofOutCnt] = i;
            tofOutCnt++;
        }
    }

    if (tofOutCnt == 0 || tofOutCnt == 4) return -1; //NO or ALL tof detect out so fuck these cases
    if (tofOutCnt == 1) return mod(tofOut[0]*90 + 180,360);
    if (tofOutCnt == 2) {
        if ((tofOut[0] == 0 && tofOut[1] == 2) || (tofOut[0] == 1 && tofOut[1] == 3)) {
            return -1;    //fuck cases of front && back or left && right
        }
        if(tofOut[0] == 0 && tofOut[1] == 3) tofOut[0] = 4; //special case to make avg method work
        return mod((float)(tofOut[0] + tofOut[1])*0.5 * 90 + 180,360);
    }
    if (tofOutCnt == 3) {
        if(tofOut[0] == 0) {
            if(tofOut[1] == 1) {
                if (tofOut[2] == 2) return 270; //detect 0,1,2, move in 270 deg
                else return 180;    //detect 0,1,3, move in 180 deg
            } 
            else return 90; //detect 0,2,3, move in 90 deg
        }
        else return 0;   //detect 1,2,3, move in 0 deg
    }
    Serial.println("wtf?");
    return -1;  //if program comes here smth fked up
}

float * calcPos() { //origin starts from centre of field
    static float currPos[4];   //X coord, Y coord, box length X, box length Y
    int topDist = FIELD_Y - tof[0]; // dist of field top to bounding box top
    int bottomDist = tof[2];    // dist of field bottom to bounding box bottom
    int leftDist = tof[3] ;   // dist of field left to bounding box left
    int rightDist = FIELD_X-tof[1];    //dist of field right to bounding box right
    currPos[3] = abs(topDist-bottomDist);    //height of bounding box
    currPos[2]= abs(rightDist-leftDist);    //length of bounding box
    currPos[1] = bottomDist + (topDist-bottomDist)*0.5 - FIELD_Y*0.5;   //height from centre
    currPos[0] = leftDist + (rightDist-leftDist)*0.5 - FIELD_X*0.5;     //length from centre
    return currPos;
}

// void calcToCentre() {
//     //settle vertical axis
//     // if (!tofFlag[0] && !tofFlag[2]) {   //no flags on BOTH tof, take average to find pos of robot
//     //     float midY = (tof[0] + tof[2])*0.5;
//     //     tofCentreVectY = tof[0]-midY;
//     // }
//     // else if (!tofFlag[0]) tofCentreVectY = tof[0]-CENTRE_DIST_Y;     //no flags on tof[0], only trust tof[0] to find pos   
//     // else if (!tofFlag[2]) tofCentreVectY = CENTRE_DIST_Y - tof[2];   //no flags on tof[2], only trust tof[2] to find pos
//     // else {    // double flagged
//     //     if(tofFlag[0] == 2 && tofFlag[2]==1) tofCentreVectY = tof[0]-CENTRE_DIST_Y;          //tof[0] soft flagged, tof[2] hard flagged, only trust tof[0]
//     //     else if(tofFlag[0] == 1 && tofFlag[0]==2) tofCentreVectY = CENTRE_DIST_Y - tof[2];   //tof[2] soft flagged, tof[0] hard flagged, only trust tof[2]
//     //     else if(tofFlag[0] == 2 && tofFlag[2]==2)  { //BOTH SOFT flagged, trust tof which soft flagged for longer time already
//     //         tofCentreVectY = (flagTimer[0] < flagTimer[2]) ? tof[0]-CENTRE_DIST_Y : CENTRE_DIST_Y - tof[2] ; 
//     //     } else {    //BOTH HARD flagged
//     //         tofCentreVectY = 0;  //better to not move than move in wrong direction
//     //     }
//     // }
//     //settle horizontal axis
//     if (!tofFlag[1] && !tofFlag[3]) {   //no flags on BOTH tof, take average to find pos of robot
//         float midX = (tof[1] + tof[3])*0.5;
//         tofCentreVectX = tof[1]-midX;
//     }
//     else if (!tofFlag[1]) tofCentreVectX = tof[1]-CENTRE_DIST_X;     //no flags on tof[1], only trust tof[1] to find pos   
//     else if (!tofFlag[3]) tofCentreVectX = CENTRE_DIST_X - tof[3];   //no flags on tof[3], only trust tof[3] to find pos
//     else {    // double flagged
//         if(tofFlag[1] == 2 && tofFlag[3]==1) tofCentreVectX = tof[1]-CENTRE_DIST_X;          //tof[1] soft flagged, tof[3] hard flagged, only trust tof[1]
//         else if(tofFlag[1] == 1 && tofFlag[1]==2) tofCentreVectX = CENTRE_DIST_X - tof[3];   //tof[3] soft flagged, tof[1] hard flagged, only trust tof[3]
//         else if(tofFlag[1] == 2 && tofFlag[3]==2)  { //BOTH SOFT flagged, trust tof which soft flagged for longer time already
//             tofCentreVectX = (flagTimer[1] < flagTimer[3]) ? tof[1]-CENTRE_DIST_X : CENTRE_DIST_X - tof[3] ; 
//         } else {    //BOTH HARD flagged
//             tofCentreVectX = 0;  //better to not move than move in wrong direction
//         }
//     }
//     tofCentreAng = mod(atan2(tofCentreVectX, tofCentreVectY)*180/PI,360);
//     tofCentreDist = sqrt(pow(tofCentreVectX,2) + pow(tofCentreVectY,2));

//     // Serial.print(tofCentreVectX);Serial.print(" ");
//     // Serial.print(tofCentreVectY); Serial.print(" ");
//     // Serial.print(tofCentreDist);Serial.print(" ");
//     // Serial.println(tofCentreAng);
// }

float * calcToPoint(float x, float y) {    //calc vector to x, y displacements from centre in mm
    float *vect;
    vect = calcPos();

    static float polarVector[6];  //distance, angle, x, y, length of bounding box, height of bounding box
    x -= *vect;
    y -= *(vect+1);
    polarVector[0] = sqrt(pow(x,2) + pow(y,2));
    polarVector[1] = mod(atan2(x, y)*180/PI,360);
    polarVector[2] = x;
    polarVector[3] = y;
    polarVector[4] = *(vect+2);
    polarVector[5] = *(vect+3);
    // if (abs(x) < *(vect+2)/1.5 && abs(y) < *(vect+3)/1.5) {	//if x and y coord of robot within bounding box
    //     angular_drive(0,0,cmpCorrection,900);
    //     //Serial.println("stop!");
    // } else {
    //     angular_drive(min(polarVector[0]*8.0,1300),polarVector[1],cmpCorrection,300);
    // }
    return polarVector;
    // Serial.print(x);Serial.print(" ");Serial.print(y);Serial.print(" ");
    // Serial.print(polarVector[0]);Serial.print(" ");Serial.println(polarVector[1]);
}

float * calcToTOFpoint(float FBdist, float LRdist, int FB, int LR) {   //calc vector needed to reach desired tof readings (uses only 1 vertical and 1 horizontal tof)
    float xyVector[2];
    static float polarVector[2];
    //set X vector
    if(FB == 0) xyVector[1] = tof[FB] - FBdist;
    else xyVector[1] = FBdist - tof[FB];
    //set Y vector
    if(LR == 1) xyVector[0] = tof[LR] - LRdist;
    else xyVector[0] = LRdist - tof[LR];

    polarVector[0] = sqrt(pow(xyVector[0],2) + pow(xyVector[1],2));
    polarVector[1] = mod(atan2(xyVector[0], xyVector[1])*180/PI,360);
    //angular_drive(min(polarVector[0]*5.0,1300),polarVector[1],cmpCorrection,300);
    //Serial.print(xyVector[0]);Serial.print(" ");Serial.print(xyVector[1]);Serial.print(" ");Serial.print(polarVector[0]);Serial.print(" ");Serial.println(polarVector[1]);

    return polarVector;
}

void calcToLinePoint(int point) {    //move to preset line points on field, negative value of same point means use only tof (dont track line)
    float *vectorToPoint, lineAngleRef, lineDistRef;
    switch(abs(point)) {
        case 1: //top left corner line
            vectorToPoint = calcToTOFpoint(300,300,0,3);
            break;
        case 2: //top right corner line
            vectorToPoint = calcToTOFpoint(300,300,0,1);
            break;
        case 3: //mid left line
            vectorToPoint = calcToPoint(-750,0);
            break;
        case 4: //mid right line
            vectorToPoint = calcToPoint(750,0);
            break;
        case 5: //bottom left corner line
            vectorToPoint = calcToTOFpoint(250,250,2,3);
            break;
        case 6: //bottom right corner line
            vectorToPoint = calcToTOFpoint(250,250,2,1);
            break;
        case 7: //top middle of penalty area
            vectorToPoint = calcToPoint(0,700);
            break;
        case 8:
            vectorToPoint = calcToPoint(0,-700);
            break;
    }

    if (onLine && point > 0) {
        lineAngleRef = smallestAngleBetween(clusterStart,*(vectorToPoint+1)) < smallestAngleBetween(clusterEnd,*(vectorToPoint+1)) ? clusterStart : clusterEnd;
        lineDistRef = lineLen > 1 ? lineLen-1 : 1-lineLen;
        angular_drive(min(*vectorToPoint*5.0,1000)+lineDistRef*200.0,lineAngleRef,cmpCorrection,200);
    }
    else {
        angular_drive(min(*vectorToPoint*5.0,1300),*(vectorToPoint+1),cmpCorrection,300);
    }
}