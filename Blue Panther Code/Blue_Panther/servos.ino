//Modify to resemble revised flameSPA for two servos. Use arays and pointers


int servoDir[2] = {UNKNOWN, UNKNOWN};  //Servo direction: UNKNOWN, LEFT or RIGHT

int servoPos[2];

long cmilCycle[2] = {0, 0};
long pmilCycle[2] = {0, 0};
long dlymilCycle[2] = {20, 20};    //Servo delay

void cycleServo(int servoID, int startPos, int endPos, int stepSize, long interval){
//Cycles the servo continuously between two positions.
//This is non-blocking, and this function needs to be repeatedly called to cycle the servos as expected.
//The servo's position and direction are stored in servoPos[] and servoDir[]. These may be viewed using the observer methods: getServoPos and getServoDir
//The vertical servo's directions may only be up or down. This code was originally written for left or right.
//My quick fix is a simple conditional at the end. Horizontal left is equivalent to vertical down, and right to up.
  
  dlymilCycle[servoID] = interval;
  cmilCycle[servoID] = millis();
  if (dlymilCycle[servoID] < (cmilCycle[servoID] - pmilCycle[servoID])) {
    switch (servoDir[servoID]) {
      case UNKNOWN:    //First method call, no direction yet
        if (startPos < endPos) {
          servoDir[servoID] = RIGHT;
          servoPos[servoID] = startPos;
        }
        else {
          //Swap startPos and endPos, and start moving the servo from the right.
          int temp = startPos;
          startPos = endPos;
          endPos = temp;
          servoDir[servoID] = LEFT;
          servoPos[servoID] = endPos;
        }
        break;
      case LEFT:    //Left
        //Order startPos and endPos from left to right, always.
        if (endPos < startPos) {
          int temp = startPos;
          startPos = endPos;
          endPos = temp;
        }
        if (startPos < (servoPos[X] - stepSize)) {
          servoPos[servoID] -= stepSize;
        }
        else {
          servoDir[servoID] = RIGHT;
          servoPos[servoID] = startPos;
        }
        break;
      case RIGHT:    //Right
        //Order startPos and endPos from left to right, always.
        if (endPos < startPos) {
          int temp = startPos;
          startPos = endPos;
          endPos = temp;
        }
        if ((servoPos[servoID] + stepSize) < endPos) {
          servoPos[servoID] += stepSize;
        }
        else {
          servoDir[servoID] = LEFT;
          servoPos[servoID] = endPos;
        }
        break;
    }
    
    if (servoID == Y) {
      if (servoDir[Y] == LEFT) {
        servoDir[Y] = DOWN;
      }
      else if (servoDir[Y] == RIGHT) {
        servoDir[Y] = UP;
      }
    }
    
    servo[servoID]->write(posProcess(servoID, servoPos[servoID]));
    pmilCycle[servoID] = millis();
  }
}

int getServoPos(int servoID) {
  return servoPos[servoID];
}

int getServoDir(int servoID) {
  return servoDir[servoID];
}


void servoPowerOn() {
//Only turns on one servo (horizontal-servo by default). The power to x and y may be toggled by flameSPA. Only one servo should be on at a given time.
  digitalWrite(servoPowerPin[X], HIGH);
  servoPower[X] = ON;
  servoPower[Y] = OFF;
//  digitalWrite(servoPowerPin[Y], HIGH);
//  attachServos();
}

void servoPowerOff() {
  digitalWrite(servoPowerPin[X], LOW);
  digitalWrite(servoPowerPin[Y], LOW);
  servoPower[X] = OFF;
  servoPower[Y] = OFF;
//  detachServos();
}

void servoOn(int servoID) {
  digitalWrite(servoPowerPin[servoID], HIGH);
}

void servoOff(int servoID) {
  digitalWrite(servoPowerPin[servoID], LOW);
}

void attachServos() {
  servoX.attach(servoXPin);
  servoY.attach(servoYPin, 544, 2600);  //Push the vertical servo's range
}

void detachServos() {
  servoX.detach();
  servoY.detach();
}

