//_______________________________Variables and constants________________________________________

//Refer to:
  //Extinguisher

  //Flame sensor
//  FlameSensor* fSensor = &flameSensor;
  const int numFlameSensors = NUM_FLAME_SENSORS;  //NUM_FLAME_SENSORS defined in main tab
//  FlameSensor fSensor[numFlameSensors] = flameSensor;
//  FlameSensor* fSensor = flameSensor;
  FlameSensor *fSensor[numFlameSensors] = {&flameSensor[0], &flameSensor[1], &flameSensor[2], &flameSensor[3], &flameSensor[4], &flameSensor[5], &flameSensor[6]};
  
  //Scan
  int heatSig[2] = {0, 0};    //Flame reading
  int maxHeatSig[2] = {0, 0};    //Will store the largest flame reading per scan
  int maxHeatSigPW[2] = {0, 0};  //Initialized to a safe near-zero number. WIll store the pulsw-width corresponding to the largest flame reading.
  int heatSigAllow[2]  = {50, 50};  //For first flameSPA method, i.e. sensor-guided robot flame pointing. Heat sig allowance when pointing robot towards flame.
  int posStep[2] = {2, 1};    //Position step. PW increment for sweep-scanning.
  
  int minFlameVal = 50;  //The minimum value that a flame reading can be.
  int flameStopVal = 970;
  
  //Servo
  Servo *servo[2] = {&servoX, &servoY};
  
  int servoDelay[2] = {20, 20};  //Delay between pulses in ms.
  //Add an offset to better suit the servo's desired scanning range
  //Center potition is straight ahead for horizontal, and level for vertical.
  //Horizontal zero degrees is understood to be 90 degrees left of center. Vertical zero is defined to be 90 degrees less than level.
  int servoOffset[2] = {32, -57};  //Clockwise, or upwards rotation of servo from zero degrees. Lower moves the servo towards zero.
  int sensorFlamePointOffset[2] = {-8, -12};  //How much to adjust/correct the robot arm to point the hose directly at flame, after obtaining max readings from flame scans
  
  //Key positions
  int centerPos[2] = {90, 84};
    //Horizontal servo- 90 degrees means pointing straight
  int leftPos = 50;  //***Left-most position. 90 degrees is straight, less than 90 is left, and greater than 90 is right
  int rightPos = 200;
  int left22 = 70;    //22 degrees left of center
  int right22 = 110;    //22 degrees right of center
  int left30 = 70;
  int right30 = 110;
  int left45 = 50;
  int right45 = 135;
    //Vertical servo-90 degrees means pointing level
  int upPos = 110;  //Pointing straight up
  int downPos = 50;  //Pointing as low as possible without physically touching the horizontal servo
  int up22 = 110;
  int down22 = 75;
  int up45 = 100;
  int down45 = 75;
  
  int lastServoPos;
  
  int sprayDelay = 400;  //Spray time in ms.
  int floodDelay = 1000;
  
  //Avg sensor
  int flameNOR = 5;
  
//______________________________________________________________________________________________

int readFlameSensor(int servoID) {
//Reads the flame sensor array, stores the peak heat reading in the heatsig variable (not the returned value), and returns the index of the sensor with the highest reading.
//Takes a parameter because it has to store the result in the correct servo's heatSig[] variable
  int maxHeatVal = 0;
  int maxFlameSensorIndex = 0;
  for(int i = 0; i < numFlameSensors; i++) {
    int sensorVal = fSensor[i]->read();
    if (maxHeatVal < sensorVal) {
      maxHeatVal = sensorVal;
      maxFlameSensorIndex = i;
    }
  }
  heatSig[servoID] = maxHeatVal;
  return maxFlameSensorIndex;
}

void servoAlign(int servoID, int pos){
//***Blocking method
//Aligns the horizontal servo to a particular position.
//Implement new methods to operate the vertical servo.
  servoOff(1-servoID);  //Turn off the servo not being written to
  servoOn(servoID);  //Turn on the servo being written to
  
  int minPos = 90;
  int maxPos = 90;
  
  if (servoID == X) {
    minPos = leftPos;
    maxPos = rightPos;
  }
  else if (servoID == Y) {
    minPos = downPos;
    maxPos = upPos;
  }
//  int tempPos = constrain(posProcess(servoID, pos), minPos, maxPos);    //Compute the correct servo angle to suit the desired real-world angle
  int tempPos = constrain(pos, minPos, maxPos);    //Compute the correct servo angle to suit the desired real-world angle
  tempPos = posProcess(servoID, pos);
  servo[servoID]->write(tempPos);
  delay(servoDelay[servoID]);
  lastServoPos = pos;
}

boolean flameDetect() {
//Performs a full horizontal flame scan. Returns true f a candle has been detected, false otherwise.
  centerServo(Y);
  flameScanFull(X);
  if (minFlameVal < maxHeatSig[X]) {
    return true;
  }
  else {
    return false;
  }
}

boolean flameScanFull(int servoID){
//Sweep-scan right (and records the maximum heat reading)
//Optimized to use flameSensor array
  int beginPos;
  int endPos;
  
  if (servoID == X) {
//    beginPos = leftPos;
//    endPos = rightPos;
    beginPos = left30; 
    endPos = right30;
  } else if (servoID == Y) {
    beginPos = downPos;
    endPos = upPos;
  }
  sweepScan(servoID, beginPos, endPos, posStep[servoID]);
}

void flameScanMini(int servoID){
//Sweep-scan right (and records the maximum heat reading)
//Obsolete because of flame sensor array
  int beginPos;
  int endPos;
  
  if (servoID == X) {
    beginPos = left45;
    endPos = right45;
  }
  else if (servoID == Y) {
    beginPos = up45;
    endPos = down45;
  }
  sweepScan(servoID, beginPos, endPos, posStep[servoID]);
}

void flameSPA(){
//Flame Search-Point-Approach (SPA)

  //Search for flame
  centerServo(Y);
  flameScanFull(X);
  //If there is a flame
      //Point towards it and approach it. Otherwise exit the room.
  if (minFlameVal < maxHeatSig[X]){
    //Approach the flame. If within extinguishing range then run an extinguishing routine.
    centerServo(X);
    robotFlamePoint();
    flameApproach();
    while (flame.isLit() == true) {
      flameScanMini(X);
      autoRobotFlamePoint();
      flameApproach();    //Extinguisher is called from within this.
    }
  }
  else {
    servoAlign(X, leftPos);
  }

}

void flameApproach(){
//Approaches the flame based n the strangth of the heart-reading. When close enough to the flame, run extinguishing method.
  if (maxHeatSig[X] < 100) {
//    forwardFastFor(1000);
  }
  else if (maxHeatSig[X] < 800) {
//    forwardFastFor(1000);
  }
  else if (maxHeatSig[X] < 900) {    //Prev 975, 950
//    forwardFastFor(1000);
  }
/*
  else if (maxHeatSig < ) {
    forwardFastFor(1000);
  }
*/
  else {  //When close enough to flame
//    flameScanMini(X);
//    sensorFlamePoint(X);
//    flameExtinguishSP();
//    verifyExtinguish();
  }
}

void sensorFlamePoint(int servoID){
//Point sensor and nozzle towards flame
  if (servoID == X) {
    lockIn(servoID, maxHeatSigPW[servoID]+sensorFlamePointOffset[servoID]);
  }
  else {
    lockIn(servoID, maxHeatSigPW[servoID]+sensorFlamePointOffset[servoID]);
  }
}

void robotFlamePoint(){
//Points the robot towards flame (maximum heat reading) immediately after a flame-scan
  int rotationAngle = maxHeatSigPW[X] - centerPos[X];
  rotate(rotationAngle);
}

void autoRobotFlamePoint(){
//Point robot towards flame if its more than five degrees away on either side
  if ((maxHeatSigPW[X] < (centerPos[X] - 5) || (centerPos[X] + 5) < maxHeatSigPW[X])) {
    robotFlamePoint();
  }
}

void flameExtinguishSP(){
//Point sensor towards flame
//  lockIn(X, maxHeatSigPW[X]);
//Fire a single pulse of CO2
  spray(sprayDelay);
}

void flameExtinguishMP(){
//Fire multiple pulses of CO2 in a sweep

//Lock-in left
  lockIn(X, left22);
//Pulse-spray right
  for (int pos = left22; pos < right22; pos+= posStep[X]) {
    //Take one step
    lockInQuick(X, pos);
    //Spray
    spray(sprayDelay);
  }
}

void flameExtinguishFlood(){
//Precondition: The flame is not at the servo edge or 
  int floodAngle[2] = {10, 10};    //flooding range in degrees
  int pwStep[2] = {2, 2};    //Step-size when pulsing the servo to a new angle
//Fire a long pulse of CO2 in an x-y sweep over the flame's detected position, +-the floodAngle.

  int leftBound = maxHeatSigPW[X] - floodAngle[X];
  int rightBound = maxHeatSigPW[X] + floodAngle[X];
  int upBound = maxHeatSigPW[Y] - floodAngle[Y];
  int downBound = maxHeatSigPW[Y] + floodAngle[Y];
  
//Lock-in left of flame
  lockIn(X, leftBound);
//Lock-in above flame
  lockIn(Y, upBound);

//Open valve
  openExtinguisher();  //Open the valve, or push the handle
  
//Sweep the servo ovet the flood-space
  long cmilFlood = millis();
  long pmilFlood = cmilFlood;
  long dlymilFlood = floodDelay;
  
  while ((cmilFlood - pmilFlood) < dlymilFlood) {
    cmilFlood = millis();
    cycleServo(X, leftBound, rightBound, pwStep[X], servoDelay[X]);
    cycleServo(Y, upBound, downBound, pwStep[Y], servoDelay[Y]);
  }
  
//Close valve
  closeExtinguisher();

}

void flameExtinguishDoubleFlood(){
//Fire a long pulse of CO2 in a sweep over the candle range twice.
//***May want to incrementally pulse the servo to its final sweep point, rather than directly pulse it there.

//Lock-in left
  lockIn(X, left22);
//Open valve
//  openExtinghusher();

//Pulse-sweep right
  for (int pos = left22; pos < right22; pos+= 1) {  //Prev. pos += 2, changed to 1 for baloon
    //Take one step
    lockInQuick(X, pos);
    delay(100);
  }
//Pulse-sweep left
  for (int pos = right22; left22 < pos; pos-= 1) {  //Prev. pos += 2, changed to 1 for baloon
    //Take one step
    lockInQuick(X, pos);
    delay(100);
  }
//Close valve
//  closeExtinguisher();

}

void verifyExtinguish(){
//If the flame is still alight, move forward a little and flood the flame with CO2. Otherwise if no flame is seen, note that the flame has been extinguished and 
  while (flame.isLit() == true) {
    //Scan for flame
    flameScanFull(X);
    //If flame is still lit
    if (minFlameVal < maxHeatSig[X]) {    //Prev. minFlameVal
      //Go forward a little, get closer to the flame.
      robotFlamePoint();
      moveDist(5);
      flameExtinguishFlood();
    }
    //Otherwise, if it's extinguished
    else {    //Perhaps add an extinguishVerifiedState, of sorts, to say whether the flame has been put out. Then handle next action from the master program.
      flame.off();    //Notes that flame has been extinguished, so that the master program knows to return home.
    }
  }
}

void openExtinguisher() {
//Opens the CO2 valve and leaves it open
  dcMotorRelayOn();
//  versaOn();
}

void closeExtinguisher() {
//Closes the CO2 valve and leaves it closed
  dcMotorRelayOff();
//  versaOff();
}

void dcMotorRelayOn() {
//Turns on the dc motor that pushed the extinguisher handle to release CO2
  digitalWrite(dcMotorRelayPin, HIGH);
}

void dcMotorRelayOff() {
//Turns off the dc motor that pushed the extinguisher handle to stop releasing CO2
  digitalWrite(dcMotorRelayPin, LOW);
}

void versaOn(){
  digitalWrite(versaPin, HIGH);
}

void versaOff(){
  digitalWrite(versaPin, LOW);
}

void setOutputVersaPins(){
  pinMode(versaPin, OUTPUT);
}

void spray(int delayTime){
//Fires a burst of CO2
  //Open valve
  openExtinguisher();
  //Keep valve open for the interval then close it.
  delay(delayTime);
  //Close valve
  closeExtinguisher();
}

void sweepScan(int servoID, int startPos, int endPos, int pwStep){
//pwStep is not to be confused with posStep, which is a global 'constant'. It is a local variable/parameter.
  //Keep start-and endPos values within rotational range. Does not map their values.
  startPos = constrain(startPos, leftPos, rightPos);
  endPos = constrain(endPos, leftPos, rightPos);
  
  //If from left to right, scan left to right
  if (startPos < endPos){
    //Lock into start position
    lockIn(servoID, startPos);
    //Scan
    maxHeatSig[servoID] = 0;    //Initialize before each scan
    for (int pos = startPos; pos < endPos; pos+= pwStep) {
      servoAlign(servoID, pos);
      //Read the heat signature
      int maxFlameSensorIndex = readFlameSensor(servoID);
      //Note the maximum heat signature and it's position
      if (servoID == X) {
        if (maxHeatSig[servoID] < heatSig[servoID]) {
          maxHeatSig[servoID] = heatSig[servoID];
          maxHeatSigPW[servoID] = (pos - 90) + (maxFlameSensorIndex * 30);
        }
      } else if (servoID == Y) {
        if (maxHeatSig[servoID] < heatSig[servoID]) {
          maxHeatSig[servoID] = heatSig[servoID];
          maxHeatSigPW[servoID] = pos;
        }
      }

      //***Testing only: Print the values
//      Serial.print(servoID);
//      Serial.print("   ");
//      Serial.print(pos);
//      Serial.print("   ");
//      Serial.print(heatSig[servoID]);
//      Serial.print("   ");
//      Serial.print(maxHeatSigPW[servoID]);
//      Serial.print("   ");
//      Serial.println(maxHeatSig[servoID]);

    }
  } else if (endPos < startPos) {
    //Lock into start position
    lockIn(servoID, startPos);
    //Scan
    maxHeatSig[servoID] = 0;    //Initialize before each scan
    for (int pos = startPos; pos > endPos; pos-= pwStep) {
      servoAlign(servoID, pos);
      //Read the heat signature
      int maxFlameSensorIndex = readFlameSensor(servoID);
      //Note the maximum heat signature and it's position
      //Note the maximum heat signature and it's position
      if (servoID == X) {
        if (maxHeatSig[servoID] < heatSig[servoID]) {
          maxHeatSig[servoID] = heatSig[servoID];
          maxHeatSigPW[servoID] = (pos - 90) + (maxFlameSensorIndex * 30);
        }
      } else if (servoID == Y) {
        if (maxHeatSig[servoID] < heatSig[servoID]) {
          maxHeatSig[servoID] = heatSig[servoID];
          maxHeatSigPW[servoID] = pos;
        }
      }
    }
  }

}

void lockIn(int servoID, int pos){
//Takes a servo, with certainty, to a specitic position (because there are sufficient pulses to get there)
  //If the the vertical servo, briefly turn the power off and on before locing in, to clear the resonant power-torque which causes the power failure.
  if (servoID == Y) {
    servoOff(X);
    servoOff(Y);
    delay(100);
    servoOn(Y);
  }
  for (int i = 0; i < 40; i++) {
    servoAlign(servoID, pos);
  }
}

void lockInQuick(int servoID, int pos){
//Less accurate lock-in method for long rotations, but useful for small, quick servo steps (rotations).
  for (int i = 0; i < 5; i++) {
    servoAlign(servoID, pos);
  }
}

int posProcess(int servoID, int newPos){
/*
  //Inverts the servo's PWM position scale
  newPos = 180 - newPos;
*/
  newPos = newPos - servoOffset[servoID];
  //Invert the x-axis scale
  if (servoID == X) {
    newPos = 180 - newPos;
  }
  return newPos;
}

void centerServo(int servoID){
//Used by external programs to center the front servo
  lockIn(servoID, centerPos[servoID]);
}

void extinguisherTest(){
  openExtinguisher();
  delay(100);
  closeExtinguisher();
  delay(5000);
}

//Update flameApproach2() to use the flame sensor array
void flameApproach2() {
//Moves the robot towards the candle in a continuous motion.
//This method will terminate once the robot is close enough to the candle to extinguish it, measured by a strong enough heat signature, or a dip in the readings.
//The horizontal servo tracks the flame as the robot approached it.
//The robot's heading is corrected after every horizontal servo sweep.
//If the robot sees a dip in the flame readings, then adjust the vertical servo and use a scan and step approach until the flame has been succesfully approached.
  
  int posStep = 3;
  long servoInterval = 20;
  int tempMaxHeatSig = 0;  //The changing max during scans (servo cycles)
  int tempMaxHeatSigPW = 0;  //The changing max during scans (servo cycles)
  int tolerance = 5;  //Robot can move straight if flame is within +/- this (angle/degree) value of straight ahead
  int pmaxHeatSig[2];
  
  flameScanFull(X);
  pmaxHeatSig[X] = maxHeatSig[X];
  
  lockIn(X, left30);
  cycleServo(X, left30, right30, posStep, servoInterval);
  //Only scans horizontally using cycleServo, so only need to keep track of horizontal cycle direction
  //Perhaps change to use array indexed by [X], just to be consistent with the rest of the program. Will still only use X.
  int curServoXDir = getServoDir(X);
  int prevServoXDir = curServoXDir;
  
  while ((minFlameVal < maxHeatSig[X]) && ((pmaxHeatSig[X] - 60) < maxHeatSig[X])) {  //Prev: 150, 60. While a flame can be seen and its heat-sig is becoming stronger with time
//  flameStopVal = 9999;  //Temp. Disable stopping when close to flame.
//  while (minFlameVal < maxHeatSig[X]) {  //While a flame can be seen
//  while (1) {
    cycleServo(X, left30, right30, posStep, servoInterval);
    curServoXDir = getServoDir(X);
    
    //If the direction has changed (i.e. half-cycle has completed) then update maxHeatSig stuff
    if (curServoXDir != prevServoXDir) {
      pmaxHeatSig[X] = maxHeatSig[X];
      maxHeatSig[X] = tempMaxHeatSig;
      maxHeatSigPW[X] = tempMaxHeatSigPW;
      tempMaxHeatSig = 0;
      tempMaxHeatSigPW = getServoPos(X);
      prevServoXDir = curServoXDir;
/*
      Serial.print(maxHeatSigPW[X]);
      Serial.print("   ");
      Serial.println(maxHeatSig[X]);
*/
    }

    int tempMaxFlameSensorIndex = readFlameSensor(X);
    
    if (tempMaxHeatSig < heatSig[X]) {
      tempMaxHeatSig = heatSig[X];
      //tempMaxHeatSigPW = getServoPos(X);
      tempMaxHeatSigPW = (getServoPos(X) - 90) + (tempMaxFlameSensorIndex * 30);  //Get the flame angle from the sensor array
    }
    if (flameStopVal < maxHeatSig[X]) {    //If close flame is detected, stop the robot
      //Stop the motors
      stopMotors();  //Bypassed PD control. controlBase must not be called after this, since it may compute a new speed for the motors.
      break;
    }
    //Turn toward the flame
    if (maxHeatSigPW[X] < (centerPos[X] - tolerance)) {  //If the flame is more than x degrees left of center, arc-turn left
      setVelocities(40, 60);  //Prev: (10, 15), (20, 30)
    }
    else if ((centerPos[X] + tolerance) < maxHeatSigPW[X]) {  //If the flame is more than x degrees right of center, arc-turn right
      setVelocities(60, 40);
    }
    else {    //If the flame is ahead, roughly
      setVelocities(40);
    }
    controlBase();
  }
  //Stop the motors
  stopMotors();
}

void flameApproach3(){
//Scan-and-step approach
//Approaches the flame based n the strangth of the heart-reading. When close enough to the flame, stop.
  while (maxHeatSig[X] < 900) {
    flameScanFull(X);
    autoRobotFlamePoint();
    if (maxHeatSig[X] < 100) {
      moveDist(20);
    }
    else if (maxHeatSig[X] < 800) {
      moveDist(10);
    }
    else if (maxHeatSig[X] < 900) {    //Prev 975, 950
      moveDist(7);
    }
  }
}

void flameSPA2(){
  centerServo(Y);
  flameScanFull(X);
  robotFlamePoint();
//  lockIn(Y, centerPos[Y]);
  while (minFlameVal < maxHeatSig[X]) {
    flameApproach2();
    flameScanFull(X);
    sensorFlamePoint(X);
    flameScanFull(Y);
    sensorFlamePoint(Y);
    flameExtinguishSP();
    //Verify that the flame has been extinguished
    sensorFlamePoint(Y);
    flameScanFull(X);
    if (maxHeatSig[X] < minFlameVal) {
      //Perhaps do a full 360 scan somehow before exiting
      break;
    }
    else {
      //Perhaps orient the robot toward the flame, then continue the loop
      flameExtinguishFlood();
/*
      forwardFast();
      delay(800);
      stopMotors();
*/
    }
  }
}

long cmilFlameTest = 0;
long pmilFlameTest = 0;
long dlymilFlameTest = 50;

void flameSensorTest() {
  for (int i = 0; i < numFlameSensors; i++) {
    Serial.print(fSensor[i]->read());
    Serial.print("   ");
    delay(dlymilFlameTest);
  }
  Serial.println();
}

