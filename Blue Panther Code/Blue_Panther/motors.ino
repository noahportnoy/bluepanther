//_______________________________Variables and constants________________________________________

  //Wheels
  float wheelRadius = 6.0;  //Radius in centimeters
  float wheelSeparation = 20.0;  //Distance between wheels in cm
  float ds = (1.0/encoderCPR) * 2 * 3.14159 * wheelRadius;  //In cm
  float dt; //In seconds
  float wheel_set_position[2] = {0,0};  //This set point, once set, will be reduced as the robot converges on the target distance from robot's frame of ref.
  float wheel_cur_position[2] = {0,0};  //In cm. This holds the distance travelled since the last call to computePositionAndVelocity()
  float wheel_cur_velocity[2];  //In cm/s. This is computed by computeVelocityAndAcceleration() and written into this array. 0 = left, 1 = right
  float wheel_pres_velocity[2] = {0,0};  //Velocity measured from encoders, used locally. Processed before being written to wheel_cur_velocity, which is used publicly.
  float wheel_prev_velocity[2] = {0,0};  //Most recent prev vel
  float wheel_set_velocity[2];
  float wheel_cur_acceleration[2];  //In cm/s^2. Also computed by computeVelocityAndAccel().
  float wheel_pres_acceleration[2] = {0,0};  //In cm/s^2. Also computed by computeVelocityAndAccel().
  float wheel_prev_acceleration[2];  //In cm/s^2. Also computed by computeVelocityAndAccel().
  
  long cmicCompPos = 0;
  long pmicCompPos = 0;
  long dlymicCompPos = 1;  //To prevent divide by zero, unlikely but possible.

  long cmicCompVel = 0;
  long pmicCompVel = 0;
  long dlymicCompVel = 1;  //To prevent divide by zero, unlikely but possible.

  //Motor speeds
    //Slow
  int slowFSpeedL;    //slow forward speed left(motor)
  int slowFSpeedR;
  int slowRSpeedL;
  int slowRSpeedR;
    //Fast
  int fastFSpeedL;    //fast forward speed left(motor)
  int fastFSpeedR;
  int fastRSpeedL;
  int fastRSpeedR;
    //Maximum (used for constraints)
  int maxFSpeedL = 255;    //max forward speed left(motor)
  int maxFSpeedR = 255;
  int maxRSpeedL = -255;
  int maxRSpeedR = -255;
  
  
  int cmilUpdateVel = 0;
  int pmilUpdateVel = 0;
  int dlymilUpdateVel = 5;


//Refer to:
/*
  //Motor pins
    extern int enablePinL;    //Shift register pin
    extern int enablePinR;    //Shift register pin
    extern int leftFPin;    //PWM pin
    extern int leftBPin;    //PWM pin
    extern int rightFPin;    //PWM pin
    extern int rightBPin;    //PWM pin
*/

//______________________________________________________________________________________________

//Slow-motion
  //One wheel only
void forwardSlowL(){
  motors.setM1Speed(slowFSpeedL);
}
void reverseSlowL(){
  motors.setM1Speed(slowRSpeedL);
}
void forwardSlowR(){
  motors.setM2Speed(slowFSpeedR);
}
void reverseSlowR(){
  motors.setM2Speed(slowRSpeedR);
}
  //Both wheels
void forwardSlow(){
  motors.setSpeeds(slowFSpeedL, slowFSpeedR);
}
void reverseSlow(){
  motors.setSpeeds(slowRSpeedL, slowRSpeedR);
}
  //Point-turns
void spinSlowL(){
  motors.setSpeeds(slowRSpeedL, slowFSpeedR);
}
void spinSlowR(){
  motors.setSpeeds(slowFSpeedL, slowRSpeedR);
}

//Fast-motion
  //One wheel only
void forwardFastL(){
  motors.setM1Speed(fastFSpeedL);
}
void reverseFastL(){
  motors.setM1Speed(fastRSpeedL);
}
void forwardFastR(){
  motors.setM2Speed(fastFSpeedR);
}
void reverseFastR(){
  motors.setM2Speed(fastRSpeedR);
}
  //Both wheels
void forwardFast(){
  motors.setSpeeds(fastFSpeedL, fastFSpeedR);
}
void reverseFast(){
  motors.setSpeeds(fastRSpeedL, fastRSpeedR);
}
  //Point-turns
void spinFastL(){
  motors.setSpeeds(fastRSpeedL, fastFSpeedR);
}
void spinFastR(){
  motors.setSpeeds(fastFSpeedL, fastRSpeedR);
}

//Arc-turns
  //Forward
void fArcL(){
  motors.setSpeeds(slowFSpeedL, fastFSpeedR);
}
void fArcR(){
  motors.setSpeeds(fastFSpeedL, slowFSpeedR);
}
  //Reverse
void rArcL(){
  motors.setSpeeds(slowRSpeedL, fastRSpeedR);
}
void rArcR(){
  motors.setSpeeds(fastRSpeedL, slowRSpeedR);
}

void stopMotorL(){
//Brakes the left motor only
  motors.setM1Speed(0);
}

void stopMotorR(){
//Brakes the right motor only
  motors.setM2Speed(0);
}

void stopMotors(){
//Brakes both motors
  motors.setSpeeds(0, 0);
}

/*
void setOutputMotorPins(){
  pinMode(leftFPin, OUTPUT);
  pinMode(leftBPin, OUTPUT);
  pinMode(rightFPin, OUTPUT);
  pinMode(rightBPin, OUTPUT);
}
*/

/*
void drive(int leftSpeed, int rightSpeed){
//Controls motors
//Positive speed means forward, negative speed means reverse.
//Speed ranges from -255 to 255
//Each motor's speed is limited by the motor's maxSpeeds (forward and reverse).

  //Limit the motor speeds
  leftSpeed = constrain(leftSpeed, maxRSpeedL, maxFSpeedL);
  rightSpeed = constrain(rightSpeed, maxRSpeedR, maxFSpeedR);

  //Left motor forward
  if (0 <= leftSpeed) {
    analogWrite(leftFPin, leftSpeed);
    digitalWrite(leftBPin, LOW);
  }
  //Left motor reverse
  if (leftSpeed < 0) {
    digitalWrite(leftFPin, LOW);
    analogWrite(leftBPin, -leftSpeed);
  }
  //Right motor forward
  if (0 <= rightSpeed) {
    analogWrite(rightFPin, rightSpeed);
    digitalWrite(rightBPin, LOW);
  }
  //Right motor reverse
  if (rightSpeed < 0) {
    digitalWrite(rightFPin, LOW);
    analogWrite(rightBPin, -rightSpeed);
  }
}
*/

void setTopSpeed(int topSpeed){
//  //Limits, but does not set, the top speed. When limiting (if the topSpeed is exceeded by other speed constraints), it adjusts the slowSpeed values.
//  if (topSpeed < fastFSpeedL || topSpeed < fastRSpeedL || topSpeed < fastFSpeedR || topSpeed < fastRSpeedR) {
  //Slow
  slowFSpeedL = topSpeed/2;
  slowFSpeedR = topSpeed/2;
  slowRSpeedL = -topSpeed/2;
  slowRSpeedR = -topSpeed/2;
  //Fast
  fastFSpeedL = topSpeed;
  fastFSpeedR = topSpeed;
  fastRSpeedL = -topSpeed;
  fastRSpeedR = -topSpeed;
//  }
}

void setSpeedLevel(int speedLevel){
  //Adjust global speeds to desired levels
  switch (speedLevel) {
    case 0:    //Slowest
      //Slow
      slowFSpeedL = 20;    //slow forward speed left(motor)
      slowFSpeedR = 20;
      slowRSpeedL = -20;
      slowRSpeedR = -20;
      //Fast
      fastFSpeedL = 40;    //fast forward speed left(motor)
      fastFSpeedR = 40;
      fastRSpeedL = -40;
      fastRSpeedR = -40;
      break;
    case 1:
      //Slow
      slowFSpeedL = 47;    //slow forward speed left(motor)
      slowFSpeedR = 50;
      slowRSpeedL = -50;
      slowRSpeedR = -50;
      //Fast
      fastFSpeedL = 65;    //fast forward speed left(motor)
      fastFSpeedR = 65;
      fastRSpeedL = -65;
      fastRSpeedR = -65;
      break;
    case 2:
      //Slow
      slowFSpeedL = 70;    //slow forward speed left(motor)
      slowFSpeedR = 70;
      slowRSpeedL = -70;
      slowRSpeedR = -70;
      //Fast
      fastFSpeedL = 140;    //fast forward speed left(motor)
      fastFSpeedR = 150;
      fastRSpeedL = -150;
      fastRSpeedR = -150;
      break;
    case 3:    //Fastest
      //Slow
      slowFSpeedL = 180;    //slow forward speed left(motor)
      slowFSpeedR = 180;
      slowRSpeedL = -180;
      slowRSpeedR = -180;
      //Fast
      fastFSpeedL = 255;    //fast forward speed left(motor)
      fastFSpeedR = 255;
      fastRSpeedL = -255;
      fastRSpeedR = -255;
      break;
  }
}

void motorTest(){
  int delayOn = 1000;
  int delayOff = 1000;
  
  ///////////////////////
  forwardSlowL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  reverseSlowL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  forwardSlowR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  reverseSlowR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  ///////////////////////
  forwardSlow();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  reverseSlow();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  ///////////////////////
  spinSlowL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  spinSlowR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  //////////////////////
  
  //////////////////////
  forwardFastL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  reverseFastL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  forwardFastR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  reverseFastR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  ///////////////////////
  forwardFast();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  reverseFast();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  ///////////////////////
  spinFastL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  spinFastR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  ///////////////////////

  ///////////////////////
  fArcL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  rArcL();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  fArcR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);
  
  rArcR();
  delay(delayOn);
  stopMotors();
  delay(delayOff);

  stopMotors();
  delay(5000);

}

void forwardSlowFor(int delayTime){
//Goes forward for said amount of time
  forwardSlow();
  delay(delayTime);
  stopMotors();
}

void forwardFastFor(int delayTime){
//Goes forward for said amount of time
  forwardFast();
  delay(delayTime);
  stopMotors();
}

void reverseSlowFor(int delayTime){
//Reverses for said amount of time
  reverseSlow();
  delay(delayTime);
  stopMotors();
}

void reverseFastFor(int delayTime){
//Reverses for said amount of time
  reverseFast();
  delay(delayTime);
  stopMotors();
}


void computePositionAndVelocity(){
  cmicCompPos = micros();
  if (dlymicCompPos <= (cmicCompPos - pmicCompPos)) {
    dt = ((float)(cmicCompPos - pmicCompPos))/1000000.0;  //In seconds
    wheel_prev_velocity[LEFT] = wheel_cur_velocity[LEFT];
    wheel_prev_velocity[RIGHT] = wheel_cur_velocity[RIGHT];
    
    wheel_cur_position[LEFT] = ((float) encoders.getCountsAndResetM1())*ds;
    wheel_cur_position[RIGHT] = ((float) encoders.getCountsAndResetM2())*ds;
    
    wheel_cur_velocity[LEFT] = wheel_cur_position[LEFT]/dt;
    wheel_cur_velocity[RIGHT] = wheel_cur_position[RIGHT]/dt;
    
    //Update odometers
    float avgWheelDist = (wheel_cur_position[LEFT] + wheel_cur_position[RIGHT])/2.0;
    for (int i = 0; i < numOdometers; i++) {
      odometer[i] += avgWheelDist;
    }

    pmicCompPos = cmicCompPos;
  }
}

void computeVelocityAndAcceleration(){
  cmicCompVel = micros();
  if (dlymicCompVel <= (cmicCompVel - pmicCompVel)) {
    dt = ((float)(cmicCompVel - pmicCompVel))/1000000.0;  //In seconds
    wheel_prev_velocity[LEFT] = wheel_cur_velocity[LEFT];
    wheel_prev_velocity[RIGHT] = wheel_cur_velocity[RIGHT];
    
    wheel_cur_velocity[LEFT] = ((float) encoders.getCountsAndResetM1())*ds/dt;
    wheel_cur_velocity[RIGHT] = ((float) encoders.getCountsAndResetM2())*ds/dt;
    
    wheel_cur_acceleration[LEFT] = (wheel_cur_velocity[LEFT] - wheel_prev_velocity[LEFT])/dt;
    wheel_cur_acceleration[RIGHT] = (wheel_cur_velocity[RIGHT] - wheel_prev_velocity[RIGHT])/dt;
    
    //Update odometers
    float avgWheelDist = (wheel_cur_velocity[LEFT] + wheel_cur_velocity[RIGHT])*dt/2.0;
    for (int i = 0; i < numOdometers; i++) {
      odometer[i] += avgWheelDist;
    }

    pmicCompVel = cmicCompVel;
  }
}

long cmilTestCompVel = 0;
long pmilTestCompVel = 0;
long dlymilTestCompVel = 50;

void testCompVelocityAndAcceleration() {
//Tests computeVelocities();
//Prints velocity readings from M1 and M2 in cm/s
  cmilTestCompVel = millis();
  if (dlymilTestCompVel < (cmilTestCompVel - pmilTestCompVel)) {
    computeVelocityAndAcceleration();
    Serial.print(wheel_cur_velocity[LEFT]);
    Serial.print("   ");
    Serial.print(wheel_cur_velocity[RIGHT]);
    Serial.print("   ");
    Serial.print(wheel_cur_acceleration[LEFT]);
    Serial.print("   ");
    Serial.println(wheel_cur_acceleration[RIGHT]);
    pmilTestCompVel = cmilTestCompVel;
  }
}

long cmilTestEncoders = 0;
long pmilTestEncoders = 0;
long dlymilTestEncoders = 50;

void testEncoders() {
//Prints encoder counts from M1 and M2
  cmilTestEncoders = millis();
  if (dlymilTestEncoders < (cmilTestEncoders - pmilTestEncoders)) {
//    int countsM1 = encoders.getCountsM1();
    long countsM1 = encoders_get_counts_m1();
    long countsM2 = encoders.getCountsM2();
    Serial.print(countsM1);
    Serial.print("   ");
    Serial.println(countsM2);
    pmilTestEncoders = cmilTestEncoders;
  }
}

void moveDist(int distance) {
//***Blocking function. Suspends program until the robot's wheels have moved the set distance.
//Distance in cm
  base_control_mode = POSITION;
  wheel_set_position[LEFT] = distance;
  wheel_set_position[RIGHT] = distance;
  while (primitive2() == TRANSIENT) {
    controlBase();
  }
  stopMotors();
}

void moveDist(int distanceL, int distanceR) {
//***Blocking function. Suspends program until the robot's wheels have moved the set distances
//Distances in cm
  base_control_mode = POSITION;
  wheel_set_position[LEFT] = distanceL;
  wheel_set_position[RIGHT] = distanceR;
  while (primitive2() == TRANSIENT) {
    controlBase();
  }
  stopMotors();
}

void setVelocities(int velocity) {
//Assigns one set-velocity to both wheels
  base_control_mode = VELOCITY;
  wheel_set_velocity[LEFT] = velocity;
  wheel_set_velocity[RIGHT] = velocity;
}

void setVelocities(int velocityL, int velocityR) {
//Assigns individual set-velocities to each wheel
  base_control_mode = VELOCITY;
  wheel_set_velocity[LEFT] = velocityL;
  wheel_set_velocity[RIGHT] = velocityR;
}

void rotate(int angle) {
//***Watch the negative numbers!!!!
//***Blocking function
//Converts a rotation angle in degrees into distance set-points for the wheels to turn and asigns them
//Positive angle means rotate right, negative angle means rotate left
  base_control_mode = ROTATION;
  float radToWheel = wheelSeparation/2.0;
  float wheelRotDist;
  
  //These are ratios applied to the wheels that move in those directions-- to compensate for the wheels' vertical offset, so the robot may turn more on-point 
  float forwardDistCorr = 1.2;
  float reverseDistCorr = 0.8;
  
  if (angle < 0) {
    wheelRotDist = -1*((float)angle)/360.0 * 2* 3.14159 * radToWheel;
    wheel_set_position[LEFT] = -wheelRotDist*reverseDistCorr;
    wheel_set_position[RIGHT] = wheelRotDist*forwardDistCorr;
//    Serial.println("neg angle");
  }
  else {
    wheelRotDist = ((float)angle)/360.0 * 2* 3.14159 * radToWheel;
    wheel_set_position[LEFT] = wheelRotDist*forwardDistCorr;
    wheel_set_position[RIGHT] = -wheelRotDist*reverseDistCorr;
//    Serial.println("pos angle");
  }
  while (primitive2() == TRANSIENT) {
    controlBase();
  }
  stopMotors();
}

