int wall_follow_direction = RIGHT;  //Default
float base_speed_f = 70;  //In cm/s. The max speed that robot can move forward while wall-following. Prev: 30, 40
//float wall_follow_dist = 10.0; //Prev: 20

extern float wheel_set_velocity[2];  //This array holds the velocity setpoint, set by one of the primitive or macro controllers.

int primitiveWallConverge(){
  //To test basic PD control
  //Stay a set distance from a wall in front
  //Figure out return states
  updateSonars();
  
  int stopDist = 20;    //This is the distance the robot will try to keep from the wall
  float distError = (float)(stopDist - sonarDist[FRONTSONAR]);
  float maxSpeed = 50.0;  //In cm/s. Max speed the robot may travel towards the wall. Actual speed, based on error, is a proportion of this.
  float minSpeed = 10.0;
  float multiplier = 300.0;
  float targetVelocity;

  int state = NO_REFERENCE;

  static int init = TRUE;
  if (init) {
    base_control_mode = VELOCITY;
    frontSonar.enable();
    init = FALSE;
  }

  if (abs(distError) < 2.0) {  //Dist error is an int in float format, so this is tolerance of +-1
    targetVelocity = 0.0;
    state = CONVERGED;
  }
  else {
    //Invert dist-error scale so that positive error means go forward
    distError = -distError/370.0;  //-1 < error < 1
    if (0 < distError) {
      targetVelocity = multiplier*distError;  //-speedBound < set_velocity < speedBound
      if (targetVelocity < minSpeed) {
        targetVelocity = minSpeed;
      }
      else if (maxSpeed < targetVelocity) {
        targetVelocity = maxSpeed;
      }
    }
    else if (distError < 0) {
      targetVelocity = multiplier*distError;  //-speedBound < set_velocity < speedBound
      if (targetVelocity < -maxSpeed) {
        targetVelocity = -maxSpeed;
      }
      else if (-minSpeed < targetVelocity) {
        targetVelocity = -minSpeed;
      }
      state = TRANSIENT;
    }
  }

  wheel_set_velocity[LEFT] = targetVelocity;  //-speedBound < set_velocity < speedBound
  wheel_set_velocity[RIGHT] = targetVelocity;  //-speedBound < set_velocity < speedBound

  /*
  Serial.print(sonarDist[0]);
   Serial.print("   ");
   Serial.print(distError);
   Serial.print("   ");
   Serial.print(wheel_set_velocity[LEFT]);
   Serial.print("   ");
   Serial.println(wheel_set_velocity[RIGHT]);
   */
  return state;
}

/*
int primitive0(){
//Continuous rotation version
//If there is a wall ahead then rotate away from the direction of wall-following motion, otherwise move forward
//Still need to figure out return states

  int stopDist = 15;    //Within this distance, measured from the front, the robot turns away from the wall ahead. Prev. 15, 20 for sonar, 500 for IR
  int curDist = sonarDist[FRONTSONAR];
  float targetVelocity[2];

  int state = NO_REFERENCE;

  frontSonar.enable();

  base_control_mode = VELOCITY;
  static int init = TRUE;
  if (init) {
    init = FALSE;
  }

  if (curDist <= stopDist) {
    if (wall_follow_direction == LEFT) {
      targetVelocity[LEFT] = base_speed_f;
      targetVelocity[RIGHT] = -base_speed_f;
    }
    else if (wall_follow_direction == RIGHT) {
      targetVelocity[LEFT] = -base_speed_f;
      targetVelocity[RIGHT] = base_speed_f;
    }
    state = TRANSIENT;
  }
  else {
    targetVelocity[LEFT] = base_speed_f;
    targetVelocity[RIGHT] = base_speed_f;
    state = NO_REFERENCE;
  }

  wheel_set_velocity[LEFT] = targetVelocity[LEFT];
  wheel_set_velocity[RIGHT] = targetVelocity[RIGHT];

  return state;
}
*/


int primitive0(){
//Discrete rotation version
//If there is a wall ahead then rotate away from the direction of wall-following motion, otherwise move forward
//Still need to figure out return states
  frontSonar.enable();
  updateSonars();

  int stopDist = 20;    //Within this distance, measured from the front, the robot turns away from the wall ahead. Prev. 15, 20. 12, 15 for sonar
  int curDist = sonarDist[FRONTSONAR];
  float targetVelocity[2];

  int state = NO_REFERENCE;

//  base_control_mode = POSITION;
  static int init = TRUE;
  if (init) {
    init = FALSE;
  }

  if (curDist <= stopDist) {
    head_wall_count++;
    if (wall_follow_direction == LEFT) {
//      rotate(140);
//      rotate(90);
      rotate(45);
    }
    else if (wall_follow_direction == RIGHT) {
      rotate(-90);
//      rotate(-45);
    }

    //Forcibly refresh the front sonar distance because if the front distance isn't automatically refreshed the robot may think there is an obstacle infront until the refresh
    delay(50);
    sonarDist[FRONTSONAR] = frontSonar.read();

    state = TRANSIENT;
  }
  else {

//    targetVelocity[LEFT] = base_speed_f;
//    targetVelocity[RIGHT] = base_speed_f;

    state = NO_REFERENCE;
  }

//  wheel_set_velocity[LEFT] = targetVelocity[LEFT];
//  wheel_set_velocity[RIGHT] = targetVelocity[RIGHT];

  return state;
}


int primitive1(){
  //IR + sonar version
  //Moves forward while keeping one of the front-corners (on the wall-following-side) a set distance from the wall
  //Ideally takes input from a proximity sensor mounted at 45deg from the normal and directed at the wall being followed
  //Corrects for error by reducing the velocity of one wheel, or the other, proportional to the error
  //Adjusted outerTurn behaviour with sonarBufferDist
  updateSonars();

  int idealWallDistLeft = 500;    //This is the distance the robot will try to keep from the wall. 450, 360, 380, 450, 400, 500 for leftIR.
  int idealWallDistRight = 450;    //This is the distance the robot will try to keep from the wall.  for rightIR. Prev: 450, 360
  int tolerance = 10;    //Plus or minus this from the wallDist. Prev: 2 for sonar, 10 for IR
  float sensorDist = 0.0;
  float sensorDistRange = 850.0;  //370 for sonars, 850 for IRs
  float distError = 0.0;
  float targetVelocity[2];
  float velocityCorrection;
  float minVelocityTC = 12.0;  //Prev: 6.0, 20.0, 6.0, 10.0
  float minVelocityTF = 5.0;  //Prev: 6.0, 20.0, 6.0, 10.0
  //Multipliers to scale the error measurements--proportional to speed reduction of one wheel
  float multiplierLeftTC = 3.0;  //When too close to wall. Prev: 40.0, 4.0, 2.0, 6.0, 12.0, 9.0, 4.0, 6.0, 5.0
  float multiplierLeftTF = 2.0;  //When too far from wall. Prev: 40.0, 4.0, 2.0, 6.0, 4.0, 9.0, 10.0, 6.0, 4.0
  float multiplierRightTC = 12.0;  //When too close to wall. Prev: 40.0, 4.0, 2.0, 12.0
  float multiplierRightTF = 6.0;  //When too far from wall. Prev: 40.0, 4.0, 2.0, 12.0
//  float outerTurnWheelSpeedReducRatio = 3.6/4.0;  //Prev. 2/3. Reduce the speed of the wheel closest to the wall to this ratio of base_speed_f, to avoid clipping the wall
  int outerTurnSonarBufferDist = 15;  //15 worked well for leftfollow, test left. Prev: 20, 15, 11, 20, 15, 13, 12
  
  int state = NO_REFERENCE;
  
  static int init = TRUE;
  base_control_mode = VELOCITY;
  if (init) {
//    disableSonars();
    init = FALSE;
  }

  if (wall_follow_direction == LEFT) {
    lfSonar.enable();
    rfSonar.disable();
    sensorDist = (float)getDist(LEFTIR);
    distError = (float)(idealWallDistLeft) - sensorDist;
  }
  else if (wall_follow_direction == RIGHT) {
    lfSonar.disable();
    rfSonar.enable();
    sensorDist = (float)getDist(RIGHTIR);
    distError = (float)(idealWallDistRight) - sensorDist;
  }

  if (distError < -tolerance) {  //Too far from wall
    if (wall_follow_direction == LEFT) {
      velocityCorrection = -1*base_speed_f*multiplierLeftTF*distError/sensorDistRange;  //0 < error < 1
      if (sonarDist[LFSONAR] < outerTurnSonarBufferDist) {
//        targetVelocity[LEFT] = base_speed_f*outerTurnWheelSpeedReducRatio;
        targetVelocity[LEFT] = base_speed_f;
        targetVelocity[RIGHT] = base_speed_f;
      }
      else {
        targetVelocity[LEFT] = base_speed_f - velocityCorrection;
        targetVelocity[RIGHT] = base_speed_f;
      }
    }
    else if (wall_follow_direction == RIGHT) {
      velocityCorrection = -1*base_speed_f*multiplierRightTF*distError/sensorDistRange;  //0 < error < 1
      //Handle corner-right, go straight (***no longer arc-wide right) while along the leading wall, otherwise correct as normal
      if (sonarDist[RFSONAR] < outerTurnSonarBufferDist) {
        targetVelocity[LEFT] = base_speed_f;
        targetVelocity[RIGHT] = base_speed_f;
//        targetVelocity[RIGHT] = base_speed_f*outerTurnWheelSpeedReducRatio;
      }
      else {
        targetVelocity[LEFT] = base_speed_f;
        targetVelocity[RIGHT] = base_speed_f - velocityCorrection;
      }
    }
    if (targetVelocity[LEFT] < minVelocityTF) {
      targetVelocity[LEFT] = minVelocityTF;
    }
    if (targetVelocity[RIGHT] < minVelocityTF) {
      targetVelocity[RIGHT] = minVelocityTF;
    }
    state = TRANSIENT;
  }
  else if (tolerance < distError) {  //Too close to wall
    if (wall_follow_direction == LEFT) {
      velocityCorrection = base_speed_f*multiplierLeftTC*distError/sensorDistRange;  //0 < error < 1
      targetVelocity[LEFT] = base_speed_f;
      targetVelocity[RIGHT] = base_speed_f - velocityCorrection;
    }
    else if (wall_follow_direction == RIGHT) {
      velocityCorrection = base_speed_f*multiplierRightTC*distError/sensorDistRange;  //0 < error < 1
      targetVelocity[LEFT] = base_speed_f - velocityCorrection;
      //Handle outer-turn right
      targetVelocity[RIGHT] = base_speed_f;
    }
    if (targetVelocity[LEFT] < minVelocityTC) {
      targetVelocity[LEFT] = minVelocityTC;
    }
    if (targetVelocity[RIGHT] < minVelocityTC) {
      targetVelocity[RIGHT] = minVelocityTC;
    }
    state = TRANSIENT;
  }
  else {
    targetVelocity[LEFT] = base_speed_f;
    targetVelocity[RIGHT] = base_speed_f;
    state = CONVERGED;
  }

  wheel_set_velocity[LEFT] = targetVelocity[LEFT];
  wheel_set_velocity[RIGHT] = targetVelocity[RIGHT];
  /*
  Serial.print(sonarDist[5]);
  Serial.print("   ");
  Serial.print(distError);
  Serial.print("   ");
  Serial.print(wheel_set_velocity[LEFT]);
  Serial.print("   ");
  Serial.println(wheel_set_velocity[RIGHT]);
  */
  return state;
}

//int primitive1_old[works well when base_speed_f = 40) (){
//  //IR + sonar version
//  //Moves forward while keeping one of the front-corners (on the wall-following-side) a set distance from the wall
//  //Ideally takes input from a proximity sensor mounted at 45deg from the normal and directed at the wall being followed
//  //Corrects for error by reducing the velocity of one wheel, or the other, proportional to the error
//  //Adjusted outerTurn behaviour with sonarBufferDist
//  updateSonars();
//
//  int idealWallDistLeft = 450;    //This is the distance the robot will try to keep from the wall. 450, 360, 380 for leftIR.
//  int idealWallDistRight = 450;    //This is the distance the robot will try to keep from the wall.  for rightIR. Prev: 450, 360
//  int tolerance = 10;    //Plus or minus this from the wallDist. Prev: 2 for sonar, 10 for IR
//  float sensorDist = 0.0;
//  float sensorDistRange = 850.0;  //370 for sonars, 850 for IRs
//  float distError = 0.0;
//  float targetVelocity[2];
//  float velocityCorrection;
//  float minVelocityTC = 6.0;  //Prev: 6.0, 20.0, 6.0
//  float minVelocityTF = 10.0;  //Prev: 6.0, 20.0, 6.0
//  float multiplierLeftTC = 12.0;  //When too close to wall. Prev: 40.0, 4.0, 2.0, 6.0
//  float multiplierLeftTF = 9.0;  //When too far from wall. Prev: 40.0, 4.0, 2.0, 6.0, 4.0
//  float multiplierRightTC = 12.0;  //When too close to wall. Prev: 40.0, 4.0, 2.0, 12.0
//  float multiplierRightTF = 6.0;  //When too far from wall. Prev: 40.0, 4.0, 2.0, 12.0
////  float outerTurnWheelSpeedReducRatio = 3.6/4.0;  //Prev. 2/3. Reduce the speed of the wheel closest to the wall to this ratio of base_speed_f, to avoid clipping the wall
//  int outerTurnSonarBufferDist = 15;  //15 worked well for leftfollow, test left. Prev: 20, 15, 11, 20, 
//  
//  int state = NO_REFERENCE;
//  
//  static int init = TRUE;
//  base_control_mode = VELOCITY;
//  if (init) {
////    disableSonars();
//    init = FALSE;
//  }
//
//  if (wall_follow_direction == LEFT) {
//    lfSonar.enable();
//    rfSonar.disable();
//    sensorDist = (float)getDist(LEFTIR);
//    distError = (float)(idealWallDistLeft) - sensorDist;
//  }
//  else if (wall_follow_direction == RIGHT) {
//    lfSonar.disable();
//    rfSonar.enable();
//    sensorDist = (float)getDist(RIGHTIR);
//    distError = (float)(idealWallDistRight) - sensorDist;
//  }
//
//  if (distError < -tolerance) {  //Too far from wall
//    if (wall_follow_direction == LEFT) {
//      velocityCorrection = -1*base_speed_f*multiplierLeftTF*distError/sensorDistRange;  //0 < error < 1
//      if (sonarDist[LFSONAR] < outerTurnSonarBufferDist) {
////        targetVelocity[LEFT] = base_speed_f*outerTurnWheelSpeedReducRatio;
//        targetVelocity[LEFT] = base_speed_f;
//        targetVelocity[RIGHT] = base_speed_f;
//      }
//      else {
//        targetVelocity[LEFT] = base_speed_f - velocityCorrection;
//        targetVelocity[RIGHT] = base_speed_f;
//      }
//    }
//    else if (wall_follow_direction == RIGHT) {
//      velocityCorrection = -1*base_speed_f*multiplierRightTF*distError/sensorDistRange;  //0 < error < 1
//      //Handle corner-right, go straight (***no longer arc-wide right) while along the leading wall, otherwise correct as normal
//      if (sonarDist[RFSONAR] < outerTurnSonarBufferDist) {
//        targetVelocity[LEFT] = base_speed_f;
//        targetVelocity[RIGHT] = base_speed_f;
////        targetVelocity[RIGHT] = base_speed_f*outerTurnWheelSpeedReducRatio;
//      }
//      else {
//        targetVelocity[LEFT] = base_speed_f;
//        targetVelocity[RIGHT] = base_speed_f - velocityCorrection;
//      }
//    }
//    if (targetVelocity[LEFT] < minVelocityTF) {
//      targetVelocity[LEFT] = minVelocityTF;
//    }
//    if (targetVelocity[RIGHT] < minVelocityTF) {
//      targetVelocity[RIGHT] = minVelocityTF;
//    }
//    state = TRANSIENT;
//  }
//  else if (tolerance < distError) {  //Too close to wall
//    if (wall_follow_direction == LEFT) {
//      velocityCorrection = base_speed_f*multiplierLeftTC*distError/sensorDistRange;  //0 < error < 1
//      targetVelocity[LEFT] = base_speed_f;
//      targetVelocity[RIGHT] = base_speed_f - velocityCorrection;
//    }
//    else if (wall_follow_direction == RIGHT) {
//      velocityCorrection = base_speed_f*multiplierRightTC*distError/sensorDistRange;  //0 < error < 1
//      targetVelocity[LEFT] = base_speed_f - velocityCorrection;
//      //Handle outer-turn right
//      targetVelocity[RIGHT] = base_speed_f;
//    }
//    if (targetVelocity[LEFT] < minVelocityTC) {
//      targetVelocity[LEFT] = minVelocityTC;
//    }
//    if (targetVelocity[RIGHT] < minVelocityTC) {
//      targetVelocity[RIGHT] = minVelocityTC;
//    }
//    state = TRANSIENT;
//  }
//  else {
//    targetVelocity[LEFT] = base_speed_f;
//    targetVelocity[RIGHT] = base_speed_f;
//    state = CONVERGED;
//  }
//
//  wheel_set_velocity[LEFT] = targetVelocity[LEFT];
//  wheel_set_velocity[RIGHT] = targetVelocity[RIGHT];
//  /*
//  Serial.print(sonarDist[5]);
//  Serial.print("   ");
//  Serial.print(distError);
//  Serial.print("   ");
//  Serial.print(wheel_set_velocity[LEFT]);
//  Serial.print("   ");
//  Serial.println(wheel_set_velocity[RIGHT]);
//  */
//  return state;
//}

int primitive2() {
  //Distance controller
  base_control_mode = POSITION;
  int state = NO_REFERENCE;
  float delta = 0.1;  //Prev: 1.1

  //  controlBase();
  if ((abs(wheel_set_position[LEFT] - wheel_cur_position[LEFT]) < delta) && (abs(wheel_set_position[RIGHT] - wheel_cur_position[RIGHT]) < delta)) {
    state = CONVERGED;
  }
  else {
    state = TRANSIENT;
  }
  return state;
}

/*
int primitive3() {
//An almost-macro controller
//Turns 90 degrees left or right, based on wall-following-direction, to avoid a wall ahead, otherwise follows the side-wall using primitive1
//Need to figure out return states

  frontSonar.enable();
  if (wall_follow_direction == LEFT) {
    lfSonar.enable();
  }
  else if (wall_follow_direction == RIGHT) {
    rfSonar.enable();
  }

  updateSonars();
  int state = NO_REFERENCE;
  int stopDist = 20;    //Within this distance, measured from the front, the robot turns away from the wall ahead. Prev. 15 for sonar, 500 for IR
  int frontDist = sonarDist[FRONTSONAR];
  if (frontDist < stopDist) {
    if (wall_follow_direction == LEFT) {  //Turn right if there's a wall ahead
      rotate(90);
    }
    else if (wall_follow_direction == RIGHT) {  //Turn left if there's a wall ahead
      rotate(-90);
    }
    //Forcible refresh the front sonar distance because if the front distance isn't automatically refreshed the robot may think there is an obstacle infront until the refresh
    delay(50);
    sonarDist[FRONTSONAR] = frontSonar.read();
  }
  else {
    primitive1();
  }
  return state;
}
*/

int primitive4() {
  frontSonar.enable();
  if (wall_follow_direction == LEFT) {
    lfSonar.enable();
  }
  else if (wall_follow_direction == RIGHT) {
    rfSonar.enable();
  }
  updateSonars();
  int state = NO_REFERENCE;
  int stopDist = 15;    //Within this distance, measured from the front, the robot turns away from the wall ahead. Prev. 15 for sonar, 500 for IR
  int frontDist = sonarDist[FRONTSONAR];
  if (frontDist < stopDist) {
    if (wall_follow_direction == LEFT) {  //Turn right if there's a wall ahead
      rotate(100);
    }
    else if (wall_follow_direction == RIGHT) {  //Turn left if there's a wall ahead
      rotate(-90);
    }
    //Forcibly refresh the front sonar distance because if the front distance isn't automatically refreshed the robot may think there is an obstacle infront until the refresh
    delay(50);
    sonarDist[FRONTSONAR] = frontSonar.read();
  }
  else {
    wallFollow(wall_follow_direction);
  }
}

