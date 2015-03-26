float f[2] = {0.0,0.0};  //Holds previous PWM value sent to the motors

extern float wheel_set_position[2];  //This set point, once set, will be reduced as the robot converges on the target distance from robot's frame of ref.
extern float wheel_cur_position[2];  //In cm, computed by computePositionandVelocity(). Holds the distance travelled. Is reset after every call to PD pos controller
extern float wheel_set_velocity[2];  //This array holds the velocity setpoint, set by one of the primitive or macro controllers.
extern float wheel_cur_velocity[2];  //This is computed by computeVelocityAndAcceleration() and written into this array. 0 = left, 1 = right
extern float wheel_cur_acceleration[2];

long cmicPDMotors = 0;
long pmicPDMotors = 0;
long dlymicPDMotors = 10000;    //Prevents motor stutter, allows system response to take effect and be measured

void controlBase() {
  switch (base_control_mode) {
    case(POSITION):
      computePositionAndVelocity();
      PDController_base_position();
      break;
    case(VELOCITY):
      cmicPDMotors = micros();
      if (dlymicPDMotors < (cmicPDMotors - pmicPDMotors)) {  //Allow a sufficient measurement interval
        computeVelocityAndAcceleration();
        PDController_base_velocity();
        pmicPDMotors = cmicPDMotors;
      }
      break;
    case(ROTATION):  //Assumes that before first call, angle has been converted to desired position set-points for wheels
      computePositionAndVelocity();
      PDController_base_position();
      break;
  }
}

void PDController_base_position() {
//Controls distance the wheels turn through
  float pos_error[2];  //Will hold position errors (setpoint - actual). Max = ? relevant?
  float vel[2];  //Will hold current wheel velocities, which are already computed
  float df[2];  //Change in force applied to wheels (pwm value)
  //Tolerance in cm. Not feasible to move the precise distance. This is within how close the controller will get to it
  float tolerance = 0.3;  //Prev. 1.1,
  
  float max_speed = 150.0;    //300, 200, 150, 300, 200
  
  //When printing data to monitor, software delay impacts pd controller. use gains k = 1.0, b = 0.6 for critically damped motion.
  float k = 2.0;  //Prev: 0.5, 3.0, 3.0, 3.0, 1.0, 3.0, 
  float b = 0.4;  //Prev: 1.0, 1.7, 1.5, 1.4, 0.2, 0.2, 
  
  pos_error[LEFT] = wheel_set_position[LEFT] - wheel_cur_position[LEFT];
  pos_error[RIGHT] = wheel_set_position[RIGHT] - wheel_cur_position[RIGHT];
  
  if (abs(pos_error[LEFT]) < tolerance) {
    pos_error[LEFT] = 0.0;
    f[LEFT] = 0;
//    wheel_set_position[LEFT] = 0;
  }
  if (abs(pos_error[RIGHT]) < tolerance) {
    pos_error[RIGHT] = 0.0;
    f[RIGHT] = 0;
//    wheel_set_position[RIGHT] = 0;
  }
  
  //Reset current positions to zero
  wheel_cur_position[LEFT] = 0;
  wheel_cur_position[RIGHT] = 0;
  //Adjust set positions to equal errror reading
  wheel_set_position[LEFT] = pos_error[LEFT];
  wheel_set_position[RIGHT] = pos_error[RIGHT];
  
  vel[LEFT] = wheel_cur_velocity[LEFT];
  vel[RIGHT] = wheel_cur_velocity[RIGHT];
  
  df[LEFT] = k*pos_error[LEFT] - b*vel[LEFT];
  df[RIGHT] = k*pos_error[RIGHT] - b*vel[RIGHT];
/*
  Serial.print(wheel_set_position[LEFT]);
  Serial.print("   ");
  Serial.print(wheel_set_position[RIGHT]);
  Serial.print("   ");
  Serial.print(wheel_cur_position[LEFT]);
  Serial.print("   ");
  Serial.print(wheel_cur_position[RIGHT]);
  Serial.print("   ");
  Serial.print(f[LEFT] + df[LEFT]);
  Serial.print("   ");
  Serial.println(f[RIGHT] + df[RIGHT]);
*/
  //Run motor control on an interval to prevent stutter
  cmicPDMotors = micros();
  if (dlymicPDMotors < (cmicPDMotors - pmicPDMotors)) {
    //Constrain the force applied to the motors
    f[LEFT] = f[LEFT] + df[LEFT];
    if (f[LEFT] < -max_speed) {
      f[LEFT] = -max_speed;
    }
    else if (max_speed < f[LEFT]) {
      f[LEFT] = max_speed;
    }
  
    f[RIGHT] = f[RIGHT] + df[RIGHT];
    if (f[RIGHT] < -max_speed) {
      f[RIGHT] = -max_speed;
    }
    else if (max_speed < f[RIGHT]) {
      f[RIGHT] = max_speed;
    }

/*
    if (abs(df[LEFT] < 40)) {
      f[LEFT] = 0.0;
    }
    if (abs(df[RIGHT] < 40)) {
      f[RIGHT] = 0.0;
    }
*/

    motors.setM1Speed((int)f[LEFT]);
    motors.setM2Speed((int)f[RIGHT]);
    pmicPDMotors = cmicPDMotors;
  }
}

void PDController_base_velocity() {
//Controls velocity the wheels spin at
//Takes ref and setpoints and computes appropriate motor torques
//  float vel[2];  //Will hold velocity setpoints
  float vel_error[2];  //Will hold velicity errors (setpoint - actual). Max = 800
  float accel[2];  //Will hold current wheel accelerations, which are already computed
  float df[2];  //Force applied to wheels (pwm value)
  
  float max_speed = 300.0;
  float min_speed = 4.0;  //Prev: 6.0
  
  //When printing data to monitor, software delays impact pd controller. use gains k = 0.5, b = 0.1 for critically damped motion.
  float k = 2.5;   //Prev: 0.5, 1.0, 2.0,  2.5,  2.5
  float b = 0.09;  //Prev: 1.0, 0.1, 0.1, 0.08, 0.09
  
  //Constrain the base_set_velocity (find appropriate values by a motor test)
  /*
  
  */
  
  vel_error[LEFT] = wheel_set_velocity[LEFT] - wheel_cur_velocity[LEFT];
  vel_error[RIGHT] = wheel_set_velocity[RIGHT] - wheel_cur_velocity[RIGHT];
  
  accel[LEFT] = wheel_cur_acceleration[LEFT];
  accel[RIGHT] = wheel_cur_acceleration[RIGHT];
  
  df[LEFT] = k*vel_error[LEFT] - b*accel[LEFT];
  df[RIGHT] = k*vel_error[RIGHT] - b*accel[RIGHT];
/*
  Serial.print(wheel_set_velocity[LEFT]);
  Serial.print("   ");
  Serial.print(wheel_set_velocity[RIGHT]);
  Serial.print("   ");
  Serial.print(wheel_cur_velocity[LEFT]);
  Serial.print("   ");
  Serial.print(wheel_cur_velocity[RIGHT]);
  Serial.print("   ");
  Serial.print(f[LEFT]);
  Serial.print("   ");
  Serial.print(df[LEFT]);
  Serial.print("   ");
  Serial.print(f[LEFT] + df[LEFT]);
  Serial.print("   ");
  Serial.println(f[RIGHT] + df[RIGHT]);
*/
  //Constrain the max force applied to the motors
  f[LEFT] = f[LEFT] + df[LEFT];
  if (f[LEFT] < -max_speed) {
    f[LEFT] = -max_speed;
  }
  else if (max_speed < f[LEFT]) {
    f[LEFT] = max_speed;
  }

  f[RIGHT] = f[RIGHT] + df[RIGHT];
  if (f[RIGHT] < -max_speed) {
    f[RIGHT] = -max_speed;
  }
  else if (max_speed < f[RIGHT]) {
    f[RIGHT] = max_speed;
  }
  
  if ((abs(wheel_set_velocity[LEFT]) < min_speed) && (abs(df[LEFT]) < 40)) {
    f[LEFT] = 0.0;
  }
  if ((abs(wheel_set_velocity [RIGHT]) < min_speed) && (abs(df[RIGHT]) < 40)) {
    f[RIGHT] = 0.0;
  }
  
  motors.setM1Speed((int)f[LEFT]);
  motors.setM2Speed((int)f[RIGHT]);
}

