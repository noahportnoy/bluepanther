
void fireFightDemo() {
//Robot exits a starting room using arbitrary start, then continuously navigates around the maze.
//When a candle flame is found, it is extinguished and the robot exits the room, and resumes its navigation, looking for another candle.
  ledOn();
  buttonStart();
  ledOff();
  arbitraryStart();
  roomDetermine();
  //Tell the robot the demo conditions
  room1_wall_location = SOUTH;
  room4_orientation = OPENDOWN;

  continuousFlameHunt();
}

void fireFight(){
  ledOn();
  sensorStart();  //Start on button push or fire-alarm
//  buttonStart();
  ledOff();
  arbitraryStart();   //Gets to doorway of starting room (facing outward) and stops
  roomDetermine();  //Determine the starting room using sonar(s)
  flameHunt();  //Navigates to the next room and searches for candle repeatedly, until candle is located and extinguished
  arbitraryStart();
  // exitLeft();
  roomTapeExit();
  returnHome();  //Drives the robot back to the starting room without entering any other rooms
  finalStop();  //Stops the robot safely inside the room where it started.
}

void flameHunt(){
  while (flame.isLit() == true) {
    goToNextRoom();
    //servoPowerOn();
    if (flameDetect() == true) {  //Flame is detected in the room, begin to approach it and stop when close enough.
      ledOn();
      robotFlamePoint();
      flameApproach2();   //Approach the flame until within range to extinguish it. Some flameApproach.
      flameScanFull(X);
      sensorFlamePoint(X);
      flameScanFull(Y);
      sensorFlamePoint(Y);
      flameExtinguishSP();  //Spray a burst of CO2 at the flame
      verifyExtinguish();  //Verifies that the flame has been extinguished, otherwise will attempt to extinguish it again by flooding it with CO2.
//      arbitraryStart();
      ledOff();
      //servoPowerOff();
    }
    else {
      //servoPowerOff();
      //If pass_through is false, the robot does not  passed through the room, so it is still at the tape facing into the room. It is not facing outwards at another exit.
      if (pass_through == false) {
        
        exit180();
      }
      else {
        //The robot needs to pass through a room. The direction to follow must already have been stored in pass_through_direction
        passThrough(pass_through_direction);
        //Deactivate the completed pass_through call
        pass_through = false;
        pass_through_direction = UNKNOWN;
      }
    }
  }
}

void continuousFlameHunt(){
  while (1) {
    goToNextRoom();
    servoPowerOn();
    if (flameDetect() == true) {  //Flame is detected in the room, begin to approach it and stop when close enough.
      ledBlink();
      robotFlamePoint();
      flameApproach2();   //Approach the flame until within range to extinguish it. Some flameApproach.
      flameScanFull(X);
      sensorFlamePoint(X);
      flameScanFull(Y);
      sensorFlamePoint(Y);
      flameExtinguishSP();  //Spray a burst of CO2 at the flame
      verifyExtinguish();  //Verifies that the flame has been extinguished, otherwise will attempt to extinguish it again by flooding it with CO2.
//      exitLeft();
      arbitraryStart();
      ledBlink();
      servoPowerOff();
    }
    else {
      servoPowerOff();
      //If pass_through is false, the robot does not  passed through the room, so it is still at the tape facing into the room. It is not facing outwards at another exit.
      if (pass_through == false) {
        exit180();
      }
      else {
        //The robot needs to pass through a room. The direction to follow must already have been stored in pass_through_direction
        passThrough(pass_through_direction);
        //Deactivate the completed pass_through call
        pass_through = false;
        pass_through_direction = UNKNOWN;
      }
    }
  }
}

void buttonStart() {
  while (startButton.isPushed() == false) {
  }
}

void sensorStart() {
  Serial1.begin(9600);

  bool alarmDetected = false;

  while (1) {
    // Read the mic frequency
    if (Serial1.available()) {
      int slaveData = (Serial1.read() & 0xFF);
      if (slaveData == 0x1) {
        alarmDetected = true;
      }
    }
    
    // Break out on button push, or on hearing approx frequency of fire alarm
    if (startButton.isPushed() || alarmDetected == true) {
      break;
    }
    
  }
  Serial1.end();
}

void wallFollow(int wallDir){
//wallDir = either LEFT or RIGHT only
  wall_follow_direction = wallDir;
  updateSonars();
  macro0();
  controlBase();
}


void finalStop() {
  moveDist(15);
  stopMotors();
  while (1) {
    
  }
  
}

void exitLeft() {
  rotate(90);
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
  }
}

