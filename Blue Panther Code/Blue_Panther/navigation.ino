//These methods do what they can to determine the unknown locations of the dog and room-1 door, and the orientation of room 4
//They assume that the robot is in the white-tape marking the room's doorway, and the robot must move past the line before beginning to wallfollow to the next room
//They take the robot a little  bit past the white-tape in the destination room's doorway before stopping.
//They are direct routes, so the robot must not, and will not, enter any other rooms along the way.
//Pass-through methods for room 1 may save time while navigating.
  //These may be called by the over-arching function in the special circumstances they may be needed. E.g. dog_location unknown after rm4to1?
boolean dogAt1;
int startRoomNum;  //Room the robot started in, assigned by arbitrary-start method
int curRoomNum;  //The room the robot enters, or has just exited by arbitrary start. This is updated each time the robot enters a new room.

int room4NextRoom = 0;
boolean specialExitRoom1 = false;

void roomTapeExit() {
//Moves the robot forward a little bit, just past the white tape, if it is lined up
  moveDist(5);
}

void passThrough(int wallDir) {
//Follow wall in wallDir, and stops at white-tape
//Intended fro room 1 when designed
  roomTapeExit();
  while(autoLineDetect() == false) {
    wallFollow(wallDir);
  }
}

void room1Eto2() {
//Room 1 east-exit to room 2
  roomTapeExit();
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
  }
  lineUpEnter();
  curRoomNum = 2;
}

// Todo: needs testing
void room1Eto3() {
//Room 1 east-exit to room 3
  roomTapeExit();
  moveDist(20);
  rotate(90);
  if (room1_wall_location == NORTH) {
    moveDist(60);
  }
  else if (room1_wall_location == SOUTH) {
    moveDist(10);
  }
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
  }

// ROOM to ROOM method, invalid?
//  while(autoLineDetect() == false) {
//    wallFollow(LEFT);
//  }
//  rotate(180);
//  moveDist(15);
//  while(autoLineDetect() == false) {
//    wallFollow(LEFT);
//  }
  lineUpEnter();
  curRoomNum = 3;
}

// TODO NOAH: Needs testing
void room1Nto3() {
//Room 1 north-exit to room 3
  int distanceTraveledToDog;
  dog_location = UNKNOWN;
  //locateDogFromRoom1N();
  roomTapeExit();
  if ((dog_location == UNKNOWN) || (dog_location == ONE) || (dog_location == TWO)) {
    odometer[0] = 0;
    while(autoLineDetect() == false) {
      wallFollow(LEFT);
      if(getDist(FRONTIR) < 400) {
        //dog detected
        dog_location = THREE;
        break;
      }
    }
  
    //If dog was detected at location 3, then capture distance traveled to the dog
    //Then rotate 180, follow left wall and then switch to following the right wall
    //Into room 3
    //ALTERNATIVE - wall follow right back to room 1 north entrance, then rotate180
    //and wall follow right
    if(dog_location == THREE) {
      distanceTraveledToDog = odometer[0];
      rotate(180);
      odometer[0] = 0;
      while(autoLineDetect() == false && odometer[0] < distanceTraveledToDog) {
        wallFollow(LEFT);
      }
      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }
    }
    //dog was not detected at location 3. now at room 1 east entrance
    //rotate 180, follow left wall to room 2, rotate 180, then follow left wall to room 3
    else {
      rotate(180);
      while(autoLineDetect() == false) {
        wallFollow(LEFT);
      }
      rotate(180);
      while(autoLineDetect() == false) {
        wallFollow(LEFT);
      }
    }
    
      //Go to center, turn left, move alongside room 2 wall, turn 180 then leftwallfollow into room 3
//    moveDist(18);
//    //make delay shorter
//    delay(3000);
//    rotate(-90);
//    //make delay shorter
//    delay(2000);
    //If dog has not yet been discovered, see if dog is down this hallway
//    if(dog_location == UNKNOWN) {
//      int frontIRdist = getDist(FRONTIR);
//      Serial.println(frontIRdist);
//      delay(1);
//      //If the front IR gives a distance less than 700, the dog is there at location THREE
//      //Turn clockwise 90 degrees and wallfollow right into room 3
//      if(frontIRdist < 700) {
//        dog_location = THREE;
//        rotate(90);
//        while(autoLineDetect() == false) {
//          wallFollow(RIGHT);
//        }
//      }
//    }
    //If dog was not found at location THREE or the dog is known to be at ONE or TWO,
    //then continue down the hall to room 3
//    if ((dog_location == UNKNOWN) || (dog_location == ONE) || (dog_location == TWO)) {
//      ledBlink();
//      delay(8000);
//      moveDist(90);
//      rotate(-90);
//      moveDist(40);
//      rotate(180);
//      while(autoLineDetect() == false) {
//        wallFollow(LEFT);
//      }
//    }
  }
  //If we exit room1N and the dog location is THREE, then wallfollow right up to room 3
  else if (dog_location == THREE) {
    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }
  }
  lineUpEnter();
  curRoomNum = 3;
}

//Todo: Needs testing
void room1Eto4() {
//Room 1 east-exit to room 4
//Assumes that room 1's var door location is known. Room 4's orientation does not need to be known.
  roomTapeExit();
  moveDist(20);
  rotate(90);
  
  if (room1_wall_location == NORTH) {
    moveDist(90);
  }
  else if (room1_wall_location == SOUTH) {
    moveDist(50);
  }
  
  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
    if (sonarDist[FRONTSONAR] < 20) {
      rotate(180);
      while(autoLineDetect() == false) {
        wallFollow(LEFT);
      }
      break;
    }
  }
  lineUpEnter();
  curRoomNum = 4;
}

//Todo: Needs testing
void room1Nto4() {
//Room 1 north-exit to room 4
  locateRoom1VarDoorFromRoom1N();  //Determine the location of room 1's var door from room 1N
  roomTapeExit();
  //Move to room 4
  moveDist(40);
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
    if (sonarDist[FRONTSONAR] < 25) {
      rotate(180);
      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }
      lineUpEnter();
      break;
    }
  }
  lineUpEnter();
  curRoomNum = 4;
}

void room2to1E() {
//Room 2 exit to room 1 east-entrance
  disableSonars();
  frontSonar.enable();
  delay(50);    //Allow any previous echo to clear
  int frontDist = frontSonar.read();
  if (room1_wall_location == UNKNOWN) {
    if (frontDist < 60) {  //Wall is ahead
      room1_wall_location = SOUTH;
    }
    else {
      room1_wall_location = NORTH;
    }
  }
  roomTapeExit();
  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }
  lineUpEnter();
  if (room1_wall_location == NORTH) {
    rotate(-55);
    specialExitRoom1 = true;
  }
  curRoomNum = 1;
}

void room2to3() {
  roomTapeExit();
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
  }
  lineUpEnter();
  curRoomNum = 3;
}

//Todo: Needs testing
void room2to4() {
//Room 2 to room 4, dog location does not matter
//***Add dog-location-detection using odometry
  roomTapeExit();
  moveDist(20);
  rotate(-90);
  moveDist(90);
  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
    if (sonarDist[FRONTSONAR] < 20) {
      rotate(180);
      while(autoLineDetect() == false) {
        wallFollow(LEFT);
      }
      break;
    }
  }
  lineUpEnter();
  curRoomNum = 4;
}

//Todo: Needs testing
void room3to1E() {
//Room 3 exit to room 1 east-entrance
  moveDist(20);
  rotate(-90);
  moveDist(65);
  rotate(90);
  if (room1_wall_location == NORTH) {
    moveDist(60);
  }
  else if (room1_wall_location == SOUTH) {
    moveDist(40);
  }
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
  }
  lineUpEnter();
  curRoomNum = 1;
}

void room3to2() {
  roomTapeExit();
  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }
  lineUpEnter();
  curRoomNum = 2;
}

//Todo: Need to test with dog
void room3to4() {
//***This method may need to change depending on what trinity say about the dog locations when room 4 is oriented opendown
  //Use a timer to measure this on an interval and allow time for primitive0 to rotate away from the wall
  roomTapeExit();
  //Get to the left wall of room 4 and rightwallfollow until the entrance. If there is an head-obstacle, it can only be a dog.
  //Determine its location, using odometry, turn 180, and then leftwallfollow until room 4 entrance.
  moveDist(20);
  rotate(-85);
  moveDist(65);
  rotate(-85);
  moveDist(30);
  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }
  lineUpEnter();
  curRoomNum = 4;
}

//Todo: Needs testing
void room4to1E() {
//Specific entrance to room 1 from room 4 is unknown because the orientation of room 4 may not be known
//Measure front distance to gauge room size before entering, to know which entrance robot is at, and to then determine the location of the var. door
  roomTapeExit();
  moveDist(26);
  while (autoLineDetect() == false) {
    wallFollow(LEFT);
  }
  lineUpEnter();
  curRoomNum = 1;
}

//Todo: Consider other orientations of room 4
void room4to1N() {
  roomTapeExit();
  if (room4_orientation == OPENDOWN) {
    disableSonars();
    frontSonar.enable();
    updateSonarsSeq();
    //Move forward until the wall ahead is close, then right-wall-follow into room 1N
//    while(sonarDist[FRONTSONAR] < 20) {
//      updateSonars();
//      setVelocities(20);
//      controlBase();
//    }
    moveDist(18);
    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }
    lineUpEnter();
  }
  curRoomNum = 1;
}

//void room4to2() {
//  roomTapeExit();
//  if(room4_orientation == OPENUP) {
//      moveDist(15);
//      rotate(-90);
//      moveDist(70);
//      rotate(-90);
//      moveDist(194);
//      rotate(90);
//  } else {
//    moveDist(15);
//    rotate(90);
//    moveDist(50);
//    rotate(-90);
//    moveDist(91);
//    rotate(90);
//  }
//  while(autoLineDetect() == false) {
//    wallFollow(LEFT);
//  }
//  lineUpEnter();
//  curRoomNum = 2;
//}
//
//void room4to3() {
//  roomTapeExit();
//  if(room4_orientation == OPENUP) {
//    moveDist(15);
//    rotate(-90);
//    while(autoLineDetect() == false) {
//      wallFollow(RIGHT);
//    }
//  }
//  else {
//    moveDist(15);
//    rotate(97);
//    moveDist(60);
//    while(autoLineDetect() == false) {
//      wallFollow(LEFT);
//    }
//  }
//  lineUpEnter();
//  curRoomNum = 3;
//}
//Todo: Needs testing
void room4to2() {
  dogAt1 = false;
  odometer[0] = 0;
  int odometerDogApproached = 0;
  frontSonar.enable();
  rfSonar.enable();
  roomTapeExit();
  if(room4_orientation == OPENUP) {
   
    moveDist(15);
    rotate(-90);
    
    while(odometer[0] < 20) {              //wall follow right, check for dog
      wallFollow(RIGHT);
      if(sonarDist[FRONTSONAR] < 25) {
        dogAt1 = true;
        break;
      }
    }
   
    if(dogAt1 == true) {
      rotate(180);
      odometerDogApproached = odometer[0];
      odometer[0] = 0;
      while(odometer[0] < (odometerDogApproached + 25)) {
        wallFollow(LEFT);
      }
      while(sonarDist[RFSONAR] < 50) {
        wallFollow(RIGHT);
      }
      moveDist(30);
      rotate(-90);
      moveDist(30);
    } else if (dogAt1 == false) {  
      while(sonarDist[FRONTSONAR] > 15) {        
        wall_follow_direction = RIGHT;
        primitive1();
        controlBase();
      }
      rotate(-90);
      moveDist(140);
    }
///////////////////////////////////////////////////////////    
  } else if(room4_orientation == OPENDOWN) {
      moveDist(15);
      rotate(90);
      moveDist(50);
      rotate(-90);
      moveDist(40);
  }
  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }
  lineUpEnter();
  curRoomNum = 2;
}

//Todo: Needs testing
void room4to3() {
  dogAt1 = false;
  odometer[0] = 0;
  int odometerDogApproached = 0;
  frontSonar.enable();
  rfSonar.enable();
  roomTapeExit();
  if(room4_orientation == OPENUP) {
   
    moveDist(15);
    rotate(-90);
    
    while(odometer[0] < 20) {              //wall follow right, check for dog
      wallFollow(RIGHT);
      if(sonarDist[FRONTSONAR] < 25) {
        dogAt1 = true;
        break;
      }
    }
   
    if(dogAt1 == true) {
      rotate(180);
      odometerDogApproached = odometer[0];
      odometer[0] = 0;
      while(odometer[0] < (odometerDogApproached + 25)) {
        wallFollow(LEFT);
      }
      while(sonarDist[RFSONAR] < 50) {
        wallFollow(RIGHT);
      }
      rotate(13);
      moveDist(55);
      while(autoLineDetect() == false) {
        wallFollow(LEFT);
      }   
    } else {  
      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }
    }
///////////////////////////////////////////////////////////    
  } else if(room4_orientation == OPENDOWN) {
    moveDist(15);
    rotate(99);
    moveDist(71);
  }
  lineUpEnter();
  curRoomNum = 3;
}

void goToNextRoom() {
//Programed to search the rooms in ascending order, assumed with no dog, room 1 wall location = south, and room 4 opening = opendown.
//Once the robot has exited a room, this method determines the next room to search and calls the correct room-x-to-room-y- method to take the robot there.
  if (startRoomNum == 1) {
    if (room1_start_room_exit == EAST) {
    //1E -> 2 -> 3 -> 4 -> 1N -> 1E
      switch (curRoomNum) {
        case 1:
          room1Eto2();
          break;
        case 2:
          room2to3();
          break;
        case 3:
          room3to4();
          break;
        case 4:
          room4to1N();
          pass_through = true;
          pass_through_direction = LEFT;
          break;
      }
    }
    else if (room1_start_room_exit == NORTH) {
    //Figure out dog location
    //Go from 1N to 1E (passthrough), then -> 2 -> 3 -> 4 -> 1N (-> passthrough) 1E
      switch (curRoomNum) {
        case 1:
          rotate(180);
          moveDist(10);
          passThrough(LEFT);
          roomTapeExit();
          room1Eto2();
          break;
        case 2:
          room2to3();
          break;
        case 3:
          room3to4();
          break;
        case 4:
          room4to1N();
          break;
      }
    }
  }
  else if (startRoomNum == 2) {
  //2 -> 3 -> 4 -> 1N (->passthrough) -> 1E -> 2
    switch (curRoomNum) {
      case 2:
        room2to3();
        break;
      case 3:
        room3to4();
        break;
      case 4:
        room4to1N();
        pass_through = true;
        pass_through_direction = LEFT;
        break;
      case 1:
        room1Eto2();
        break;
    }
  }
  else if (startRoomNum == 3) {
  //3 -> 4 -> 1N (-> Pass through to 1E) -> 2 -> 3
    switch (curRoomNum) {
      case 3:
        room3to4();
        break;
      case 4:
        room4to1N();
        pass_through = true;  //Tell the superior flameHunt program that it should not exit180 since the robot will be facing outwards at another exit, not inwards where it started
        pass_through_direction = LEFT;
        break;
      case 1:
        room1Eto2();
        break;
      case 2:
        room2to3();
        break;
    }
  }
  else if (startRoomNum == 4) {
    //4 -> 1N (-> passthrough) 1E -> 2 -> 3 -> 4
    switch (curRoomNum) {
      case 4:
        room4to1N();
        pass_through = true;
        pass_through_direction = LEFT;
        break;
      case 1:
        room1Eto2();
        break;
      case 2:
        room2to3();
        break;
      case 3:
        room3to4();
        break;
    }
  }
}

void locateDogFromRoom1N() {
//This comprehensively determines the dog's location
//Assumes robot is on the white-tape facing out from room 1's north doorway
/*
  disableSonars();
  frontSonar.enable();
  lfSonar.enable();

  delay(50);    //Allow any previous echo to clear
  int frontDist = frontSonar.read();
  delay(50);
  int leftFrontDist = lfSonar.read();
  //This comprehensively determines the dog's location
  if (dog_location == UNKNOWN) {
    if (frontDist < 90) {  //Dog is ahead
      dog_location = TWO;
    }
    else if (leftFrontDist < 60) {  //Dog is to the left
      dog_location = THREE;
    }
    else {  //Dog is around the corner, by process of elimination
      dog_location = ONE;
    }
  }
*/
}

void locateRoom1VarDoorFromRoom1N() {
//Determine the location of room 1's variable door from room 1N
  lbSonar.enable();
  delay(50);    //Allow any previous echo to clear
  int leftBackDist = lbSonar.read();
  if (room1_wall_location == UNKNOWN) {
    if (leftBackDist < 180) {
      room1_wall_location = NORTH;
    }
    else {
      room1_wall_location = SOUTH;
    }
  }
}

void arbitraryStart() {
//Takes the robot from a random position in a room to the white tape.
//Does not move the robot past the white-tape at the doorway

//Before the robot moves, try to detect a wall nearby
//If a wall is detected, set the wall-following-direction
//Otherwise, move fowrward until a wall is detected around the robot
//Then follow that wall out or the room and stop at the white tape

  int frontSonarBuffer = 25;
  int leftIRBuffer = 400;
  int rightIRBuffer = 400;
  int leftSonarBuffer = 20;
  int rightSonarBuffer = 20;
  
  enableSonars();
  updateSonarsSeq();
  disableSonars();
  int wallDir = UNKNOWN;
  int leftIRDist = getDist(LEFTIR);
  int rightIRDist = getDist(RIGHTIR);
  
  //Check the front sonar
  if (sonarDist[FRONTSONAR] < frontSonarBuffer) {  //If there's a wall ahead
    if (leftIRDist < rightIRDist) {  //If the left side of the robot is closer to the wall, follow the left wall
      wallDir = LEFT;
    }
    else {
      wallDir = RIGHT;
    }
  }
  
  //Check the distanceIRs
  else if ((leftIRDist < leftIRBuffer) && (rightIRDist < rightIRBuffer)) {  //If both front corners of the robot are close to a wall, rotate left some then follow the right wall
    rotate(-90);
    wallDir = RIGHT;
  }
  else if (leftIRDist < leftIRBuffer) {  //If the left side of the robot is closer to the wall, follow the left wall
    wallDir = LEFT;
  }
  else if (rightIRDist < rightIRBuffer) {  //If the right side of the robot is closer to the wall, follow the right wall
    wallDir = RIGHT;
  }
  
  //Check the side-sonars at the front
  else if (sonarDist[LFSONAR] < leftSonarBuffer) {  //If the left side of the robot is closer to the wall, follow the left wall
    wallDir = LEFT;
  }
  else if (sonarDist[RFSONAR] < rightSonarBuffer) {  //If the right side of the robot is closer to the wall, follow the right wall
    wallDir = RIGHT;
  }
  
  //If a wall has been detected, follow it to the white tape. Otherwise move forward until a wall is detected then follow that wall to the white tape.
  if (wallDir != UNKNOWN) {
    while (autoLineDetect() == false) {
      wallFollow(wallDir);
    }
  }
  else {
    lfSonar.enable();
    frontSonar.enable();
    rfSonar.enable();

    while(wallDir == UNKNOWN) {
      setVelocities(20);
      updateSonars();
      leftIRDist = getDist(LEFTIR);
      rightIRDist = getDist(RIGHTIR);
      if (autoLineDetect() == true) {
        break;
      }
      //***Repeat the wall-detection used when the robot was stationary
      //Check the front sonar
      if (sonarDist[FRONTSONAR] < 30) {
        //If there's a wall ahead, check the IR distance sensors
        if (leftIRDist < rightIRDist) {  //If the left side of the robot is closer to the wall, follow the left wall
          wallDir = LEFT;
        }
        else {
          wallDir = RIGHT;
        }
      }
      //Check the distanceIRs
      else if ((leftIRDist < leftIRBuffer) && (rightIRDist < rightIRBuffer)) {
        //If both front corners of the robot are close to a wall, rotate left some then follow the right wall
        rotate(-90);
        wallDir = RIGHT;
      }
      else if (leftIRDist < leftIRBuffer) {  //If the left side of the robot is closer to the wall, follow the left wall
        wallDir = LEFT;
      }
      else if (rightIRDist < rightIRBuffer) {  //If the right side of the robot is closer to the wall, follow the right wall
        wallDir = RIGHT;
      }
      //Check the side-sonars at the front
      else if (sonarDist[LFSONAR] < leftSonarBuffer) {  //If the left side of the robot is closer to the wall, follow the left wall
        wallDir = LEFT;
      }
      else if (sonarDist[RFSONAR] < rightSonarBuffer) {  //If the right side of the robot is closer to the wall, follow the right wall
        wallDir = RIGHT;
      }
      
      //If it has found a wall on this iteration, follow it until the line
      if (wallDir != UNKNOWN) {
        while (autoLineDetect() == false) {
          wallFollow(wallDir);
        }
        break;
      }
      controlBase();
    }
  }
  lineUp();
}

void roomDetermine() {
//Determines the robots starting room number, and if in room 1 identifies which specific exit the robot is at.
  enableSonars();
  updateSonarsSeq();
  disableSonars();
  
  if (sonarDist[REARSONAR] < 50) {
    startRoomNum = 4;
    if (sonarDist[RFSONAR] < 100) {
      room4_orientation = OPENUP;
    }
    else {
      room4_orientation = OPENDOWN;
    }
  }
  else if (sonarDist[REARSONAR] < 62) {
    startRoomNum = 2;
    //Determine location of room 1's variable door
    if (sonarDist[FRONTSONAR] < 70) {
      room1_wall_location = SOUTH;
    }
    else {
      room1_wall_location = NORTH;
    }
  }
  else if (sonarDist[REARSONAR] < 90) {
    if (sonarDist[LBSONAR] < 50) {
      startRoomNum = 3;
    }
    else {
      startRoomNum = 1;
      room1_start_room_exit = NORTH;
      //Determine location of room 1's variable door
      if (sonarDist[LBSONAR] < 100) {
        room1_wall_location = NORTH;
      }
      else {
        room1_wall_location = SOUTH;
      }
    }
  }
  else {
    startRoomNum = 1;
    room1_start_room_exit = EAST;
    if (sonarDist[LBSONAR] < 40) {
      room1_wall_location = NORTH;
    }
    else {
      room1_wall_location = SOUTH;
    }
  }
/*
  else if (rearDist < ) {
    
  }
*/
  curRoomNum = startRoomNum;
//  moveDist(20);
//  ledBlink(startRoomNum);
}

void exit180() {
  if (specialExitRoom1 == true) {
    rotate(235);    //Robot is rotated left 45 degrees to scan for candle
    specialExitRoom1 = false;
  }
  else {
    rotate(180);
  }
  lineUp();
}

void returnHome() {
//Assumes robot is in a random position and direction inside a room.
//Takes robot out of the current room, and directly to the destination room.
  if(curRoomNum == 1) {
    switch(startRoomNum) {
      case 2:
        room1Eto2();
        break;
      case 3:
        room1Eto3(); //can also use room1Nto3();
        break;
      case 4:
        room1Eto4(); //can also use room1Nto4();
        break;
    }
  }
  else if(curRoomNum == 2) {
    switch(startRoomNum) {
      case 1:
        room2to1E();
        break;
      case 3:
        room2to3();
        break;
      case 4:
        room2to4();
        break;
    }
  }
  else if(curRoomNum == 3) {
    switch(startRoomNum) {
      case 1:   room3to1E();  //can also use room3to1N();
        break;
      case 2:   room3to2();
        break;
      case 4:   room3to4();
        break;
    }
  }
  else if(curRoomNum == 4) {
    switch(startRoomNum) {
      case 1:
        room4to1E();
        break;
      case 2:
        room4to2();
        break;
      case 3:
        room4to3();
        break;
    }
  }
}

