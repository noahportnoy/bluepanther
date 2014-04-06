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
int room4to1ArrivalEntrance = 0;
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

// Chi - looks good!
void determineRoom4OrientationFromRoom4() {
  rfSonar.enable();
  updateSonarsSeq();
  if (sonarDist[RFSONAR] < 100) {
    room4_orientation = OPENUP;
  }
  else {
    room4_orientation = OPENDOWN;
  }
}

// Chi - looks good!
void moveForwardUntilWall(int stopDist) {
  //Uses the front-mounted sonar
  disableSonars();
  frontSonar.enable();
  updateSonarsSeq();
  while(sonarDist[FRONTSONAR] > stopDist) {
    updateSonars();
    setVelocities(20);
    controlBase();
  }
  stopMotors();
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

// Noah - looks good!
void room1Nto2() {
//Room 1 north-exit to room 3
  locateRoom1VarDoorFromRoom1N();  // Determine the location of room 1's var door from room 1N
  rotate(180);
  roomTapeExit();

  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }
  lineUp();
  room1Eto2();
}

// Noah - looks good!
void room1Eto3() {
//Room 1 east-exit to room 3

  roomTapeExit();
  int distToWallSwitch = 40;

  if(room1_wall_location == NORTH) {
    odometer[0] = 0;

    while(odometer[0] < distToWallSwitch) {
      wallFollow(RIGHT);
    }
    while(autoLineDetect() == false) {
      wallFollow(LEFT);
    }

  } else if(room1_wall_location == SOUTH) {
    moveForwardUntilWall(15);
    rotate(90);

    while(autoLineDetect() == false) {
      wallFollow(LEFT);
    }
  }

  lineUpEnter();
  curRoomNum = 3;
}

// Noah - looks good!
void room1Nto3() {
//Room 1 north-exit to room 3
  locateRoom1VarDoorFromRoom1N();  // Determine the location of room 1's var door from room 1N
  rotate(180);
  roomTapeExit();

  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }
  lineUp();
  room1Eto3();
}

//Todo Noah: Needs testing
void room1Eto4() {
  //Get from room 1 east exit to room 1 north exit

  // Turn around and move past tape
  rotate(180);
  roomTapeExit();
  
  // wall follow left to room 1 north exit
  while(autoLineDetect() == false) {
    wallFollow(LEFT);
  }

  lineUpEnter();

  // call room 1 north to room 4 method
  room1Nto4();
}

// Noah: When dog is at location 1 against north wall, robot can't see the dog
void room1Nto4() {
// Room 1 north exit to room 4
  locateRoom1VarDoorFromRoom1N();  // Determine the location of room 1's var door from room 1N
  roomTapeExit();

  int distToWallSwitch = 70;     // distance to wall switch location, outside of room 4, east hallway
  int distToDog2 = 120;          // from room 1 north exit to dog 2 (at most)
  odometer[0] = 0;
  int frontirreading;

  // wall follow right to wall switch location
  while(odometer[0] <= distToWallSwitch) {
    wallFollow(RIGHT);

    // if we see the dog, it's at location 2
    if(getDist(FRONTIR) < 500) {
      dog_location = TWO;
      break;
    }
  }

  // if dog was not yet discovered at location 2, wall follow left until we've traveled distToDog2
  // if we see the dog, it's at location 2
  if(dog_location != TWO) {
    while(odometer[0] <= distToDog2) {
      wallFollow(LEFT);
      if(getDist(FRONTIR) < 500) {
        dog_location = TWO;
        break;
      }
    }
  }

  // if the dog is at location 2, turn around and wall follow right into room 4
  if(dog_location == TWO) {
    rotate(180);
    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }

  // if the dog was not found at location 2, wall follow left
  // if we see the dog, it's at location 1
  } else {
    while(autoLineDetect() == false) {
      wallFollow(LEFT);
      if(getDist(FRONTIR) < 500) {
        dog_location = ONE;
        //Serial.println("Dog seen at ONE");
        break;
      }
    }
  }

  // if we discovered the dog at location 1, turn around and wall follow right into the room 4
  if(dog_location == ONE) {
    rotate(180);
    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
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

// Noah - looks good!
void room2to4() {
//Room 2 to room 4
  int distToWallSwitch = 80;   // wall switch location between 3 and 4

  //Strategy: Left wall-follow almost to entrance of room 3; rotate 180 in the hallway then follow the room 3 to 4 navigation strategy.

  odometer[0] = 0.0;
  float distToTravel = 140;
  //Left wall-follow almost to entrance of room 3
  while (odometer[0] < distToTravel) {
    wallFollow(LEFT);
  }
  //Rotate 180 in the hallway then follow the room 3 to 4 navigation strategy.
  rotate(180);
  
  ////////////// begin room3to4 strategy ///////////////
  odometer[0] = 0;

  // wall follow left to wall switch location
  while(odometer[0] < distToWallSwitch) {
    wallFollow(LEFT);
  }

  rotate(180);

  while(autoLineDetect() == false) {
    wallFollow(LEFT);
    if(getDist(FRONTIR) < 500) {
      dog_location = TWO;
      break;
    }
  }

  if(dog_location == TWO) {
    rotate(180);

    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }
  }

  ////////////// end room3to4 strategy ///////////////

  lineUpEnter();
  curRoomNum = 4;
}

// Todo Noah: Need to correct against right wall after distToWallSwitch, then test
void room3to1E() {
//Room 3 exit to room 1 east-entrance
  int distToWallSwitch = 140;
  odometer[0] = 0;

  roomTapeExit();
  while(odometer[0] < distToWallSwitch) {
    wallFollow(RIGHT);
  }

  //rotate(15);
  moveForwardUntilWall(15);
  rotate(-100);

  while(autoLineDetect() == false) {
    wallFollow(RIGHT);
  }

  // if(room1_wall_location == NORTH) {
    
  //   while(autoLineDetect() == false) {
  //     wallFollow(LEFT);
  //   }
  // } else if(room1_wall_location == SOUTH) {
  //   rotate(-90);
  //   moveDist(15);
  // }

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

// Noah - looks good!
void room3to4() {
  roomTapeExit();
  
  int distToWallSwitch = 120;   // wall switch location between 3 and 4
  odometer[0] = 0;

  // wall follow left out of room 3 to wall switch location
  while(odometer[0] < distToWallSwitch) {
    wallFollow(LEFT);
  }

  rotate(180);

  while(autoLineDetect() == false) {
    wallFollow(LEFT);
    if(getDist(FRONTIR) < 500) {
      dog_location = TWO;
      break;
    }
  }

  if(dog_location == TWO) {
    rotate(180);

    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }
  }
  
  lineUpEnter();
  curRoomNum = 4;
}
// Noah - looks good!
void room4to1() {
  int distToWallSwitch = 150;
  int distToWallSwitch2 = 140;
  int distToWallSwitch3 = 50;
  int distToDogAt2 = 0;
  determineRoom4OrientationFromRoom4();
  roomTapeExit();

  if(room4_orientation == OPENDOWN) {
    moveForwardUntilWall(15);
    rotate(90);

    while(autoLineDetect() == false) {
      wallFollow(LEFT);
    }

    room4to1ArrivalEntrance = EAST;

  } else if(room4_orientation == OPENUP) {
    odometer[0] = 0;

    while(odometer[0] < distToWallSwitch3) {
      wallFollow(RIGHT);
      if(getDist(FRONTIR) < 500) {
        // dog seen at location 2
        dog_location = TWO;
        Serial.println("Dog seen at TWO");
        break;
      }
    }

    if(dog_location == TWO) {
      rotate(-150);
      odometer[0] = 0;

      while(odometer[0] < distToWallSwitch) {
        wallFollow(RIGHT);
      }

      odometer[0] = 0;
      while(odometer[0] < distToWallSwitch2) {
        wallFollow(LEFT);
      }

      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }

      room4to1ArrivalEntrance = NORTH;

    } else {
      odometer[0] = 0;
      while(autoLineDetect() == false) {
        wallFollow(LEFT);
        if(getDist(FRONTIR) < 500) {
          // dog seen at location 2
          dog_location = TWO;
          Serial.println("Dog seen at TWO");
          distToDogAt2 = odometer[0];
          break;
        }
      }

      if(dog_location == TWO) {
        rotate(180);
        odometer[0] = 0;

        while(odometer[0] < distToWallSwitch + distToDogAt2) {
          wallFollow(RIGHT);
        }

        odometer[0] = 0;
        while(odometer[0] < distToWallSwitch2) {
          wallFollow(LEFT);
        }

        while(autoLineDetect() == false) {
          wallFollow(RIGHT);
        }
      }

      room4to1ArrivalEntrance = NORTH;
    }
  }

  lineUpEnter();
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

//Todo Chi: Needs testing. Tested, works ~75% of time. 4-OPENDOWN to 2 needs most testing
void room4to2() {
  dogAt1 = false;
  odometer[0] = 0;
  int odometerDogApproached = 0;
  frontSonar.enable();
  rfSonar.enable();

  determineRoom4OrientationFromRoom4();
  roomTapeExit();

  if(room4_orientation == OPENUP) {
    
    moveForwardUntilWall(15);
    rotate(-90);
    odometer[0] = 0.0;
    while(odometer[0] < 40) {              //wall follow right, check for dog
      wallFollow(RIGHT);
      if(getDist(FRONTIR) < 500) {
        dogAt1 = true;
        break;
      }
    }
  
    if(dogAt1 == true) {
      rotate(180);
      odometerDogApproached = odometer[0];
      odometer[0] = 0;
      //Move past room 4's doorway, before switching wall-following direction
//      while(odometer[0] < (odometerDogApproached + 25)) {
      while(odometer[0] < 60) {
        wallFollow(LEFT);
      }
      //Follow around room 4 until room 3 wall is on the left, before rotating to follow room 3's wall
      odometer[0] = 0.0;
      while(odometer[0] < 250) {
        wallFollow(RIGHT);
      }
      rotate(-180);
      //Follow room 3's wall until room 2's wall is on the left, before rotating to follow room 2's wall
      odometer[0] = 0.0;
      while(odometer[0] < 85) {
        wallFollow(RIGHT);
      }
      rotate(-180);
      //Follow into room 2
      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }
    } else if (dogAt1 == false) {
      //Follow right wall until room 2's wall is on the left, before rotating to follow room 2's wall
      odometer[0] = 0.0;
      while(odometer[0] < 200) {
        wallFollow(RIGHT);
      }
      rotate(-180);
      //Follow into room 2
      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }
    }
    ///////////////////////////////////////////////////////////
  } else if(room4_orientation == OPENDOWN) {
    //Follow right wall until room 3's wall is on the left, before rotating to follow room 3's wall on the right
    odometer[0] = 0.0;
    while(odometer[0] < 90) {
      wallFollow(RIGHT);
    }
    rotate(-180);
    //Follow right wall until room 2's wall is on the left, before rotating to follow room 2's wall
    odometer[0] = 0.0;
    while(odometer[0] < 70) {
      wallFollow(RIGHT);
    }
    rotate(-180);
    //Follow into room 2
    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }
  }
  lineUpEnter();
  curRoomNum = 2;
}

// Noah - looks good!
void room4to3() {
	determineRoom4OrientationFromRoom4();
	roomTapeExit();
  odometer[0] = 0;
  int distToWallSwitch1 = 60;     // distance from room4 up entrance, left, to wall switch between 4 and 3
  int distToWallSwitch2 = 110;    // distance from dog location 1 to wall switch location outside room 4, east hallway
  int distToWallSwitch3 = 320;    // distance from dog location 1 to wall switch location between 4 and 3
  int distToWallSwitch4 = 90;     // distance from room4 down entrance to wall switch between 4 and 3
  int distToWallSwitch5 = 290;    // distance from room4 up entrance, right, to wall switch between 4 and 3
  dog_location = ONE;

  // if room 4 open up
  if(room4_orientation == OPENUP) {

    // if dog not known to be at location 1
    if(dog_location == UNKNOWN || dog_location == TWO || dog_location == THREE || dog_location == TWO_THREE) {

      // wall follow left to wall switch 1 location between rooms 4 and 3
      while(odometer[0] < distToWallSwitch1) {
        wallFollow(LEFT);

        // if dog is discovered at location 1, set dog_location and break
        if(getDist(FRONTIR) < 500 && dog_location == UNKNOWN) {
          dog_location = ONE;
          break;
        }
      }

      // if the dog was discovered at location 1, turn around and loop around room 4 to other side of dog
      if(dog_location == ONE) {
        moveDist(-5);
        rotate(150);
        odometer[0] = 0;

        // wall follow left until wall switch 2 location outside room 4, east hallway
        while(odometer[0] < distToWallSwitch2) {
          wallFollow(LEFT);
        }

        // wall follow right until we're between rooms 3 and 4, then turn around
        while(odometer[0] < distToWallSwitch3) {
          wallFollow(RIGHT);
        }
        rotate(180);

        // wall follow right into room 3
        while(autoLineDetect() == false) {
          wallFollow(RIGHT);
        }

      // if dog was not seen at location 1, wall follow right into room 3
      } else {
        // if dog location was unknown, set it to TWO_THREE
        if(dog_location == UNKNOWN) {
          dog_location = TWO_THREE;
        }

        // wall follow right into room 3
        while(autoLineDetect() == false) {
          wallFollow(RIGHT);
        }
      }
    // if we know dog location to be 1 at the outset, then loop around room 4 to wall switch between 4 and 3
    } else if(dog_location == ONE) {
      odometer[0] = 0;

      // wall follow right out of room 4 until we're between rooms 4 and 3, then turn around
      while(odometer[0] < distToWallSwitch5) {
          wallFollow(RIGHT);
      }
      rotate(180);

      // wall follow right into room 3
      while(autoLineDetect() == false) {
        wallFollow(RIGHT);
      }

    }

  // if room 4 orientation is OPENDOWN, wall follow right until we've traveled distToWallSwitch4
  } else if(room4_orientation == OPENDOWN) {
    odometer[0] = 0;

    // wall follow right until we've traveled distToWallSwitch4, then turn around
    while(odometer[0] < distToWallSwitch4) {
      wallFollow(RIGHT);
    }
    rotate(180);

    // wall follow right into room 3
    while(autoLineDetect() == false) {
      wallFollow(RIGHT);
    }
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
          room4to1();
          break;
      }
    }
    else if (room1_start_room_exit == NORTH) {
    //Figure out dog location
    //Current plan: 1N -> 4 -> 3 -> 2 -> 1E
    //Plan B: Go from 1N to 1E (passthrough), then -> 2 -> 3 -> 4 -> 1N (-> passthrough) 1E
      switch (curRoomNum) {
        case 1:
          room1Nto4();
          // rotate(180);
          // moveDist(10);
          // passThrough(LEFT);
          // roomTapeExit();
          // room1Eto2();
          break;
        case 4:
          room4to3();
          break;
        case 3:
          room3to2();
          break;
        case 2:
          room2to1E();
          break;
      }
    }
  }
  else if (startRoomNum == 2) {
  //Current: 2 -> 1E -> 1N (passthrough) -> 4 -> 3 -> 2
  //Old: 2 -> 3 -> 4 -> 1N (->passthrough) -> 1E -> 2
    switch (curRoomNum) {
      case 2:
        room2to1E();
    		pass_through = true;
    		pass_through_direction = LEFT;
        break;
      case 1:
        room1Nto4();
        break;
      case 4:
        room4to3();
        break;
      case 3:
        room3to2();
        break;
    }
  }
  else if (startRoomNum == 3) {
  
  // 3 -> 2 -> 1E -> (1N passthrough) -> 4 -> 3
    switch (curRoomNum) {
      case 3:
        room3to2();
        break;
      case 2:
        room2to1E();
    		pass_through = true;
    		pass_through_direction = LEFT;
        break;
      case 1:
        //passThrough(LEFT);
        room1Nto4();
        break;
      case 4:
        room4to3();
        break;
    }
  }
  else if (startRoomNum == 4) {
    //4 -> 1N (-> passthrough) 1E -> 2 -> 3 -> 4
    switch (curRoomNum) {
      case 4:
        room4to1();

        if(room4to1ArrivalEntrance == NORTH) {
          pass_through = true;
          pass_through_direction = LEFT;
        }
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

void locateRoom1VarDoorFromRoom1N() {   // was north for < 180, else south
//Determine the location of room 1's variable door from room 1N
  lbSonar.enable();
  delay(50);    //Allow any previous echo to clear
  int leftBackDist = lbSonar.read();
  if (room1_wall_location == UNKNOWN) {
    if (leftBackDist < 110) {
      Serial.print("wall is at NORTH. leftBackDist is ");
      Serial.println(leftBackDist);
      room1_wall_location = NORTH;
    }
    else {
      Serial.print("wall is at SOUTH. leftBackDist is ");
      Serial.println(leftBackDist);
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
  int room1entrance;

  if(curRoomNum == 1) {
    delay(50);
    rfSonar.enable();
    sonarDist[REARSONAR] = rfSonar.read();

    if(sonarDist[REARSONAR] < 90) {
      room1entrance = NORTH;
    } else {
      room1entrance = EAST;
    }


    if( room1entrance == EAST ) {
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

    } else if( room1entrance == NORTH ) {
      switch(startRoomNum) {
        case 2:
          room1Nto2();
          break;
        case 3:
          room1Nto3();
          break;
        case 4:
          room1Nto4();
          break;
      }
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
      case 1:   room3to1E();
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
        room4to1();           // <-- This wasn't here before, is it okay?
        //room4to1E();
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

