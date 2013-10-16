

boolean autoLineDetect() {
//Reads the two line sensors with each function call and returns true if a line has been detected.
  updateLineSensors();
  boolean leftTriggered = lineSensor[LEFT]->detectLine();
  boolean rightTriggered = lineSensor[RIGHT]->detectLine();
  if (leftTriggered || rightTriggered) {  //If a line has been detected
    return true;
  }
  else {
    return false;
  }
}

void lineUp() {
//Reverses the robot a little, then stops along a white line ahead of it.
//White returns higher value to sensors than black
  
  moveDist(-5);
  
  int leftVal = lineSensor[LEFT]->read();
  int rightVal = lineSensor[RIGHT]->read();
  int leftWhiteVal = lineSensor[LEFT]->getWhiteVal();
  int rightWhiteVal = lineSensor[RIGHT]->getWhiteVal();

  while ((leftVal < leftWhiteVal) || (rightVal < rightWhiteVal)) {  //While at least one sensor if not above the line
    //Read the line sensors
    leftVal = lineSensor[LEFT]->read();
    rightVal = lineSensor[RIGHT]->read();
/*
    //Testing
    Serial.print(leftVal);
    Serial.print("   ");
    Serial.println(rightVal);
*/
    if ((leftWhiteVal < leftVal) && (rightWhiteVal < rightVal)) {  //If both line sensors are above the line, then stop the robot and terminate the loop
      stopMotors();
      break;
    }
    else if (leftWhiteVal < leftVal) {  //If only the left sensor is above the line, rotate left
      setVelocities(-6, 6);
    }
    else if (rightWhiteVal < rightVal) {  //If only the right sensor is above the line, spin right
      setVelocities(6, -6);
    }
    else {  //If both sensors are not above the line, then move forward
      setVelocities(6);
    }
    controlBase();
  }
  stopMotors();
}

void lineUpEnter() {
  int roomEntryDist = 13;
  lineUp();
  moveDist(roomEntryDist);
}

void updateLineSensors() {
  for (int i = 0; i < 2; i++) {
    lineSensor[i]->update();
  }
}

