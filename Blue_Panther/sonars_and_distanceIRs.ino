//IR methods
int distNOR = 10;

//Sonar methods
long cmilUpdateSonar = 0;
long pmilUpdateSonar = 0;
long dlymilUpdateSonar = 50;

void updateSonars(){
  //Non-blocking method
  //Read the curSonar if it's enabled, otherwise find the next enabled sonar and read it.
  //If no sonars are enabled then move on.
  cmilUpdateSonar = millis();
  if (dlymilUpdateSonar < (cmilUpdateSonar - pmilUpdateSonar)) {
    if (sonar[curSonar]->isEnabled() == true) {
      sonarDist[curSonar] = sonar[curSonar]->read();
      curSonar++;
      if ((numSonars - 1) < curSonar) {
        curSonar = 0;
      }
      pmilUpdateSonar = cmilUpdateSonar;
    }
    else {
      int startSonar = curSonar;
      curSonar++;
      if ((numSonars - 1) < curSonar) {
        curSonar = 0;
      }
      int nextSonar = curSonar + 1;
      if ((numSonars - 1) < nextSonar) {
        nextSonar = 0;
      }
      boolean valid = true;
      while (sonar[curSonar]->isEnabled() == false) {
        if (nextSonar == startSonar) {  //This condition is true if all sonars are disabed. If so, move on.
          valid = false;
          break;
        }
        curSonar++;
        if ((numSonars - 1) < curSonar) {
          curSonar = 0;
        }
        nextSonar++;
        if ((numSonars - 1) < nextSonar) {
          nextSonar = 0;
        }
      }
      if (valid == true) {
        sonarDist[curSonar] = sonar[curSonar]->read();
        curSonar++;
        if ((numSonars - 1) < curSonar) {
          curSonar = 0;
        }
        pmilUpdateSonar = cmilUpdateSonar;
      }
    }
  }
}

void updateSonarsSeq() {
//***Blocking method
//Reads enabled sonars in sequence and updates their values in sonarDist[]

  delay(50);  //Allow any previous echo to clear
  for (int i = 0; i < numSonars; i++) {
    if (sonar[i]->isEnabled() == true) {
      sonarDist[i] = sonar[i]->read();
      delay(50);
    }
  }
}

long cmilTestSonar = 0;
long pmilTestSonar = 0;
long dlymilTestSonar = 20;

void testSonars(){
  //Non-blocking function
  updateSonars();
  cmilTestSonar = millis();
  if (dlymilTestSonar < (cmilTestSonar - pmilTestSonar)) {
    for (int i = 0; i < (numSonars - 1); i++) {
      Serial.print(sonarDist[i]);
      Serial.print("   ");
    }
    Serial.println(sonarDist[numSonars - 1]);
    pmilTestSonar = cmilTestSonar;
  }
}

void enableSonars(){
  for (int i = 0; i < numSonars; i++) {
    sonar[i]->enable();
  }
}

void disableSonars(){
  for (int i = 0; i < numSonars; i++) {
    sonar[i]->disable();
  }
}


//IRs
int getDist(int irSensor) {
  int pinNum = 0;
  if (irSensor == LEFTIR) {
    pinNum = leftDistIRPin;
  }
  else if (irSensor == RIGHTIR) {
    pinNum = rightDistIRPin;
  }
  else if (irSensor == FRONTIR) {
    pinNum = frontDistIRPin;
  }
  
  int distance = avgSensor(pinNum, distNOR);
  distance =  850 - (254.0/1024.0) *2.0* distance * 2.54;
  return distance;
}

long cmilDistTest = 0;
long pmilDistTest = 0;
long dlymilDistTest = 50;

void distTest(int irSensor){
  cmilDistTest = millis();
  if (dlymilDistTest < (cmilDistTest - pmilDistTest)) {
    int pinNum = 0;
    if (irSensor == LEFTIR) {
      pinNum = leftDistIRPin;
    }
    else if (irSensor == RIGHTIR) {
      pinNum = rightDistIRPin;
    }
    else if (irSensor == FRONTIR) {
      pinNum = frontDistIRPin;
    }
    int dist = getDist(pinNum);
    Serial.println(dist);
    pmilDistTest = cmilDistTest;
  }
}

int avgSensor(int sensorPin, int NOR){
//Average sensor function
  int sensorTotal = 0;    //Initialize sensor total
  for (int i = 0; i < NOR; i++) {
    sensorTotal = sensorTotal + analogRead(sensorPin);    //Add the current sensor reading to the previous total
  }
  sensorTotal = sensorTotal / NOR;    //Return the average sensor value
  return sensorTotal;
}

