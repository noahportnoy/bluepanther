/*Notes
 //Checklist:
 -Ensure all wires and cabled are plugged in and secured
 -Are batteries fully charged
 -Has CO2 been refilled
 
 //Battery
 
 
 //Encoders
 -Tested. Working well.
 
 //Flame-detection
 -Create a new flameExtinguishFlood2() method, which uses the cycleServo method to continuously sweep over the candle's flame  zone with CO2.
 
 //Line-up
 
 //Motors
 -DualVNH5019MotorShield Library uses pins 9 and 10 exclusively for timer 1 pwm control. This cannot be changed because of how the library was designed.
 -75:1 motors max speed = 1 meter/sec
 -Perhaps rename moveDist() to moveCm()
 -It does not make sense to have a moveVelocity() method which waits for the wheels to converge on the velocity before terminating.
   -Once it terminates the PD-velocity-control-correction stops.
   -Instead, make a setVelocity methods which set mode to velocity and the global wheel_set_velocity variables, then call the controlBase method after.

 
 //Navigation
 -Obsolete. Have two basic methods: leftWallFollowUntilLine(), and rightWallFollowUntilLine()
 -Obsolete. Possibly have dog-detecting versions of the ones above that can tally the number of times a head-wall has been hit to locate the dog, esp. from room 4
 -Actually, have no xwallfollowUntilLine methods. have room x to room y methods for simple rooms, and room1North to room4South, for example, for more complicated rooms.
 -These room to room methods should incorporate the dog detection.
 -Dog has three positions, numbered 1 to 3, going clockwise from the top.
 -Room 3 to 4, with dog detection
 -Go to center spot, go mid-way along room 4 west-wall, roate 180, then follow room 4's wall, now on the left.
 -No head-walls should be hit, so if any head-obstacle is detected before entering room 4, it is the dog.
 -In this case, save the location of the dog, rotate 180 degrees, then follow room 4 on the right, until reaching the entrance.
 -Safely reach the entrance of room 4, knowing you will not hit the dog.
 -Make odometers, e.g. odometer[0], etc that hold the distance travelled by the robot (average of two wheels)
 -Can be reset e.g. by odometer[0] = 0.0
 -Use this odometry to aid dog-detection during room-to-room navigation
 
 //PD_Controllers
 -Printing out wheel-motion data takes significant computation time, which impacts the PD controller.
 -Removing the print statements changes the way the controller behaves; i.e. jitter without print statements  when it worked fine with them.
 -Changing wheel radius changes the appropriate gains
 -Perhaps reduce controller-code to for-loops, rather than identical individual wheel operations.
 -Tune velocity controller gains for quicker response
 
 //Start_Button
 -***MUST MOUNT HIGHER!!!!!!!!!!
 -Must be within 2cm of top servo, or other mechanical part
 -Perhaps standoffs and mounted on a plank of wood?
 -Possibly even mount mic ridiculously high, too, for consistency
 
 //Servos
 -Incorporate servo offset into the posProcess() method
 
 //Sonars
 -Coded sonars on my own, tested successfully. If working with sonars, be familiar with default library settings, e.g. sonars disabled by default.
 -Sonars can be read by calling updateSonars(). Only enabled sonars are read. Distance values are stored into the array named sonarDist[]
 -Sonars are numbered 0 to 5, starting with the front sonar moving clockwise.
 -Processed readings of 0 to 370cm
 -Side note- don't get confused between left and right sonars when enabling/disabling them. This is easier said than done.
 */

//Versa valve

#define OFF 0
#define ON 1

#include <Button.h>
#include <DualVNH5019MotorShield.h>
//#include <FreqCounter.h>
#include <Flame.h>
#include <FlameSensor.h>
#include <LineSensor.h>
#include <PololuWheelEncoders.h>
#include <Servo.h>
#include <Sonar.h>

//---------Button--------
int buttonPin = 30;
Button startButton(buttonPin);

//------Distance IRs-----
//Two sensors, mounted at left 45 and right 45 degrees from normal
//No class ready to construct IR-proximity sensor objects, so read the sensors directly from the pins
int frontDistIRPin = A3;
int leftDistIRPin = A9;
int rightDistIRPin = A10;

#define FRONTIR A3
#define LEFTIR  A9
#define RIGHTIR A10

//-----Direction-Constants-and-Arena-Unknowns-----
//Dog locations, Room 1 door location, Room 4 orientation
enum {
  //Direction constants
  LEFT = 0,
  RIGHT = 1,
  DOWN,
  UP,
  UNKNOWN,  //Dog, room 1, room 4
  //Dog-location
  ONE,
  TWO,
  THREE,
  ONE_TWO,
  ONE_THREE,
  TWO_THREE,
  //Room 1 variable door locations, start room exit locations
  NORTH,
  SOUTH,
  EAST,
  //Room 4 orientation
  OPENUP,  //Entrance is North
  OPENDOWN  //Entrance is South
};
int dog_location = UNKNOWN;
int room1_wall_location = UNKNOWN;  //***The location of the moveable wall, NOT the changing opening
int room4_orientation = UNKNOWN;

int room1_start_room_exit = UNKNOWN;  //NORTH or EAST. Used by arbitrary-start and maze-navigator. Stores doorway of room 1 the robot is exiting through if room 1 is the starting room.

boolean pass_through = false;  //Used by flameHunt and goToNextRoom to remember if the robot passed through a searched room to a different exit, so exit180 isn't needed.
int pass_through_direction = UNKNOWN;    //This allows the goToNextRoom to tell flameHunt which wall to follow to pass through the room.

int head_wall_count = 0;  //Incremented by primitive0 every time a wall ahead is encountered and the robot is rotated away by 90 deg. This should only be read, and never reset.

//------Candle-Flame------
Flame flame;    //Models the candle flame.

//--------Encoders--------
PololuWheelEncoders encoders;
float encoderCPR = 48*74.83;    //Encoder's counts per revolution = resolution * gearbox ratio

int m1aPin = 51;
int m1bPin = 50;
int m2aPin = 52;
int m2bPin = 53;

//--------Flame-sensing-------
//int flameSensorPin = A8;
//FlameSensor flameSensor(flameSensorPin);
#define NUM_FLAME_SENSORS 7
//const int numFlameSensors = 7;
int flameSensorPin[NUM_FLAME_SENSORS] = {A4, A5, A6, A7, A13, A14, A15};
//TODO CHI - check this line
FlameSensor flameSensor[NUM_FLAME_SENSORS] = FlameSensor(0);

//------------LEDs------------
int ledPin = 31;

//-------Line-sensing---------
int leftLineSensorPin = A12;
int rightLineSensorPin = A11;
LineSensor leftLineSensor(leftLineSensorPin);
LineSensor rightLineSensor(rightLineSensorPin);

LineSensor *lineSensor[2] = {&leftLineSensor, &rightLineSensor};

int leftWhite = 80;  //Prev: 500. Minimum value for line sensor that can be accapted as white.
int rightWhite = 80;  //Prev: 700

long dlyMinWhiteLineInterval = 20;  //The time interval over which a continuous white reading will be accepted as a white line, not a stray mark.

//--------Motors----------
DualVNH5019MotorShield motors;  // Define our motor controller

int dcMotorRelayPin = 48;

//-----PD-Controllers-----
//Odometers hold the average distance travelled by both wheels since their last reset. (Reset odometer by assigning it value zero)
//Used for odometry.
const int numOdometers = 2;
float odometer[numOdometers];  //Odometer 0 used for dog_detection

//States assigned to the base_control_mode to operate the PDControllers
enum {
  POSITION = 0,
  VELOCITY,
  ROTATION
};

//States returned by controllers
enum {
  NO_REFERENCE = 0,
  DONT_CARE = 1,
  TRANSIENT = 2,
  CONVERGED = 3
};

enum {
  FALSE = 0,
  TRUE = 1
};

int base_control_mode;  //Used to select the PD control mode. May be set to POSITION, VELOCYTY, or ROTATION

//--------Servos----------
Servo servoX;  //x-axis servo
Servo servoY;  //y-axis servo
int servoXPin = 45;
int servoYPin = 44;

int servoPowerPin[2] = {49, 34};  //Servo noise blacks out the sonar readings. Power the servos when needed using opto-isolators connected to these pins for x and y.
int servoPower[2] = {0, 0};  //This variable keeps track of which servos are turned on. Intended to only have one servo on at a time.

#define X 0
#define Y 1

//--------Sonars-------------
int lfSonarPin = 25;  // Arduino pin tied to trigger and echo pin on ping sensor.
int lbSonarPin = 24;  // Arduino pin tied to trigger and echo pin on ping sensor.
int rfSonarPin = 27;  // Arduino pin tied to trigger and echo pin on ping sensor.
int rbSonarPin = 22;  // Arduino pin tied to trigger and echo pin on ping sensor.
int frontSonarPin = 26;  // Arduino pin tied to trigger and echo pin on ping sensor.
int rearSonarPin = 23;  // Arduino pin tied to trigger and echo pin on ping sensor.

//Construct sonar objects
Sonar lfSonar(lfSonarPin); // NewPing setup of pins and maximum distance.
Sonar lbSonar(lbSonarPin); // NewPing setup of pins and maximum distance.
Sonar rfSonar(rfSonarPin); // NewPing setup of pins and maximum distance.
Sonar rbSonar(rbSonarPin); // NewPing setup of pins and maximum distance.
Sonar frontSonar(frontSonarPin); // NewPing setup of pins and maximum distance.
Sonar rearSonar(rearSonarPin); // NewPing setup of pins and maximum distance.

int numSonars = 6;
int curSonar = 0;  //Current sonar to be read
Sonar *sonar[6];  //Numbered clockwise from 0 starting with front sonar
int sonarDist[6];  //Array to hold distance values of sonars. Will be updated constantly as the program runs and publicly accessible.

//Sonar names corresponding to sonarDist array
enum {
  FRONTSONAR = 0,
  RFSONAR = 1,
  RBSONAR = 2,
  REARSONAR = 3,
  LBSONAR = 4,
  LFSONAR = 5
};

//------Versa-Valve------
int versaPin = 30;

void setup(){
  //Set Serial baud rate
  Serial.begin(9600);
  //Initialize stuff
  setupMotors();
  pinMode(dcMotorRelayPin, OUTPUT);
  pinMode(servoPowerPin[X], OUTPUT);
  pinMode(servoPowerPin[Y], OUTPUT);
  pinMode(ledPin, OUTPUT);
  setupEncoders();
  setupServos();  //Turns on servos to orient them, then then turns them off
  setupSonars();
  setupLineSensors();
  setupFlameSensors();
  initializeOdometers();
  flame.on();   //The candle is alight
  ledBlink();
//  sensorStart();
  //For test-coding
//  buttonStart();
  ledOn();
  buttonStart();
  ledOff();
}

long cmilTest = 0;
long pmilTest = 0;
long dlymilTest = 6000;

void loop(){
  
  //NOAH - room test
  //Serial.println(getDist(FRONTIR));
    lineUp();
    room1Nto2();
    delay(15000);


  // NOAH - working dog detection and stop while wall following
  // wallFollow(RIGHT);
  // int sensorValue = getDist(FRONTIR);
  // Serial.println(sensorValue);
  // delay(50);
  // if(sensorValue < 400) {
  //   stopMotors();
  //   delay(10000);
  // }



  // Chi old code
//  wallFollow(RIGHT);
//  flameApproach2();
//  delay(2000);
//  flameScanFull(X);
//  sensorFlamePoint(X);
//  flameScanFull(Y);
//  sensorFlamePoint(Y);
//  delay(2000);
//  flameExtinguishSP();
//  delay(2000);
//  setupServos();
//  ledOn();
//  buttonStart();
//  ledOff();
//  flameScanFull(X);
//  sensorFlamePoint(X);
//  flameExtinguishSP();
//  setupServos();
//  ledOn();
//  buttonStart();
//  ledOff();
//  flameSensorTest();
//  Serial.println(analogRead(A14));
//  delay(50);
//  wallFollow(LEFT);
//  delay(2000);
//  flameSPA2();
//  delay(2000);
/*
  ledOn();
  buttonStart();
  ledOff();
  delay(1000);
  while(1) {
    wallFollow(LEFT);
    if (startButton.isPushed() == true) {
      break;
    }
  }
  stopMotors();
  delay(2000);
  buttonStart();
  delay(2000);
  flameSPA2();
  delay(2000);
*/
/*
  flame.on();
  buttonStart();
  flameScanFull(X);
  sensorFlamePoint(X);
  flameScanFull(Y);
  sensorFlamePoint(Y);
  delay(2000);
  flameExtinguishSP();
  delay(2000);
  setupServos();
  delay(2000);
*/
//  fireFight();
//  fireFightDemo();
//  digitalWrite(34, HIGH);
//  delay(10000);
//  servoPowerOn();
//  servoOn(X);
//  centerServo(X);
//  rightLineSensor.test();
//  extinguisherTest();
//  if (startButton.isPushed() == true) {
//    openExtinguisher();
//  }
//  else {
//    closeExtinguisher();
//  }
/*
  flameScanFull(X);
  sensorFlamePoint(X);
  flameScanFull(Y);
  sensorFlamePoint(Y);
  flameExtinguishSP();  //Spray a burst of CO2 at the flame
  delay(5000);
*/
/*
  servoPowerOn();
  flameApproach2();
  flameScanFull(X);
  sensorFlamePoint(X);
  flameScanFull(Y);
  sensorFlamePoint(Y);
  flameExtinguishSP();  //Spray a burst of CO2 at the flame
  delay(10000);
*/
//  room1_wall_location = SOUTH;
//  lineUp();
//  room1Eto3();
//  delay(5000);
}


void setupMotors(){
  motors.init();
}

void setupEncoders(){
  encoders.init(m1aPin, m1bPin, m2aPin, m2bPin);
}

void setupServos(){
  //Assigns servo objects to their I/O pins, and moves both servos to their starting positions
  servoPowerOn();
  servoX.attach(servoXPin);
  servoY.attach(servoYPin, 544, 2600);  //Push the vertical servo's range
  servoY.write(170);  //Stop the servo from swinging down before centering itself
  centerServo(X);
  centerServo(Y);
  servoPowerOff();
}

void setupSonars(){
  //Assign sonars to array
  sonar[FRONTSONAR] = &frontSonar;
  sonar[RFSONAR] = &rfSonar;
  sonar[RBSONAR] = &rbSonar;
  sonar[REARSONAR] = &rearSonar;
  sonar[LBSONAR] = &lbSonar;
  sonar[LFSONAR] = &lfSonar;

  //Measure and initialize distance readings.
  for (int i = 0; i < numSonars; i++) {
    sonar[i]->enable();
    sonarDist[i] = sonar[i]->read();
    delay(50);
  }
  disableSonars();
}

void setupLineSensors() {
  leftLineSensor.setWhiteVal(leftWhite);
  rightLineSensor.setWhiteVal(rightWhite);
  
  leftLineSensor.setVerifyInterval(dlyMinWhiteLineInterval);
  rightLineSensor.setVerifyInterval(dlyMinWhiteLineInterval);
}

void setupFlameSensors() {
  for (int i = 0; i < NUM_FLAME_SENSORS; i++) {
    flameSensor[i] = FlameSensor(flameSensorPin[i]);
  }
}

void initializeOdometers() {
  for (int i = 0; i < numOdometers; i++) {
    odometer[i] = 0.0;
  }
}

void ledBlink() {
//Blinks the led once
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
}

void ledBlink(int n) {
//Blinks the led n times
  for (int i = 0; i < n; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

void ledOn() {
  digitalWrite(ledPin, HIGH);
}

void ledOff() {
  digitalWrite(ledPin, LOW);
}

