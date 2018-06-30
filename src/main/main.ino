/* https://github.com/raphiniert/DA_arduino
 * 2018.06.22
 * main.ino
 * Authors: Jakob Baltter, Raphael Kamper
 * This is the arduino code running on an Arduino Mega Rev 3
 * board to control a guitar robot. Including multiple servo
 * motors controlled via 2 Pololu Maestro Mini 18 servo
 * controllers, stepper motors controlled via a DVR8825 
 * stepper controller and light sensors as a feedback system
 * for the stepper motor.
 */

#include <PololuMaestro.h>
#include <AccelStepper.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial_1 SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial_2 SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial_1(19, 18);
  SoftwareSerial maestroSerial_2(17, 16);
#endif

// servo control
MiniMaestro maestro_1(maestroSerial_1);
MiniMaestro maestro_2(maestroSerial_2);

// show DEBUG, WARNING or ERROR messages
boolean DEBUG = true;
boolean WARNINGS = true;
boolean ERRORS = true;

// Music
const int bpm = 180;

const byte melodyStrings = 3;
const byte melodyFrets = 10;
const byte accompanimentStrings = 3;
const byte accompanimentFrets = 13;

// melody servos
const byte melodyServoSpeed = 0; // speed limit in units of (1/4 microseconds)/(10 milliseconds), 0 = unlimitied
const byte melodyServoAcceleration = 0; // 0 ... infinity (max) to 255; 
// melodyServoMapping contains the servo id for a fret on a string
const int melodyServoMapping[melodyStrings][melodyFrets] = {
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};  // TODO: think about pin mapping for all servos
  
const int melodyFretDownVal[melodyStrings][melodyFrets] = {
  {6000, 7400, 5800, 7600, 6100, 7500, 5800, 7600, 6400, 6000},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  
const int melodyFretUpVal[melodyStrings][melodyFrets] = {
  {6550, 6900, 6500, 7000, 6700, 6900, 6500, 7200, 5800, 6600},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

// accompanimentServoMapping contains the servo positions in
// microseconds per fret
int accompanimentServoMapping[accompanimentStrings][accompanimentFrets] = {}; // TODO: find out positions

// damping servos
const byte maxDampingServos = 6;
const byte dampingServoSpeed = 0; // speed limit in units of (1/4 microseconds)/(10 milliseconds), 0 = unlimitied
const byte dampingServoAcceleration = 128; // 0 ... infinity (max) to 255; 

const int dampingServoMapping[maxDampingServos] = {10, -1, -1, -1, -1, -1};
const int dampingServoValues[maxDampingServos][2] = {{7100, 6900}, {6000, 6000}, {6000, 6000}, {6000, 6000}, {6000, 6000}, {6000, 6000}};  //{{damp, free}}

// stepper control
const byte maxSteppers = 6;
const int stepperMaxSpeed = 1000;
const int stepperSpeed = 1000;
const int stepperAcceleration = 4000;
const byte stepsPerPluck = 40; // n = # pleks (5), s = number of steps per revolution (200) s/n = number of steps: 200/5 = 40
const byte stepperLightThreshold = 100;
const byte lightSensorPins[maxSteppers] = {A0, A1, A2, A3, A4, A5};

// stepPin, dirPin per stepper motor
const byte stepperPinMapping[maxSteppers][2] = {{30, 31}, {32, 33}, {34, 35}, {36, 37}, {38, 39}, {40, 41}};

// melody steppers
//AccelStepper stepper0(AccelStepper::DRIVER, stepperPinMapping[0][0], stepperPinMapping[0][1]); // TODO switch on arduino
//AccelStepper stepper1(AccelStepper::DRIVER, stepperPinMapping[1][0], stepperPinMapping[1][1]);
//AccelStepper stepper2(AccelStepper::DRIVER, stepperPinMapping[2][0], stepperPinMapping[2][1]);
// accompaniment steppers
//AccelStepper stepper3(AccelStepper::DRIVER, stepperPinMapping[3][0], stepperPinMapping[3][1]);
//AccelStepper stepper4(AccelStepper::DRIVER, stepperPinMapping[4][0], stepperPinMapping[4][1]);
//AccelStepper stepper5(AccelStepper::DRIVER, stepperPinMapping[5][0], stepperPinMapping[5][1]);

AccelStepper steppers[maxSteppers] = { // TODO: test this
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[0][0], stepperPinMapping[0][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[1][0], stepperPinMapping[1][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[2][0], stepperPinMapping[2][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[3][0], stepperPinMapping[3][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[4][0], stepperPinMapping[4][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[5][0], stepperPinMapping[5][1])};

void setup() {
  // put your setup code here, to run once:
  maestroSerial_1.begin(9600);
  // maestroSerial_2.begin(9600);

  // stepper init and positioning
  setupLightSensors();
  setupSteppers();
  setStepperStartPosition(0);  // TODO: add this for all stepper motors
  
  // servo init
  setupFrettingServos();
  setupDampingServos();
  
  Serial.begin(9600);
  if(DEBUG) {
    Serial.println("Setup done.");
    Serial.print("bpm: ");
    Serial.print(bpm);
    Serial.println();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // TODO wait for serial cmd to play melody and accompaniment
  // Serial.println(getStepperLightVal(0));
  int s = 0;  // stirng
  int f = 0;  // fret
  for(int s = 0; s < 1; s++) {
    for(int f = 0; f < melodyFrets; f++){
      fretMelody(s, f);
      delay(80);
      pluck_2(0);
      delay(60000/bpm);
      damp(s);
      delay(80);
      releaseFretMelody(s, f);
      releaseDamping(s);
      
    }
  }
}

/**
 * TODO: write comment
 */
void fretMelody(byte s, byte f) {
  // TODO implement this
  if(s < 2) { // use maestroSerial_1 for strings 0 and 1
    maestro_1.setTarget(melodyServoMapping[s][f], melodyFretDownVal[s][f]);
  } else {
    if(ERRORS){
      Serial.println("Not implemented yet");
    }
  }
}

/**
 * TODO: write comment
 */
void releaseFretMelody(byte s, byte f) {
  if(s < 2) { // use maestroSerial_1 for strings 0 and 1
    maestro_1.setTarget(melodyServoMapping[s][f], melodyFretUpVal[s][f]);
  } else {
    if(ERRORS){
      Serial.println("Not implemented yet");
    }
  }
}

/**
 * TODO: write comment
 */
void fretAccompaniment(byte s, byte f) {
  // TODO implement this
}

/**
 * Sets new move to position for stepper s, runs to the according position,
 * checks if stepper is at right position and prints warning otherwise.
 * Resets position of stepper s to 0.
 */
void pluck(byte s) {
  steppers[s].moveTo(stepsPerPluck);
  steppers[s].runToPosition();
  if(getStepperLightVal(s) < stepperLightThreshold) {
    setStepperStartPosition(s);
    if(WARNINGS){
      Serial.print("Recalibration might be needed for string: ");
      Serial.println(s);
    }
  }
  steppers[s].setCurrentPosition(0);
}

/**
 * Plucks string s by running motor s as long as stepperLightThreshold isn't reached
 * or x ms didn't pass. This is neccessary, because otherwise motor wouldn't run, because
 * it should always stop above stepperLightThreshold. Resets current stepper position to 0.
 */
void pluck_2(byte s) {
  steppers[s].setSpeed(stepperSpeed);
  long t1 = millis();
  while(analogRead(lightSensorPins[s]) < stepperLightThreshold || (millis() - t1) < 20) {
    steppers[s].runSpeed();
  }
  steppers[s].stop();
  steppers[s].setCurrentPosition(0);
}

/**
 * TODO: write comment
 */
void damp(byte s) {
  if(s < 2) { // use maestroSerial_1 for strings 0 and 1
    maestro_1.setTarget(dampingServoMapping[s], dampingServoValues[s][0]);
  } else {
    if(ERRORS){
      Serial.println("Not implemented yet");
    }
  }
}

/**
 * TODO: write comment
 */
void releaseDamping(byte s) {
  if(s < 2) { // use maestroSerial_1 for strings 0 and 1
    maestro_1.setTarget(dampingServoMapping[s], dampingServoValues[s][1]);
  } else {
    if(ERRORS){
      Serial.println("Not implemented yet");
    }
  }
}

/**
 * TODO: write comment
 */
void setupLightSensors() {
  for(byte i = 0; i < maxSteppers; i++) {
    pinMode(lightSensorPins[i], INPUT);
  }
}

/**
 * Sets start position for sepper motor. Stepper i runs as long
 * as the according light sensor returns a value below the threshold.
 * Afterwars stepper is stopped an position set to 0.
 */
void setStepperStartPosition(byte i) {
  if (DEBUG) {
    Serial.print("Stepper: ");
    Serial.print(i);
    Serial.println(" seeks start Position.");
  }
  steppers[i].setSpeed(stepperSpeed);
  while(analogRead(lightSensorPins[i]) < stepperLightThreshold) { // TODO test this
    steppers[i].runSpeed();
    if (DEBUG) {
    Serial.print("Lightval for stepper: ");
    Serial.print(i);
    Serial.print(" is: ");
    Serial.println(getStepperLightVal(i));
    }
  }
  steppers[i].stop();
  steppers[i].setCurrentPosition(0);
  steppers[i].setSpeed(stepperSpeed);
}

/**
 * Sets maximum speed, current speed and acceleration values
 * for all stepper motors
 */
void setupSteppers() {
  for(byte i = 0; i < maxSteppers; i++) {
    steppers[i].setMaxSpeed(stepperMaxSpeed);
    steppers[i].setAcceleration(stepperAcceleration);
    steppers[i].setSpeed(stepperSpeed);
  }
}

/**
 * returns current value of the light sensor i
 */
int getStepperLightVal(byte i) {
  return analogRead(lightSensorPins[i]);
}

/**
 * TODO: write comment
 */
void setupTiming(int timing) {
  // TODO implement this
}

void setupFrettingServos() {
  for(int s = 0; s < melodyStrings; s++) {
    for(int f = 0; f < melodyFrets; f++) {
      if(s < 2) { // use maestroSerial_1 for strings 0 and 1
        maestro_1.setSpeed(melodyServoMapping[s][f], melodyServoSpeed);
        maestro_1.setAcceleration(melodyServoMapping[s][f], melodyServoAcceleration);
        releaseFretMelody(s, f);
      } else {
        if(ERRORS) {
          Serial.println("Not implemented yet");
        }
      }
    }
  }
}

void setupDampingServos() {
  for(int s = 0; s < maxDampingServos; s++) {
    if(s < 2) { // use maestroSerial_1 for strings 0 and 1
      maestro_1.setSpeed(dampingServoMapping[s], dampingServoSpeed);
      maestro_1.setAcceleration(dampingServoMapping[s], dampingServoAcceleration);
      releaseDamping(s);
    } else {
      if(ERRORS) {
        Serial.println("Not implemented yet");
      }
    }
  }
}

