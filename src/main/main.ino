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


const byte melodyStrings = 3;
const byte melodyFrets = 10;
const byte accompanimentStrings = 3;
const byte accompanimentFrets = 13;

// melodyServoMapping contains the servo id for a fret on a string
int melodyServoMapping[melodyStrings][melodyFrets] = {};  // TODO: think about pin mapping for all servos

// accompanimentServoMapping contains the servo positions in
// microseconds per fret
int accompanimentServoMapping[accompanimentStrings][accompanimentFrets] = {}; // TODO: find out positions

// stepper control
const byte maxSteppers = 6;
const int stepperMaxSpeed = 5000;
const int stepperSpeed = 5000;
const int stepperAcceleration = 4000;
const byte stepperLightThreshold = 250;
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
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[0][0], stepperPinMapping[0][1]), // TODO: switch wires
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
  setStepperStartPosition(0);  // TODO: add this for all stepper motors
  
  // servo init

  Serial.begin(9600);
  Serial.println("Setup done.");
}

void loop() {
  // put your main code here, to run repeatedly:
  // TODO wait for serial cmd to play melody and accompaniment
}

/**
 * TODO: write comment
 */
void fretMelody(byte s, byte f){
  // TODO implement this
}

/**
 * TODO: write comment
 */
void fretAccompaniment(byte s, byte f){
  // TODO implement this
}

/**
 * TODO: write comment
 */
void pluck(byte s){
  // TODO implement this
}

/**
 * TODO: write comment
 */
void damp(byte s){
  // TODO implement this
}

/**
 * TODO: write comment
 */
void setupLightSensors(){
  for(byte i = 0; i < maxSteppers; i++){
    pinMode(lightSensorPins[i], INPUT);
  }
}

/**
 * Sets start position for sepper motor. Stepper i runs as long
 * as the according light sensor returns a value below the threshold.
 * Afterwars stepper is stopped an position set to 0.
 */
void setStepperStartPosition(byte i){
  steppers[i].setSpeed(1000);
  while(analogRead(lightSensorPins[i]) > stepperLightThreshold) { // TODO test this
    steppers[i].runSpeed();
  }
  steppers[i].stop();
  steppers[i].setCurrentPosition(0);
  steppers[i].setSpeed(stepperSpeed);
}

/**
 * Sets maximum speed, current speed and acceleration values
 * for all stepper motors
 */
void setupSteppers(){
  for(byte i = 0; i < maxSteppers; i++){
    steppers[i].setMaxSpeed(stepperMaxSpeed);
    steppers[i].setAcceleration(stepperAcceleration);
    steppers[i].setSpeed(stepperSpeed);
  }
}

/**
 * returns current value of the light sensor i
 */
int getStepperLightVal(byte i){
  return analogRead(lightSensorPins[i]);
}

/**
 * TODO: write comment
 */
void setupTiming(int timing){
  // TODO implement this
}

