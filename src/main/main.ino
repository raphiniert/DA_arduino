/* https://github.com/raphiniert/DA_arduino
   2018.06.22
   main.ino
   Authors: Jakob Blattner, Raphael Kamper
   This is the arduino code running on an Arduino Mega Rev 3
   board to control a guitar robot. Including multiple servo
   motors controlled via 2 Pololu Maestro Mini 18 servo
   controllers, stepper motors controlled via a DVR8825
   stepper controller and light sensors as a feedback system
   for the stepper motor.
*/

#include <PololuMaestro.h>
#include <AccelStepper.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial_1 SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial_2 SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
//SoftwareSerial maestroSerial_1(19, 18);
//SoftwareSerial maestroSerial_2(17, 16);
#endif

// servo control
MiniMaestro maestro_1(Serial1);
MiniMaestro maestro_2(Serial2);

// show DEBUG, WARNING or ERROR messages
boolean DEBUG = false;
boolean WARNINGS = true;
boolean ERRORS = true;

// Music
int bpm = 100;

// Timing
long noteDuration = 60000 / bpm;
const int servoMoveDownTime = 80;  // milliseconds
const int servoDampingTime = 80;  // milliseconds
long t1 = 0;

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
  {11, 12, 13, 14, 15, 16, 17, 0, 1, 2},
  {3, 4, 5, 6, 7, 8, 9, 10, 11, 12}
};  // TODO: think about pin mapping for all servos

const int melodyFretDownVal[melodyStrings][melodyFrets] = {
  {6000, 7000, 5800, 7000, 5800, 7000, 5700, 7000, 7000, 5700},
  {6000, 7400, 5800, 7600, 6100, 7500, 5800, 7600, 6400, 6000},
  {6000, 7700, 5500, 7500, 5800, 7300, 5800, 7300, 7400, 5400}
};

const int melodyFretUpVal[melodyStrings][melodyFrets] = {
  {6550, 6200, 6500, 6200, 6200, 6200, 6500, 6050, 6550, 6300},
  {6550, 6900, 6500, 7000, 6700, 6900, 6500, 7000, 5800, 6600},
  {6800, 7200, 6100, 6800, 6300, 6600, 6500, 6700, 6700, 6000}
};

// accompanimentServoMapping contains the servo positions in
// microseconds per fret
int accompanimentServoMapping[accompanimentStrings][accompanimentFrets] = {}; // TODO: find out positions

// damping servos
const byte maxDampingServos = 6;
const byte dampingServoSpeed = 0; // speed limit in units of (1/4 microseconds)/(10 milliseconds), 0 = unlimitied
const byte dampingServoAcceleration = 128; // 0 ... infinity (max) to 255;

const int dampingServoMapping[maxDampingServos] = {10, 13, 14, -1, -1, -1};
const int dampingServoValues[maxDampingServos][2] = {{7000, 6500}, {8000, 7500}, {7000, 6500}, {6000, 6000}, {6000, 6000}, {6000, 6000}};  //{{damp, free}}

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

AccelStepper steppers[maxSteppers] = { // TODO: test this
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[0][0], stepperPinMapping[0][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[1][0], stepperPinMapping[1][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[2][0], stepperPinMapping[2][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[3][0], stepperPinMapping[3][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[4][0], stepperPinMapping[4][1]),
  AccelStepper(AccelStepper::DRIVER, stepperPinMapping[5][0], stepperPinMapping[5][1])
};

//walk this way
const byte song_notes = 18;
const byte song[song_notes][3] = {  //string, fret, duration
  {2, 6, 1},
  {2, 7, 1},
  {2, 8, 1},
  {1, 5, 1},
  {2, 6, 1},
  {2, 7, 1},
  {2, 8, 1},
  {1, 5, 1},
  {2, 0, 2},
  {2, 6, 1},
  {2, 7, 1},
  {2, 8, 1},
  {1, 5, 1},
  {2, 6, 1},
  {2, 7, 1},
  {2, 8, 1},
  {1, 5, 1},
  {2, 3, 4}
};

//Serial communication
String receivedMsg = "";
int serialReceivedString = 0;
int serialReceivedFret = 0;
int serialReceivedDuration = 0;
int serialReceivedDamping = 0;
int serialReceivedBpm = 100;
int serialReceivedFrets[3] = {-1, -1, -1};
int serialReceivedDurations[3] = {1, 1, 1};
int serialReceivedDampings[3] = {0, 0, 0};

boolean activeStrings[3] = {false, false, false};
int durations[3] = {0, 0, 0};
int oldFrets[3] = {0, 0, 0};
int damping[3] = {0, 0, 0};
int oldDamping[3] = {0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600); //maestroSerial_1.begin(9600);
  Serial2.begin(9600); //maestroSerial_2.begin(9600);

  // stepper init and positioning
  setupLightSensors();
  setupSteppers();
  setStepperStartPosition(0);  // TODO: add this for all stepper motors
  setStepperStartPosition(1);
  setStepperStartPosition(2);
  
  // servo init
  setupFrettingServos();
  setupDampingServos();

  Serial.begin(9600);
  if (DEBUG) {
    Serial.println("Setup done.");
    Serial.print("bpm: ");
    Serial.println(bpm);
  }
}

//int msgInd = 0; // only for DEBUGGING

void loop(){
  // Arduino receives one message per beat
  if (Serial.available() > 0){
    // waiting to receive something.
    receivedMsg = Serial.readStringUntil('\n');
    
    /* DEBUG
    if (msgInd % 2 == 0){
      receivedMsg = "10,100,3,3,3,1,0\n";
    } else {
      receivedMsg = "10,100,5,6,7,1,0\n";
    }
    msgInd++;*/
    int msgIndStart = 0;
    int msgIndEnd = 0;
    msgIndStart = receivedMsg.indexOf(",") + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedBpm = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedFrets[0] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedDurations[0] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedDampings[0] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedFrets[1] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedDurations[1] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedDampings[1] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedFrets[2] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    msgIndEnd = receivedMsg.indexOf(",", msgIndStart);
    serialReceivedDurations[2] = receivedMsg.substring(msgIndStart, msgIndEnd).toInt();

    msgIndStart = msgIndEnd + 1;
    serialReceivedDampings[2] = receivedMsg.substring(msgIndStart, msgIndStart + 1).toInt();

    noteDuration = 60000 / serialReceivedBpm;

    if (DEBUG) {
      Serial.print("serialReceivedBpm: ");
      Serial.print(serialReceivedBpm);
      Serial.print(" serialReceivedFrets[0]: ");
      Serial.print(serialReceivedFrets[0]);
      Serial.print(" serialReceivedDurations[0]: ");
      Serial.print(serialReceivedDurations[0]);
      Serial.print(" serialReceivedDampings[0]: ");
      Serial.println(serialReceivedDampings[0]);
      Serial.print("serialReceivedFrets[1]: ");
      Serial.print(serialReceivedFrets[1]);
      Serial.print(" serialReceivedDurations[1]: ");
      Serial.print(serialReceivedDurations[1]);
      Serial.print(" serialReceivedDampings[1]: ");
      Serial.println(serialReceivedDampings[1]);
      Serial.print(" serialReceivedFrets[2]: ");
      Serial.print(serialReceivedFrets[2]);
      Serial.print(" serialReceivedDurations[2]: ");
      Serial.print(serialReceivedDurations[2]);
      Serial.print(" serialReceivedDampings[2]: ");
      Serial.println(serialReceivedDampings[2]);

      //calculated values
      Serial.print("strigns[3] = {");
      Serial.print(activeStrings[0]);
      Serial.print(", ");
      Serial.print(activeStrings[1]);
      Serial.print(", ");
      Serial.print(activeStrings[2]);
      Serial.println("}");
    }
    
    releaseFretSimultan(oldFrets, durations);
    releaseDampingSimultan(serialReceivedDampings);

    for(int i = 0; i < 3; i++) {
      if (serialReceivedFrets[i] >= 0) {
        activeStrings[i] = true;
        durations[i] = serialReceivedDurations[i];
      } else {
        activeStrings[i] = false;
      }
    }
    
    t1 = millis();
    fretSimultan(serialReceivedFrets);
    while (millis() - t1 < servoMoveDownTime) {
      ;
    }
    t1 = millis();
    dampSimultan(serialReceivedDampings);
    while (millis() - t1 < servoDampingTime) {
      ;
    }
    pluckSimultan(activeStrings);

    for(int i = 0; i < 3; i++) {
      durations[i] -= durations[i] > 0 ? 1 : 0;
    }
    
    while(Serial.availableForWrite() <= 0){
      ; // wait
    }
    Serial.println(receivedMsg); // indicates, that Arduino is done with the current beat
  }
}

/**
   TODO: write comment
*/
void fretMelody(byte s, byte f) {
  if (DEBUG) {
    Serial.print("in fretMelody(");
    Serial.print(s);
    Serial.print(", ");
    Serial.print(f);
    Serial.println(")");
  }
  if (s < 1) { // use maestroSerial_1 for strings 0 and 1
    maestro_1.setTarget(melodyServoMapping[s][f], melodyFretDownVal[s][f]);
  } else if (s == 1 &&  f < 7){
    maestro_1.setTarget(melodyServoMapping[s][f], melodyFretDownVal[s][f]);
  } else if (s == 1 &&  f >= 7){
    maestro_2.setTarget(melodyServoMapping[s][f], melodyFretDownVal[s][f]);
  } else if(s == 2) {
    maestro_2.setTarget(melodyServoMapping[s][f], melodyFretDownVal[s][f]);
  } else {
    if (ERRORS) {
      Serial.println("Not implemented yet! In fretMelody().");
    }
  }
}

/**
   TODO: write comment
*/
void releaseFretMelody(byte s, byte f) {
  if (DEBUG) {
    Serial.print("Release fret melody with s: ");
    Serial.print(s);
    Serial.print(" and f: ");
    Serial.println(f);
  }
  if (s < 1) { // use maestroSerial_1 for strings 0 and 1
    maestro_1.setTarget(melodyServoMapping[s][f], melodyFretUpVal[s][f]);
  } else if (s == 1 &&  f < 7){
    maestro_1.setTarget(melodyServoMapping[s][f], melodyFretUpVal[s][f]);
  } else if (s == 1 &&  f >= 7){
    maestro_2.setTarget(melodyServoMapping[s][f], melodyFretUpVal[s][f]);
  } else if(s == 2) {
    maestro_2.setTarget(melodyServoMapping[s][f], melodyFretUpVal[s][f]);
  } else {
    if (ERRORS) {
      Serial.println("Not implemented yet! In releaseFretMelody().");
    }
  }
}

/**
 * TODO: write comment
 */
void releaseFretSimultan(int frets[], int durations[]) {
  if (DEBUG) {
      Serial.println("in releaseFretSimultan()");
  }
  for(int s = 0; s < 3; s++){
    if (DEBUG) {
      Serial.print("frets[");
      Serial.print(s);
      Serial.print("] = ");
      Serial.println(frets[s]);
    }
    if (frets[s] > 0 && durations[s] == 0) {
      releaseFretMelody(s, frets[s]);
    }
  }
}

void dampSimultan(int dampings[]){
  if (DEBUG) {
      Serial.println("in dampSimultan()");
  }
  for(int s = 0; s < 3; s++){
    if(dampings[s]){
      damp(s);
    }
  }
}

/**
 * TODO: write comment
 */
void releaseDampingSimultan(int dampStrings[]){
  if (DEBUG) {
      Serial.println("in releaseDampingSimultan()");
  }
  for(int s = 0; s < 3; s++){
    if(dampStrings[s] == 0){
      releaseDamping(s);
    }
  }
}

/**
 * TODO: write comment
 */
void fretSimultan(int frets[]) {
  if (DEBUG) {
      Serial.println("in fretSimultan()");
  }
  for(int s = 0; s < 3; s++){
    if (DEBUG) {
      Serial.print("frets[");
      Serial.print(s);
      Serial.print("] = ");
      Serial.println(frets[s]);
    }
    if (frets[s] > 0) {
      fretMelody(s, frets[s]);
      oldFrets[s] = frets[s];
    }
  }
}

/**
 * checks if all strings should be stopped for plucking
 */
boolean stopAllStrings(boolean stopStrings[]) {
  if (DEBUG) {
      Serial.println("in stopAllStrings()");
      Serial.print("stopStrings[3] = {");
      Serial.print(stopStrings[0]);
      Serial.print(", ");
      Serial.print(stopStrings[1]);
      Serial.print(", ");
      Serial.print(stopStrings[2]);
      Serial.println("};");
  }
  for(int i = 0; i < 3; i++){
    if(!stopStrings[i]){
      return false;
    }
  }
  return true;
}

/**
 * TODO: assuming there are 3 strings for now, but needs to be generalized
 */
void pluckSimultan(boolean strings[]) {
  if (DEBUG) {
      Serial.println("in pluckSimultan()");
  }
  steppers[0].setSpeed(stepperSpeed);
  steppers[1].setSpeed(stepperSpeed);
  steppers[2].setSpeed(stepperSpeed);
  

  boolean stopStrings[3] = {false, false, false};
  for(int i = 0; i < 3; i++){
    stopStrings[i] = !strings[i]; //if string should be played set stop to false
  }
  if (DEBUG) {
      Serial.print("stopStrings[3] = {");
      Serial.print(stopStrings[0]);
      Serial.print(", ");
      Serial.print(stopStrings[1]);
      Serial.print(", ");
      Serial.print(stopStrings[2]);
      Serial.println("};");
  }
  long t1 = millis();
  while (!stopAllStrings(stopStrings)) {
    /*if (DEBUG) {
      Serial.println("something must be true...");
      Serial.print("stopAllStrings() = ");
      Serial.print(stopAllStrings(stopStrings));
    }*/
    if(!stopStrings[0]){
      if(analogRead(lightSensorPins[0]) < stepperLightThreshold || (millis() - t1) < 20) {
        steppers[0].runSpeed();
          if (DEBUG) {
            Serial.println("String 0 run");
          }
      } else {
        stopStrings[0] = true;
        steppers[0].stop();
      }
    }
    if(!stopStrings[1]){
      if(analogRead(lightSensorPins[1]) < stepperLightThreshold || (millis() - t1) < 20) {
        steppers[1].runSpeed();
        if (DEBUG) {
            Serial.println("String 1 run");
          }
      } else {
        stopStrings[1] = true;
        steppers[1].stop();
      }
    }
    if(!stopStrings[2]){
      if(analogRead(lightSensorPins[2]) < stepperLightThreshold || (millis() - t1) < 20) {
        steppers[2].runSpeed();
        if (DEBUG) {
            Serial.println("String 2 run");
          }
      } else {
        stopStrings[2] = true;
        steppers[2].stop();
      }
    }
  }

  steppers[0].stop();
  steppers[1].stop();
  steppers[2].stop();
  
  steppers[0].setCurrentPosition(0);
  steppers[1].setCurrentPosition(0);
  steppers[2].setCurrentPosition(0);
}

/**
   TODO: write comment
*/
void damp(byte s) {
  if (s < 1) { // use maestroSerial_1 for string 0 
    maestro_1.setTarget(dampingServoMapping[s], dampingServoValues[s][0]);
  } else if (s < 3) { // use maestroSerial_1 for strings 1 and 2 
    maestro_2.setTarget(dampingServoMapping[s], dampingServoValues[s][0]);
  } else {
    if (ERRORS) {
      Serial.println("Not implemented yet");
    }
  }
}

/**
   TODO: write comment
*/
void releaseDamping(byte s) {
  if (s < 1) { // use maestroSerial_1 for string 0 
    maestro_1.setTarget(dampingServoMapping[s], dampingServoValues[s][1]);
  } else if (s < 3) { // use maestroSerial_1 for strings 1 and 2 
    maestro_2.setTarget(dampingServoMapping[s], dampingServoValues[s][1]);
  } else {
    if (ERRORS) {
      Serial.println("Not implemented yet");
    }
  }
}

/**
   TODO: write comment
*/
void setupLightSensors() {
  for (byte i = 0; i < maxSteppers; i++) {
    pinMode(lightSensorPins[i], INPUT);
  }
}

/**
   Sets start position for sepper motor. Stepper i runs as long
   as the according light sensor returns a value below the threshold.
   Afterwars stepper is stopped an position set to 0.
*/
void setStepperStartPosition(byte i) {
  if (DEBUG) {
    Serial.print("Stepper: ");
    Serial.print(i);
    Serial.println(" seeks start Position.");
  }
  steppers[i].setSpeed(stepperSpeed);
  while (analogRead(lightSensorPins[i]) < stepperLightThreshold) { // TODO test this
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
   Sets maximum speed, current speed and acceleration values
   for all stepper motors
*/
void setupSteppers() {
  for (byte i = 0; i < maxSteppers; i++) {
    steppers[i].setMaxSpeed(stepperMaxSpeed);
    steppers[i].setAcceleration(stepperAcceleration);
    steppers[i].setSpeed(stepperSpeed);
  }
}

/**
   returns current value of the light sensor i
*/
int getStepperLightVal(byte i) {
  return analogRead(lightSensorPins[i]);
}

/**
   TODO: write comment
*/
void setTiming(int newBpm) {
  // TODO implement this
  bpm = newBpm;
}

/**
 * TODO: write comment
 */
void setupFrettingServos() {
  for (int s = 0; s < melodyStrings; s++) {
    for (int f = 0; f < melodyFrets; f++) {
      if (s < 1) { // use maestroSerial_1 for strings 0 and 1
        maestro_1.setSpeed(melodyServoMapping[s][f], melodyServoSpeed);
        maestro_1.setAcceleration(melodyServoMapping[s][f], melodyServoAcceleration);
      } else if(s == 1 && f < 7) {
        maestro_1.setSpeed(melodyServoMapping[s][f], melodyServoSpeed);
        maestro_1.setAcceleration(melodyServoMapping[s][f], melodyServoAcceleration);
      } else if(s == 1 && f >= 7) {
        maestro_2.setSpeed(melodyServoMapping[s][f], melodyServoSpeed);
        maestro_2.setAcceleration(melodyServoMapping[s][f], melodyServoAcceleration);
      } else if(s == 2) {
        maestro_2.setSpeed(melodyServoMapping[s][f], melodyServoSpeed);
        maestro_2.setAcceleration(melodyServoMapping[s][f], melodyServoAcceleration);
      } else {
        if (ERRORS) {
          Serial.println("Not implemented yet! In setupFrettingServos()");
          return;
        }
      }
      releaseFretMelody(s, f);
      delay(500);
    }
  }
}

/**
 * TODO: write comment
 */
void setupDampingServos() {
  for (int s = 0; s < maxDampingServos; s++) {
    if (s < 1) { // use maestroSerial_1 for strings 0 and 1
      maestro_1.setSpeed(dampingServoMapping[s], dampingServoSpeed);
      maestro_1.setAcceleration(dampingServoMapping[s], dampingServoAcceleration);
      releaseDamping(s);
    } else if(s < 3){
      maestro_2.setSpeed(dampingServoMapping[s], dampingServoSpeed);
      maestro_2.setAcceleration(dampingServoMapping[s], dampingServoAcceleration);
      releaseDamping(s);
    } else {
      if (ERRORS) {
        Serial.println("Not implemented yet");
      }
    }
  }
}
