// LIBRARIES
// VVVVVVVVV

#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

// PIN DECLAIRATION
// VVVVVVVVVVVVVVVV

#define dirPinA 12
#define dirPinB 13

#define pwmPinA  3
#define pwmPinB 11

#define brkPinA  9
#define brkPinB  8

#define curPinA A0
#define curPinB A1

#define strtBtn 41

#define SwithFast 43
const int ledPin     = 45;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



// PARAMETERS
// VVVVVVVVVV

// PID parameters
int defSpd  = 220; // Default speed. The speed setting for straght lines.
int RturnSpd; // Turning speed. The speed setting for corners.
int LturnSpd;
int error;
int lastError = 0;

// Speed settings
int maxSpeed = 220;
int rotationSpeed = 150;
int stopSpeed = 50;

// Delays
int allignDel = 1;
int pauzeDel = 5000;
int turnDelay = 20;
int preTurnSpeed = 90;


// CUSTOM VARIABLES
// VVVVVVVVVVVVVVVV

typedef enum {MOTOR_R, MOTOR_L} SELECT_MOTOR;
typedef enum {FORWARD, REVERSE} DIRECTION;

bool stop = false;

int normArray[8];

// FUNCTIONS
// VVVVVVVVV


// Change the speed depending on the current switch position  (fast or slow)
void speedControl() {
  if(digitalRead(SwithFast) == HIGH){
    defSpd = 255;
    rotationSpeed = 180;
    stopSpeed = 15;
    turnDelay = 175;
    preTurnSpeed = 90;
  }
  else {
    defSpd = 255;
    rotationSpeed = 140;
    stopSpeed = 15;
    turnDelay = 100;
    preTurnSpeed = 100;
  }

}

// Function to simplify the motor control
void driveMotor(SELECT_MOTOR motor, DIRECTION dirSel, int power, bool brake = false){

  int dir = 0;
  int pwm = 0;
  int brk = 0;

  if(motor == MOTOR_R){
    dir = 12;
    pwm = 3;
    brk = 9;
  }
  else{
    dir = 13;
    pwm = 11;
    brk = 8;
  }


  if(dirSel == FORWARD){
    digitalWrite(dir, HIGH);
  }
  else{
    digitalWrite(dir, LOW);
  }

  analogWrite(pwm, power);

  digitalWrite(brk, brake);

}

// MOST IMPORTANT FUNCTION to make the turn
int makeTurn(){

  SELECT_MOTOR turnFor;
  SELECT_MOTOR turnRev;

  if((normArray[0] == 1  && normArray[7] == 0)){
    Serial.println("RIGHT");
    turnFor = MOTOR_R;
    turnRev = MOTOR_L;
  }
  else if((normArray[0] == 0  && normArray[7] == 1)){
    Serial.println("LEFT");
    turnFor = MOTOR_L;
    turnRev = MOTOR_R;
  }
  else{
    return(0);
  }

  Serial.println("turning");

  driveMotor(turnFor, FORWARD, preTurnSpeed);
  driveMotor(turnRev, FORWARD, preTurnSpeed);

  //DO NOT CHANGE!!!
  delay(turnDelay);

  // unsigned int stepNrStart = nrStepsA;

  // while(nrStepsA < stepNrStart + stepGoal){
  // }
  
  driveMotor(MOTOR_R, FORWARD, 0, true);
  driveMotor(MOTOR_L, FORWARD, 0, true);


  
  driveMotor(turnFor, FORWARD, rotationSpeed);
  driveMotor(turnRev, REVERSE, rotationSpeed);
  
  

  qtr.readLineBlack(sensorValues);
  
  int stopt = 0;

  while(stopt == 0){
    
    qtr.readLineBlack(sensorValues);
    if(sensorValues[3] > 800 || sensorValues[4] > 800 ){
      stopt = 1;
    }
  }

  

  return(0);
}

// Refresh sensor
void readSensor(unsigned int threshold){
  qtr.readLineBlack(sensorValues);
  
  for(int i = 0; i < SensorCount; i++){
    
    if(sensorValues[i] >= threshold){
      normArray[i] = 1;
    }
    else{
      normArray[i] = 0;
    }
    Serial.print(normArray[i]);
    Serial.print(" ");
  }

  Serial.println();
}

// PID control function
void PIDSteer(){
  //PID control

  uint16_t position = qtr.readLineBlack(sensorValues);

  error = position - 3500;

  RturnSpd = defSpd - (2 * error + 3 * (error - lastError));
  LturnSpd = defSpd + (2 * error + 3 * (error - lastError));

  lastError = error;
    
  if(RturnSpd > maxSpeed){
    RturnSpd = maxSpeed;
  }
  if(LturnSpd > maxSpeed){
    LturnSpd = maxSpeed;
  }

  driveMotor(MOTOR_R, FORWARD, RturnSpd);
  driveMotor(MOTOR_L, FORWARD, LturnSpd);

}

void blink(){
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void pauzeStop(){
  if(normArray[0] == 1 && normArray[7] == 1){
      
    driveMotor(MOTOR_R, FORWARD, 0, true);
    driveMotor(MOTOR_L, FORWARD, 0, true);

    delay(10);

    driveMotor(MOTOR_R, FORWARD, stopSpeed);
    driveMotor(MOTOR_L, FORWARD, stopSpeed);

    int starttime = millis();

    while(normArray[0] == 1 || normArray[7] == 1){
      readSensor(700);
    }
    int endTime = millis();

    delay(allignDel);

    driveMotor(MOTOR_R, FORWARD, 0, true);
    driveMotor(MOTOR_L, FORWARD, 0, true);

      
    // millis delay for the pause
    if(endTime - starttime < 500){
      stop = false;
      delay(pauzeDel);
    }
    else{
      stop = true;
    }
  }

  for(int i = 0; i < SensorCount; i++){
    Serial.print(normArray[i]);
    Serial.print("\t");
  }

}


// SETUP
// VVVVV
void setup(){
  
  Serial.begin(9600);

  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);

  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);

  pinMode(brkPinA, OUTPUT);
  pinMode(brkPinB, OUTPUT);

  pinMode(curPinA, INPUT);
  pinMode(curPinB, INPUT);

  pinMode(SwithFast, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(47);

  delay(500);

  pinMode(45, OUTPUT);
  
  digitalWrite(45, HIGH);

  for (uint16_t i = 0; i < 150; i++){
    qtr.calibrate();
    Serial.println(i);
  }

  
  digitalWrite(45, LOW);
  
}



// LOOP
// VVVV


void loop(){

  while(digitalRead(strtBtn) != HIGH) {}
  stop = false;

  

  //main while loop
  while(!stop){  

    PIDSteer();

    readSensor(750);

    makeTurn();

    pauzeStop();
  }
}
