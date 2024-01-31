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

#define strtBtn 4

#define encoderPinA 20  // Encoder channel A for motor A connected to digital pin 20
#define encoderPinB 21  // Encoder channel A for motor B connected to digital pin 21


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



// PARAMETERS
// VVVVVVVVVV

int defSpd  = 255; // Default speed. The speed setting for straght lines.
int RturnSpd; // Turning speed. The speed setting for corners.
int LturnSpd;
int error;
int lastError = 0;
int maxSpeed = 255;

int allignDel = 200;
int pauzeDel = 1000;



// CUSTOM VARIABLES
// VVVVVVVVVVVVVVVV

typedef enum {MOTOR_R, MOTOR_L} SELECT_MOTOR;
typedef enum {FORWARD, REVERSE} DIRECTION;

bool stop = false;

int normArray[8];

const float mmPerStep = (65*PI)/20;
const float mmToMiddle = 160;
unsigned int  stepGoal = (mmToMiddle/mmPerStep)-2;

volatile unsigned long nrStepsA = 0;
volatile unsigned long nrStepsB = 0;



// FUNCTIONS
// VVVVVVVVV

void updateEncoderA() {
  // Increment or decrement the encoder count for motor A based on the rising and falling edges of the encoder signal
  nrStepsA++;
}

void updateEncoderB() {
  // Increment or decrement the encoder count for motor B based on the rising and falling edges of the encoder signal
  nrStepsB++;
}



void driveMotor(SELECT_MOTOR motor, DIRECTION dirSel, int power, bool brake = false){

  int dir = 0;
  int pwm = 0;
  int brk = 0;

  if(motor == MOTOR_L){
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



int makeTurn(int array[]){

  SELECT_MOTOR turnFor;
  SELECT_MOTOR turnRev;

  if((array[0] == 1  && array[7] == 0)){
    Serial.println("RIGHT");
    turnFor = MOTOR_R;
    turnRev = MOTOR_L;
  }
  else if((array[0] == 0  && array[7] == 1)){
    Serial.println("LEFT");
    turnFor = MOTOR_L;
    turnRev = MOTOR_R;
  }
  else{
    return(0);
  }

  Serial.println("turning");

  delay(150);

  // unsigned int stepNrStart = nrStepsA;

  // while(nrStepsA < stepNrStart + stepGoal){
  // }
  
  driveMotor(MOTOR_R, FORWARD, 0, true);
  driveMotor(MOTOR_L, FORWARD, 0, true);


  
  driveMotor(turnFor, FORWARD, 200);
  driveMotor(turnRev, REVERSE, 200);
  
  

  uint16_t position = qtr.readLineBlack(sensorValues);
  
  int stop = 0;

  while(stop == 0){
    
    position = qtr.readLineBlack(sensorValues);
    if(sensorValues[4] > 900 || sensorValues[3] > 900 ){
      stop = 1;
    }
  }

  

  return(0);
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

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoderB, CHANGE);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);


  for (uint16_t i = 0; i < 150; i++){
    qtr.calibrate();
    Serial.println(i);
  }


}



// LOOP
// VVVV
void loop(){


  

  while(digitalRead(strtBtn) != HIGH) {
  }
  stop = false;
  while(!stop){




    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensorValues);

    for(int i = 0; i < SensorCount; i++){
      if(sensorValues[i] >= 700){
        normArray[i] = 1;
      }
      else{
        normArray[i] = 0;
      }
    }

    

    int error = position - 3500;

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



    
    makeTurn(normArray);
    
    
    if(normArray[0] == 1 && normArray[7] == 1){

      driveMotor(MOTOR_R, FORWARD, 0, true);
      driveMotor(MOTOR_L, FORWARD, 0, true);

      bool pauze = false;

      for(int i = 500; i > 0; i--){
        position = qtr.readLineBlack(sensorValues);

        for(int i = 0; i < SensorCount; i++){
          if(sensorValues[i] >= 700){
            normArray[i] = 1;
          }
          else{
            normArray[i] = 0;
          }
        }
        if(normArray[0] == 0 || normArray[7] == 0){
          pauze = true;
          Serial.println("pauze");
        }
      }

      delay(allignDel);

      driveMotor(MOTOR_R, FORWARD, 0, true);
      driveMotor(MOTOR_L, FORWARD, 0, true);
      if(pauze == false){
        stop = true;
      }

      delay(pauzeDel);
      
    }

    for(int i = 0; i < SensorCount; i++){
      Serial.print(normArray[i]);
      Serial.print("\t");
    }
    Serial.println();
  }


}
