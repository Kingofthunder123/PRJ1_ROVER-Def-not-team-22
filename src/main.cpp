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

#define btnOne  5
#define btnTwo  6
#define btnThr  7

#define encoderPinA 20  // Encoder channel A for motor A connected to digital pin 20
#define encoderPinB 21  // Encoder channel A for motor B connected to digital pin 21


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



// PARAMETERS
// VVVVVVVVVV

int defSpd  = 255; // Default speed. The speed setting for straght lines.
int turnSpd = 140; // Turning speed. The speed setting for corners.



// CUSTOM VARIABLES
// VVVVVVVVVVVVVVVV

typedef enum {MOTOR_A, MOTOR_B} SELECT_MOTOR;
typedef enum {FORWARD, REVERSE} DIRECTION;

bool stop = false;

int normArray[8];

const float mmPerStep = (65*PI)/20;
const float mmToMiddle = 160;

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

  if(motor == MOTOR_A){
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



bool lineNormal(int array[]){
  int total = 0;
  for(int i = 0; i < SensorCount; i++){
    total += array[i];
  }

  if(total < 8){
    return(true);
  }

  return(false);
}



void makeTurn(int array[]){
  unsigned int  stepGoal = mmToMiddle/mmPerStep;

  unsigned int stepNrStartA = nrStepsA;
  unsigned int stepNrStartB = nrStepsB;

  while((nrStepsA < stepNrStartA + stepGoal) || (nrStepsB < stepNrStartB + stepGoal)){
    if(nrStepsA > stepNrStartA + stepGoal){
      driveMotor(MOTOR_A, FORWARD, 0, true);
    }

    if(nrStepsB > stepNrStartB + stepGoal){
      driveMotor(MOTOR_B, FORWARD, 0, true);
    }
  }
  driveMotor(MOTOR_A, FORWARD, 0, true);
  driveMotor(MOTOR_B, FORWARD, 0, true);
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

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoderB, CHANGE);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(2);
  Serial.println("hoi");

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    Serial.println(i);
  }
  digitalWrite(LED_BUILTIN, LOW);



}



// LOOP
// VVVV
void loop(){

  while(digitalRead(strtBtn) != HIGH) {}
  stop = false;
  while(!stop){




    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensorValues);

    for(int i = 0; i < SensorCount; i++){
      if(sensorValues[i] >= 900){
        normArray[i] = 1;
      }
      else{
        normArray[i] = 0;
      }
    }

    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      if(sensorValues[i] >= 900){
        Serial.print(1);
      }
      else{
        Serial.print(0);
      }
      Serial.print('\t');
    }
    Serial.println(position);



    driveMotor(MOTOR_A, FORWARD, defSpd);
    driveMotor(MOTOR_B, FORWARD, defSpd);



    if(lineNormal(normArray)){ // Is line normal?
      if(digitalRead(btnTwo)){
        //turn
      }
    }
    else{
      if(digitalRead(btnThr)){
          // It's a pause sign, pause for a bit and then go on
          driveMotor(MOTOR_A, FORWARD, 0, true);
          driveMotor(MOTOR_B, FORWARD, 0, true);
          delay(2000);
      }
      else{
        driveMotor(MOTOR_A, FORWARD, 0, true);
        driveMotor(MOTOR_B, FORWARD, 0, true);
        stop = true;
      }
    }
  }


}
