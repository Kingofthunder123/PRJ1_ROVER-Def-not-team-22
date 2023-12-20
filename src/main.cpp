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


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



// CUSTOM VARIABLES
// VVVVVVVVVVVVVVVV

typedef enum {MOTOR_A, MOTOR_B} SELECT_MOTOR;
typedef enum {FORWARD, REVERSE} DIRECTION;



// FUNCTIONS
// VVVVVVVVV

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


  // qtr.setTypeAnalog();
  // qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  // qtr.setEmitterPin(2);


  // delay(500);
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);


  // for (uint16_t i = 0; i < 400; i++){
  //   qtr.calibrate();
  // }
  // digitalWrite(LED_BUILTIN, LOW);



}



// LOOP
// VVVV
void loop(){
  // // read calibrated sensor values and obtain a measure of the line position
  // // from 0 to 5000 (for a white line, use readLineWhite() instead)
  // uint16_t position = qtr.readLineBlack(sensorValues);

  // // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // // reflectance and 1000 means minimum reflectance, followed by the line
  // // position
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  // delay(250);

  driveMotor(MOTOR_A, FORWARD, 230);
  driveMotor(MOTOR_B, FORWARD, 230);

  delay(1000);
  driveMotor(MOTOR_A, FORWARD, 0, true);
  driveMotor(MOTOR_B, FORWARD, 0, true);
  delay(1000);

  driveMotor(MOTOR_A, FORWARD, 230);
  driveMotor(MOTOR_B, REVERSE, 230);

  delay(1000);
  driveMotor(MOTOR_A, FORWARD, 0, true);
  driveMotor(MOTOR_B, FORWARD, 0, true);
  delay(1000);

}
