// LIBRARIES
// VVVVVVVVV

#include <Arduino.h>
#include <QTRSensors.h>
#include <FastLED.h>

QTRSensors qtr;


#define NUM_LEDS 22
#define DATA_PIN 17
#define Brightness 255

CRGB leds[NUM_LEDS];
CRGB FontysPurple = CRGB(102, 0, 153);


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

#define strtBtn 52

#define encoderPinA 20  // Encoder channel A for motor A connected to digital pin 20
#define encoderPinB 21  // Encoder channel A for motor B connected to digital pin 21

#define SwithFast 44
#define SwithSlow 45

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



// PARAMETERS
// VVVVVVVVVV

// PID parameters
int defSpd  = 255; // Default speed. The speed setting for straght lines.
int RturnSpd; // Turning speed. The speed setting for corners.
int LturnSpd;
int error;
int lastError = 0;

// Speed settings
int maxSpeed = 255;
int rotationSpeed = 100;
int stopSpeed = 50;

// Delays
int allignDel = 1;
int pauzeDel = 5000;
int turnDelay = 175;
int preTurnSpeed = 0;


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


// Change the speed depending on the current switch position  (fast or slow)
void speedControl() {
//TEST THIS!!!
  if(digitalRead(SwithFast) == HIGH){
    defSpd = 255;
    rotationSpeed = 180;
    stopSpeed = 50;
    turnDelay = 175;
    preTurnSpeed = 255;
  }
  else {
    defSpd = 255;
    rotationSpeed = 140;
    stopSpeed = 35;
    turnDelay = 100;
    preTurnSpeed = 100;
  }

}

// Update the encoder count for motor A
void updateEncoderA() {
  // Increment or decrement the encoder count for motor A based on the rising and falling edges of the encoder signal
  nrStepsA++;
}

// Update the encoder count for motor B
void updateEncoderB() {
  // Increment or decrement the encoder count for motor B based on the rising and falling edges of the encoder signal
  nrStepsB++;
}

// Function to simplify the motor control
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

//MOST IMPORTANT FUNCTION to make the turn
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


// SETUP
// VVVVV
void setup(){
  
  Serial.begin(9600);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(Brightness);
  FastLED.clear();

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

  pinMode(SwithFast, INPUT);
  pinMode(SwithSlow, INPUT);

  // attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoderB, CHANGE);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);

  fill_solid(leds, NUM_LEDS, FontysPurple);
  FastLED.show();


  for (uint16_t i = 0; i < 150; i++){
    qtr.calibrate();
    Serial.println(i);
  }
}



// LOOP
// VVVV


void loop(){

  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  

  while(digitalRead(strtBtn) != HIGH) {
  }
  stop = false;

  speedControl();
  Serial.println(rotationSpeed);
  Serial.println(stopSpeed);

//main while loop
  while(!stop){  

  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();


    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensorValues);

    for(int i = 0; i < SensorCount; i++){
      if(sensorValues[i] >= 750){
        normArray[i] = 1;
      }
      else{
        normArray[i] = 0;
      }
    }

    
    //PID control
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
    
    
    //IF to stop
    if(normArray[0] == 1 && normArray[7] == 1){
      
      driveMotor(MOTOR_R, FORWARD, 0, true);
      driveMotor(MOTOR_L, FORWARD, 0, true);

      delay(10);

      driveMotor(MOTOR_R, FORWARD, stopSpeed);
      driveMotor(MOTOR_L, FORWARD, stopSpeed);

      int starttime = millis();

      while(normArray[0] == 1 || normArray[7] == 1){
        qtr.readLineBlack(sensorValues);

        for(int i = 0; i < SensorCount; i++){
          if(sensorValues[i] >= 700){
            normArray[i] = 1;
          }
          else{
            normArray[i] = 0;
          }
        }
      }
      int endTime = millis();

      delay(allignDel);

      driveMotor(MOTOR_R, FORWARD, 0, true);
      driveMotor(MOTOR_L, FORWARD, 0, true);

      
    // millis delay for the pause
      if(endTime - starttime < 500){
        fill_solid(leds, NUM_LEDS, CRGB::Blue);
        FastLED.show();
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
    Serial.println();
  }
}

// to try in the future
// qtr.setCalibration();
// test speed settings with the switch
