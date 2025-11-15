#include <Arduino.h>
#include <pins.h>
#include <SensorsControl.h>
#include "MotorControl.h"
#include "PID.h"
#include "PIDSettings.h"

Sensor left {
  .xshutPin = 2,
  .i2cAddress = 0x2A,
  .index = 0
};

Sensor mid {
  .xshutPin = 11,
  .i2cAddress = 0x2B,
  .index = 1
};

Sensor right {
  .xshutPin = 10,
  .i2cAddress = 0x2C,
  .index = 2
};


PIDcontroller pid{

  .proportional = PID_PROPORTIONAL,
  .integral = PID_INTEGRAL,
  .derivative = PID_DERIVATIVE,

  .dLowPassFilter = DLOWPASSFILTER,

  .limMin = PID_LIM_MIN,
  .limMax = PID_LIM_MAX,

  .time = PID_TIME

};

SensorControl sensors(left, mid, right);

int sameEnough = 0;

int prevLeft = 0;
int prevMid = 0;
int prevRight = 0;

// Reads if the sensor data is close to the old sensor data. If that has been the case for 1 second then it goes backwards.

void SamePlace(int leftSensorData, int midSensorData, int rightSensorData, int &sameEnough, int &prevLeft, int &prevMid, int &prevRight){

  const float tolerance = 20.0; 

  bool sameLeft  = fabs(leftSensorData  - prevLeft)  < tolerance;
  bool sameMid   = fabs(midSensorData   - prevMid)   < tolerance;
  bool sameRight = fabs(rightSensorData - prevRight) < tolerance;

  
  if (sameLeft && sameMid && sameRight) {
    sameEnough++;
  } else {
    sameEnough = 0; 
  }

  prevLeft  = leftSensorData;
  prevMid   = midSensorData;
  prevRight = rightSensorData;


}

float scaleFromDistance(int distance) {
  const float maxBoost = 1.5;
  const float maxDist  = 400.0;

  
  distance = constrain(distance, 0, maxDist);

  
  float factor = 1.0 + (maxBoost - 1.0) * (1.0 - (distance / maxDist));

  return factor;
}

void printPWMValues(int PWMValueLeft, int PWMValueRight) {
  Serial.print(F("Left PWM: "));
  Serial.print(String(PWMValueLeft));
  Serial.print(F("\nRight PWM: "));
  Serial.print(String(PWMValueRight));
  Serial.print(F("\n"));
  }

void setup(){
  // put your setup code here, to run once:
  Serial.begin(115200);

  if(!sensors.begin()){
    Serial.println(F("Sensors init failed. Halt."));
    while(true){
      delay(100);
    }
  }

  MotorControl::begin();
  Serial.println(F("Motor test start \n"));

  PIDController_Init(&pid);

}


void loop() {
  // put your main code here, to run repeatedly:
  int leftSensorDistance = sensors.readSensorData(left.index);
  int midSensorDistance = sensors.readSensorData(mid.index);
  int rightSensorDistance = sensors.readSensorData(right.index);

  sensors.printValues(leftSensorDistance, midSensorDistance, rightSensorDistance);

  float error;
  float PIDoutput;

  float PIDInputNormalization = 100;

  // The error is just left sensor data vs right sensor data devided by a magic number.
  // The PIDnormalaztion variable came through testing. The error would be too big and the PID sytem thingy would always output it's max, so it had to be lowered.

  error = (float) (leftSensorDistance - rightSensorDistance) / PIDInputNormalization;

  PIDoutput = PIDController_Update(&pid, error);

  SamePlace(leftSensorDistance, midSensorDistance, rightSensorDistance, sameEnough, prevLeft, prevMid, prevRight);

  int baseSpeed = 200;

  int leftMotorSpeed  = (int) baseSpeed - PIDoutput * 5;
  int rightMotorSpeed = (int) baseSpeed + PIDoutput * 5;

  // PID system as we have it cant dodge things in it way, 
  // so this code chunk does it.

  if(rightSensorDistance < 400 && midSensorDistance < 400){
    float scale = scaleFromDistance(leftSensorDistance);
    leftMotorSpeed = min(rightMotorSpeed * scale, 255.0f);
    rightMotorSpeed = min(leftMotorSpeed / scale, 255.0f);
  }

  else if(leftSensorDistance < 400 && midSensorDistance < 400){
    float scale = scaleFromDistance(rightSensorDistance);
    rightMotorSpeed = min(leftMotorSpeed * scale, 255.0f);
    leftMotorSpeed = min(rightMotorSpeed / scale, 255.0f);
  }

  else if(midSensorDistance < 300){
    float scale = scaleFromDistance(midSensorDistance);
    rightMotorSpeed = min(rightMotorSpeed * scale, 255.0f);
  }

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  int backwardsMovementThreshold = 21;

  if (sameEnough == backwardsMovementThreshold) {
    MotorControl::moveBackward(200);
    delay(2500);
    MotorControl::stop(); 

    sameEnough = 0;
    prevLeft = 0;
    prevMid = 0;
    prevRight = 0;

  }

  printPWMValues(leftMotorSpeed, rightMotorSpeed);

  MotorControl::drive(leftMotorSpeed, rightMotorSpeed);

  delay(20);

}


