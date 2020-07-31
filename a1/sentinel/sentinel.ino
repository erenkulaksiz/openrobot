#include <SharpIR.h>
#include <Wire.h>
#include <MAX44009.h>

#define rightSensorPin A3
#define leftSensorPin A4
#define midSensorPin A5

SharpIR rightSensor( SharpIR::GP2Y0A41SK0F, rightSensorPin );
SharpIR leftSensor( SharpIR::GP2Y0A41SK0F, leftSensorPin );
SharpIR midSensor( SharpIR::GP2Y0A41SK0F, midSensorPin );
MAX44009 lightSensor;

int rightCM, leftCM, midCM;

void setup(){
  Serial.begin(115200);
  Wire.begin();
}

void loop(){
  rightCM = cmSensorLimit(rightSensor.getDistance());
  leftCM = cmSensorLimit(leftSensor.getDistance());
  midCM = cmSensorLimit(midSensor.getDistance());


  delay(100);
}

int cmSensorLimit(int input){
  int output = 0;
  if(input > 20) output = 20;
  else if(input < 3) output = 3;
  else output = input;
  return output;
}
