/////////////////////////////////////////
// Project: Sentinel Pro v0.2
// Author: Eren Kulaksız
// For: UORG 17 Robotic Days / Robotics Competition

/////////////////////////////////////////
// LIBRARIES

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "FastLED.h"
#include <Wire.h>
#include <MPU6050.h>
#include <MAX44009.h>
#include <SharpIR.h>
#include <QTRSensors.h>
#include <MemoryFree.h>

/////////////////////////////////////////
// LEDS

#define leds_NUM_LEDS 8
CRGB leds[leds_NUM_LEDS];
int leds_fadeAmount = 5;
int leds_brightness = 0;
unsigned long leds_previousMillis = 0;
const long leds_interval = 20;
#define leds_DATA_PIN 12

/////////////////////////////////////////
// OBJECTS

RF24 rf24(49, 47);
MPU6050 mpu;
MAX44009 max44;
SharpIR sagsensor( SharpIR::GP2Y0A41SK0F, A3 );
SharpIR solsensor( SharpIR::GP2Y0A41SK0F, A4 );
SharpIR ortasensor( SharpIR::GP2Y0A41SK0F, A5 );
QTRSensors qtr;

/////////////////////////////////////////
// NRF24L01 - RADIO

const uint64_t radio_pipe = 0xE8E8F0F0E1LL;

/////////////////////////////////////////
// MPU6050 - GYRO

int16_t ax, ay, az;
int16_t gx, gy, gz;

/////////////////////////////////////////
// QTR - LINE FOLLOW

const uint8_t qtrSensorCount = 6;
uint16_t qtrsensorValues[qtrSensorCount];

/////////////////////////////////////////
// DATA

int data[] = { 0, 0, 0, 0 };

/////////////////////////////////////////
// VARIABLES

int joystickX, joystickY;
int solMotorHiz, sagMotorHiz; // 0-512 geri // 512-1023 ileri
#define SOL_MOTOR_PIN_A 8
#define SOL_MOTOR_PIN_B 9
#define SAG_MOTOR_PIN_A 10
#define SAG_MOTOR_PIN_B 11
#define UST_BUTTON 3
#define ALT_BUTTON 4

int lightlux;
int valy;
int valx;

int sagcm;
int solcm;
int ortacm;

uint16_t qtrposition;
uint16_t freememory;

int yMapped;

unsigned long leds_currentMillis;

bool isUstButtonPressing = false;
bool isAltButtonPressing = false;

/////////////////////////////////////////
// PID FOR LINE FOLLOW

#define Kp 0.07  // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed 
// 0.09


#define Kd 0.15   // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
// 0.16


#define rightMaxSpeed 1023 // max speed of the robot

#define leftMaxSpeed 1023 // max speed of the robot

#define rightBaseSpeed 800 // this is the speed at which the motors should spin when the robot is perfectly on the line

#define leftBaseSpeed 800  // this is the speed at which the motors should spin when the robot is perfectly on the line

#define TIMEOUT 2500

/////////////////////////////////////////
// CURRENT MODE

int CURRENT_MODE = 0; // 0 = unselected // 1 Çizgi izleme // 2 Joystick Kontrolü // 3 arama kurtarma

/////////////////////////////////////////
// MAIN FUNCTION - SETUP
void setup()
{

  for (int i = 0; i < leds_NUM_LEDS; i++ )
  {
    leds[i].setRGB(255, 0, 0);
  }

  FastLED.show();

  delay(3000);

  /////////////////////////////////////////
  // LEDS

  FastLED.addLeds<WS2812B, leds_DATA_PIN, GRB>(leds, leds_NUM_LEDS).setCorrection( TypicalLEDStrip );

  /////////////////////////////////////////
  // INITIAL SETUP

  Serial.begin(115200);
  printf_begin();
  Wire.begin();

  rf24.begin();
  rf24.setPayloadSize(32);
  rf24.setPALevel(RF24_PA_LOW);
  rf24.openReadingPipe(1, radio_pipe);
  rf24.startListening();
  //rf24.printDetails();

  mpu.initialize();

  if (max44.begin())
  {
    printdb("Light sensor cannot found", "error");
    while (1);
  }

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    22, 24, 26, 28, 30, 32
  }, qtrSensorCount);

  /////////////////////////////////////////
  // PINMODES

  pinMode(SOL_MOTOR_PIN_A, OUTPUT);
  pinMode(SOL_MOTOR_PIN_B, OUTPUT);
  pinMode(SAG_MOTOR_PIN_A, OUTPUT);
  pinMode(SAG_MOTOR_PIN_B, OUTPUT);
  pinMode(UST_BUTTON, INPUT_PULLUP);
  pinMode(ALT_BUTTON, INPUT_PULLUP);

  delay(1000);

  isUstButtonPressing = digitalRead(UST_BUTTON);
  isAltButtonPressing = digitalRead(ALT_BUTTON);

  isUstButtonPressing = !isUstButtonPressing;
  isAltButtonPressing = !isAltButtonPressing;

  printdb(String(isUstButtonPressing), "debugln");
  printdb(String(isAltButtonPressing), "debugln");

  CURRENT_MODE = 0;

  // show yellow color for selection
  for (int i = 0; i < leds_NUM_LEDS; i++ )
  {
    leds[i].setRGB(255, 225, 0);
  }
  FastLED.show();


  while (CURRENT_MODE == 0) {

    printdb("waiting for input", "debugln");

    isUstButtonPressing = digitalRead(UST_BUTTON);
    isAltButtonPressing = digitalRead(ALT_BUTTON);

    isUstButtonPressing = !isUstButtonPressing;
    isAltButtonPressing = !isAltButtonPressing;

    delay(500);
    
    if (isUstButtonPressing && !isAltButtonPressing) {
      CURRENT_MODE = 1; // Line follow
      printdb("Line follow DETECTED BY BUTTON", "debugln");
    }

    if (!isUstButtonPressing && isAltButtonPressing) {
      CURRENT_MODE = 2; // Joystick mode
      printdb("joystick DETECTED BY BUTTON", "debugln");
    }

    if (isUstButtonPressing && isAltButtonPressing) {
      CURRENT_MODE = 3; // Joystick mode
      printdb("arama DETECTED BY BUTTON", "debugln");
    }
    
  }

  // show green color for correct selection
  for (int i = 0; i < leds_NUM_LEDS; i++ )
  {
    leds[i].setRGB(0, 225, 0);
  }
  FastLED.show();

  delay(2000);

  for (int i = 0; i < leds_NUM_LEDS; i++ )
  {
    leds[i].setRGB(0, 0, 0);
  }

  FastLED.show();

  /////////////////////////////////////////
  // QTR - CALIBRATE

  printdb("Start of calibration", "debugln");

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  for (uint8_t i = 0; i < qtrSensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < qtrSensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  printdb("End of calibration", "debugln");


  /////////////////////////////////////////
  // DETECT CURRENT MODE

  switch (CURRENT_MODE) {
    case 1:
      printdb("Detected mode: Line Follower", "debugln");
      break;

    case 2:
      printdb("Detected mode: Joystick Controller", "debugln");
      break;


    case 3:
      printdb("Detected mode: Arama Modu", "debugln");
      break;

  }

  printdb(" ", "space");
  for (int i = 0; i < leds_NUM_LEDS; i++ )
  {
    leds[i].setRGB(0, 225, 0);
  }

  FastLED.show();
  
  delay(3000);

  for (int i = 0; i < leds_NUM_LEDS; i++ )
  {
    leds[i].setRGB(0, 0, 0);
  }

  FastLED.show();

}

int lastError = 0;
bool didSeenStopLine = false;

/////////////////////////////////////////
// MAIN FUNCTION - LOOP
void loop()
{

  /////////////////////////////////////////
  // READ SENSOR DATA

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  lightlux = max44.get_lux();
  valy = map(ay, -16000, 16000, 0, 360);
  valx = map(ax, -16000, 16000, 0, 360);

  sagcm = sagsensor.getDistance();
  solcm = solsensor.getDistance();
  ortacm = ortasensor.getDistance();

  qtrposition = qtr.readLineBlack(qtrsensorValues);

  isUstButtonPressing = digitalRead(UST_BUTTON);
  isAltButtonPressing = digitalRead(ALT_BUTTON);

  isUstButtonPressing = !isUstButtonPressing;
  isAltButtonPressing = !isAltButtonPressing;

  /////////////////////////////////////////
  // LIMIT SENSOR DATA

  if (sagcm < 4) sagcm = 4;
  else if (sagcm > 30) sagcm = 30;
  if (solcm < 4) solcm = 4;
  else if (solcm > 30) solcm = 30;
  if (ortacm < 4) ortacm = 4;
  else if (ortacm > 30) ortacm = 30;

  if (valy < 0) valy = 0;
  else if (valy > 360) valy = 360;
  if (valx < 0) valx = 0;
  else if (valx > 360) valx = 360;

  /////////////////////////////////////////
  // PRINT SENSOR DATA

  freememory = freeMemory();
  printdb("freeMemory()=", "debug");
  printdb(String(freememory), "debuglnsz");
  printdb("ustbutton =", "debug");
  printdb(String(isUstButtonPressing), "debuglnsz");
  printdb("altbutton =", "debug");
  printdb(String(isAltButtonPressing), "debuglnsz");
  printdb(" ", "space");
  printdb("valx: ", "debug");
  printdb(String(valx), "debuglnsz");
  printdb("valy: ", "debug");
  printdb(String(valy), "debuglnsz");
  printdb(String(qtrposition), "debugln");
  printdb(" ", "space");
  printdb("sagcm: ", "debug");
  printdb(String(sagcm), "debuglnsz");
  printdb("solcm: ", "debug");
  printdb(String(solcm), "debuglnsz");
  printdb("ortacm: ", "debug");
  printdb(String(ortacm), "debuglnsz");
  printdb(" ", "space");
  printdb("lightlux: ", "debug");
  printdb(String(lightlux), "debuglnsz");
  printdb(" ", "space");
  printdb(String(CURRENT_MODE), "debugln");

  switch(CURRENT_MODE) {
    case 1:{

      /////////////////////////////////////////
      // LINE FOLLOWER MODE

      int error = 2500 - qtrposition;

      int motorSpeed = Kp * error + Kd * (error - lastError);
      lastError = error;

      int rightMotorSpeed = rightBaseSpeed + motorSpeed;
      int leftMotorSpeed = leftBaseSpeed - motorSpeed;


      printdb("LFFT: " + String(leftMotorSpeed), "debuglnsz");
      printdb("RIGHT: " + String(rightMotorSpeed), "debuglnsz");

      int toplam = 0;

      for (uint8_t i = 0; i < qtrSensorCount; i++)
      {

        toplam = toplam + qtrsensorValues[i];
        //printdb(String(qtrsensorValues[i]), "debuglnsz");

        if (toplam < 3600) {
          if (ortacm > 5) {
            if (!didSeenStopLine) {
              solMotorKontrol(solMotorHiz);
              sagMotorKontrol(sagMotorHiz);
            }
          } else {
            solMotorKontrol(512);
            sagMotorKontrol(512);
          }
        } else {
          didSeenStopLine = true;
          solMotorKontrol(512);
          sagMotorKontrol(512);
        }


      }

      if (solcm < 5) {
        didSeenStopLine = false;
      }

      solMotorHiz = leftMotorSpeed;
      sagMotorHiz = rightMotorSpeed;

      break;}

    case 2: {

      /////////////////////////////////////////
      // JOYSTICK MODE

      if ( rf24.available() ) {

        /////////////////////////////////////////
        // RF CONNECTED

        rf24.read( data, 4 );
        joystickX = data[0];
        joystickY = data[1];

        /////////////////////////////////////////
        // PRINT DATA

        printdb(" ", "space");
        printdb("datax: ", "debug");
        printdb(String(data[0]), "debuglnsz");
        printdb("datay: ", "debug");
        printdb(String(data[1]), "debuglnsz");

        /////////////////////////////////////////
        // MOTOR CONTROL FOR X AXIS, GOES FORWARD AND BACKWARD

        if (joystickX < 380) {
          solMotorHiz = map(joystickX, 0, 420, 0, 512);
          sagMotorHiz = map(joystickX, 0, 420, 0, 512);
        } else if (joystickX > 490) {
          solMotorHiz = map(joystickX, 512, 980, 512, 1023);
          sagMotorHiz = map(joystickX, 512, 980, 512, 1023);
        } else {
          solMotorHiz = 512;
          sagMotorHiz = 512;
        }

        /////////////////////////////////////////
        // MOTOR CONTROL FOR Y AXIS, GOES LEFT AND RIGHT

        if (joystickY < 380) {
          yMapped = map(joystickY, 380, 0, 0, 1023);

          solMotorHiz = solMotorHiz + yMapped;
          sagMotorHiz = sagMotorHiz - yMapped;
        } else if (joystickY > 512) {
          yMapped = map(joystickY, 512, 0, 0, 1023);

          solMotorHiz = solMotorHiz + yMapped;
          sagMotorHiz = sagMotorHiz - yMapped;
        }

        
      printdb("LFFT: " + String(solMotorHiz), "debuglnsz");
      printdb("RIGHT: " + String(sagMotorHiz), "debuglnsz");

        /////////////////////////////////////////
        // CONTROL MOTORS

        solMotorKontrol(solMotorHiz);
        sagMotorKontrol(sagMotorHiz);

        /////////////////////////////////////////
        // LED CONTROL

        for (int i = 0; i < leds_NUM_LEDS; i++ )
        {
          leds[i].setRGB(0, 225, 0);
          leds[i].fadeLightBy(leds_brightness);
        }

        leds_currentMillis = millis();

        if (leds_currentMillis - leds_previousMillis >= leds_interval) {
          leds_previousMillis = leds_currentMillis;

          leds_brightness = leds_brightness + leds_fadeAmount;

          if (leds_brightness == 0 || leds_brightness == 255)
          {
            leds_fadeAmount = -leds_fadeAmount ;
          }
        }

        FastLED.show();

      } else {

        /////////////////////////////////////////
        // RF NOT CONNECTED

        /////////////////////////////////////////
        // SHUTDOWN MOTORS

        digitalWrite(SOL_MOTOR_PIN_A, 0);
        digitalWrite(SOL_MOTOR_PIN_B, 0);
        digitalWrite(SAG_MOTOR_PIN_A, 0);
        digitalWrite(SAG_MOTOR_PIN_B, 0);

        /////////////////////////////////////////
        // LED CONTROL

        for (int i = 0; i < leds_NUM_LEDS; i++ )
        {
          leds[i].setRGB(225, 0, 0);
          leds[i].fadeLightBy(leds_brightness);
        }

        leds_currentMillis = millis();

        if (leds_currentMillis - leds_previousMillis >= leds_interval) {
          leds_previousMillis = leds_currentMillis;

          leds_brightness = leds_brightness + leds_fadeAmount;

          if (leds_brightness == 0 || leds_brightness == 255)
          {
            leds_fadeAmount = -leds_fadeAmount ;
          }
        }

        FastLED.show();

        /////////////////////////////////////////
        // SERIAL INFORMATION

        Serial.println("No connection");

      }

      break;}

    case 3:{

      printdb(String("DDDDDDDDDDDDDDDDDDDDD"), "debugln");

      break;}
  }

  delay(25);

}

/////////////////////////////////////////
// SERIAL PRINT DEBUGGER
void printdb(String input, String type) {
  String outprint = "";
  int casetype = 0;
  bool enableOutput = true;

  if (type == "error") casetype = 0;
  else if (type == "debug") casetype = 1;
  else if (type == "debugln") casetype = 2;
  else if (type == "space") casetype = 3;
  else if (type == "debugsz") casetype = 4;
  else if (type == "debuglnsz") casetype = 5;

  if (enableOutput) {
    switch (casetype) {
      case 0:
        outprint = "[ERROR] " + String(millis()) + " " + input;
        Serial.println(outprint);
        break;

      case 1:
        outprint = "[DEBUG] " + String(millis()) + " " + input;
        Serial.print(outprint);
        break;

      case 2:
        outprint = "[DEBUG] " + String(millis()) + " " + input;
        Serial.println(outprint);
        break;

      case 3:
        Serial.println("_____");
        break;

      case 4:
        Serial.print(input);
        break;

      case 5:
        Serial.println(input);
        break;
    }
  }


}

/////////////////////////////////////////
// FUNCTION: Controlling left motor with
// single input variable
//
// PARAM: (int)girdi
//
// CONTROL:
// If girdi is < 512, motor goes
// backwards according to input variable
//
// If girdi is > 512, motor goes forwards
// according to input variable
//
void solMotorKontrol(int girdi) {
  int solMotorGeriHiz, solMotorIleriHiz;

  // Limiter
  if (girdi < 0) {
    girdi = 0;
  } else if (girdi > 1023) {
    girdi = 1023;
  }

  // Geri git
  if (girdi < 512) {
    solMotorGeriHiz = map(girdi, 512, 0, 0, 110);
    if (solMotorGeriHiz > 10) {
      analogWrite(SOL_MOTOR_PIN_A, solMotorGeriHiz);
      digitalWrite(SOL_MOTOR_PIN_B, LOW);
    } else {
      digitalWrite(SOL_MOTOR_PIN_A, LOW);
      digitalWrite(SOL_MOTOR_PIN_B, LOW);
    }
  } else if (girdi > 512) {
    // İleri Git
    solMotorIleriHiz = map(girdi, 512, 1023, 0, 110);
    if (solMotorIleriHiz > 20) {
      analogWrite(SOL_MOTOR_PIN_B, solMotorIleriHiz);
      digitalWrite(SOL_MOTOR_PIN_A, LOW);
    } else {
      digitalWrite(SOL_MOTOR_PIN_A, LOW);
      digitalWrite(SOL_MOTOR_PIN_B, LOW);
    }
  } else {
    digitalWrite(SOL_MOTOR_PIN_A, LOW);
    digitalWrite(SOL_MOTOR_PIN_B, LOW);
  }
}


/////////////////////////////////////////
// FUNCTION: Controlling right motor with
// single input variable
//
// PARAM: (int)girdi
//
// CONTROL:
// If girdi is < 512, motor goes
// backwards according to input variable
//
// If girdi is > 512, motor goes forwards
// according to input variable
//
void sagMotorKontrol(int girdi) {
  int sagMotorGeriHiz, sagMotorIleriHiz;

  // Limiter
  if (girdi < 0) {
    girdi = 0;
  } else if (girdi > 1023) {
    girdi = 1023;
  }

  // Geri git
  if (girdi < 512) {
    sagMotorGeriHiz = map(girdi, 512, 0, 0, 110);
    if (sagMotorGeriHiz > 10) {
      analogWrite(SAG_MOTOR_PIN_A, sagMotorGeriHiz);
      digitalWrite(SAG_MOTOR_PIN_B, LOW);
    } else {
      digitalWrite(SAG_MOTOR_PIN_A, LOW);
      digitalWrite(SAG_MOTOR_PIN_B, LOW);
    }
  } else if (girdi > 512) {
    // İleri Git
    sagMotorIleriHiz = map(girdi, 512, 1023, 0, 110);
    if (sagMotorIleriHiz > 20) {
      analogWrite(SAG_MOTOR_PIN_B, sagMotorIleriHiz);
      digitalWrite(SAG_MOTOR_PIN_A, LOW);
    } else {
      digitalWrite(SAG_MOTOR_PIN_A, LOW);
      digitalWrite(SAG_MOTOR_PIN_B, LOW);
    }
  } else {
    digitalWrite(SAG_MOTOR_PIN_A, LOW);
    digitalWrite(SAG_MOTOR_PIN_B, LOW);
  }
}
