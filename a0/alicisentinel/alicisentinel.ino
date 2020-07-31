#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "FastLED.h"
#include <Wire.h>
#include <MPU6050.h>
#include <MAX44009.h>

#define NUM_LEDS_SAG 8
#define NUM_LEDS_SOL 8

CRGB leds_sag[NUM_LEDS_SAG];
CRGB leds_sol[NUM_LEDS_SOL];
RF24 radio(49, 47);
MPU6050 mpu;
MAX44009 light;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define DATA_PIN_SAG 2
#define DATA_PIN_SOL 3

int fadeAmount = 5;
int brightness = 0;
const uint64_t pipe = 0xE8E8F0F0E1LL;
unsigned long LIGHTS_previousMillis = 0;
const long LIGHTS_interval = 20;
int data[] = { 0, 0, 0, 0 };

int joystickX, joystickY;
int solMotorHiz, sagMotorHiz; // 0-512 geri // 512-1023 ileri

#define SOL_MOTOR_PIN_A 8
#define SOL_MOTOR_PIN_B 9
#define SAG_MOTOR_PIN_A 10
#define SAG_MOTOR_PIN_B 11


void setup()
{
  FastLED.addLeds<WS2812B, DATA_PIN_SAG, GRB>(leds_sag, NUM_LEDS_SAG).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2812B, DATA_PIN_SOL, GRB>(leds_sol, NUM_LEDS_SOL).setCorrection( TypicalLEDStrip );
  Serial.begin(115200);
  printf_begin();
  Wire.begin();
  pinMode(13, OUTPUT);
  pinMode(SOL_MOTOR_PIN_A, OUTPUT);
  pinMode(SOL_MOTOR_PIN_B, OUTPUT);
  pinMode(SAG_MOTOR_PIN_A, OUTPUT);
  pinMode(SAG_MOTOR_PIN_B, OUTPUT);

  radio.begin();
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  radio.printDetails();

  mpu.initialize();

  if (light.begin())
  {
    Serial.println("Could not find a valid MAX44009 sensor, check wiring!");
    while (1);
  }

}

void loop()
{

  if ( radio.available() ) {

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int valy = map(ay, -16000, 16000, 0, 360);
    int valx = map(ax, -16000, 16000, 0, 360);

    Serial.println("____");
    Serial.print("Lux: ");
    Serial.println(light.get_lux());

    radio.read( data, 4 );
    Serial.println("_____");
    Serial.print("datax:");
    Serial.println(data[0]);
    Serial.print("datay:");
    Serial.println(data[1]);

    joystickX = data[0];
    joystickY = data[1];

    // Motor kontrol

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

    if (joystickY < 380) {
      int yMapped = map(joystickY, 380, 0, 0, 1023);

      solMotorHiz = solMotorHiz + yMapped;
      sagMotorHiz = sagMotorHiz - yMapped;

    } else if (joystickY > 512) {
      int yMapped = map(joystickY, 512, 0, 0, 1023);

      solMotorHiz = solMotorHiz + yMapped;
      sagMotorHiz = sagMotorHiz - yMapped;

    }

    solMotorKontrol(solMotorHiz);
    sagMotorKontrol(sagMotorHiz);


    //////////////////////


    for (int i = 0; i < NUM_LEDS_SAG; i++ )
    {
      leds_sag[i].setRGB(0, 25, 0);
      leds_sag[i].fadeLightBy(brightness);
    }

    for (int i = 0; i < NUM_LEDS_SOL; i++ )
    {
      leds_sol[i].setRGB(0, 25, 0);
      leds_sol[i].fadeLightBy(brightness);
    }

    FastLED.show();

    unsigned long LIGHTS_currentMillis = millis();

    if (LIGHTS_currentMillis - LIGHTS_previousMillis >= LIGHTS_interval) {
      LIGHTS_previousMillis = LIGHTS_currentMillis;

      brightness = brightness + fadeAmount;

      if (brightness == 0 || brightness == 255)
      {
        fadeAmount = -fadeAmount ;
      }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  } else {

    digitalWrite(SOL_MOTOR_PIN_A, 0);
    digitalWrite(SOL_MOTOR_PIN_B, 0);
    digitalWrite(SAG_MOTOR_PIN_A, 0);
    digitalWrite(SAG_MOTOR_PIN_B, 0);


    for (int i = 0; i < NUM_LEDS_SAG; i++ )
    {
      leds_sag[i].setRGB(25, 0, 0);
      leds_sag[i].fadeLightBy(brightness);
    }

    for (int i = 0; i < NUM_LEDS_SOL; i++ )
    {
      leds_sol[i].setRGB(25, 0, 0);
      leds_sol[i].fadeLightBy(brightness);
    }

    FastLED.show();

    unsigned long LIGHTS_currentMillis = millis();

    if (LIGHTS_currentMillis - LIGHTS_previousMillis >= LIGHTS_interval) {
      LIGHTS_previousMillis = LIGHTS_currentMillis;

      brightness = brightness + fadeAmount;

      if (brightness == 0 || brightness == 255)
      {
        fadeAmount = -fadeAmount ;
      }
    }

    Serial.println("No connection");
  }


  delay(25);

}

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
