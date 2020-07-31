#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(49, 47);

const uint64_t pipe = 0xE8E8F0F0E1LL;

int data[] = { 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  printf_begin();
  pinMode(13, OUTPUT);
  radio.begin();
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.printDetails();
}

void loop() {
  data[0] = analogRead(A0);
  data[1] = analogRead(A1);
  bool ok = radio.write( data, 4 );
  Serial.println("_____");
  Serial.print("datax:");
  Serial.println(data[0]);
  Serial.print("datay:");
  Serial.println(data[1]);
  if (ok)
    printf("ok\n\r");
  else
    printf("failed\n\r");

  delay(5);
}
