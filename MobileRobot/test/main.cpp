#include <Arduino.h>
#include <IRremote.h>
#include "MecanumCar_v2.h"

#define RECV_PIN A3

IRrecv irrecv(RECV_PIN);
decode_results results;
mecanumCar robot(3, 2);

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  robot.Init();
  Serial.println("Ready.");
}

void loop() {
  if (irrecv.decode(&results)) {
    if (results.value != 0xFFFFFFFF) {  // ignore NEC repeat frames
      Serial.print("IR: 0x");
      Serial.println(results.value, HEX);

      switch (results.value) {
        case 0xFF629D: robot.Advance(); break;
        case 0xFF02FD: robot.Stop();    break;
      }
    }

    irrecv.resume();
  }
}
