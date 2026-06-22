//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 10.1
  IRremote
  http://www.keyestudio.com
*/

#include <IRremote.h>

int RECV_PIN = A3; //IR receiver is connected to A3
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup(){
  Serial.begin(9600); //Set the baud rate to 9600
  irrecv.enableIRIn(); 
}

void loop() {
  if (irrecv.decode(&results)) {
  Serial.println(results.value, HEX);
  irrecv.resume(); // Receive the next value
  }
}
//*************************************************************************
