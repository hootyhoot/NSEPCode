//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 10.2
  IRremote Control LED
  http://www.keyestudio.com
*/
#include <MecanumCar_v2.h>
mecanumCar mecanumCar(3, 2);  //sda-->D3,scl-->D2
#include <IRremote.h>

int RECV_PIN = A3;
IRrecv irrecv(RECV_PIN);
decode_results results;
bool flag = true;

void setup(){
  mecanumCar.Init();//Initialize the motor and the color light driver
  irrecv.enableIRIn(); 
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    if (results.value == 0xFF02FD && flag == true) //The value for turning on the light
    {
    mecanumCar.right_led(1);
    mecanumCar.left_led(1);
    flag = false;
    }
     else if (results.value == 0xFF02FD && flag == false) //The value for turning off the lights
    {
    mecanumCar.right_led(0);
    mecanumCar.left_led(0);
    flag = true;
    }
    irrecv.resume(); // Receive the next value
  }
}
//*************************************************************************
