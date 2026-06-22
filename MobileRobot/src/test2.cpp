#include <Arduino.h>
#include "MecanumCar_v2.h"

mecanumCar car(3, 2);

#define SensorLeft   A0
#define SensorMiddle A1

void setup()
{
    Serial.begin(9600);

    car.Init();

    int calibrated_speed_Upper_L = 24;
    int calibrated_speed_Lower_L = 21;
    int calibrated_speed_Upper_R = 28;
    int calibrated_speed_Lower_R = 28;

    int speed_multiplier = 1.5;

    speed_Upper_L = calibrated_speed_Upper_L * speed_multiplier;
    speed_Lower_L = calibrated_speed_Lower_L * speed_multiplier;
    speed_Upper_R = calibrated_speed_Upper_R * speed_multiplier;
    speed_Lower_R = calibrated_speed_Lower_R * speed_multiplier;

    pinMode(SensorLeft, INPUT);
    pinMode(SensorMiddle, INPUT);

    Serial.println("2-Sensor Line Following");
}

void loop()
{
    int L = digitalRead(SensorLeft);
    int M = digitalRead(SensorMiddle);

    Serial.print("L=");
    Serial.print(L);
    Serial.print(" M=");
    Serial.println(M);

    // Both sensors on black line
    if(L == 1 && M == 1)
    {
        Serial.println("FORWARD");
        car.Advance();
        delay(1000);
    }

    // Line moved towards left sensor
    if(L == 1 && M == 0)
    {
        Serial.println("CORRECT LEFT");
        car.Turn_Left();
        // or car.LU_Move();
    }

    // Line moved towards middle sensor
    if(L == 0 && M == 1)
    {
        Serial.println("CORRECT RIGHT");
        car.Turn_Right();
        // or car.RU_Move();
    }

    // No line detected
    else
    {
        Serial.println("STOP");
        car.Stop();
    }

    delay(20);
}