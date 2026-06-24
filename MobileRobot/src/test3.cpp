#include <Arduino.h>
#include "MecanumCar_v2.h"
#include "IRremote.h"

mecanumCar car(3, 2);

// ===== Sensors =====
#define SensorLeft   A0
#define SensorMiddle A1
#define SensorRight  A2

// ===== IR =====
IRrecv irrecv(A3);
decode_results results;

#define BTN_1  0xFF6897
#define BTN_OK 0xFF02FD

// ===== Variables =====
bool missionRunning = false;

int intersectionCount = 0;
bool intersectionFlag = false;

void startMission()
{
    missionRunning = true;

    intersectionCount = 0;
    intersectionFlag = false;

    Serial.println("MISSION STARTED");
}

void stopMission()
{
    missionRunning = false;

    car.Stop();

    Serial.println("MISSION STOPPED");
}

void setup()
{
    Serial.begin(9600);

    car.Init();

    car.left_led(1);
    car.right_led(1);

    // Smooth speed
    // speed_Upper_L = 45;
    // speed_Lower_L = 45;
    // speed_Upper_R = 45;
    // speed_Lower_R = 45;

    pinMode(SensorLeft, INPUT);
    pinMode(SensorMiddle, INPUT);
    pinMode(SensorRight, INPUT);

    irrecv.enableIRIn();

    Serial.println("=================");
    Serial.println("GRID NAVIGATION");
    Serial.println("=================");
    Serial.println("BTN 1  = START");
    Serial.println("BTN OK = STOP");
}

void loop()
{
    // ==========================
    // REMOTE CONTROL
    // ==========================
    if(irrecv.decode(&results))
    {
        unsigned long key = results.value;

        if(key != 0xFFFFFFFF)
        {
            Serial.print("HEX = 0x");
            Serial.println(key, HEX);

            switch(key)
            {
                case BTN_1:
                    startMission();
                    break;

                case BTN_OK:
                    stopMission();
                    break;
            }
        }

        irrecv.resume();
    }

    // ==========================
    // WAIT FOR START
    // ==========================
    if(!missionRunning)
    {
        return;
    }

    // ==========================
    // SENSOR READING
    // ==========================
    int L = digitalRead(SensorLeft);
    int M = digitalRead(SensorMiddle);
    int R = digitalRead(SensorRight);

    Serial.print("L=");
    Serial.print(L);
    Serial.print(" M=");
    Serial.print(M);
    Serial.print(" R=");
    Serial.println(R);

    // ==========================
    // INTERSECTION
    // ==========================
    if(L == 1 && M == 1 && R == 1)
    {
        if(!intersectionFlag)
        {
            intersectionCount++;

            Serial.print("INTERSECTION #");
            Serial.println(intersectionCount);

            intersectionFlag = true;
        }

        if(intersectionCount >= 5 && intersectionFlag)
        {
            Serial.println("5TH INTERSECTION");
            Serial.println("TURN RIGHT");

            car.Stop();
            delay(200);

            car.Advance();
            delay(100);

            car.Turn_Right();
            delay(780);

            car.Stop();

            car.L_Move();
            delay(3000);

            car.Stop();
            delay(200);

            exit(0);

            intersectionCount = 0;
        }
        else
        {
            car.Stop();
        }
    }
    else
    {
        intersectionFlag = false;

        // ==========================
        // FORWARD
        // ==========================
        if(L == 0 && M == 1 && R == 0)
        {
            car.Advance();
        }

        // ==========================
        // LEFT CORRECTION
        // ==========================
        else if(L == 1 && R == 0)
        {
            car.Turn_Left();
        }

        // ==========================
        // RIGHT CORRECTION
        // ==========================
        else if(L == 0 && R == 1)
        {
            car.Turn_Right();
        }

        // ==========================
        // LINE LOST
        // ==========================
        else
        {
            car.Stop();
        }
    }

    delay(20);
}
