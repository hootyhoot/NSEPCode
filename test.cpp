#include <Arduino.h>
#include "MecanumCar_v2.h"
#include "IRremote.h"

// Robot
mecanumCar car(3, 2);

// IR Receiver
IRrecv irrecv(A3);
decode_results results;

#define EchoPin 13
#define TrigPin 12

int get_distance()
{
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(TrigPin, LOW);

    long duration = pulseIn(EchoPin, HIGH, 30000);

    if(duration == 0)
    {
        return 999; // No echo detected
    }

    return duration / 58;
}

// =====================
// Full NEC Codes
// =====================
#define BTN_UP      0xFF629D
#define BTN_LEFT    0xFF22DD
#define BTN_OK      0xFF02FD
#define BTN_RIGHT   0xFFC23D
#define BTN_DOWN    0xFFA857

#define BTN_1       0xFF6897
#define BTN_2       0xFF9867
#define BTN_3       0xFFB04F

#define BTN_4       0xFF30CF
#define BTN_5       0xFF18E7
#define BTN_6       0xFF7A85

#define BTN_7       0xFF10EF
#define BTN_8       0xFF38C7
#define BTN_9       0xFF5AA5

#define BTN_STAR    0xFF42BD
#define BTN_0       0xFF4AB5
#define BTN_HASH    0xFF52AD

void mission1()
{
    Serial.println("Mission 1 - Zig Zag Left");

    car.Advance();
    delay(1000);

    car.Turn_Left();
    delay(700);

    car.Advance();
    delay(1000);

    car.Stop();
}

void mission2()
{
    Serial.println("Mission 2 - Zig Zag Right");

    car.Advance();
    delay(1000);

    car.Turn_Right();
    delay(700);

    car.Advance();
    delay(1000);

    car.Stop();
}

void mission3()
{
    Serial.println("Mission 3 - Side Slide");

    car.L_Move();
    delay(1000);

    car.R_Move();
    delay(1000);

    car.L_Move();
    delay(1000);

    car.Stop();
}

void mission4()
{
    Serial.println("Mission 4 - Square");

    for(int i = 0; i < 4; i++)
    {
        car.Advance();
        delay(1000);

        car.Turn_Right();
        delay(580);
    }

    car.Stop();
}

void mission5()
{
    Serial.println("Mission 5 - Dance");

    car.L_Move();
    delay(500);

    car.R_Move();
    delay(500);

    car.Turn_Left();
    delay(500);

    car.Turn_Right();
    delay(500);

    car.Advance();
    delay(500);

    car.Back();
    delay(500);

    car.Stop();
}

void mission6()
{
    Serial.println("Mission 6 - Patrol");

    car.Advance();
    delay(1000);

    car.LU_Move();
    delay(1000);

    car.RU_Move();
    delay(1000);

    car.Back();
    delay(1000);

    car.Stop();
}

void mission7()
{
    Serial.println("Mission 7 - Triangle");

    for(int i = 0; i < 3; i++)
    {
        car.Advance();
        delay(1000);

        car.Turn_Right();
        delay(773);
    }

    car.Stop();
}

void mission8()
{
    Serial.println("Mission 8 - Follow Mode");
    Serial.println("Press OK to Exit");

    while(true)
    {
        int distance = get_distance();

        Serial.print("Distance = ");
        Serial.print(distance);
        Serial.println(" cm");

        if(distance == 999)
        {
            car.Stop();
        }
        else if(distance < 15)
        {
            car.Back();
        }
        else if(distance >= 15 && distance <= 25)
        {
            car.Stop();
        }
        else if(distance > 25 && distance <= 45)
        {
            car.Advance();
        }
        else
        {
            car.Stop();
        }

        // Check for exit command
        if(irrecv.decode(&results))
        {
            if(results.value == BTN_OK)
            {
                car.Stop();
                Serial.println("Exit Follow Mode");
                irrecv.resume();
                break;
            }

            irrecv.resume();
        }

        delay(100);
    }
}

// =====================
// Setup
// =====================
void setup()
{
    Serial.begin(9600);

    car.Init();

    speed_Upper_L = 150;
    speed_Lower_L = 150;
    speed_Upper_R = 150;
    speed_Lower_R = 150;

    pinMode(EchoPin, INPUT);
    pinMode(TrigPin, OUTPUT);

    irrecv.enableIRIn();

    Serial.println();
    Serial.println("==============================");
    Serial.println(" IR ROBOT READY");
    Serial.println("==============================");
    Serial.println("1 = Zig Zag Left");
    Serial.println("2 = Zig Zag Right");
    Serial.println("3 = Side Slide");
    Serial.println("4 = Square");
    Serial.println("5 = Dance");
    Serial.println("6 = Patrol");
    Serial.println("7 = Triangle");
    Serial.println("8 = Follow Mode");
    Serial.println("OK = Emergency Stop");
    Serial.println();
}

// =====================
// Main Loop
// =====================
void loop()
{
    if (irrecv.decode(&results))
    {
        unsigned long key = results.value;

        // Ignore NEC repeat code
        if (key != 0xFFFFFFFF)
        {
            Serial.print("HEX = 0x");
            Serial.println(key, HEX);

            switch(key)
            {
                case BTN_1:
                    mission1();
                    break;

                case BTN_2:
                    mission2();
                    break;

                case BTN_3:
                    mission3();
                    break;

                case BTN_4:
                    mission4();
                    break;

                case BTN_5:
                    mission5();
                    break;

                case BTN_6:
                    mission6();
                    break;

                case BTN_7:
                    mission7();
                    break;

                case BTN_8:
                    mission8();
                    break;

                case BTN_UP:
                    car.Advance();
                    delay(300);
                    car.Stop();
                    break;

                case BTN_DOWN:
                    car.Back();
                    delay(300);
                    car.Stop();
                    break;

                case BTN_LEFT:
                    car.Turn_Left();
                    delay(300);
                    car.Stop();
                    break;

                case BTN_RIGHT:
                    car.Turn_Right();
                    delay(300);
                    car.Stop();
                    break;

                case BTN_9:
                    car.L_Move();
                    delay(1000);
                    car.Stop();
                    break;

                case BTN_0:
                    car.R_Move();
                    delay(1000);
                    car.Stop();
                    break;

                case BTN_STAR:
                    car.LU_Move();
                    delay(1000);
                    car.Stop();
                    break;

                case BTN_HASH:
                    car.RU_Move();
                    delay(1000);
                    car.Stop();
                    break;

                case BTN_OK:
                    Serial.println("EMERGENCY STOP");
                    car.Stop();
                    break;

                default:
                    Serial.println("Unknown Button");
                    break;
            }
        }

        irrecv.resume();
    }
}