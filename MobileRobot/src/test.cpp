#include <Arduino.h>
#include "MecanumCar_v2.h"
#include "IRremote.h"
#include <Servo.h>


// Robot
mecanumCar car(3, 2);

// Line sensors
#define SensorLeft   A0
#define SensorMiddle A1
#define SensorRight  A2

// IR Receiver
IRrecv irrecv(A3);
decode_results results;


#define EchoPin 13
#define TrigPin 12

#define GRIP_OPEN   20
#define GRIP_CLOSE  85

// Gripper Servo
Servo gripper;

int gripPos = GRIP_OPEN;

void openGripper()
{
    Serial.println("Opening Gripper");

    for(int pos = gripPos; pos >= GRIP_OPEN; pos--)
    {
        gripper.write(pos);
        delay(10);
    }

    gripPos = GRIP_OPEN;
}

void closeGripper()
{
    Serial.println("Closing Gripper");

    for(int pos = gripPos; pos <= GRIP_CLOSE; pos++)
    {
        gripper.write(pos);
        delay(10);
    }

    gripPos = GRIP_CLOSE;
}


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
void mission9()
{
    Serial.println("Mission 9 - Auto Pick Object");
    Serial.println("Press OK to Cancel");

    openGripper();

    delay(1000);

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
        else if(distance > 30)
        {
            // Fast speed
            speed_Upper_L = 84;
            speed_Lower_L = 81;
            speed_Upper_R = 88;
            speed_Lower_R = 88;

            car.Advance();
        }
        else if(distance > 15)
        {
            // Medium speed
            speed_Upper_L = 64;
            speed_Lower_L = 61;
            speed_Upper_R = 68;
            speed_Lower_R = 68;

            car.Advance();
        }
        else if(distance > 5)
        {
            // Slow speed for accurate pickup
            speed_Upper_L = 39;
            speed_Lower_L = 36;
            speed_Upper_R = 43;
            speed_Lower_R = 43;

            car.Advance();
        }
        else if(distance > 3)
        {
            // Slow speed for accurate pickup
            speed_Upper_L = 15;
            speed_Lower_L = 15;
            speed_Upper_R = 15;
            speed_Lower_R = 15;

            car.Advance();
        }
        else
        {
            car.Stop();

            Serial.println("Object Reached");

            delay(500);

            closeGripper();

            Serial.println("Object Picked");

            delay(1000);

            // Move back after pickup
            speed_Upper_L = 50;
            speed_Lower_L = 50;
            speed_Upper_R = 50;
            speed_Lower_R = 50;

            car.Back();
            delay(1000);

            car.Stop();

            Serial.println("Mission Complete");

            break;
        }

        // Cancel mission with OK button
        if(irrecv.decode(&results))
        {
            if(results.value == BTN_OK)
            {
                car.Stop();

                Serial.println("Mission Cancelled");

                irrecv.resume();

                break;
            }

            irrecv.resume();
        }

        delay(100);
    }
}
void missionLineFollow()
{
    Serial.println("Line Follow Mode - Press OK to Exit");

    unsigned long lastPrint = 0;

    while(true)
    {
        int L = !digitalRead(SensorLeft);
        int M = !digitalRead(SensorMiddle);
        int R = !digitalRead(SensorRight);

        const char* direction;

        if((L == 0 && M == 1 && R == 0) || (L == 1 && M == 1 && R == 0) || (L == 0 && M == 1 && R == 1))
        {
            direction = "FORWARD";
            car.Motor_Upper_L(1, 60);
            car.Motor_Lower_L(1, 58);
            car.Motor_Upper_R(1, 65);
            car.Motor_Lower_R(1, 65);
        }
        else if(L == 1 && M == 1 && R == 1)
        {
            direction = "INTERSECTION";
            car.Motor_Upper_L(1, 60);
            car.Motor_Lower_L(1, 58);
            car.Motor_Upper_R(1, 65);
            car.Motor_Lower_R(1, 65);
        }
        else if(L == 1 && M == 0 && R == 0)
        {
            direction = "LEFT";
            car.Motor_Upper_L(0, 0);
            car.Motor_Lower_L(1, 100);
            car.Motor_Upper_R(1, 100);
            car.Motor_Lower_R(0, 0);
        }
        else if(L == 0 && M == 0 && R == 1)
        {
            direction = "RIGHT";
            car.Motor_Upper_L(1, 100);
            car.Motor_Lower_L(0, 0);
            car.Motor_Upper_R(0, 0);
            car.Motor_Lower_R(1, 100);
        }
        else
        {
            direction = "No line detected";
            car.Motor_Upper_L(0, 20);
            car.Motor_Lower_L(0, 20);
            car.Motor_Upper_R(0, 20);
            car.Motor_Lower_R(0, 20);
        }

        if(millis() - lastPrint >= 200)
        {
            Serial.print("L="); Serial.print(L);
            Serial.print(" M="); Serial.print(M);
            Serial.print(" R="); Serial.print(R);
            Serial.print(" -> "); Serial.println(direction);
            lastPrint = millis();
        }

        if(irrecv.decode(&results))
        {
            if(results.value == BTN_OK)
            {
                car.Stop();
                Serial.println("Exit Line Follow Mode");
                irrecv.resume();
                break;
            }
            irrecv.resume();
        }

        delay(20);
    }

}

void initState()
{
    Serial.println();
    Serial.println("=========================");
    Serial.println("INITIALIZATION STATE");
    Serial.println("=========================");

    bool ultrasonicOK = false;
    bool servoOK = false;
    bool motorOK = false;

    car.left_led(1);
    car.right_led(1);

    // -------------------------
    // Ultrasonic Test
    // -------------------------
    Serial.println("[CHECK] Ultrasonic");

    delay(1000);

    int distance = get_distance();

    ultrasonicOK = true;

    Serial.print("Ultrasonic Reading: ");
    Serial.print(distance);
    Serial.println(" cm");

    // -------------------------
    // Servo Test
    // -------------------------
    Serial.println("[CHECK] Gripper Servo");

    openGripper();
    delay(500);

    closeGripper();
    delay(500);

    openGripper();

    servoOK = true;

    Serial.println("[PASS] Servo");

    // -------------------------
    // Motor Test
    // -------------------------
    Serial.println("[CHECK] Motor Driver");

    car.Advance();
    delay(100);

    car.Stop();
    delay(100);

    car.Back();
    delay(100);

    car.Stop();
    delay(100);

    motorOK = true;

    Serial.println("[PASS] Motors");

        speed_Upper_L = 114;
        speed_Lower_L = 111;
        speed_Upper_R = 118;
        speed_Lower_R = 118;

    // -------------------------
    // Result
    // -------------------------
    if(ultrasonicOK && servoOK && motorOK)
    {
        Serial.println();
        Serial.println("SYSTEM STATUS : READY");

        car.left_led(0);
        car.right_led(0);
    }
    else
    {
        Serial.println();
        Serial.println("SYSTEM STATUS : FAULT");

        while(true)
        {
            car.left_led(1);
            car.right_led(1);
            delay(200);

            car.left_led(0);
            car.right_led(0);
            delay(200);
        }
    }

    Serial.println("=========================");
}



// =====================
// Setup
// =====================
void setup()
{
    Serial.begin(9600);

    car.Init();

    gripper.attach(9);
    gripper.write(GRIP_OPEN);

    speed_Upper_L = 114;
    speed_Lower_L = 111;
    speed_Upper_R = 118;
    speed_Lower_R = 118;

    pinMode(EchoPin, INPUT);
    pinMode(TrigPin, OUTPUT);

    pinMode(SensorLeft, INPUT);
    pinMode(SensorMiddle, INPUT);
    pinMode(SensorRight, INPUT);

    irrecv.enableIRIn();

    // Run self-audit automatically
    initState();

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
    Serial.println("9 = Auto Pick Object");
    Serial.println("0 = Open Gripper");
    Serial.println("# = Close Gripper");
    Serial.println("* = Line Follow Mode");
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
                
                case BTN_9:
                    mission9();
                    break;

                case BTN_UP:
                    car.Advance();
                    delay(3000);
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

                case BTN_0:
                    openGripper();
                    break;

                case BTN_HASH:
                    closeGripper();
                    break;

                case BTN_STAR:
                    missionLineFollow();
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