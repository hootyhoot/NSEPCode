#include <Arduino.h>
#include "MecanumCar_v2.h"
#include "IRremote.h"
#include <Servo.h>

mecanumCar car(3, 2);
IRrecv irrecv(A3);
decode_results results;
Servo gripper;

// ===== Pins =====
#define SensorLeft   A0
#define SensorMiddle A1
#define SensorRight  A2
#define EchoPin      13
#define TrigPin      12
#define SERVO_PIN    9

// ===== Buttons =====
#define BTN_1  0xFF6897
#define BTN_OK 0xFF02FD

// ===== Gripper =====
#define GRIP_OPEN  20
#define GRIP_CLOSE 85
int gripPos = GRIP_OPEN;

// ===== Speeds =====
#define SPEED_NORMAL  40
#define SPEED_SLOW    25
#define SLOW_AT_CM    6
#define GRAB_AT_CM    3

// Strafe left speeds — tune to correct diagonal drift and rotation.
//
// Forward/backward drift  → adjust the backward pair (UL+LR) vs forward pair (LL+UR)
//   Drifts FORWARD  → increase UL+LR, or decrease LL+UR
//   Drifts BACKWARD → decrease UL+LR, or increase LL+UR
//
// Rotation during strafe  → shift power between left side (UL+LL) and right side (UR+LR)
//   Rotates CCW → left side too strong: decrease UL or LL, or increase UR or LR
//   Rotates CW  → right side too strong: increase UL or LL, or decrease UR or LR
#define STRAFE_UL  41   // Upper Left  — goes BACKWARD during left strafe
#define STRAFE_LL  40   // Lower Left  — goes FORWARD  during left strafe
#define STRAFE_UR  40   // Upper Right — goes FORWARD  during left strafe
#define STRAFE_LR  45   // Lower Right — goes BACKWARD during left strafe

// ===== Strafe timing (tune per grid square size) =====
#define STRAFE_2_BLOCKS_MS 2450

// ===== Intersection debounce =====
#define INTERSECTION_DEBOUNCE_MS 200

// ===== Mission phases =====
enum Phase {
    PHASE_IDLE,
    PHASE_STRAIGHT_3,   // line follow, count 3 intersections then strafe
    PHASE_STRAFE_LEFT,  // timed strafe left 2 blocks
    PHASE_FORWARD_2,    // line follow, count 2 intersections then approach
    PHASE_APPROACH,     // slow approach — grab at GRAB_AT_CM
    PHASE_DONE
};

Phase phase = PHASE_IDLE;

// ===== Intersection state =====
int intersectionCount    = 0;
bool intersectionFlag    = false;
unsigned long leftIntersectionAt = 0;

// ===== Strafe state =====
unsigned long strafeStartAt = 0;

// ===== Helpers =====
int get_distance()
{
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    long d = pulseIn(EchoPin, HIGH, 30000);
    return d == 0 ? 999 : (int)(d / 58);
}

void setSpeed(uint8_t spd)
{
    speed_Upper_L = spd;
    speed_Lower_L = spd;
    speed_Upper_R = spd;
    speed_Lower_R = spd;
}

void openGripper()
{
    for(int pos = gripPos; pos >= GRIP_OPEN; pos--) { gripper.write(pos); delay(10); }
    gripPos = GRIP_OPEN;
    Serial.println("GRIPPER: OPEN");
}

void closeGripper()
{
    for(int pos = gripPos; pos <= GRIP_CLOSE; pos++) { gripper.write(pos); delay(10); }
    gripPos = GRIP_CLOSE;
    Serial.println("GRIPPER: CLOSED");
}

void resetIntersectionState()
{
    intersectionCount    = 0;
    intersectionFlag     = false;
    leftIntersectionAt   = 0;
}

void transitionTo(Phase next)
{
    phase = next;
    resetIntersectionState();

    switch(next)
    {
        case PHASE_STRAIGHT_3:  Serial.println("PHASE: straight x3");    break;
        case PHASE_STRAFE_LEFT: Serial.println("PHASE: strafe left x2");
                                strafeStartAt = millis();                 break;
        case PHASE_FORWARD_2:   Serial.println("PHASE: forward x2");     break;
        case PHASE_APPROACH:    Serial.println("PHASE: approach + grab"); break;
        case PHASE_DONE:        Serial.println("PHASE: done"); car.Stop();break;
        default: break;
    }
}

// Returns true if a new intersection was just counted
bool checkIntersection(int L, int M, int R)
{
    if(L == 1 && M == 1 && R == 1)
    {
        leftIntersectionAt = 0;
        if(!intersectionFlag)
        {
            intersectionCount++;
            intersectionFlag = true;
            Serial.print("INTERSECTION #"); Serial.println(intersectionCount);
            return true;
        }
    }
    else
    {
        if(intersectionFlag)
        {
            if(leftIntersectionAt == 0) leftIntersectionAt = millis();
            if(millis() - leftIntersectionAt >= INTERSECTION_DEBOUNCE_MS)
                intersectionFlag = false;
        }
    }
    return false;
}

void lineFollow(int L, int M, int R, uint8_t spd)
{
    setSpeed(spd);
    if(L == 0 && M == 1 && R == 0)   car.Advance();
    else if(L == 1 && R == 0)        car.Turn_Left();
    else if(L == 0 && R == 1)        car.Turn_Right();
    else                              car.Stop();
}

// ===== Setup =====
void setup()
{
    Serial.begin(9600);
    car.Init();
    gripper.attach(SERVO_PIN);
    gripper.write(GRIP_OPEN);

    setSpeed(SPEED_NORMAL);

    pinMode(EchoPin, INPUT);
    pinMode(TrigPin, OUTPUT);
    pinMode(SensorLeft,   INPUT);
    pinMode(SensorMiddle, INPUT);
    pinMode(SensorRight,  INPUT);

    irrecv.enableIRIn();

    Serial.println("=================");
    Serial.println("GRID MISSION");
    Serial.println("=================");
    Serial.println("BTN 1  = START");
    Serial.println("BTN OK = STOP");
}

// ===== Loop =====
void loop()
{
    // Remote control
    if(irrecv.decode(&results))
    {
        unsigned long key = results.value;
        if(key != 0xFFFFFFFF)
        {
            Serial.print("HEX=0x"); Serial.println(key, HEX);
            if(key == BTN_1)  transitionTo(PHASE_STRAIGHT_3);
            if(key == BTN_OK) { transitionTo(PHASE_IDLE); car.Stop(); Serial.println("STOPPED"); }
        }
        irrecv.resume();
    }

    if(phase == PHASE_IDLE) return;

    int L = digitalRead(SensorLeft);
    int M = digitalRead(SensorMiddle);
    int R = digitalRead(SensorRight);

    switch(phase)
    {
        // ── Phase 1: straight, count 3 intersections ──────────
        case PHASE_STRAIGHT_3:
        {
            checkIntersection(L, M, R);
            if(intersectionCount >= 3 && !intersectionFlag) {
                car.Stop();
                transitionTo(PHASE_STRAFE_LEFT);
            }
            else {
                lineFollow(L, M, R, SPEED_NORMAL);
            }
            break;
        }

        // ── Phase 2: timed strafe left 2 blocks ───────────────
        case PHASE_STRAFE_LEFT:
        {
            if(millis() - strafeStartAt < STRAFE_2_BLOCKS_MS)
            {
                // Direct motor control with independent speeds to correct
                // diagonal drift. UL+LR run backward, LL+UR run forward.
                // Raising the backward pair (UL/LR) counters forward drift.
                car.Motor_Upper_L(0, STRAFE_UL);
                car.Motor_Lower_L(1, STRAFE_LL);
                car.Motor_Upper_R(1, STRAFE_UR);
                car.Motor_Lower_R(0, STRAFE_LR);
            }
            else {
                car.Stop();
                transitionTo(PHASE_FORWARD_2);
            }
            break;
        }

        // ── Phase 3: forward, count 2 intersections ───────────
        case PHASE_FORWARD_2:
        {
            checkIntersection(L, M, R);
            if(intersectionCount >= 2 && !intersectionFlag)
                transitionTo(PHASE_APPROACH);
            else
                lineFollow(L, M, R, SPEED_NORMAL);
            break;
        }

        // ── Phase 4: slow approach and grab ───────────────────
        case PHASE_APPROACH:
        {
            int dist = get_distance();
            Serial.print("Dist="); Serial.println(dist);

            if(dist <= GRAB_AT_CM)
            {
                car.Stop();
                delay(200);
                closeGripper();
                transitionTo(PHASE_DONE);
            }
            else if(dist <= SLOW_AT_CM)
            {
                setSpeed(SPEED_SLOW);
                lineFollow(L, M, R, SPEED_SLOW);
            }
            else
            {
                lineFollow(L, M, R, SPEED_NORMAL);
            }
            break;
        }

        case PHASE_DONE:
        default:
            car.Stop();
            break;
    }

    delay(20);
}
