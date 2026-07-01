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
#define SPEED_NORMAL  50
#define SPEED_SLOW    25
#define SPEED_ROTATE  60
#define SLOW_AT_CM    8
#define GRAB_AT_CM    5

// ===== Turn tuning =====
// Don't check the right sensor at all for this long after the turn starts.
// Prevents the robot from immediately re-detecting its own starting line.
#define TURN_MIN_ROTATE_MS   250

// After the right sensor first sees the new line, coast this many ms before stopping.
// Helps the robot settle squarely onto the new line rather than clipping the edge.
#define TURN_LINE_SETTLE_MS  80

// ===== Intersection debounce =====
#define INTERSECTION_DEBOUNCE_MS 200

// After the turn completes, advance straight for this long to center
// the robot's body over the intersection (the turn only aligns the right sensor).
#define TURN_ALIGN_ADVANCE_MS   150

// ===== Mission phases =====
enum Phase {
    PHASE_IDLE,
    PHASE_STRAIGHT_3,    // line follow, count 3 intersections then turn
    PHASE_TURN_LEFT_90,  // pivot-rotate left until right sensor sees new line
    PHASE_FORWARD_2,     // line follow, count 2 intersections then approach
    PHASE_TURN_RIGHT_90, // pivot-rotate right until right sensor sees new line
    PHASE_APPROACH,      // slow approach — grab at GRAB_AT_CM
    PHASE_DROP,          // drop the object
    PHASE_TURN_180,     // rotate 180 degree
    PHASE_DONE
};

enum TurnDir { TURN_DIR_LEFT, TURN_DIR_RIGHT };

Phase phase = PHASE_IDLE;

// ===== Intersection state =====
int  intersectionCount       = 0;
bool intersectionFlag        = false;
unsigned long leftIntersectionAt = 0;

// ===== Turn state =====
unsigned long turnStartedAt      = 0;   // when PHASE_TURN_LEFT_90 began
bool          turnLineDetected   = false; // right sensor has hit the new line
unsigned long turnLineDetectedAt = 0;   // timestamp of that first detection

// ===== 180-turn state =====
int turn180Stage = 0;   // counts how many 90-deg segments completed (0, 1, 2)

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
    const float trimUL = 24.0 / 28.0; 
    const float trimLL = 21.0 / 28.0;
    const float trimUR = 28.0 / 28.0;
    const float trimLR = 27.0 / 28.0;

    speed_Upper_L = (uint8_t)(spd * trimUL);
    speed_Lower_L = (uint8_t)(spd * trimLL);
    speed_Upper_R = (uint8_t)(spd * trimUR);
    speed_Lower_R = (uint8_t)(spd * trimLR);
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
    intersectionCount  = 0;
    intersectionFlag   = false;
    leftIntersectionAt = 0;
}

void resetTurnState()
{
    turnStartedAt      = millis();   // clock starts NOW so the guard is accurate
}

void transitionTo(Phase next)
{
    phase = next;
    resetIntersectionState();
    resetTurnState();

    switch(next)
    {
        case PHASE_STRAIGHT_3:   Serial.println("PHASE: straight x3");           break;
        case PHASE_TURN_LEFT_90: Serial.println("PHASE: rotate left — waiting for right sensor"); break;
        case PHASE_FORWARD_2:    Serial.println("PHASE: forward x2");             break;
        case PHASE_TURN_RIGHT_90: Serial.println("PHASE: rotate right — waiting for right sensor"); break;
        case PHASE_APPROACH:     Serial.println("PHASE: approach + grab");        break;
        case PHASE_TURN_180:     Serial.println("PHASE: 180 turn (2x90)"); turn180Stage = 0; break;
        case PHASE_DROP:         Serial.println("PHASE: drop");                 break;
        case PHASE_DONE:         Serial.println("PHASE: done"); car.Stop();        break;
        default: break;
    }
}

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
    if     (L == 0 && M == 1 && R == 0)  car.Advance();
    else if(L == 1 && R == 0)            car.Turn_Left();
    else if(L == 0 && R == 1)            car.Turn_Right();
    else                                  car.Stop();
}

// Watches ONLY the right sensor during a left pivot-turn.
//
// Logic:
//   1. TURN_MIN_ROTATE_MS guard — ignore the right sensor completely for the
//      first 400 ms so the robot can't snap back onto the line it just left.
//   2. Once the guard expires, watch for the right sensor to go HIGH.
//   3. When it first goes HIGH, latch the timestamp (turnLineDetectedAt).
//   4. After TURN_LINE_SETTLE_MS ms of the sensor staying HIGH, stop and
//      return true — the robot is now squarely on the new line.
//
// Returns true when the turn is complete.
bool checkTurnLine(TurnDir dir)
{
    unsigned long now = millis();

    // Guard: minimum rotation time not yet elapsed
    if(now - turnStartedAt < TURN_MIN_ROTATE_MS) return false;

    int sensorVal = (dir == TURN_DIR_LEFT) ? digitalRead(SensorLeft)
                                            : digitalRead(SensorRight);

    if(sensorVal == HIGH)
    {
        car.Stop();
        Serial.println(dir == TURN_DIR_LEFT
            ? "TURN: 90 deg complete — right sensor on line"
            : "TURN: 90 deg complete — left sensor on line");
        return true;
    }


    return false;
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
            if(intersectionCount >= 4 && !intersectionFlag) {
                car.Stop();
                delay(50);
                transitionTo(PHASE_TURN_RIGHT_90);
            }
            else {

                lineFollow(L, M, R, SPEED_NORMAL);
            }
            break;
        }

        // ── Phase 2: pivot-rotate left, stop when right sensor sees new line ──
        case PHASE_TURN_LEFT_90:
        {
            if(checkTurnLine(TURN_DIR_LEFT))
            {
                delay(100);
                setSpeed(SPEED_SLOW);
                car.Advance();
                delay(TURN_ALIGN_ADVANCE_MS);
                car.Stop();
                delay(50);
                transitionTo(PHASE_APPROACH);
            }
            else
            {
                setSpeed(SPEED_ROTATE);
                car.Turn_Left();
            }
            break;
        }

        // ── Phase 3: forward, count 2 intersections ───────────
        case PHASE_FORWARD_2:
        {
            checkIntersection(L, M, R);
            if(intersectionCount >= 2 && !intersectionFlag) {
                car.Stop();
                delay(50);
                transitionTo(PHASE_TURN_LEFT_90);
            }
            else
                lineFollow(L, M, R, SPEED_NORMAL);
            break;
        }

        case PHASE_TURN_RIGHT_90:
        {
            if(checkTurnLine(TURN_DIR_RIGHT))
            {
                delay(100);
                setSpeed(SPEED_SLOW);
                car.Advance();
                delay(TURN_ALIGN_ADVANCE_MS);
                car.Stop();
                delay(50);
                transitionTo(PHASE_FORWARD_2);
            }
            else
            {
                setSpeed(SPEED_ROTATE);
                car.Turn_Right();
            }
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
                delay(50);
                transitionTo(PHASE_TURN_180);
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


        case PHASE_TURN_180:
        {
            setSpeed(SPEED_NORMAL);

            // ---- First 90-degree pivot ----
            car.Turn_Left();
            delay(TURN_MIN_ROTATE_MS); // ignore sensor briefly so it doesn't trigger on starting line

            while (true)
            {
                    int L = digitalRead(SensorLeft);
                    if (L == HIGH)
                        {
                        car.Stop();
                        Serial.println("TURN180: first 90 deg done");
                        break;
                        }
            }

            delay(50); // small settle pause before next pivot

            // ---- Second 90-degree pivot ----
            car.Turn_Left();
            delay(TURN_MIN_ROTATE_MS);

            while (true)
            {
                 int L = digitalRead(SensorLeft);
                if (L == HIGH)
                    {
                    car.Stop();
                    Serial.println("TURN180: second 90 deg done — 180 complete");
                    break;
                    }
            }

            transitionTo(PHASE_DROP);

            delay(100);
        }


        
        case PHASE_DROP:
        {
            openGripper();
            delay(50);
            transitionTo(PHASE_DONE);
            break;
        }

        case PHASE_DONE:
        default:
            car.Stop();
            break;
    }

    delay(20);
}