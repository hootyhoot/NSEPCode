#include <Arduino.h>
#include "MecanumCar_v2.h"
#include "IRremote.h"
#include <Servo.h>

// ============================================================
//  HARDWARE OBJECTS
// ============================================================
mecanumCar car(3, 2);
IRrecv irrecv(A3);
decode_results results;
Servo gripper;

// ============================================================
//  PINS
// ============================================================
#define SensorLeft   A0
#define SensorMiddle A1
#define SensorRight  A2
#define EchoPin      13
#define TrigPin      12
#define SERVO_PIN    9

// ============================================================
//  REMOTE BUTTONS
// ============================================================
#define BTN_1  0xFF6897   // start the mission
#define BTN_OK 0xFF02FD   // stop / reset

// ============================================================
//  GRIPPER
// ============================================================
#define GRIP_OPEN  20
#define GRIP_CLOSE 85
int gripPos = GRIP_OPEN;

// ============================================================
//  SPEEDS
// ============================================================
#define SPEED_NORMAL 50   // normal line-follow speed
#define SPEED_SLOW   25   // slowest speed, used right next to the object
   // speed while pivoting 90 degrees

// Distance window for the slow approach. Outside APPROACH_START_CM the
// robot drives at SPEED_NORMAL. Inside it, speed scales down smoothly
// toward SPEED_SLOW as the object gets closer. Below GRAB_AT_CM it stops
// and grabs.
#define APPROACH_START_CM 20
#define GRAB_AT_CM         3

// Per-wheel calibration trims — the 4 wheels don't spin at exactly the
// same real speed for the same PWM value, so we scale each one to match.
const float trimUL = 36.0 / 40.0;
const float trimLL =  31.0 / 40.0;
const float trimUR = 40.0 / 40.0;
const float trimLR = 38.0 / 40.0;




// ============================================================
//  MISSION PHASES
// ============================================================
enum Phase {
    PHASE_IDLE,      // waiting for start button
    PHASE_OUTBOUND,  // drive straight, counting junctions, toward the object
    PHASE_APPROACH,  // slow approach to the object, then grab it
    PHASE_TURN180,   // spin around to face back the way we came
    PHASE_RETURN,    // drive straight, counting junctions, back to the drop point
    PHASE_DROP,      // release the object
    PHASE_DONE       // mission complete, stay stopped
};
Phase phase = PHASE_IDLE;

enum TurnDir { TURN_DIR_LEFT, TURN_DIR_RIGHT };

// ============================================================
//  INTERSECTION COUNTING STATE
// ============================================================
int  intersectionCount = 0;     // how many intersections crossed this segment
bool junction    = false; // true while all 3 sensors currently see black

// ============================================================
//  MOTOR SPEED
//  mecanumCar reads these 4 globals internally whenever you call
//  car.Advance() / car.Turn_Left() / etc, so "setting the speed"
//  just means writing to them (scaled by each wheel's trim).
// ============================================================
void setSpeed(uint8_t spd)
{
    speed_Upper_L = (uint8_t)(spd * trimUL);
    speed_Lower_L = (uint8_t)(spd * trimLL);
    speed_Upper_R = (uint8_t)(spd * trimUR);
    speed_Lower_R = (uint8_t)(spd * trimLR);
}



// ============================================================
//  GRIPPER
// ============================================================
void openGripper()
{
    for (int pos = gripPos; pos >= GRIP_OPEN; pos--) { gripper.write(pos); delay(10); }
    gripPos = GRIP_OPEN;
    Serial.println("GRIPPER: OPEN");
}

void closeGripper()
{
    for (int pos = gripPos; pos <= GRIP_CLOSE; pos++) { gripper.write(pos); delay(10); }
    gripPos = GRIP_CLOSE;
    Serial.println("GRIPPER: CLOSED");
}

// ============================================================
//  DISTANCE SENSOR (HC-SR04 style ultrasonic)
// ============================================================
int getDistanceCm()
{
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    long d = pulseIn(EchoPin, HIGH, 30000);
    return d == 0 ? 999 : (int)(d / 58);
}

// Maps a distance reading to a motor speed: far away = SPEED_NORMAL,
// right next to the object = SPEED_SLOW, smoothly in between.
uint8_t approachSpeed(int dist)
{
    if (dist >= APPROACH_START_CM) return SPEED_NORMAL;
    if (dist <= GRAB_AT_CM)        return SPEED_SLOW;
    return (uint8_t)map(dist, GRAB_AT_CM, APPROACH_START_CM, SPEED_SLOW, SPEED_NORMAL);
}

// ============================================================
//  LINE FOLLOWING
//  L / M / R are the 3 forward IR sensors: 1 = sees black line.
//  Steer to keep the MIDDLE sensor on the line.




// ============================================================
void lineFollow(int L, int M, int R, uint8_t spd)
{
    if (L == 1 && M == 1 && R == 1) {
        setSpeed(spd);              // junction — drive straight through
        car.Advance();
    } else if (M == 1 && L == 0 && R == 0) {
        setSpeed(spd);
        car.Advance();
    } else if (L == 1 && M == 0 && R == 0) {
        setSpeed(spd * 0.6);
        car.Turn_Left();
    } else if (R == 1 && M == 0 && L == 0) {
        setSpeed(spd * 0.6);
        car.Turn_Right();
    } else if (L == 1 && M == 1 && R == 0) {
        setSpeed(spd * 0.8);
        car.Turn_Left();
    } else if (R == 1 && M == 1 && L == 0) {
        setSpeed(spd * 0.8);
        car.Turn_Right();
    } else {
        setSpeed(spd * 0.5);        // line lost
        car.Advance();
    }
}

// ============================================================
//  INTERSECTION DETECTION
//  An intersection = all 3 sensors see black at once.
//  onIntersection stops us counting the same intersection twice
//  while we're still physically sitting on top of it.
// ============================================================
void checkIntersection(int L, int M, int R)
{
    

    if (L ==1 && M == 1 && R == 1 && !junction) {
        intersectionCount++;
        junction = true;
        
    } else if (L==0 || M==0 || R==0) {
        junction = false;   // left the intersection — ready to count the next one
    }
}

// ============================================================
//  90 DEGREE PIVOT TURN
//  Spins in place until the sensor on the turn's target side
//  (left sensor for a left turn, right sensor for a right turn)
//  sees the new line, then stops.
//  Blocking — robot does nothing else while this runs.
// ============================================================
bool performTurn90(TurnDir dir)
{
    setSpeed(55);
    if (dir == TURN_DIR_LEFT) car.Turn_Left(); else car.Turn_Right();

    // Ignore sensors briefly — we start this turn already sitting on a
    // line, so checking immediately would just re-detect that same line.
    delay(200);

    unsigned long start = millis();

    while (millis() - start < 3000)
    {
        int L = digitalRead(SensorLeft);
        int R = digitalRead(SensorRight);

        bool foundLine = (dir == TURN_DIR_LEFT) ? (L == HIGH) : (R == HIGH);

        if (foundLine) {
            car.Stop();
            Serial.println("TURN: complete");
            return true;
        }
    }

    car.Stop();
    Serial.println("TURN: timeout — line not found");
    return false;
}

// ============================================================
//  PHASE TRANSITIONS
//  Call this whenever the mission moves to a new phase. Resets
//  the per-segment counters so each new phase starts clean.
// ============================================================
void transitionTo(Phase next)
{
    phase = next;
    intersectionCount = 0;
    junction = false;

    switch (next) {
        case PHASE_OUTBOUND: Serial.println("PHASE: outbound (5 junctions)"); break;
        case PHASE_APPROACH: Serial.println("PHASE: approach + grab");        break;
        case PHASE_TURN180:  Serial.println("PHASE: 180 turn");               break;
        case PHASE_RETURN:   Serial.println("PHASE: return (5 junctions)");   break;
        case PHASE_DROP:     Serial.println("PHASE: drop");                   break;
        case PHASE_DONE:     Serial.println("PHASE: done"); car.Stop();       break;
        default: break;
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(9600);
    car.Init();
    gripper.attach(SERVO_PIN);
    gripper.write(GRIP_OPEN);

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

// ============================================================
//  MAIN LOOP
// ============================================================
void loop()
{
    // --- remote control: works no matter which phase we're in ---
    if (irrecv.decode(&results))
    {
        unsigned long key = results.value;
        if (key != 0xFFFFFFFF) {
            Serial.print("HEX=0x"); Serial.println(key, HEX);
            if (key == BTN_1)  transitionTo(PHASE_OUTBOUND);
            if (key == BTN_OK) { transitionTo(PHASE_IDLE); car.Stop(); Serial.println("STOPPED"); }
        }
        irrecv.resume();
    }

    if (phase == PHASE_IDLE) return;   // nothing else to do until started

    int L = digitalRead(SensorLeft);
    int M = digitalRead(SensorMiddle);
    int R = digitalRead(SensorRight);

    switch (phase)
    {
        // ---- drive straight toward the object, counting junctions ----
        case PHASE_OUTBOUND:
            checkIntersection(L, M, R);
            if (intersectionCount >= 5 && !junction) {
                car.Stop();
                transitionTo(PHASE_APPROACH);
            } else {
                lineFollow(L, M, R, SPEED_NORMAL);
            }
            break;

        // ---- creep toward the object, slowing smoothly, grab when close ----
        case PHASE_APPROACH:
        {
            int dist = getDistanceCm();
            Serial.print("Dist="); Serial.println(dist);

            if (dist <= GRAB_AT_CM) {
                car.Stop();
                delay(200);
                closeGripper();
                transitionTo(PHASE_TURN180);
            } else {
                lineFollow(L, M, R, approachSpeed(dist));
            }
            break;
        }

        // ---- two 90 degree turns back-to-back = a 180 ----
        case PHASE_TURN180:
        {
            if (performTurn90(TURN_DIR_LEFT)) {
                if (performTurn90(TURN_DIR_LEFT)) {
                    transitionTo(PHASE_RETURN);
                }
            }
            break;
        }

        // ---- drive straight back to the drop point, counting junctions ----
        case PHASE_RETURN:
            checkIntersection(L, M, R);
            if (intersectionCount >= 7 && !junction) {
                car.Stop();
                transitionTo(PHASE_DROP);
            } else {
                lineFollow(L, M, R, SPEED_NORMAL);
            }
            break;

        case PHASE_DROP:
            openGripper();
            transitionTo(PHASE_DONE);
            break;

        case PHASE_DONE:
        default:
            car.Stop();
            break;
    }

    delay(20);
}
