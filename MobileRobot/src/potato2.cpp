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
#define GRAB_AT_CM    3

// Inner-wheel speed as a percentage of outer-wheel speed during line-follow
// correction. Lower = gentler steer, less zigzag, slower recovery.
// Higher = snappier correction, more prone to overshoot.
#define SOFT_TURN_PCT 35

// Per-wheel calibration trims (measured motor speed mismatch)
const float trimUL = 24.0 / 28.0;
const float trimLL = 21.0 / 28.0;
const float trimUR = 28.0 / 28.0;
const float trimLR = 27.0 / 28.0;

// ===== Turn tuning =====
// Don't check sensors at all for this long after the turn starts (blind, fast spin).
// Prevents the robot from immediately re-detecting its own starting line.
#define TURN_MIN_ROTATE_MS   250

// Rotation speed during the blind window — can be fast since sensors aren't watched yet.
#define SPEED_ROTATE_BLIND   SPEED_ROTATE

// Rotation speed during the closed-loop seek window — kept slow so the robot travels
// only a small angle between sensor polls, preventing the 90°->120°+ overshoot.
#define SPEED_ROTATE_SEEK    28

// How often to poll sensors during the seek window (ms). Small = tighter feedback loop.
#define TURN_POLL_MS          5

// Once a sensor first reports the new line, it must stay reporting it for this long
// (continuously) before the turn is considered complete. Filters out single-sample noise.
#define TURN_LINE_SETTLE_MS  60

// Safety cutoff so a missed line can't spin the robot forever.
#define TURN_SEEK_TIMEOUT_MS 3000

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
    speed_Upper_L = (uint8_t)(spd * trimUL);
    speed_Lower_L = (uint8_t)(spd * trimLL);
    speed_Upper_R = (uint8_t)(spd * trimUR);
    speed_Lower_R = (uint8_t)(spd * trimLR);
}

// Gentle differential steer for line-follow correction: outer side runs at
// full commanded speed, inner side is slowed (not reversed) to bank the
// robot back onto the line without the overshoot a full pivot turn causes.
void softSteer(uint8_t spd, bool steerLeft)
{
    uint8_t outer = spd;
    uint8_t inner = (uint8_t)((uint32_t)spd * SOFT_TURN_PCT / 100);

    if(steerLeft)
    {
        car.Motor_Upper_L(1, (uint8_t)(inner * trimUL));
        car.Motor_Lower_L(1, (uint8_t)(inner * trimLL));
        car.Motor_Upper_R(1, (uint8_t)(outer * trimUR));
        car.Motor_Lower_R(1, (uint8_t)(outer * trimLR));
    }
    else
    {
        car.Motor_Upper_L(1, (uint8_t)(outer * trimUL));
        car.Motor_Lower_L(1, (uint8_t)(outer * trimLL));
        car.Motor_Upper_R(1, (uint8_t)(inner * trimUR));
        car.Motor_Lower_R(1, (uint8_t)(inner * trimLR));
    }
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
    if     (L == 0 && M == 1 && R == 0)  { setSpeed(spd); car.Advance(); }
    else if(L == 1 && R == 0)            softSteer(spd, true);
    else if(L == 0 && R == 1)            softSteer(spd, false);
    else                                  car.Stop();
}

// Closed-loop 90 deg pivot turn using all 3 forward IR sensors.
//
// Stage 1 (blind): rotate fast for TURN_MIN_ROTATE_MS so the robot can't
// immediately re-detect the line it just left.
//
// Stage 2 (seek): rotate slowly and poll sensors every TURN_POLL_MS. Stopping
// is decided by:
//   - the DIRECTIONAL sensor (Left for a left turn, Right for a right turn)
//     going HIGH — this is the expected, on-target detection.
//   - the MIDDLE sensor going HIGH as a fallback — catches the case where the
//     turn overshot slightly and the directional sensor swept past the line
//     before the front-center sensor crossed it, so the robot doesn't spin
//     past the line entirely.
// Either trigger must stay HIGH continuously for TURN_LINE_SETTLE_MS before
// the turn is accepted, filtering out single-sample noise/glitches.
//
// Blocking by design — matches the rest of this file's phase-transition style
// (see the delay()-based sequences elsewhere) and keeps the seek poll rate
// independent of the main loop's ~20ms cadence, which is what caused the
// original overshoot (too much angle traveled between checks at full speed).
bool performTurn90(TurnDir dir)
{
    // Stage 1: blind fast spin
    setSpeed(SPEED_ROTATE_BLIND);
    if(dir == TURN_DIR_LEFT) car.Turn_Left(); else car.Turn_Right();
    delay(TURN_MIN_ROTATE_MS);

    // Stage 2: slow closed-loop seek
    setSpeed(SPEED_ROTATE_SEEK);
    if(dir == TURN_DIR_LEFT) car.Turn_Left(); else car.Turn_Right();

    unsigned long seekStart   = millis();
    unsigned long settleStart = 0;
    bool          settling    = false;

    while(millis() - seekStart < TURN_SEEK_TIMEOUT_MS)
    {
        int L = digitalRead(SensorLeft);
        int M = digitalRead(SensorMiddle);
        int R = digitalRead(SensorRight);

        bool targetHit = (dir == TURN_DIR_LEFT) ? (L == HIGH) : (R == HIGH);
        bool centerHit = (M == HIGH);

        if(targetHit || centerHit)
        {
            if(!settling) { settling = true; settleStart = millis(); }
            if(millis() - settleStart >= TURN_LINE_SETTLE_MS)
            {
                car.Stop();
                Serial.println(dir == TURN_DIR_LEFT
                    ? "TURN LEFT: 90 deg complete"
                    : "TURN RIGHT: 90 deg complete");
                return true;
            }
        }
        else
        {
            settling = false;  // lost the line before settle completed — false alarm
        }

        delay(TURN_POLL_MS);
    }

    car.Stop();
    Serial.println("TURN: timeout — line not found");
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
                intersectionCount = 0;
                transitionTo(PHASE_TURN_RIGHT_90);
            }
            else {

                lineFollow(L, M, R, SPEED_NORMAL);
            }
            break;
        }

        // ── Phase 2: closed-loop pivot-rotate left until left sensor sees new line ──
        case PHASE_TURN_LEFT_90:
        {
            if(performTurn90(TURN_DIR_LEFT))
            {
                delay(100);
                setSpeed(SPEED_SLOW);
                car.Advance();
                delay(TURN_ALIGN_ADVANCE_MS);
                car.Stop();
                delay(50);
                transitionTo(PHASE_APPROACH);
            }
            // else: timed out — stays in this phase, retries next loop() call
            break;
        }

        // ── Phase 3: forward, count 2 intersections ───────────
        case PHASE_FORWARD_2:
        {
            checkIntersection(L, M, R);
            if(intersectionCount >= 2 && !intersectionFlag) {
                car.Stop();
                delay(50);
                intersectionCount = 0;
                transitionTo(PHASE_TURN_LEFT_90);
            }
            else
                lineFollow(L, M, R, SPEED_NORMAL);
            break;
        }

        case PHASE_TURN_RIGHT_90:
        {
            if(performTurn90(TURN_DIR_RIGHT))
            {
                delay(100);
                setSpeed(SPEED_SLOW);
                car.Advance();
                delay(TURN_ALIGN_ADVANCE_MS);
                car.Stop();
                delay(50);
                transitionTo(PHASE_FORWARD_2);
            }
            // else: timed out — stays in this phase, retries next loop() call
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

            for(int i=0; i < 2; i++) {
                car.Turn_Left();
                delay(TURN_MIN_ROTATE_MS);
                bool turnFlag = false;
                while (!turnFlag) {
                    int L = digitalRead(SensorLeft);
                    if (L == HIGH) {
                        turnFlag = true;
                        car.Stop();
                        Serial.println("TURN180: first 90 deg done");
                        delay(50);
                    }
                }
            }


            transitionTo(PHASE_DROP);
            break;

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