#include <Arduino.h>
#include <IRremote.h>
#include "MecanumCar_v2.h"

#define RECV_PIN         A3
#define EchoPin          13
#define TrigPin          12
#define STOP_DISTANCE_CM 20
#define DOUBLE_PRESS_MS  400

// ── Key codes (hex from IRremote) — fill blanks from Serial monitor ──
#define KEY_FORWARD      0xFF629D  // confirmed
#define KEY_BACKWARD     0xFFA857       // TODO
#define KEY_SLEFT        0xFF22DD       // TODO
#define KEY_SRIGHT       0xFFC23D       // TODO
#define KEY_ROTATE_CW    0xFF7A85       // TODO
#define KEY_ROTATE_CCW   0xFF30CF       // TODO
#define KEY_CIRCLE_CW    0xFF7A85       // TODO  (double-press KEY_SRIGHT)
#define KEY_CIRCLE_CCW   0xFF30CF       // TODO  (double-press KEY_SLEFT)
#define KEY_DRIFT_LEFT   0xFF42BD       // TODO
#define KEY_DRIFT_RIGHT  0xFF52AD       // TODO
#define KEY_STOP         0xFF02FD  // confirmed
#define KEY_UL           0xFF6897
#define KEY_UR           0xFFB04F
#define KEY_DL           0xFF10EF
#define KEY_DR           0xFF5AA5

IRrecv irrecv(RECV_PIN);
decode_results results;
mecanumCar robot(3, 2);

enum Move {
  MOVE_STOP,
  MOVE_FORWARD, MOVE_BACKWARD,
  MOVE_SLEFT,   MOVE_SRIGHT,
  MOVE_DIAG_NW, MOVE_DIAG_NE, MOVE_DIAG_SW, MOVE_DIAG_SE,
  MOVE_CIRCLE_CW, MOVE_CIRCLE_CCW,
  MOVE_ROTATE_CW, MOVE_ROTATE_CCW,
  MOVE_DRIFT_LEFT, MOVE_DRIFT_RIGHT
};

Move currentMove = MOVE_STOP;

unsigned long lastKey     = 0;
unsigned long lastKeyTime = 0;

// ── Ultrasonic ────────────────────────────────────────────────
float Get_Distance() {
  float dis;
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  dis = pulseIn(EchoPin, HIGH) / 58.2;
  delay(50);
  return dis;
}

bool forwardBlocked() {
  float d = Get_Distance();
  return d > 0 && d <= STOP_DISTANCE_CM;
}

// ── Custom circle movements ───────────────────────────────────
void circle_cw() {
  robot.Motor_Upper_L(1, speed_Upper_L);
  robot.Motor_Lower_L(1, speed_Lower_L);
  robot.Motor_Upper_R(0, 0);
  robot.Motor_Lower_R(0, 0);
}

void circle_ccw() {
  robot.Motor_Upper_L(0, 0);
  robot.Motor_Lower_L(0, 0);
  robot.Motor_Upper_R(1, speed_Upper_R);
  robot.Motor_Lower_R(1, speed_Lower_R);
}

// ── Movement dispatcher ───────────────────────────────────────
void handleKey(unsigned long key, bool isDouble) {
  Serial.print("Key: 0x"); Serial.print(key, HEX);
  Serial.println(isDouble ? " (double)" : " (single)");

  if (!isDouble) {
    if      (key == KEY_FORWARD)     currentMove = MOVE_FORWARD;
    else if (key == KEY_BACKWARD)    currentMove = MOVE_BACKWARD;
    else if (key == KEY_SLEFT)       currentMove = MOVE_SLEFT;
    else if (key == KEY_SRIGHT)      currentMove = MOVE_SRIGHT;
    else if (key == KEY_ROTATE_CW)   currentMove = MOVE_ROTATE_CW;
    else if (key == KEY_ROTATE_CCW)  currentMove = MOVE_ROTATE_CCW;
    else if (key == KEY_DRIFT_LEFT)  currentMove = MOVE_DRIFT_LEFT;
    else if (key == KEY_DRIFT_RIGHT) currentMove = MOVE_DRIFT_RIGHT;
    else if (key == KEY_UL)          currentMove = MOVE_DIAG_NW;
    else if (key == KEY_UR)          currentMove = MOVE_DIAG_NE;
    else if (key == KEY_DL)          currentMove = MOVE_DIAG_SW;
    else if (key == KEY_DR)          currentMove = MOVE_DIAG_SE;
    else if (key == KEY_STOP)      { currentMove = MOVE_STOP; robot.Stop(); }
  } else {
    // Double-press actions
    if      (key == KEY_ROTATE_CW)  currentMove = MOVE_CIRCLE_CW;
    else if (key == KEY_ROTATE_CCW) currentMove = MOVE_CIRCLE_CCW;
  }
}

void applyMove() {
  switch (currentMove) {
    case MOVE_FORWARD:
      if (forwardBlocked()) robot.Stop();
      else                  robot.Advance();
      break;
    case MOVE_BACKWARD:    robot.Back();        break;
    case MOVE_SLEFT:       robot.L_Move();      break;
    case MOVE_SRIGHT:      robot.R_Move();      break;
    case MOVE_DIAG_NW:     robot.LU_Move();     break;
    case MOVE_DIAG_NE:     robot.RU_Move();     break;
    case MOVE_DIAG_SW:     robot.LD_Move();     break;
    case MOVE_DIAG_SE:     robot.RD_Move();     break;
    case MOVE_CIRCLE_CW:   circle_cw();         break;
    case MOVE_CIRCLE_CCW:  circle_ccw();        break;
    case MOVE_ROTATE_CW:   robot.Turn_Right();  break;
    case MOVE_ROTATE_CCW:  robot.Turn_Left();   break;
    case MOVE_DRIFT_LEFT:  robot.drift_left();  break;
    case MOVE_DRIFT_RIGHT: robot.drift_right(); break;
    case MOVE_STOP:
    default:               robot.Stop();        break;
  }
}

// ── Arduino entry points ──────────────────────────────────────
void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);
  robot.Init();
  Serial.println("Ready.");
}

void loop() {
  if (irrecv.decode(&results)) {
    unsigned long val = results.value;
    irrecv.resume();

    if (val == 0xFFFFFFFF) return;  // ignore NEC repeat frames

    unsigned long now = millis();
    bool isDouble = (val == lastKey && (now - lastKeyTime) < DOUBLE_PRESS_MS);
    lastKey     = val;
    lastKeyTime = now;

    handleKey(val, isDouble);
  }

  applyMove();
}
