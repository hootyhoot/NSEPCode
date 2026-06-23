#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LittleFS.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int GLOBAL_MIN_PULSE = 200;
const int GLOBAL_MAX_PULSE = 3000;
const int MAX_ANGLE        = 270;
const int NUM_SERVOS       = 6;
const int NUM_POSES        = 18;   // a-r

#define POSES_FILE "/poses.csv"
#define SEQ_FILE "/sequences.csv"

const int NUM_SEQUENCES = 5;
const int MAX_SEQ_STEPS = 10;
const int SEQ_PAUSE_MS  = 400;

struct PoseSequence {
    int steps[MAX_SEQ_STEPS];
    int stepCount;
};

PoseSequence sequences[NUM_SEQUENCES];
bool sequenceSet[NUM_SEQUENCES] = {false};

const int CH_SHOULDER  = 0;
const int CH_ELBOW     = 1;
const int CH_WRIST     = 2;
const int CH_WRIST_ROT = 3;
const int CH_GRIPPER   = 4;
const int CH_BASE      = 5;

const char* JOINT_NAMES[NUM_SERVOS] = {
    "Shoulder", "Elbow", "Wrist", "Wrist-Rot", "Gripper", "Base"
};

const int HOME_STEP  = 5;
const int HOME_DELAY = 20;
const int POSE_STEP  = 5;
const int POSE_DELAY = 15;

int HOME_PWM[NUM_SERVOS] = {1795, 1460, 1510, 1500, 790, 1500};

const int jointMin[NUM_SERVOS] = {425,  720,  315,  320,  790,  330};
const int jointMax[NUM_SERVOS] = {2110, 2720, 2695, 2720, 1595, 2640};

int activeChannel = 0;
int stepSize      = 5;
int currentPWM[NUM_SERVOS]            = {1500, 1500, 1500, 1500, 1500, 1500};
int savedPoses[NUM_POSES][NUM_SERVOS] = {0};
bool poseSet[NUM_POSES]               = {false};

//
enum RobotStatus {
    STATUS_INIT,
    STATUS_IDLE,
    STATUS_BUSY
};

RobotStatus robotStatus = STATUS_INIT;

// Call whenever the arm starts a motion
void setStatusBusy(const char* reason) {
    robotStatus = STATUS_BUSY;
    Serial.print("[BUSY] "); Serial.println(reason);
}

// Call whenever a motion finishes
void setStatusIdle() {
    robotStatus = STATUS_IDLE;
    Serial.println("[IDLE] Ready.");
}

// Returns true and prints a warning if a motion command should be blocked
bool isBusy() {
    if (robotStatus == STATUS_BUSY) {
        Serial.println(">>> ARM IS BUSY — wait for [IDLE] or press Q to stop.");
        return true;
    }
    return false;
}

// ── Emergency stop flag ───────────────────────────────────────────────────────
// Set to true by 'Z' in loop(); checked inside long motion loops to abort early.
volatile bool eStop = false;

// ── Input state machine ───────────────────────────────────────────────────────
enum InputState {
    IDLE_INPUT,
    WAIT_POSE_SLOT,
    WAIT_SEQ_SLOT,
    WAIT_SEQ_POSES
};

InputState inputState    = IDLE_INPUT;
int        pendingSeqIdx = -1;
String     inputBuffer   = "";

// ── Helpers ───────────────────────────────────────────────────────────────────

int poseIndex(char c) { return c - 'a'; }

int clampToJointLimits(int channel, int value) {
    return constrain(value, jointMin[channel], jointMax[channel]);
}

// ── Flash – poses ─────────────────────────────────────────────────────────────

void persistPoses() {
    File f = LittleFS.open(POSES_FILE, "w");
    if (!f) { Serial.println("Error: cannot open poses file for writing."); return; }
    for (int p = 0; p < NUM_POSES; p++) {
        if (!poseSet[p]) continue;
        f.print((char)('a' + p));
        for (int i = 0; i < NUM_SERVOS; i++) { f.print(','); f.print(savedPoses[p][i]); }
        f.println();
    }
    f.close();
    Serial.println("Poses saved to flash.");
}

bool loadPosesFromFlash() {
    if (!LittleFS.exists(POSES_FILE)) { Serial.println("No saved poses on flash."); return false; }
    File f = LittleFS.open(POSES_FILE, "r");
    if (!f) { Serial.println("Error: cannot open poses file."); return false; }
    int n = 0;
    while (f.available()) {
        String line = f.readStringUntil('\n'); line.trim();
        if (!line.length()) continue;
        char slot = line.charAt(0);
        if (slot < 'a' || slot >= ('a' + NUM_POSES)) continue;
        int idx = poseIndex(slot);
        int comma = line.indexOf(','); if (comma < 0) continue;
        line = line.substring(comma + 1);
        int i = 0;
        while (i < NUM_SERVOS) {
            comma = line.indexOf(',');
            String tok = (comma >= 0) ? line.substring(0, comma) : line;
            savedPoses[idx][i++] = tok.toInt();
            if (comma < 0) break;
            line = line.substring(comma + 1);
        }
        if (i == NUM_SERVOS) { poseSet[idx] = true; n++; }
    }
    f.close();
    Serial.print("Loaded "); Serial.print(n); Serial.println(" pose(s) from flash.");
    return n > 0;
}

void clearPoses() {
    for (int i = 0; i < NUM_POSES; i++) poseSet[i] = false;
    LittleFS.remove(POSES_FILE);
    Serial.println("Cleared all poses (RAM + flash).");
}

// ── Flash – sequences ─────────────────────────────────────────────────────────

void persistSequences() {
    File f = LittleFS.open(SEQ_FILE, "w");
    if (!f) { Serial.println("Error: cannot open sequence file for writing."); return; }
    for (int s = 0; s < NUM_SEQUENCES; s++) {
        if (!sequenceSet[s]) continue;
        f.print((char)('A' + s));
        for (int i = 0; i < sequences[s].stepCount; i++) {
            f.print(','); f.print((char)('a' + sequences[s].steps[i]));
        }
        f.println();
    }
    f.close();
    Serial.println("Sequences saved to flash.");
}

bool loadSequencesFromFlash() {
    if (!LittleFS.exists(SEQ_FILE)) { Serial.println("No saved sequences on flash."); return false; }
    File f = LittleFS.open(SEQ_FILE, "r");
    if (!f) { Serial.println("Error: cannot open sequence file."); return false; }
    int n = 0;
    while (f.available()) {
        String line = f.readStringUntil('\n'); line.trim();
        if (!line.length()) continue;
        char slot = line.charAt(0);
        if (slot < 'A' || slot >= ('A' + NUM_SEQUENCES)) continue;
        int idx = slot - 'A';
        int comma = line.indexOf(','); if (comma < 0) continue;
        line = line.substring(comma + 1);
        int count = 0;
        while (line.length() && count < MAX_SEQ_STEPS) {
            comma = line.indexOf(',');
            String tok = (comma >= 0) ? line.substring(0, comma) : line; tok.trim();
            if (tok.length()) {
                char pc = tolower(tok.charAt(0));
                if (pc >= 'a' && pc < ('a' + NUM_POSES)) sequences[idx].steps[count++] = pc - 'a';
            }
            if (comma < 0) break;
            line = line.substring(comma + 1);
        }
        sequences[idx].stepCount = count;
        if (count > 0) { sequenceSet[idx] = true; n++; }
    }
    f.close();
    Serial.print("Loaded "); Serial.print(n); Serial.println(" sequence(s) from flash.");
    return n > 0;
}

void clearSequences() {
    for (int i = 0; i < NUM_SEQUENCES; i++) { sequenceSet[i] = false; sequences[i].stepCount = 0; }
    LittleFS.remove(SEQ_FILE);
    Serial.println("Cleared all sequences (RAM + flash).");
}

// ── Display ───────────────────────────────────────────────────────────────────

void printStatus() {
    Serial.print("\n[STATUS] ");
    switch (robotStatus) {
        case STATUS_INIT: Serial.println("INIT  — hardware starting up"); break;
        case STATUS_IDLE: Serial.println("IDLE  — ready for commands");   break;
        case STATUS_BUSY: Serial.println("BUSY  — motion in progress (Q to stop)"); break;
    }
    Serial.print("Active channel: "); Serial.print(activeChannel);
    Serial.print(" ("); Serial.print(JOINT_NAMES[activeChannel]); Serial.println(")");
}

void printPose(int index) {
    Serial.print("\n--- Pose "); Serial.print((char)('a' + index)); Serial.println(" ---");
    for (int i = 0; i < NUM_SERVOS; i++) {
        int angle = map(savedPoses[index][i], GLOBAL_MIN_PULSE, GLOBAL_MAX_PULSE, 0, MAX_ANGLE);
        Serial.print("Ch["); Serial.print(i); Serial.print("] ");
        Serial.print(JOINT_NAMES[i]); Serial.print(": ");
        Serial.print(savedPoses[index][i]); Serial.print(" us | ~");
        Serial.print(angle); Serial.println("°");
    }
}

void printAllPoses() {
    Serial.println("\n=== Saved Poses ===");
    for (int i = 0; i < NUM_POSES; i++) {
        if (poseSet[i]) printPose(i);
        else { Serial.print("Pose "); Serial.print((char)('a' + i)); Serial.println(": empty"); }
    }
}

void printAllSequences() {
    Serial.println("\n=== Saved Sequences ===");
    for (int s = 0; s < NUM_SEQUENCES; s++) {
        Serial.print("Sequence "); Serial.print((char)('A' + s)); Serial.print(": ");
        if (!sequenceSet[s] || !sequences[s].stepCount) { Serial.println("empty"); continue; }
        for (int i = 0; i < sequences[s].stepCount; i++) {
            Serial.print((char)('a' + sequences[s].steps[i]));
            if (i < sequences[s].stepCount - 1) Serial.print(" -> ");
        }
        Serial.println();
    }
}

// ── Motion ────────────────────────────────────────────────────────────────────
// All motion functions check eStop each tick so 'Z' can abort mid-move.

void moveServoSmooth(int channel, int targetPWM, int incrementSize, int delayMs) {
    targetPWM = clampToJointLimits(channel, targetPWM);
    int cur = currentPWM[channel];
    if (cur == targetPWM) return;
    int dir = (targetPWM > cur) ? 1 : -1;
    while (cur != targetPWM) {
        // ── Drain serial so 'Q' is seen even mid-move ─────────────────────────
        while (Serial.available()) {
            char peek = Serial.read();
            if (peek == 'z' || peek == 'Z') {
                eStop = true;
                Serial.println("\n[E-STOP] Motion aborted by Z.");
            }
        }
        if (eStop) return;

        cur += dir * incrementSize;
        if ((dir == 1 && cur > targetPWM) || (dir == -1 && cur < targetPWM)) cur = targetPWM;
        pwm.writeMicroseconds(channel, cur);
        currentPWM[channel] = cur;
        delay(delayMs);
    }
}

void moveToPose(int index) {
    int startPWM[NUM_SERVOS];
    int maxDist = 0;
    for (int i = 0; i < NUM_SERVOS; i++) {
        startPWM[i] = currentPWM[i];
        int target  = clampToJointLimits(i, savedPoses[index][i]);
        int d = abs(target - startPWM[i]);
        if (d > maxDist) maxDist = d;
    }
    int steps = max(1, maxDist / POSE_STEP);
    for (int s = 1; s <= steps; s++) {
        // ── Check for Z mid-interpolation ────────────────────────────────────
        while (Serial.available()) {
            char peek = Serial.read();
            if (peek == 'z' || peek == 'Z') {
                eStop = true;
                Serial.println("\n[E-STOP] Motion aborted by Z.");
            }
        }
        if (eStop) return;

        for (int i = 0; i < NUM_SERVOS; i++) {
            int target = clampToJointLimits(i, savedPoses[index][i]);
            int val    = startPWM[i] + (long)(target - startPWM[i]) * s / steps;
            pwm.writeMicroseconds(i, val);
            currentPWM[i] = val;
        }
        delay(POSE_DELAY);
    }
    // Snap to exact target only if we weren't stopped
    if (!eStop) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            currentPWM[i] = clampToJointLimits(i, savedPoses[index][i]);
            pwm.writeMicroseconds(i, currentPWM[i]);
        }
    }
}

void homeArm() {
    setStatusBusy("Homing arm");
    eStop = false;

    Serial.println("\n>>> HOMING SEQUENCE STARTED <<<");
    Serial.println("Step 1/5: Opening gripper...");
    moveServoSmooth(CH_GRIPPER,   HOME_PWM[CH_GRIPPER],   HOME_STEP, HOME_DELAY);
    if (!eStop) { Serial.println("Step 2/5: Centering wrist...");
    moveServoSmooth(CH_WRIST_ROT, HOME_PWM[CH_WRIST_ROT], HOME_STEP, HOME_DELAY); }
    if (!eStop) moveServoSmooth(CH_WRIST,     HOME_PWM[CH_WRIST],     HOME_STEP, HOME_DELAY);
    if (!eStop) { Serial.println("Step 3/5: Centering elbow...");
    moveServoSmooth(CH_ELBOW,     HOME_PWM[CH_ELBOW],     HOME_STEP, HOME_DELAY); }
    if (!eStop) { Serial.println("Step 4/5: Centering shoulder...");
    moveServoSmooth(CH_SHOULDER,  HOME_PWM[CH_SHOULDER],  HOME_STEP, HOME_DELAY); }
    if (!eStop) { Serial.println("Step 5/5: Centering base...");
    moveServoSmooth(CH_BASE,      HOME_PWM[CH_BASE],      HOME_STEP, HOME_DELAY); }

    if (eStop) Serial.println(">>> HOMING ABORTED <<<\n");
    else       Serial.println(">>> HOMING COMPLETE <<<\n");

    setStatusIdle();
}

void playSequence(int index) {
    if (!sequenceSet[index] || sequences[index].stepCount == 0) {
        Serial.print("Sequence "); Serial.print((char)('A' + index));
        Serial.println(" is empty. Use M to define it.");
        return;
    }

    setStatusBusy("Playing sequence");
    eStop = false;

    Serial.print("\n>>> PLAYING SEQUENCE "); Serial.print((char)('A' + index)); Serial.println(" <<<");

    for (int i = 0; i < sequences[index].stepCount; i++) {
        if (eStop) break;

        int pIdx = sequences[index].steps[i];
        if (!poseSet[pIdx]) {
            Serial.print("  Skipping pose "); Serial.print((char)('a' + pIdx)); Serial.println(" (empty)");
            continue;
        }
        Serial.print("  Step "); Serial.print(i + 1); Serial.print("/");
        Serial.print(sequences[index].stepCount); Serial.print(": pose ");
        Serial.println((char)('a' + pIdx));
        moveToPose(pIdx);
        if (!eStop) delay(SEQ_PAUSE_MS);
    }

    if (eStop) {
        Serial.print(">>> SEQUENCE "); Serial.print((char)('A' + index)); Serial.println(" ABORTED <<<\n");
    } else {
        Serial.print(">>> SEQUENCE "); Serial.print((char)('A' + index)); Serial.println(" COMPLETE — returning home <<<");
        homeArm();
        return;   // homeArm() calls setStatusIdle() itself
    }

    setStatusIdle();
}

// ── Command dispatcher ────────────────────────────────────────────────────────

void handleIdleChar(char c) {

    // Z – emergency stop (also handled inside motion loops; caught here when IDLE)
    if (c == 'z' || c == 'Z') {
        if (robotStatus != STATUS_BUSY) Serial.println("Not moving — nothing to stop.");
        return;   // when BUSY, Z is caught inside the motion loop directly
    }

    // V – print robot status
    if (c == 'v' || c == 'V') { printStatus(); return; }

    // Y – home
    if (c == 'y' || c == 'Y') {
        if (isBusy()) return;
        homeArm();
        return;
    }

    // L – list poses and sequences
    if (c == 'l' || c == 'L') {
        if (isBusy()) return;
        printAllPoses(); printAllSequences();
        return;
    }

    // T – clear all poses
    if (c == 't' || c == 'T') {
        if (isBusy()) return;
        clearPoses();
        return;
    }

    // X – clear all sequences
    if (c == 'x' || c == 'X') {
        if (isBusy()) return;
        clearSequences();
        return;
    }

    // R – record pose
    if (c == 'R') {
        if (isBusy()) return;
        inputState = WAIT_POSE_SLOT;
        Serial.println("Enter slot (a-k) to save current pose:");
        return;
    }

    // M – define sequence
    if (c == 'M') {
        if (isBusy()) return;
        inputState = WAIT_SEQ_SLOT;
        Serial.println("Enter sequence slot (A-G) to define:");
        return;
    }

    // a-k – go to saved pose
    if (c >= 'a' && c <= 'k') {
        if (isBusy()) return;
        int idx = poseIndex(c);
        if (poseSet[idx]) {
            setStatusBusy("Moving to pose");
            eStop = false;
            Serial.print("\nMoving to Pose "); Serial.println(c);
            moveToPose(idx);
            if (!eStop) printPose(idx);
            setStatusIdle();
        } else {
            Serial.print("Pose "); Serial.print(c); Serial.println(" is empty. Use R to record.");
        }
        return;
    }

    // A-G – play sequence
    if (c >= 'A' && c <= 'G') {
        if (isBusy()) return;
        playSequence(c - 'A');
        return;
    }

    // 0-5 – select channel
    if (c >= '0' && c <= '5') {
        if (isBusy()) return;
        activeChannel = c - '0';
        Serial.print("\n>>> Switched to Channel: "); Serial.print(activeChannel);
        Serial.print(" ("); Serial.print(JOINT_NAMES[activeChannel]);
        Serial.print(") | Limits: "); Serial.print(jointMin[activeChannel]);
        Serial.print(" - "); Serial.print(jointMax[activeChannel]); Serial.println(" us");
        return;
    }

    // W/S – nudge active channel (single step, treated as brief BUSY)
    bool moved = false;
    if      (c == 'w' || c == 'W' || c == '+') { currentPWM[activeChannel] += stepSize; moved = true; }
    else if (c == 's' || c == 'S' || c == '-') { currentPWM[activeChannel] -= stepSize; moved = true; }
    else {
        Serial.println("Invalid key. (V=status Q=stop Y=home L=list R=record M=seq a-k=pose A-G=seq 0-5=ch W/S=nudge T X)");
    }

    if (moved) {
        if (isBusy()) return;
        currentPWM[activeChannel] = clampToJointLimits(activeChannel, currentPWM[activeChannel]);
        pwm.writeMicroseconds(activeChannel, currentPWM[activeChannel]);
        int approxAngle = map(currentPWM[activeChannel], GLOBAL_MIN_PULSE, GLOBAL_MAX_PULSE, 0, MAX_ANGLE);
        Serial.print("Ch["); Serial.print(activeChannel); Serial.print("] ");
        Serial.print(JOINT_NAMES[activeChannel]); Serial.print(" | PWM: ");
        Serial.print(currentPWM[activeChannel]); Serial.print(" us | ~");
        Serial.print(approxAngle); Serial.print("°");
        if (currentPWM[activeChannel] == jointMin[activeChannel]) Serial.print("  [AT MIN LIMIT]");
        if (currentPWM[activeChannel] == jointMax[activeChannel]) Serial.print("  [AT MAX LIMIT]");
        Serial.println();
    }
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────
// ── Self-audit ────────────────────────────────────────────────────────────────
// Returns true if all hardware is responding correctly.
bool selfAudit() {
    bool passed = true;

    Serial.println("[INIT] Running self-audit...");

    // 1. Check PCA9685 is reachable on I2C
    Wire.beginTransmission(0x40);
    byte i2cError = Wire.endTransmission();
    if (i2cError == 0) {
        Serial.println("  [OK] PCA9685 motor driver found on I2C (0x40)");
    } else {
        Serial.print("  [FAIL] PCA9685 not responding! I2C error: ");
        Serial.println(i2cError);
        passed = false;
    }

    // 2. Check LittleFS mounted correctly
    if (LittleFS.begin(true)) {
        Serial.println("  [OK] LittleFS flash filesystem mounted");
    } else {
        Serial.println("  [FAIL] LittleFS mount failed!");
        passed = false;
    }

    // 3. Verify PWM driver is outputting by writing a safe value
    //    and confirming no I2C error during the write
    Wire.beginTransmission(0x40);
    Wire.write(0x00);   // MODE1 register
    byte writeError = Wire.endTransmission();
    if (writeError == 0) {
        Serial.println("  [OK] PWM driver write test passed");
    } else {
        Serial.print("  [FAIL] PWM driver write error: ");
        Serial.println(writeError);
        passed = false;
    }

    if (passed) Serial.println("[INIT] Self-audit PASSED — all systems nominal.");
    else        Serial.println("[INIT] Self-audit FAILED — check connections.");

    return passed;
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    robotStatus = STATUS_INIT;
    Serial.println("\n[INIT] Power-on — starting system...");

    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    delay(10);

    // ── Step 1: Self-audit ────────────────────────────────────────────────────
    bool auditOk = selfAudit();
    if (!auditOk) {
        Serial.println("[INIT] WARNING: Audit failed. Proceeding with caution...");
        // You can halt here with while(true); if you want hard-stop on failure
    }

    // Load flash data after LittleFS is confirmed mounted
    loadPosesFromFlash();
    loadSequencesFromFlash();

    // Safe-initialise all joints before homing
    for (int i = 0; i < NUM_SERVOS; i++) {
        pwm.writeMicroseconds(i, currentPWM[i]);
        delay(150);
    }

    // ── Step 2: Autonomous Homing (Wrist → Elbow → Shoulder → Base per spec) ─
    Serial.println("[INIT] Step 2: Autonomous homing...");
    homeArm();   // homeArm() sets STATUS_BUSY then STATUS_IDLE internally

    Serial.println("\n--- 6DOF Arm Controller ---");
    Serial.println("V      print robot status");
    Serial.println("Z      emergency stop");
    Serial.println("0-5    select channel");
    Serial.println("W / +  increase PWM");
    Serial.println("S / -  decrease PWM");
    Serial.println("R      record pose -> type slot (a-r)");
    Serial.println("a-r    go to saved pose");
    Serial.println("M      define sequence -> slot (A-E) -> poses -> Enter");
    Serial.println("A-E    play saved sequence (auto-homes after)");
    Serial.println("L      list all poses and sequences");
    Serial.println("T      clear all poses");
    Serial.println("X      clear all sequences");
    Serial.println("Y      homing sequence");
    Serial.println("---------------------------");
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        // Z is always processed immediately, even during multi-char input states
        if (c == 'z' || c == 'Z') {
            eStop = true;
            Serial.println("\n[E-STOP] Q received.");
            inputState    = IDLE_INPUT;
            inputBuffer   = "";
            pendingSeqIdx = -1;
            return;
        }

        if (inputState == IDLE_INPUT) {
            if (c == '\n' || c == '\r' || c == ' ') continue;
            handleIdleChar(c);
            continue;
        }

        if (inputState == WAIT_POSE_SLOT) {
            if (c == '\n' || c == '\r' || c == ' ') continue;
            char slot = tolower(c);
            if (slot >= 'a' && slot <= 'k') {
                int idx = poseIndex(slot);
                for (int i = 0; i < NUM_SERVOS; i++) savedPoses[idx][i] = currentPWM[i];
                poseSet[idx] = true;
                Serial.print("Pose saved to slot "); Serial.println(slot);
                printPose(idx);
                persistPoses();
            } else {
                Serial.println("Invalid slot. Use a-k. Cancelled.");
            }
            inputState = IDLE_INPUT;
            continue;
        }

        if (inputState == WAIT_SEQ_SLOT) {
            if (c == '\n' || c == '\r' || c == ' ') continue;
            char slot = toupper(c);
            if (slot >= 'A' && slot <= 'G') {
                pendingSeqIdx = slot - 'A';
                inputBuffer   = "";
                inputState    = WAIT_SEQ_POSES;
                Serial.print("Sequence slot "); Serial.print(slot); Serial.println(" selected.");
                Serial.println("Type pose letters (e.g. abcfg), then press Enter to SAVE:");
            } else {
                Serial.println("Invalid slot. Use A-G. Cancelled.");
                inputState = IDLE_INPUT;
            }
            continue;
        }

        if (inputState == WAIT_SEQ_POSES) {
            if (c == '\r') continue;
            if (c == '\n') {
                inputBuffer.trim(); inputBuffer.toLowerCase();
                int count = 0;
                for (unsigned int i = 0; i < inputBuffer.length() && count < MAX_SEQ_STEPS; i++) {
                    char p = inputBuffer.charAt(i);
                    if (p >= 'a' && p < ('a' + NUM_POSES)) sequences[pendingSeqIdx].steps[count++] = p - 'a';
                }
                if (count == 0) {
                    Serial.println("No valid pose letters. Sequence NOT saved.");
                } else {
                    sequences[pendingSeqIdx].stepCount = count;
                    sequenceSet[pendingSeqIdx]         = true;
                    Serial.print("Sequence "); Serial.print((char)('A' + pendingSeqIdx));
                    Serial.print(" saved: ");
                    for (int i = 0; i < count; i++) {
                        Serial.print((char)('a' + sequences[pendingSeqIdx].steps[i]));
                        if (i < count - 1) Serial.print(" -> ");
                    }
                    Serial.println();
                    persistSequences();
                }
                inputBuffer = ""; pendingSeqIdx = -1; inputState = IDLE_INPUT;
            } else {
                Serial.print(c); inputBuffer += c;
            }
            continue;
        }
    }
}
