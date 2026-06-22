#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Channel assignments
#define CH_SHOULDER  0
#define CH_ELBOW     1
#define CH_WRIST_P   2
#define CH_WRIST_R   3
#define CH_GRABBER   4
#define CH_BASE      5
#define NUM_JOINTS   6

// Servo pulse range at 50Hz (4096 ticks = 20ms)
#define TICK_MIN  102   // 0.5ms -> 0 deg
#define TICK_MAX  512   // 2.5ms -> 180 deg
#define JOG_STEP  5     // degrees per nudge

#define MAX_POSES   5
#define POSES_FILE  "/poses.txt"

const char* JOINT_NAMES[NUM_JOINTS] = {
    "shoulder", "elbow", "wrist_pitch", "wrist_roll", "grabber", "base"
};

static uint16_t current[NUM_JOINTS];
static uint16_t saved[MAX_POSES][NUM_JOINTS];
static int      savedCount = 0;

// ---- Conversion ------------------------------------------------------------

uint16_t degToTick(int deg) {
    deg = constrain(deg, 0, 180);
    return (uint16_t)(TICK_MIN + (long)(TICK_MAX - TICK_MIN) * deg / 180);
}

int tickToDeg(uint16_t tick) {
    return (int)((long)(tick - TICK_MIN) * 180 / (TICK_MAX - TICK_MIN));
}

// ---- Persistence -----------------------------------------------------------

void persistPoses() {
    File f = LittleFS.open(POSES_FILE, "w");
    if (!f) { Serial.println("Error: could not open file for writing"); return; }
    for (int p = 0; p < savedCount; p++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            f.print(tickToDeg(saved[p][j]));
            if (j < NUM_JOINTS - 1) f.print(',');
        }
        f.println();
    }
    f.close();
    Serial.printf("Poses written to %s\n", POSES_FILE);
}

bool loadPoses() {
    if (!LittleFS.exists(POSES_FILE)) {
        Serial.println("No saved pose file found");
        return false;
    }
    File f = LittleFS.open(POSES_FILE, "r");
    if (!f) { Serial.println("Error: could not open pose file"); return false; }

    savedCount = 0;
    while (f.available() && savedCount < MAX_POSES) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        int j = 0;
        while (j < NUM_JOINTS) {
            int comma = line.indexOf(',');
            String token = (comma >= 0) ? line.substring(0, comma) : line;
            saved[savedCount][j++] = degToTick(token.toInt());
            if (comma < 0) break;
            line = line.substring(comma + 1);
        }
        if (j == NUM_JOINTS) savedCount++;
    }
    f.close();
    return savedCount > 0;
}

// ---- Motion ----------------------------------------------------------------

void moveTo(const uint16_t target[NUM_JOINTS], uint32_t durationMs) {
    const uint32_t steps = max(1u, durationMs / 20u);
    uint16_t start[NUM_JOINTS];
    memcpy(start, current, sizeof(start));

    for (uint32_t i = 1; i <= steps; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            int32_t val = (int32_t)start[j] +
                          ((int32_t)target[j] - start[j]) * (int32_t)i / (int32_t)steps;
            pca.setPWM(j, 0, (uint16_t)val);
        }
        delay(20);
    }
    memcpy(current, target, NUM_JOINTS * sizeof(uint16_t));
}

void setJointDeg(int joint, int deg) {
    uint16_t target[NUM_JOINTS];
    memcpy(target, current, sizeof(target));
    target[joint] = degToTick(deg);
    moveTo(target, 400);
}

void jogJoint(int joint, int deltaDeg) {
    setJointDeg(joint, tickToDeg(current[joint]) + deltaDeg);
}

// ---- Serial UI -------------------------------------------------------------

void printStatus() {
    Serial.println("--- Current positions ---");
    for (int j = 0; j < NUM_JOINTS; j++) {
        Serial.printf("  j%d %-12s %3d deg\n", j, JOINT_NAMES[j], tickToDeg(current[j]));
    }
    Serial.printf("Saved poses: %d / %d\n", savedCount, MAX_POSES);
}

void printPoses() {
    if (savedCount == 0) { Serial.println("No poses saved"); return; }
    // Header
    Serial.printf("%-6s", "pose");
    for (int j = 0; j < NUM_JOINTS; j++) Serial.printf("  %-11s", JOINT_NAMES[j]);
    Serial.println();
    // Rows
    for (int p = 0; p < savedCount; p++) {
        Serial.printf("%-6d", p + 1);
        for (int j = 0; j < NUM_JOINTS; j++) Serial.printf("  %-8d deg", tickToDeg(saved[p][j]));
        Serial.println();
    }
}

void printHelp() {
    Serial.println("Commands:");
    Serial.println("  j<n> <deg>   set joint n to angle 0-180  (e.g. j0 90)");
    Serial.println("  j<n>+        nudge joint n up 5 deg");
    Serial.println("  j<n>-        nudge joint n down 5 deg");
    Serial.println("  save         save current pose (also writes to flash)");
    Serial.println("  load         reload poses from flash");
    Serial.println("  poses        list all saved poses and their angles");
    Serial.println("  play         replay all saved poses");
    Serial.println("  clear        erase saved poses (and delete file)");
    Serial.println("  home         move all joints to 90 deg");
    Serial.println("  status       print current joint angles");
    Serial.println("  help         show this message");
    Serial.println("Joints: j0=shoulder j1=elbow j2=wrist_pitch j3=wrist_roll j4=grabber j5=base");
}

void handleCommand(const String& cmd) {
    String s = cmd;
    s.trim();
    s.toLowerCase();

    if (s == "help") {
        printHelp();

    } else if (s == "status") {
        printStatus();

    } else if (s == "home") {
        uint16_t home[NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; j++) home[j] = degToTick(90);
        moveTo(home, 1500);
        Serial.println("At home (90 deg)");

    } else if (s == "save") {
        if (savedCount >= MAX_POSES) {
            Serial.printf("Already have %d poses. Use 'clear' to reset.\n", MAX_POSES);
            return;
        }
        memcpy(saved[savedCount], current, NUM_JOINTS * sizeof(uint16_t));
        savedCount++;
        Serial.printf("Saved pose %d / %d\n", savedCount, MAX_POSES);
        persistPoses();
        if (savedCount == MAX_POSES)
            Serial.println("All poses recorded. Type 'play' to replay.");

    } else if (s == "poses") {
        printPoses();

    } else if (s == "load") {
        if (loadPoses())
            Serial.printf("Loaded %d poses from flash\n", savedCount);

    } else if (s == "clear") {
        savedCount = 0;
        LittleFS.remove(POSES_FILE);
        Serial.println("Cleared all saved poses");

    } else if (s == "play") {
        if (savedCount == 0) {
            Serial.println("No poses saved yet");
            return;
        }
        printPoses();
        Serial.println("Starting in 2s...");
        delay(2000);
        for (int p = 0; p < savedCount; p++) {
            Serial.printf("  -> pose %d\n", p + 1);
            moveTo(saved[p], 1000);
            delay(500);
        }
        Serial.println("Done");

    } else if (s.length() >= 2 && s[0] == 'j') {
        int joint = s[1] - '0';
        if (joint < 0 || joint >= NUM_JOINTS) {
            Serial.println("Invalid joint number (0-5)");
            return;
        }
        if (s.length() > 2 && s[2] == '+') {
            jogJoint(joint, JOG_STEP);
            Serial.printf("j%d %s = %d deg\n", joint, JOINT_NAMES[joint], tickToDeg(current[joint]));
        } else if (s.length() > 2 && s[2] == '-') {
            jogJoint(joint, -JOG_STEP);
            Serial.printf("j%d %s = %d deg\n", joint, JOINT_NAMES[joint], tickToDeg(current[joint]));
        } else {
            int spaceIdx = s.indexOf(' ');
            if (spaceIdx < 0) { Serial.println("Usage: j<n> <degrees>"); return; }
            int deg = s.substring(spaceIdx + 1).toInt();
            setJointDeg(joint, deg);
            Serial.printf("j%d %s = %d deg\n", joint, JOINT_NAMES[joint], tickToDeg(current[joint]));
        }

    } else if (s.length() > 0) {
        Serial.println("Unknown command. Type 'help'.");
    }
}

// ---- Arduino entry points --------------------------------------------------

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pca.begin();
    pca.setPWMFreq(50);

    LittleFS.begin(true);  // true = format if mount fails

    uint16_t home[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; j++) {
        current[j] = degToTick(90);
        home[j]    = degToTick(90);
    }
    moveTo(home, 1500);

    Serial.println("\nRobot arm ready.");
    if (loadPoses())
        Serial.printf("Restored %d pose(s) from flash. Type 'play' to replay.\n", savedCount);
    else
        Serial.println("No saved poses. Type 'help' for commands.");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }
}
