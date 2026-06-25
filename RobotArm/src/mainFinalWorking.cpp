#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ── Channels ──────────────────────────────────────────────────────────────────
#define CH_SHOULDER  0
#define CH_ELBOW     1
#define CH_WRIST_P   2
#define CH_WRIST_R   3
#define CH_GRABBER   4
#define CH_BASE      5
#define NUM_JOINTS   6
#define NUM_IK_DOF   5

// ── Physical geometry (cm) ────────────────────────────────────────────────────
static const float H1     = 6.0f;
static const float L2     = 8.5f;
static const float L3     = 7.5f;
static const float D_TOOL = 10.0f;  // L4+L5

// ── Workspace limits (cm) ─────────────────────────────────────────────────────
static const float Z_MIN = -2.0f;
static const float Z_MAX = 29.0f;
static const float R_MAX = 22.0f;

// ── PCA9685 ───────────────────────────────────────────────────────────────────
#define TICK_MIN 102
#define TICK_MAX 512
#define JOG_STEP 5

// ── Servo <-> IK mapping ──────────────────────────────────────────────────────
// servo_deg = zero + offset + ik_deg * dir   (clamped [0,180])
struct JointMap { uint8_t ch; float zero; float dir; float offset; };

// Zeros derived from calib-front physical measurement
static JointMap jmap[NUM_IK_DOF] = {
    {CH_BASE,    100.0f,  1.0f, 0.0f},  // [0] base
    {CH_SHOULDER, 80.0f,  1.0f, 0.0f},  // [1] shoulder
    {CH_ELBOW,    12.0f,  1.0f, 0.0f},  // [2] elbow
    {CH_WRIST_P, 136.0f,  1.0f, 0.0f},  // [3] wrist_pitch
    {CH_WRIST_R,  95.0f,  1.0f, 0.0f},  // [4] wrist_roll
};

static const char* IK_NAMES[NUM_IK_DOF]  = {"base","shoulder","elbow","wrist_pitch","wrist_roll"};
static const char* SRV_NAMES[NUM_JOINTS] = {"shoulder","elbow","wrist_pitch","wrist_roll","grabber","base"};

// ── Pose recording ────────────────────────────────────────────────────────────
#define MAX_POSES     10
#define POSES_FILE    "/poses.txt"
#define CALIB_FILE    "/calib.txt"
#define CTARGETS_FILE "/ctargets.txt"

static uint16_t current[NUM_JOINTS];
static uint16_t saved[MAX_POSES][NUM_JOINTS];
static int      savedCount = 0;

// ── IK state ──────────────────────────────────────────────────────────────────
static float ikAngles[NUM_IK_DOF];
static float defaultPitch = -999.0f;  // -999 = auto-compute from target geometry
static bool  elbowUp      = true;

// ── Serial input ──────────────────────────────────────────────────────────────
static String   inputBuf      = "";
static int      selectedJoint = -1;
static uint8_t  escState      = 0;
static uint32_t escTime       = 0;
static float    jogPos[NUM_JOINTS];

// ── Named calibration targets ─────────────────────────────────────────────────
struct CalibTarget { const char* name; float x, y, z, pitch; };
static const CalibTarget CALIB_TARGETS[] = {
    {"top",   0.0f,   1.9f, 28.1f, 90.0f},
    {"front", 0.0f,  22.0f,  3.75f, 0.0f},
    {"left", -22.0f,  0.0f,  3.75f, 0.0f},
    {"right", 22.0f,  0.0f,  3.75f, 0.0f},
};
static const int NUM_CALIB_TARGETS = 4;

// ── Servo snapshots for calib targets (direct playback, bypasses IK) ──────────
// Channel order: ch0=shoulder ch1=elbow ch2=wrist_p ch3=wrist_r ch4=grabber ch5=base
struct ServoSnap { bool valid; uint8_t deg[NUM_JOINTS]; };

static ServoSnap calibSnaps[NUM_CALIB_TARGETS];

// Hardcoded defaults — loaded into calibSnaps on first boot (no ctargets file)
// Channel order: ch0=shoulder ch1=elbow ch2=wrist_p ch3=wrist_r ch4=grabber ch5=base
static const ServoSnap CALIB_SNAP_DEFAULTS[NUM_CALIB_TARGETS] = {
    {true,  {124,  92, 102, 94, 32, 101}},  // top
    {true,  { 32,  93, 103, 95, 90, 100}},  // front
    {true,  { 31,  92, 102, 94, 90, 165}},  // left
    {true,  { 31,  92, 102, 94, 90,  36}},  // right
};


// ══════════════════════════════════════════════════════════════════════════════
// Conversion helpers
// ══════════════════════════════════════════════════════════════════════════════

uint16_t degToTick(int deg) {
    deg = constrain(deg, 0, 180);
    return (uint16_t)(TICK_MIN + (long)(TICK_MAX - TICK_MIN) * deg / 180);
}

int tickToDeg(uint16_t tick) {
    return (int)((long)(tick - TICK_MIN) * 180 / (TICK_MAX - TICK_MIN));
}

float ikToServoDeg(int i, float ik_deg) {
    return jmap[i].zero + jmap[i].offset + ik_deg * jmap[i].dir;
}

float servoToIKDeg(int i, float srv_deg) {
    if (fabsf(jmap[i].dir) < 1e-4f) return 0.0f;
    return (srv_deg - jmap[i].zero - jmap[i].offset) / jmap[i].dir;
}

// ══════════════════════════════════════════════════════════════════════════════
// Motion
// ══════════════════════════════════════════════════════════════════════════════

void moveTo(const uint16_t target[], uint32_t durationMs) {
    const uint32_t steps = max(1u, durationMs / 20u);
    uint16_t start[NUM_JOINTS];
    memcpy(start, current, sizeof(start));
    for (uint32_t i = 1; i <= steps; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            int32_t v = (int32_t)start[j] +
                        ((int32_t)target[j] - (int32_t)start[j]) * (int32_t)i / (int32_t)steps;
            pca.setPWM(j, 0, (uint16_t)v);
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

void jogJoint(int joint, int delta) {
    setJointDeg(joint, tickToDeg(current[joint]) + delta);
}

void jogDirect(int joint, int delta) {
    int deg = constrain(tickToDeg(current[joint]) + delta, 0, 180);
    current[joint] = degToTick(deg);
    pca.setPWM(joint, 0, current[joint]);
}

// ══════════════════════════════════════════════════════════════════════════════
// FK
// ══════════════════════════════════════════════════════════════════════════════

void fk(const float ang[], float &x, float &y, float &z, float &approach_deg) {
    float t1   = ang[0] * DEG_TO_RAD;
    float a_sh = ang[1] * DEG_TO_RAD;
    float a_el = (ang[1] + ang[2]) * DEG_TO_RAD;
    float a_wr = (ang[1] + ang[2] + ang[3]) * DEG_TO_RAD;
    float r = L2 * cosf(a_sh) + L3 * cosf(a_el) + D_TOOL * cosf(a_wr);
    z = H1 + L2 * sinf(a_sh) + L3 * sinf(a_el) + D_TOOL * sinf(a_wr);
    x = r * sinf(t1);
    y = r * cosf(t1);
    approach_deg = a_wr * RAD_TO_DEG;
}

// ══════════════════════════════════════════════════════════════════════════════
// IK
// ══════════════════════════════════════════════════════════════════════════════

bool ik(float x, float y, float z, float pitch_deg, bool elbow_up_f,
        float roll_deg, float out[], bool silent = false) {
    if (z < Z_MIN || z > Z_MAX) {
        if (!silent) Serial.printf("  IK fail: z=%.2f outside [%.1f, %.1f]\n", z, Z_MIN, Z_MAX);
        return false;
    }
    float r_full = sqrtf(x*x + y*y);
    if (r_full > R_MAX) {
        if (!silent) Serial.printf("  IK fail: reach=%.2f cm > R_MAX=%.1f cm\n", r_full, R_MAX);
        return false;
    }
    float alpha = pitch_deg * DEG_TO_RAD;
    float t1    = (r_full < 0.01f) ? (ikAngles[0] * DEG_TO_RAD) : atan2f(x, y);
    float r_wp  = r_full - D_TOOL * cosf(alpha);
    float z_wp  = z - H1  - D_TOOL * sinf(alpha);
    float d2    = r_wp*r_wp + z_wp*z_wp;
    float dmax  = L2 + L3;
    float dmin  = fabsf(L2 - L3);
    if (d2 > dmax*dmax) {
        if (!silent) Serial.printf("  IK fail: wrist too far (d=%.2f cm)\n", sqrtf(d2));
        return false;
    }
    if (d2 < dmin*dmin) {
        if (!silent) Serial.printf("  IK fail: wrist too close (d=%.2f cm)\n", sqrtf(d2));
        return false;
    }
    float cos3 = constrain((d2 - L2*L2 - L3*L3) / (2.0f*L2*L3), -1.0f, 1.0f);
    float sin3 = sqrtf(max(0.0f, 1.0f - cos3*cos3)) * (elbow_up_f ? 1.0f : -1.0f);
    float t3   = atan2f(sin3, cos3);
    float t2   = atan2f(z_wp, r_wp) - atan2f(L3*sin3, L2 + L3*cos3);
    float t4   = alpha - t2 - t3;
    out[0] = t1 * RAD_TO_DEG;
    out[1] = t2 * RAD_TO_DEG;
    out[2] = t3 * RAD_TO_DEG;
    out[3] = t4 * RAD_TO_DEG;
    out[4] = roll_deg;
    return true;
}

bool applyIK(const float ang[], uint32_t ms = 1000, bool verbose = true) {
    uint16_t target[NUM_JOINTS];
    memcpy(target, current, sizeof(target));
    for (int i = 0; i < NUM_IK_DOF; i++) {
        float sd = ikToServoDeg(i, ang[i]);
        if (sd < 0.0f || sd > 180.0f) {
            if (verbose) Serial.printf("  Servo limit: %s = %.1f deg\n", IK_NAMES[i], sd);
            return false;
        }
        target[jmap[i].ch] = degToTick((int)roundf(sd));
    }
    if (verbose) {
        Serial.print("  Joints:");
        for (int i = 0; i < NUM_IK_DOF; i++) {
            int from = tickToDeg(current[jmap[i].ch]);
            int to   = tickToDeg(target[jmap[i].ch]);
            if (abs(to - from) > 1)
                Serial.printf("  %s %d->%d", IK_NAMES[i], from, to);
            else
                Serial.printf("  %s=%d", IK_NAMES[i], to);
        }
        Serial.println();
    }
    moveTo(target, ms);
    memcpy(ikAngles, ang, NUM_IK_DOF * sizeof(float));
    return true;
}

void syncIK();  // forward declaration

// Cartesian-space path following: divides the straight-line path from the
// current FK position to (x,y,z) into N steps and re-runs IK at each step.
// The approach angle is also interpolated from current to target pitch so
// the wrist orientation changes smoothly throughout the motion.
bool cartesianMove(float x, float y, float z, float pitch, float roll, uint32_t ms) {
    syncIK();
    float x0, y0, z0, ap0;
    fk(ikAngles, x0, y0, z0, ap0);

    const int    N      = 10;
    const uint32_t stepMs = max(40u, ms / (uint32_t)N);
    float out[NUM_IK_DOF];
    bool reached = false;

    for (int s = 1; s <= N; s++) {
        float t      = (float)s / (float)N;
        float xi     = x0  + (x     - x0)  * t;
        float yi     = y0  + (y     - y0)  * t;
        float zi     = z0  + (z     - z0)  * t;
        float pitchi = ap0 + (pitch - ap0) * t;  // smoothly change approach angle

        if (!ik(xi, yi, zi, pitchi, elbowUp, roll, out, /*silent=*/true)) continue;

        bool ok = true;
        for (int i = 0; i < NUM_IK_DOF; i++) {
            float sd = ikToServoDeg(i, out[i]);
            if (sd < 0.0f || sd > 180.0f) { ok = false; break; }
        }
        if (!ok) continue;

        applyIK(out, stepMs, /*verbose=*/false);
        if (s == N) reached = true;
    }
    return reached;
}

void syncIK() {
    for (int i = 0; i < NUM_IK_DOF; i++)
        ikAngles[i] = servoToIKDeg(i, (float)tickToDeg(current[jmap[i].ch]));
}

// ══════════════════════════════════════════════════════════════════════════════
// Calibration persistence
// ══════════════════════════════════════════════════════════════════════════════

void saveCalib() {
    File f = LittleFS.open(CALIB_FILE, "w");
    if (!f) { Serial.println("  Error: cannot open calib file"); return; }
    for (int i = 0; i < NUM_IK_DOF; i++)
        f.printf("%.4f %.4f %.4f\n", jmap[i].zero, jmap[i].dir, jmap[i].offset);
    f.close();
    Serial.println("  Calibration saved.");
}

void loadCalib() {
    if (!LittleFS.exists(CALIB_FILE)) { Serial.println("  No calib file."); return; }
    File f = LittleFS.open(CALIB_FILE, "r");
    if (!f) { Serial.println("  Error: cannot open calib file"); return; }
    for (int i = 0; i < NUM_IK_DOF && f.available(); i++) {
        String line = f.readStringUntil('\n');
        float z = jmap[i].zero, d = jmap[i].dir, o = 0.0f;
        sscanf(line.c_str(), "%f %f %f", &z, &d, &o);
        jmap[i].zero = z; jmap[i].dir = d; jmap[i].offset = o;
    }
    f.close();
    Serial.println("  Calibration loaded.");
}

void saveCalibSnaps() {
    File f = LittleFS.open(CTARGETS_FILE, "w");
    if (!f) { Serial.println("  Error: cannot open targets file"); return; }
    for (int i = 0; i < NUM_CALIB_TARGETS; i++) {
        f.printf("%d", calibSnaps[i].valid ? 1 : 0);
        for (int j = 0; j < NUM_JOINTS; j++)
            f.printf(" %d", (int)calibSnaps[i].deg[j]);
        f.println();
    }
    f.close();
}

void loadCalibSnaps() {
    if (!LittleFS.exists(CTARGETS_FILE)) {
        memcpy(calibSnaps, CALIB_SNAP_DEFAULTS, sizeof(calibSnaps));
        saveCalibSnaps();
        return;
    }
    File f = LittleFS.open(CTARGETS_FILE, "r");
    if (!f) { memcpy(calibSnaps, CALIB_SNAP_DEFAULTS, sizeof(calibSnaps)); return; }
    for (int i = 0; i < NUM_CALIB_TARGETS && f.available(); i++) {
        String line = f.readStringUntil('\n');
        int v = 0, d[NUM_JOINTS] = {};
        sscanf(line.c_str(), "%d %d %d %d %d %d %d", &v,
               &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]);
        calibSnaps[i].valid = (v == 1);
        for (int j = 0; j < NUM_JOINTS; j++)
            calibSnaps[i].deg[j] = (uint8_t)constrain(d[j], 0, 180);
    }
    f.close();
}

// ══════════════════════════════════════════════════════════════════════════════
// Pose persistence
// ══════════════════════════════════════════════════════════════════════════════

void persistPoses() {
    File f = LittleFS.open(POSES_FILE, "w");
    if (!f) { Serial.println("  Error: cannot open poses file"); return; }
    for (int p = 0; p < savedCount; p++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            f.print(tickToDeg(saved[p][j]));
            if (j < NUM_JOINTS - 1) f.print(',');
        }
        f.println();
    }
    f.close();
}

bool loadPoses() {
    if (!LittleFS.exists(POSES_FILE)) return false;
    File f = LittleFS.open(POSES_FILE, "r");
    if (!f) return false;
    savedCount = 0;
    while (f.available() && savedCount < MAX_POSES) {
        String line = f.readStringUntil('\n'); line.trim();
        if (!line.length()) continue;
        int j = 0;
        while (j < NUM_JOINTS) {
            int c = line.indexOf(',');
            saved[savedCount][j++] = degToTick((c >= 0 ? line.substring(0, c) : line).toInt());
            if (c < 0) break;
            line = line.substring(c + 1);
        }
        if (j == NUM_JOINTS) savedCount++;
    }
    f.close();
    return savedCount > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// Print helpers
// ══════════════════════════════════════════════════════════════════════════════

void printWhere() {
    syncIK();
    float x, y, z, ap;
    fk(ikAngles, x, y, z, ap);
    Serial.printf("  x=%.2f  y=%.2f  z=%.2f cm   approach=%.1f deg\n", x, y, z, ap);
}

void printIKStatus() {
    syncIK();
    float x, y, z, ap;
    fk(ikAngles, x, y, z, ap);
    Serial.println("--- IK state ---");
    for (int i = 0; i < NUM_IK_DOF; i++)
        Serial.printf("  [%d] %-12s  ik=%7.2f deg  servo=%.1f deg\n",
            i, IK_NAMES[i], ikAngles[i], ikToServoDeg(i, ikAngles[i]));
    Serial.printf("  FK: x=%.2f  y=%.2f  z=%.2f cm   approach=%.1f deg\n", x, y, z, ap);
}

void printCalibState() {
    Serial.println("--- Servo mapping ---");
    Serial.printf("  %-4s  %-12s  %6s  %4s  %7s\n", "idx", "joint", "zero", "dir", "offset");
    for (int i = 0; i < NUM_IK_DOF; i++)
        Serial.printf("  [%d]  %-12s  %6.1f  %+.0f  %+.2f\n",
            i, IK_NAMES[i], jmap[i].zero, jmap[i].dir, jmap[i].offset);
    Serial.println("--- Calib snapshots ---");
    for (int i = 0; i < NUM_CALIB_TARGETS; i++) {
        if (calibSnaps[i].valid) {
            Serial.printf("  %-6s  sh=%d el=%d wp=%d wr=%d gr=%d base=%d\n",
                CALIB_TARGETS[i].name,
                calibSnaps[i].deg[CH_SHOULDER], calibSnaps[i].deg[CH_ELBOW],
                calibSnaps[i].deg[CH_WRIST_P],  calibSnaps[i].deg[CH_WRIST_R],
                calibSnaps[i].deg[CH_GRABBER],  calibSnaps[i].deg[CH_BASE]);
        } else {
            Serial.printf("  %-6s  (not saved — run 'calib save %s')\n",
                CALIB_TARGETS[i].name, CALIB_TARGETS[i].name);
        }
    }
}

void printStatus() {
    Serial.println("--- Servo positions ---");
    for (int j = 0; j < NUM_JOINTS; j++)
        Serial.printf("  j%d %-12s %3d deg\n", j, SRV_NAMES[j], tickToDeg(current[j]));
    Serial.printf("  Saved poses: %d/%d\n", savedCount, MAX_POSES);
}

void printPoses() {
    if (!savedCount) { Serial.println("  No poses saved"); return; }
    Serial.printf("%-5s", "pose");
    for (int j = 0; j < NUM_JOINTS; j++) Serial.printf("  %-10s", SRV_NAMES[j]);
    Serial.println();
    for (int p = 0; p < savedCount; p++) {
        Serial.printf("[%-2d] ", p);
        for (int j = 0; j < NUM_JOINTS; j++) Serial.printf("  %-7d deg", tickToDeg(saved[p][j]));
        Serial.println();
    }
}

void printHelp() {
    Serial.println("=== Commands ===");
    Serial.println(" -- IK motion --");
    Serial.println("  xyz x y z [pitch] [roll]    move to XYZ (cm)");
    Serial.println("  calib top|front|left|right   move to named target (snapshot if saved, else IK)");
    Serial.println("  calib save <name>            save current servo position as named target");
    Serial.println("  where                        FK position (XYZ + approach)");
    Serial.println("  ikstatus                     IK angles + FK position");
    Serial.println("  pitch <deg>                  default approach angle");
    Serial.println("  elbow up|down                switch elbow solution");
    Serial.println(" -- Calibration --");
    Serial.println("  offset <0-4> <deg>           trim offset for IK joint n");
    Serial.println("  zero   <0-4> <deg>           servo-zero for IK joint n");
    Serial.println("  dir    <0-4> <+1|-1>         servo direction for IK joint n");
    Serial.println("  calibstate                   show mapping + snapshot table");
    Serial.println("  savecalib / loadcalib        persist zero/dir/offset to flash");
    Serial.println("  clearcalib                   delete calib file, reset to firmware defaults");
    Serial.println(" -- Manual servo control --");
    Serial.println("  j<n>                         select joint for +/- jogging");
    Serial.println("  j<n> <deg>                   set servo n to angle");
    Serial.println("  j<n>+ / j<n>-               nudge servo n by 5 deg");
    Serial.println("  + / -                        jog selected joint by 1 deg");
    Serial.println("  Enter (empty)                deselect joint");
    Serial.println("  home                         all servos to 90 deg");
    Serial.println("  status                       all servo angles");
    Serial.println(" -- Poses --");
    Serial.println("  save              append current position as next pose");
    Serial.println("  save <i>          insert at index i (0=start, shifts rest down)");
    Serial.println("  del <i>                      delete pose at index i (shifts rest down)");
    Serial.println("  setpose <i> sh el wp wr gr base   overwrite pose i with given servo degrees");
    Serial.println("  play / poses / clear / load");
    Serial.println();
    Serial.println("IK joints: [0]=base [1]=shoulder [2]=elbow [3]=wrist_pitch [4]=wrist_roll");
    Serial.println("Servo map: j0=shoulder j1=elbow j2=wrist_p j3=wrist_r j4=grabber j5=base");
}

// ══════════════════════════════════════════════════════════════════════════════
// Command handler
// ══════════════════════════════════════════════════════════════════════════════

static int tokenise(const String& s, String* tok, int maxn) {
    int n = 0;
    String rest = s; rest.trim();
    while (rest.length() && n < maxn) {
        int sp = rest.indexOf(' ');
        if (sp < 0) { tok[n++] = rest; break; }
        tok[n++] = rest.substring(0, sp);
        rest = rest.substring(sp + 1);
        rest.trim();
    }
    return n;
}

void handleCommand(const String& raw) {
    String s = raw; s.trim();
    Serial.println();
    if (!s.length()) return;

    String t[8]; int n = tokenise(s, t, 8);
    String cmd = t[0]; cmd.toLowerCase();

    if (cmd == "where" || cmd == "fk") {
        printWhere();

    } else if (cmd == "ikstatus") {
        printIKStatus();

    } else if (cmd == "status") {
        printStatus();

    } else if (cmd == "calibstate") {
        printCalibState();

    } else if (cmd == "help") {
        printHelp();

    } else if (cmd == "home") {
        uint16_t home[NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; j++) home[j] = degToTick(90);
        moveTo(home, 1500);
        syncIK();
        Serial.println("  Home (all servos 90 deg)");

    } else if (cmd == "xyz") {
        if (n < 4) { Serial.println("  Usage: xyz x y z [pitch_deg] [roll_deg]"); return; }
        float x = t[1].toFloat(), y = t[2].toFloat(), z = t[3].toFloat();
        float roll  = (n > 5) ? t[5].toFloat() : 0.0f;
        float pitch;
        if (n > 4) {
            pitch = t[4].toFloat();
        } else if (defaultPitch > -998.0f) {
            pitch = defaultPitch;
        } else {
            float r_auto = sqrtf(x*x + y*y);
            float p0 = constrain(atan2f(z - H1, max(r_auto, 0.1f)) * RAD_TO_DEG, 0.0f, 90.0f);
            pitch = 90.0f;  // fallback
            for (float p = p0; p <= 90.0f; p += 5.0f) {
                float tmp[NUM_IK_DOF];
                if (!ik(x, y, z, p, elbowUp, 0.0f, tmp, /*silent=*/true)) continue;
                bool ok = true;
                for (int i = 0; i < NUM_IK_DOF; i++) {
                    float sd = ikToServoDeg(i, tmp[i]);
                    if (sd < 0.0f || sd > 180.0f) { ok = false; break; }
                }
                if (ok) { pitch = p; break; }
            }
            Serial.printf("  (auto-pitch: %.0f deg)\n", pitch);
        }
        // Verify target is reachable before starting motion
        float out[NUM_IK_DOF];
        if (!ik(x, y, z, pitch, elbowUp, roll, out)) return;
        Serial.printf("  IK: base=%.1f  sh=%.1f  el=%.1f  wp=%.1f  wr=%.1f\n",
            out[0], out[1], out[2], out[3], out[4]);
        // Cartesian path: straight line in XYZ, re-runs IK at each step
        if (cartesianMove(x, y, z, pitch, roll, 1200))
            printWhere();
        else
            Serial.println("  Could not reach target via Cartesian path.");

    } else if (cmd == "calib") {
        if (n < 2) { Serial.println("  Targets: top  front  left  right   (calib save <name> to record)"); return; }
        String name = t[1]; name.toLowerCase();

        if (name == "save") {
            if (n < 3) { Serial.println("  Usage: calib save <name>"); return; }
            String tname = t[2]; tname.toLowerCase();
            bool found = false;
            for (int i = 0; i < NUM_CALIB_TARGETS; i++) {
                if (tname == CALIB_TARGETS[i].name) {
                    calibSnaps[i].valid = true;
                    for (int j = 0; j < NUM_JOINTS; j++)
                        calibSnaps[i].deg[j] = (uint8_t)tickToDeg(current[j]);
                    saveCalibSnaps();
                    Serial.printf("  Saved '%s': sh=%d el=%d wp=%d wr=%d gr=%d base=%d\n",
                        CALIB_TARGETS[i].name,
                        calibSnaps[i].deg[CH_SHOULDER], calibSnaps[i].deg[CH_ELBOW],
                        calibSnaps[i].deg[CH_WRIST_P],  calibSnaps[i].deg[CH_WRIST_R],
                        calibSnaps[i].deg[CH_GRABBER],  calibSnaps[i].deg[CH_BASE]);
                    found = true; break;
                }
            }
            if (!found) Serial.println("  Unknown target. Use: top / front / left / right");
            return;
        }

        bool found = false;
        for (int i = 0; i < NUM_CALIB_TARGETS; i++) {
            if (name == CALIB_TARGETS[i].name) {
                if (calibSnaps[i].valid) {
                    Serial.printf("  -> '%s' (snapshot)\n", CALIB_TARGETS[i].name);
                    uint16_t target[NUM_JOINTS];
                    for (int j = 0; j < NUM_JOINTS; j++)
                        target[j] = degToTick(calibSnaps[i].deg[j]);
                    moveTo(target, 1500);
                    syncIK();
                } else {
                    const CalibTarget& ct = CALIB_TARGETS[i];
                    Serial.printf("  -> '%s' via IK (no snapshot — run 'calib save %s' after positioning)\n",
                        ct.name, ct.name);
                    float out[NUM_IK_DOF];
                    if (ik(ct.x, ct.y, ct.z, ct.pitch, elbowUp, 0.0f, out)) {
                        Serial.printf("  IK: base=%.1f  sh=%.1f  el=%.1f  wp=%.1f  wr=%.1f\n",
                            out[0], out[1], out[2], out[3], out[4]);
                        if (!applyIK(out)) Serial.println("  Aborted: servo out of range.");
                    }
                }
                found = true; break;
            }
        }
        if (!found) Serial.println("  Unknown target. Use: top / front / left / right");

    } else if (cmd == "pitch") {
        if (n < 2 || t[1] == "auto") {
            defaultPitch = -999.0f;
            Serial.println("  Pitch: auto (computed from target geometry)");
        } else {
            defaultPitch = t[1].toFloat();
            Serial.printf("  Default pitch: %.1f deg (type 'pitch auto' to restore auto)\n", defaultPitch);
        }

    } else if (cmd == "elbow") {
        if (n < 2 || t[1] == "up")  { elbowUp = true;  Serial.println("  Elbow: up"); }
        else if (t[1] == "down")    { elbowUp = false; Serial.println("  Elbow: down"); }
        else Serial.println("  Usage: elbow up|down");

    } else if (cmd == "offset") {
        if (n < 3) { Serial.println("  Usage: offset <0-4> <deg>"); return; }
        int i = t[1].toInt();
        if (i < 0 || i >= NUM_IK_DOF) { Serial.println("  Joint 0-4 only"); return; }
        jmap[i].offset = t[2].toFloat();
        Serial.printf("  [%d] %s  offset=%+.2f\n", i, IK_NAMES[i], jmap[i].offset);

    } else if (cmd == "zero") {
        if (n < 3) { Serial.println("  Usage: zero <0-4> <servo_deg>"); return; }
        int i = t[1].toInt();
        if (i < 0 || i >= NUM_IK_DOF) { Serial.println("  Joint 0-4 only"); return; }
        jmap[i].zero = t[2].toFloat();
        Serial.printf("  [%d] %s  zero=%.1f\n", i, IK_NAMES[i], jmap[i].zero);

    } else if (cmd == "dir") {
        if (n < 3) { Serial.println("  Usage: dir <0-4> <+1|-1>"); return; }
        int i = t[1].toInt();
        if (i < 0 || i >= NUM_IK_DOF) { Serial.println("  Joint 0-4 only"); return; }
        jmap[i].dir = (t[2].toFloat() >= 0) ? 1.0f : -1.0f;
        Serial.printf("  [%d] %s  dir=%+.0f\n", i, IK_NAMES[i], jmap[i].dir);

    } else if (cmd == "savecalib") {
        saveCalib();

    } else if (cmd == "loadcalib") {
        loadCalib(); syncIK();

    } else if (cmd == "clearcalib") {
        LittleFS.remove(CALIB_FILE);
        jmap[0] = {CH_BASE,    100.0f, 1.0f, 0.0f};
        jmap[1] = {CH_SHOULDER, 80.0f, 1.0f, 0.0f};
        jmap[2] = {CH_ELBOW,    12.0f, 1.0f, 0.0f};
        jmap[3] = {CH_WRIST_P, 136.0f, 1.0f, 0.0f};
        jmap[4] = {CH_WRIST_R,  95.0f, 1.0f, 0.0f};
        syncIK();
        Serial.println("  Calib file deleted. Zeros reset to firmware defaults.");

    } else if (cmd == "save") {
        if (savedCount >= MAX_POSES) {
            Serial.printf("  Full (%d poses). Type 'clear' to reset.\n", MAX_POSES); return;
        }
        if (n >= 2) {
            // insert at position (1-based), shifting existing poses down
            int pos = t[1].toInt();
            if (pos < 0 || pos > savedCount) {
                Serial.printf("  Index must be 0-%d\n", savedCount); return;
            }
            for (int p = savedCount; p > pos; p--)
                memcpy(saved[p], saved[p-1], NUM_JOINTS * sizeof(uint16_t));
            memcpy(saved[pos], current, NUM_JOINTS * sizeof(uint16_t));
            savedCount++;
            Serial.printf("  Pose inserted at [%d] (total: %d/%d)\n", pos, savedCount, MAX_POSES);
        } else {
            memcpy(saved[savedCount++], current, NUM_JOINTS * sizeof(uint16_t));
            Serial.printf("  Pose %d/%d saved.\n", savedCount, MAX_POSES);
        }
        persistPoses();

    } else if (cmd == "poses") {
        printPoses();

    } else if (cmd == "del") {
        if (n < 2) { Serial.println("  Usage: del <index>"); return; }
        int idx = t[1].toInt();
        if (idx < 0 || idx >= savedCount) {
            Serial.printf("  Index must be 0-%d\n", savedCount - 1); return;
        }
        for (int p = idx; p < savedCount - 1; p++)
            memcpy(saved[p], saved[p+1], NUM_JOINTS * sizeof(uint16_t));
        savedCount--;
        Serial.printf("  Deleted pose [%d] (%d remaining)\n", idx, savedCount);
        persistPoses();

    } else if (cmd == "setpose") {
        // setpose <i> <sh> <el> <wp> <wr> <gr> <base>
        if (n < 8) { Serial.println("  Usage: setpose <i> <sh> <el> <wp> <wr> <gr> <base>"); return; }
        int idx = t[1].toInt();
        if (idx < 0 || idx >= savedCount) {
            Serial.printf("  Index must be 0-%d\n", savedCount - 1); return;
        }
        // channel order matches SRV_NAMES: sh=0 el=1 wp=2 wr=3 gr=4 base=5
        for (int j = 0; j < NUM_JOINTS; j++)
            saved[idx][j] = degToTick(constrain(t[2+j].toInt(), 0, 180));
        Serial.printf("  Pose [%d] updated\n", idx);
        printPoses();
        persistPoses();

    } else if (cmd == "load") {
        if (loadPoses()) Serial.printf("  Loaded %d poses.\n", savedCount);
        else Serial.println("  No pose file found.");

    } else if (cmd == "clear") {
        savedCount = 0;
        LittleFS.remove(POSES_FILE);
        Serial.println("  Poses cleared.");

    } else if (cmd == "play") {
        if (!savedCount) { Serial.println("  No poses saved."); return; }
        printPoses();
        Serial.println("  Starting in 2 s...");
        delay(2000);
        for (int p = 0; p < savedCount; p++) {
            Serial.printf("  -> pose %d\n", p+1);
            moveTo(saved[p], 1000);
            delay(500);
        }
        Serial.println("  Done.");

    } else if (cmd.length() >= 2 && cmd[0] == 'j') {
        int joint = cmd[1] - '0';
        if (joint < 0 || joint >= NUM_JOINTS) {
            Serial.println("  Invalid joint (0-5)"); return;
        }
        if (cmd.length() > 2 && cmd[2] == '+') {
            jogJoint(joint, JOG_STEP);
            syncIK();
            Serial.printf("  j%d %-12s %3d deg\n", joint, SRV_NAMES[joint], tickToDeg(current[joint]));
        } else if (cmd.length() > 2 && cmd[2] == '-') {
            jogJoint(joint, -JOG_STEP);
            syncIK();
            Serial.printf("  j%d %-12s %3d deg\n", joint, SRV_NAMES[joint], tickToDeg(current[joint]));
        } else if (n >= 2) {
            setJointDeg(joint, t[1].toInt());
            syncIK();
            Serial.printf("  j%d %-12s %3d deg\n", joint, SRV_NAMES[joint], tickToDeg(current[joint]));
        } else {
            selectedJoint = joint;
            jogPos[joint] = (float)tickToDeg(current[joint]);
            Serial.printf("  Selected j%d (%s) at %d deg — use +/- to jog\n",
                joint, SRV_NAMES[joint], tickToDeg(current[joint]));
        }

    } else {
        Serial.println("  Unknown command. Type 'help'.");
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// Setup / Loop
// ══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pca.begin();
    pca.setPWMFreq(50);
    LittleFS.begin(true);

    for (int j = 0; j < NUM_JOINTS; j++) current[j] = degToTick(90);
    moveTo(current, 1500);

    if (LittleFS.exists(CALIB_FILE)) loadCalib();
    loadCalibSnaps();
    syncIK();
    for (int j = 0; j < NUM_JOINTS; j++) jogPos[j] = (float)tickToDeg(current[j]);

    Serial.println("\n=== Robot Arm Ready ===");
    printWhere();

    if (loadPoses())
        Serial.printf("  %d pose(s) restored. Type 'play' to replay.\n", savedCount);
    else
        Serial.println("  No saved poses. Type 'help' for commands.");
}

void loop() {
    if (escState > 0 && (millis() - escTime) > 50) {
        if (escState == 1 && selectedJoint >= 0) {
            selectedJoint = -1;
            Serial.println("\n  Deselected — back to command mode.");
        }
        escState = 0;
    }

    while (Serial.available()) {
        char c = (char)Serial.read();

        if (escState == 1) {
            if (c == '[') { escState = 2; continue; }
            escState = 0;
        }
        if (escState == 2) {
            escState = 0;
            if (selectedJoint >= 0 && (c == 'A' || c == 'B'))
                c = (c == 'A') ? '+' : '-';
        }
        if (c == '\x1b') {
            escState = 1;
            escTime  = millis();
            continue;
        }

        if (c == '\n') {
            if (inputBuf.length() > 0) {
                handleCommand(inputBuf);
                inputBuf = "";
            } else if (selectedJoint >= 0) {
                selectedJoint = -1;
                Serial.println("\n  Deselected — back to command mode.");
            }
        } else if (c == '\r') {
        } else if (c == 127 || c == '\b') {
            if (inputBuf.length()) {
                inputBuf.remove(inputBuf.length() - 1);
                Serial.print("\b \b");
            }
        } else if (selectedJoint >= 0 && inputBuf.length() == 0 && (c == '+' || c == '-')) {
            static uint32_t lastKeyMs = 0;
            uint32_t now = millis();
            if (now - lastKeyMs >= 30) {
                lastKeyMs = now;
                jogPos[selectedJoint] += (c == '+') ? 1.0f : -1.0f;
                jogPos[selectedJoint]  = constrain(jogPos[selectedJoint], 0.0f, 180.0f);
                int target = (int)roundf(jogPos[selectedJoint]);
                current[selectedJoint] = degToTick(target);
                pca.setPWM(selectedJoint, 0, current[selectedJoint]);
                syncIK();
                Serial.printf("\r  j%d %-12s %3d deg    ",
                    selectedJoint, SRV_NAMES[selectedJoint], target);
            }
            while (Serial.available()) Serial.read();
        } else {
            inputBuf += c;
            Serial.print(c);
        }
    }
}
