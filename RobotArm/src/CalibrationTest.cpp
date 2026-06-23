#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int MIN_PULSE = 75;
const int MAX_PULSE = 3000;
const int MAX_ANGLE = 270;
const int NUM_SERVOS = 6;
const int NUM_POSES = 9;  // Poses a-g

// --- Channel mapping ---------------------------------------------------
// Adjust these to match how your servos are actually wired to the PCA9685.
const int CH_SHOULDER   = 0;
const int CH_ELBOW      = 1;
const int CH_WRIST      = 2;
const int CH_WRIST_ROT  = 3;
const int CH_GRIPPER    = 4;
const int CH_BASE       = 5;

// --- Homing speed control ----------------------------------------------
// HOME_STEP  = microseconds moved per increment (lower = smoother/slower)
// HOME_DELAY = milliseconds paused between increments (higher = slower)
const int HOME_STEP  = 5;
const int HOME_DELAY = 20;

// --- Homing target positions (in microseconds) --------------------------
// 1500 = roughly centered. Adjust CH_GRIPPER's value to your measured
// "open" PWM value for your gripper hardware.
int HOME_PWM[NUM_SERVOS] = {
  1795, // CH_SHOULDER
  1460, // CH_ELBOW
  1510, // CH_WRIST
  1500, // CH_WRIST_ROT
  790,  // CH_GRIPPER  
  1500 // CH_BASE
};

int activeChannel = 0;
int stepSize = 5;

int currentPWM[NUM_SERVOS] = {1500, 1500, 1500, 1500, 1500, 1500};

int savedPoses[NUM_POSES][NUM_SERVOS] = {0};
bool poseSet[NUM_POSES] = {false};

int poseIndex(char c)
{
    return c - 'a';  // a=0, b=1, c=2 ...
}

void moveToPose(int index)
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        currentPWM[i] = savedPoses[index][i];
        pwm.writeMicroseconds(i, currentPWM[i]);
    }
}

void printPose(int index)
{
    Serial.print("\n--- Pose ");
    Serial.print((char)('a' + index));
    Serial.println(" ---");
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        int angle = map(savedPoses[index][i], MIN_PULSE, MAX_PULSE, 0, MAX_ANGLE);
        Serial.print("Ch[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(savedPoses[index][i]);
        Serial.print(" us | ~");
        Serial.print(angle);
        Serial.println("°");
    }
}

void printAllPoses()
{
    Serial.println("\n=== Saved Poses ===");
    for (int i = 0; i < NUM_POSES; i++)
    {
        if (poseSet[i])
        {
            printPose(i);
        }
        else
        {
            Serial.print("Pose ");
            Serial.print((char)('a' + i));
            Serial.println(": empty");
        }
    }
}

// Moves a single channel from its current PWM to a target PWM in small
// increments, with a short delay between each step, so the motion is
// slow and smooth instead of an instant jump.
void moveServoSmooth(int channel, int targetPWM, int incrementSize, int delayMs)
{
    int current = currentPWM[channel];
    if (current == targetPWM) return;

    int direction = (targetPWM > current) ? 1 : -1;

    while (current != targetPWM)
    {
        current += direction * incrementSize;

        // Prevent overshooting past the target on the final step
        if ((direction == 1 && current > targetPWM) ||
            (direction == -1 && current < targetPWM))
        {
            current = targetPWM;
        }

        pwm.writeMicroseconds(channel, current);
        currentPWM[channel] = current;
        delay(delayMs);
    }
}

// Sequential homing: gripper -> wrist -> elbow -> shoulder -> base.
// Each joint moves smoothly to its home position before the next one starts.
void homeArm()
{
    Serial.println("\n>>> HOMING SEQUENCE STARTED <<<");

    Serial.println("Step 1/5: Opening gripper...");
    moveServoSmooth(CH_GRIPPER, HOME_PWM[CH_GRIPPER], HOME_STEP, HOME_DELAY);

    Serial.println("Step 2/5: Centering wrist...");
    moveServoSmooth(CH_WRIST_ROT, HOME_PWM[CH_WRIST_ROT], HOME_STEP, HOME_DELAY);
    moveServoSmooth(CH_WRIST, HOME_PWM[CH_WRIST], HOME_STEP, HOME_DELAY);

    Serial.println("Step 3/5: Centering elbow...");
    moveServoSmooth(CH_ELBOW, HOME_PWM[CH_ELBOW], HOME_STEP, HOME_DELAY);

    Serial.println("Step 4/5: Centering shoulder...");
    moveServoSmooth(CH_SHOULDER, HOME_PWM[CH_SHOULDER], HOME_STEP, HOME_DELAY);

    Serial.println("Step 5/5: Centering base...");
    moveServoSmooth(CH_BASE, HOME_PWM[CH_BASE], HOME_STEP, HOME_DELAY);

    Serial.println(">>> HOMING COMPLETE <<<\n");
}

void setup()
{
    Serial.begin(9600);
    while (!Serial);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    delay(10);

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        pwm.writeMicroseconds(i, currentPWM[i]);
        delay(150);
    }

    Serial.println("\n--- 6DOF Calibrator + Pose Recorder ---");
    Serial.println("0-5         = select channel");
    Serial.println("W / +       = increase PWM");
    Serial.println("S / -       = decrease PWM");
    Serial.println("R + a-g     = record pose to slot");
    Serial.println("a-g         = go to saved pose");
    Serial.println("L           = list all saved poses");
    Serial.println("H           = slow homing sequence (gripper->wrist->elbow->shoulder->base)");
    Serial.println("---------------------------------------");
    Serial.println(">>> Channel: 0");
}

void loop()
{
    if (Serial.available() > 0)
    {
        char inChar = Serial.read();

        if (inChar == '\n' || inChar == '\r' || inChar == ' ')
            return;

        // H = slow homing sequence
        if (inChar == 'h' || inChar == 'H')
        {
            homeArm();
            return;
        }

        // L = list all poses
        if (inChar == 'l' || inChar == 'L')
        {
            printAllPoses();
            return;
        }

        // R = record pose
        if (inChar == 'r' || inChar == 'R')
        {
            Serial.println("Enter slot (a-i) to save pose:");
            while (!Serial.available());
            char slot = tolower(Serial.read());

            if (slot >= 'a' && slot <= 'g')
            {
                int index = poseIndex(slot);
                for (int i = 0; i < NUM_SERVOS; i++)
                {
                    savedPoses[index][i] = currentPWM[i];
                }
                poseSet[index] = true;
                Serial.print("Pose saved to slot ");
                Serial.println(slot);
                printPose(index);
            }
            else
            {
                Serial.println("Invalid slot. Use a-g.");
            }
            return;
        }

        // a-i = go to saved pose
        if (inChar >= 'a' && inChar <= 'g')
        {
            int index = poseIndex(inChar);
            if (poseSet[index])
            {
                Serial.print("\nMoving to Pose ");
                Serial.println(inChar);
                moveToPose(index);
                printPose(index);
            }
            else
            {
                Serial.print("Pose ");
                Serial.print(inChar);
                Serial.println(" is empty. Use R to record.");
            }
            return;
        }

        // 0-5 = select channel
        if (inChar >= '0' && inChar <= '5')
        {
            activeChannel = inChar - '0';
            Serial.print("\n>>> Switched to Channel: ");
            Serial.println(activeChannel);
            return;
        }

        bool moved = false;

        if (inChar == 'w' || inChar == 'W' || inChar == '+')
        {
            currentPWM[activeChannel] += stepSize;
            moved = true;
        }
        else if (inChar == 's' || inChar == 'S' || inChar == '-')
        {
            currentPWM[activeChannel] -= stepSize;
            moved = true;
        }
        else
        {
            Serial.println("Invalid key.");
        }

        if (moved)
        {
            currentPWM[activeChannel] = constrain(currentPWM[activeChannel], MIN_PULSE, MAX_PULSE);
            pwm.writeMicroseconds(activeChannel, currentPWM[activeChannel]);

            int approxAngle = map(currentPWM[activeChannel], MIN_PULSE, MAX_PULSE, 0, MAX_ANGLE);

            Serial.print("Ch[");
            Serial.print(activeChannel);
            Serial.print("] | PWM: ");
            Serial.print(currentPWM[activeChannel]);
            Serial.print(" us | ~");
            Serial.print(approxAngle);
            Serial.println("°");
        }
    }
}