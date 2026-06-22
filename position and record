#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int MIN_PULSE = 75;
const int MAX_PULSE = 3000;
const int MAX_ANGLE = 270;
const int NUM_SERVOS = 6;
const int NUM_POSES = 9;  // Poses a-i

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
    Serial.println("R + a-i     = record pose to slot");
    Serial.println("a-i         = go to saved pose");
    Serial.println("L           = list all saved poses");
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

            if (slot >= 'a' && slot <= 'i')
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
                Serial.println("Invalid slot. Use a-i.");
            }
            return;
        }

        // a-i = go to saved pose
        if (inChar >= 'a' && inChar <= 'i')
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
