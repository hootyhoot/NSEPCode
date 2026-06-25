#include <Arduino.h>

// TCS3200

#define S2  6
#define S3  7
#define OUT 8

int redValue;
int greenValue;
int blueValue;

int readColor(bool s2, bool s3)
{
    digitalWrite(S2, s2);
    digitalWrite(S3, s3);

    delay(20);

    // Lower value = stronger colour
    return pulseIn(OUT, LOW, 30000);
}

void setup()
{
    Serial.begin(9600);

    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(OUT, INPUT);

    Serial.println("================================");
    Serial.println("GY-31 TCS3200 COLOR SENSOR TEST");
    Serial.println("================================");
}

void loop()
{
    // Read RED
    redValue = readColor(LOW, LOW);

    // Read BLUE
    blueValue = readColor(LOW, HIGH);

    // Read GREEN
    greenValue = readColor(HIGH, HIGH);

    Serial.print("R: ");
    Serial.print(redValue);

    Serial.print("   G: ");
    Serial.print(greenValue);

    Serial.print("   B: ");
    Serial.println(blueValue);

    // -------------------------
    // Simple colour detection
    // -------------------------

    int diff = greenValue - redValue;

if(diff > 300)
{
    Serial.println("Detected: RED");
}
else if(diff > 0)
{
    Serial.println("Detected: YELLOW");
}
else
{
    Serial.println("Detected: BLUE");
}
/*
    // ---------- RED ----------
        if(redValue < greenValue)
        {
            Serial.println("Detected: RED");
        }

        // ---------- BLUE ----------
        else if(redValue > greenValue && blueValue >= 290 && blueValue<= 410)
        {
            Serial.println("Detected: BLUE");
        }

        // ---------- YELLOW ----------
        else if(redValue >= 750 && redValue <= 1200)
        //else if (greenValue > blueValue)
        {
            Serial.println("Detected: YELLOW");
        }

        else
        {
            Serial.println("Detected: UNKNOWN");
        } */

            Serial.println("--------------------------------");

            delay(500);
        }