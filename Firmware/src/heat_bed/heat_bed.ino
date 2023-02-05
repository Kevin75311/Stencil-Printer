
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define THERMISTOR_PIN A0
#define RELAY_PIN 5

LiquidCrystal_I2C lcd(0x27, 20, 4);
uint32_t lastUpdate, currentTime = 0;

// Temp read
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// PID control
double Setpoint, Input, Output;
double Kp = 40, Ki = 0, Kd = 3;
int WindowSize = 1000;
uint32_t windowStartTime = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float AveTemp = 0;
const int cnt = 64;
float RunAveBuf[cnt];
int NextAve;

void setup()
{
    Serial.begin(115200);
    lcd.init();
    lcd.backlight();

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    windowStartTime = millis();
    //initialize the variables we're linked to
    Setpoint = 40.0;

    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WindowSize);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    if (Serial.available())
    {
        Setpoint = Serial.parseFloat();
    }

    Tc = readTemp();
    Input = Tc;

    currentTime = millis();
    if (currentTime - lastUpdate >= 500)
    {
        lcd.setCursor(0, 0);
        lcd.print("PV:");
        lcd.print(Input);
        lcd.setCursor(0, 1);
        lcd.print("SV:");
        lcd.print(Setpoint);
        lastUpdate = currentTime;
    }

    // data averaging
    RunAveBuf[NextAve++] = Tc;
    if (NextAve >= cnt)
    {
        NextAve = 0;
    }
    float RunningAverageTemperature = 0;
    for (int i = 0; i < cnt; ++i)
    {
        RunningAverageTemperature += RunAveBuf[i];
    }
    RunningAverageTemperature /= cnt;

    // Serial.print(RunningAverageTemperature);
    // Serial.print(",");
    // Serial.println(Setpoint);
    sendToPC((float*)&Setpoint, &RunningAverageTemperature);

    myPID.Compute();

    // turn the output pin on/off based on pid output

    if (millis() - windowStartTime > WindowSize)
    {
        //time to shift the Relay Window
        windowStartTime += WindowSize;
    }
    if (Output < millis() - windowStartTime)
        digitalWrite(RELAY_PIN, LOW);
    else
        digitalWrite(RELAY_PIN, HIGH);
}

float readTemp()
{
    Vo = analogRead(THERMISTOR_PIN);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    return (T - 273.15);
}

void sendToPC(float* data1, float* data2)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3]};
  Serial.write(buf, 8);
}