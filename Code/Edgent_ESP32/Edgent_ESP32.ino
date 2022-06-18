
// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPL_NhPakMf"
#define BLYNK_DEVICE_NAME "Balancing Robot"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
#define USE_WROVER_BOARD

#include "BlynkEdgent.h"


#define m1pin1 A5
#define m1pin2 A18
#define m1PWM A4
#define m2pin1 A19
#define m2pin2 A17
#define m2PWM A16


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

double AngleX;
double AngleY;
double AngleZ;
int32_t lastMillis;
double cycleRate;
double sampleRate;
double timeConstant;
double alpha;
double temperature;

BLYNK_WRITE(V0)
{
  int LeftMotor = param.asInt();

  if (LeftMotor > 0) {
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin1, HIGH);
    digitalWrite(m2pin2, LOW);
  }
  else {
    digitalWrite(m1pin1, LOW);
    digitalWrite(m1pin2, HIGH);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
  }
  analogWrite(m1PWM, abs(LeftMotor));
  analogWrite(m2PWM, abs(LeftMotor));
}

BLYNK_WRITE(V1)
{
  int test = param.asInt();
  digitalWrite(A15 , test);
  Serial.print("Blynk.Cloud is writing something to V1  : ");
  Serial.println(test);
}

void setup()
{
  pinMode(m1pin1, OUTPUT);
  pinMode(m1pin2, OUTPUT);
  pinMode(m1PWM, OUTPUT);
  pinMode(m2pin1, OUTPUT);
  pinMode(m2pin2, OUTPUT);
  pinMode(m2PWM, OUTPUT);

  digitalWrite(m1pin1, LOW);
  digitalWrite(m1pin2, LOW);
  digitalWrite(m1PWM, LOW);
  digitalWrite(m2pin1, LOW);
  digitalWrite(m2pin2, LOW);
  digitalWrite(m2PWM, LOW);

  Serial.begin(115200);
  delay(100);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastMillis = 0;
  sampleRate = 100; // in ms
  AngleX = 0;
  AngleY = 0;
  AngleZ = 0;
  timeConstant = 1;
  alpha = timeConstant / (timeConstant + sampleRate / 1000) ;


  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();

  if (millis() - lastMillis > sampleRate) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    temperature = temp.temperature;
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    double AGx, AGy, AGz;
    AGx = AngleX + RAD_TO_DEG * g.gyro.x * sampleRate / 1000;
    AGy = AngleY - RAD_TO_DEG * g.gyro.y * sampleRate / 1000;
    AGz = AngleZ + RAD_TO_DEG * g.gyro.z * sampleRate / 1000;

    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad");
    double AAccx, AAccy, AAccz;
    AAccx = RAD_TO_DEG * (atan2(-a.acceleration.y, -a.acceleration.z));
    AAccy = RAD_TO_DEG * (atan2(-a.acceleration.x, -a.acceleration.z));
    AAccz = RAD_TO_DEG * (atan2(-a.acceleration.y, -a.acceleration.x));
    Serial.print("Acc angle X: ");
    Serial.print(AAccx);
    Serial.print(", Y: ");
    Serial.print(AAccy);
    Serial.print(", Z: ");
    Serial.print(AAccz);
    Serial.println(" deg");

    AngleX = alpha * AGx + (1 - alpha) * AAccx;
    AngleY = alpha * AGy + (1 - alpha) * AAccy;
    AngleZ = alpha * AGz + (1 - alpha) * AAccz;
    Serial.print("Comp angle X: ");
    Serial.print(AngleX);
    Serial.print(", Y: ");
    Serial.print(AngleY);
    Serial.print(", Z: ");
    Serial.print(AngleZ);
    Serial.println(" deg");
    
  }
  if (millis() - lastMillis > 500) {//augmenter delay quand c'est controllé
    Blynk.virtualWrite(V2, AngleY);
  }
  if (millis() - lastMillis > 2000) {
    Blynk.virtualWrite(V6, temperature);
  }
}
