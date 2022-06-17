
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

BLYNK_WRITE(V0)
{
 int LeftMotor = param.asInt();
  
  if (LeftMotor >0){
    digitalWrite(m1pin1,HIGH);
    digitalWrite(m1pin2,LOW);
    digitalWrite(m2pin1,HIGH);
    digitalWrite(m2pin2,LOW);
  }
  else{
    digitalWrite(m1pin1,LOW);
    digitalWrite(m1pin2,HIGH);
    digitalWrite(m2pin1,LOW);
    digitalWrite(m2pin2,HIGH);
  }
  analogWrite(m1PWM,abs(LeftMotor));
  analogWrite(m2PWM,abs(LeftMotor));
}

BLYNK_WRITE(V1)
{
  int test = param.asInt();
  digitalWrite(A15 ,test);
  Serial.print("Blynk.Cloud is writing something to V1  : ");
  Serial.println(test);
}

void setup()
{
   pinMode(m1pin1,OUTPUT);
    pinMode(m1pin2,OUTPUT);
    pinMode(m1PWM,OUTPUT);
  pinMode(m2pin1,OUTPUT);
  pinMode(m2pin2,OUTPUT);
  pinMode(m2PWM,OUTPUT);

  digitalWrite(m1pin1,LOW);
  digitalWrite(m1pin2,LOW);
  digitalWrite(m1PWM,LOW);
  digitalWrite(m2pin1,LOW);
  digitalWrite(m2pin2,LOW);
  digitalWrite(m2PWM,LOW);

  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();
}
