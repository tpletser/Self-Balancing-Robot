
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


#include<Wire.h>
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
int32_t counter;

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

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  counter = 0;
  
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();

  if(counter>1000){
    counter =0;
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);
     
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng));
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
     
    Serial.print("AngleX= ");
    Serial.println(x);
     
    Serial.print("AngleY= ");
    Serial.println(y);
     
    Serial.print("AngleZ= ");
    Serial.println(z);
    Serial.println("-----------------------------------------");
     
    //Blynk.virtualWrite(V2, x);
    Blynk.virtualWrite(V2, y);
    //Blynk.virtualWrite(V4, z);
  } 
  counter++;
}
