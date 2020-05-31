//=========================================================
//  rumicar_atom_V5.ino :  RumiCar application 
//  History     : V5.0  2020-05-29 New Create(K.Ohe)
//                  Support Bylnk Remoto Control
//=========================================================
#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include "M5Atom.h"
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#define EXTERN extern
#include "RumiCar_atom.h"

VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;

//=========================================================
//  Bylnk definition
//=========================================================
char auth[] = "mcJxlsL7QpjgwQ6Rr-OGJVhbWw9jWiSp";
#define DEVICE_NAME "ByRumiCar"
int16_t Joy_X;
int16_t Joy_Y;
int16_t AutoPilot = 1;

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  RC_setup();
  Blynk.setDeviceName(DEVICE_NAME);
  Blynk.begin(auth);  
}

//=========================================================
//  auto pilot function
//=========================================================
int iBuf = 0;
void auto_pilot()
{
  int s0, s1, s2;
  int ispeed = 255;
  int idist1;
  int sDist = 150;
  int K_OFF = 50;    // original 64
  int KP = 200;
  int KI = 20;
  int sP;
  long sI;
  int sDrive;
  int dDist;  

  //=========================================================
  //  get VL53L0X TOF sensor value 
  //    S0: left S1: center S2: right
  //=========================================================
  //s0=sensor0.readRangeContinuousMillimeters();
  //s1=sensor1.readRangeContinuousMillimeters();
  //s2=sensor2.readRangeContinuousMillimeters();
  s0=sensor0.readRangeSingleMillimeters();
  s1=sensor1.readRangeSingleMillimeters();
  s2=sensor2.readRangeSingleMillimeters();
/*
  Serial.print("Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor1:");
  Serial.print(s1);
  Serial.print("  Sensor2:");
  Serial.println(s2);
*/
  //=========================================================
  //  drive  
  //=========================================================
  //idist1=sensor1.readRangeSingleMillimeters();  // 距離計測
  idist1 = s1;
  dDist = idist1 - sDist;                         // 差分

  if ( idist1 < 1500 ){                           // 150cm 以内のみ動作
    sP = (dDist * KP) / 100;                      // P項
    iBuf += (sP * KI);                            // 積分
    sI = iBuf >> 8;                               // I項
    sDrive = sP + sI;                             // 指令値
//    constrain(sDrive,-255,255);                 // Limit
    sDrive = constrain(sDrive,-50, 50);                   // Limit
//    sDrive = sP;
    if ( dDist > 5 ){                             // Direction
      RC_drive(FORWARD,(sDrive + K_OFF));
    }else if (dDist < -5){
      RC_drive(REVERSE,((-sDrive) + K_OFF));
      }else{                                      // +-5mm 以内は停止
        RC_drive(BRAKE,ispeed);                   // Brake
        iBuf = 0;                                 // 積分クリア
      }
  }else{
      RC_drive(FREE,ispeed);                      // Free
      iBuf = 0;                                   // 積分クリア
  }
  //=========================================================
  //  steer  
  //=========================================================
  int dAngle;
  if   (s0 > 250) s0 = 250;
  else if (s0 < 50) s0 = 50;
  if (s2 > 250) s2 = 250;
  else if (s2 < 50) s2 = 50;
  dAngle = (s0 - s2) / 2;
  if (dAngle > 10) RC_steer(LEFT, abs(dAngle));
  else if (dAngle < -10) RC_steer(RIGHT, abs(dAngle));  
  else RC_steer(CENTER);
}

//=========================================================
//  manual pilot function
//=========================================================
void manual_pilot()
{
  int sDrive;
  int K_OFF = 50;    // original 64
  if (Joy_Y > 0) {
    sDrive = constrain( Joy_Y + K_OFF, 0, 255);
    RC_drive(FORWARD, sDrive);
  } else if (Joy_Y < 0) {
    sDrive = constrain(-Joy_Y + K_OFF, 0, 255);
    RC_drive(REVERSE, sDrive);    
  } else {
    RC_drive(BRAKE, 0); 
  }
  if (Joy_X > 0) {
    RC_steer(RIGHT, Joy_X);
  } else if (Joy_X < 0) {
    RC_steer(LEFT,-Joy_X);    
  } else {
    RC_steer(CENTER); 
  }
}

//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
  M5.update();
  Blynk.run();
  if (AutoPilot == 1) {
    auto_pilot();
  } else {
    manual_pilot();
  }
}

//=========================================================
//  Bylnk Callback functions
//=========================================================
BLYNK_WRITE(V0) {
  AutoPilot = param[0].asInt();
  Serial.println(AutoPilot);
  if(AutoPilot == 1){
    M5.dis.drawpix(0, 0x000000);
  }
  if(AutoPilot == 0){
    M5.dis.drawpix(0, 0xf00000);
  }
}

BLYNK_WRITE(V1) {
  Joy_X = param[0].asInt();
  Joy_Y = param[1].asInt();
  Serial.print("X = ");
  Serial.print(Joy_X);
  Serial.print("  Y = ");
  Serial.println(Joy_Y);
}
