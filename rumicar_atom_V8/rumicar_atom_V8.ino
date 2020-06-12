//=========================================================
//  rumicar_atom_V8.ino :  RumiCar application 
//  History     : V8.0  2020-06-11 New Create(K.Ohe)
//                      Support Bylnk Remoto Control
//                      Support VL53L1X distance sensor
//                      Support reverse steering
//=========================================================
#define USE_BLYNK
#ifdef USE_BLYNK
#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#endif
#include "M5Atom.h"
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#define EXTERN extern
#include "RumiCar_atom.h"

#ifdef  SENSOR_VL53L1X
VL53L1X sensor0;
VL53L1X sensor1;
VL53L1X sensor2;
#else
VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;
#endif
//=========================================================
//  Bylnk definition
//=========================================================
char auth[] = "mcJxlsL7QpjgwQ6Rr-OGJVhbWw9jWiSp";
#define DEVICE_NAME "ByRumiCar"
int16_t Joy_X = 0;
int16_t Joy_Y = 0;
int16_t AutoPilot = 1;
int16_t Trim = 0;
int16_t Max_Speed = 120;
#ifdef USE_BLYNK
WidgetLCD lcd(V3);
#endif
//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  RC_setup();
#ifdef USE_BLYNK
  Blynk.setDeviceName(DEVICE_NAME);
  Blynk.begin(auth);

  lcd.clear();
#endif  
}

//=========================================================
//  auto pilot function
//=========================================================
#define MAX_SPEED 1.0
#define MID_SPEED 0.9
#define LOW_SPEED 0.8
#define BRAKE_TIME 1000
int s0, s1, s2;
int CurDir = BRAKE;         // current direction
int LastDir = BRAKE;        // last direction
int sTime, eTime = 0;        //
int iBuf = 0;
void auto_pilot()
{
  //=========================================================
  //  drive  
  //=========================================================
  if(s1 < 100){
    CurDir = REVERSE;
    RC_drive(CurDir, Max_Speed * MAX_SPEED);
  }else {
    if (s1 > 250) {
      CurDir = FORWARD;
      RC_drive(CurDir, Max_Speed * MAX_SPEED);
    } else {
      CurDir = FORWARD;
      if (s1 > 150) {
        RC_drive(CurDir, Max_Speed * MID_SPEED);
      }else {
        RC_drive(CurDir, Max_Speed * LOW_SPEED);
      }
    }
  }
  if (LastDir != FORWARD && CurDir == FORWARD) {
    sTime = millis();
    eTime = 0;
//    Serial.print("REVERSE to FORWARD : ");
//    Serial.println(eTime);
  }
  if (LastDir == FORWARD && CurDir == FORWARD) {
    eTime = millis() - sTime;
    if (eTime > 10000) {
      eTime = 10000;
    }
//    Serial.print("FORWARD to FORWARD : ");
//    Serial.println(eTime);
  }
  if (eTime > BRAKE_TIME && CurDir == REVERSE) {
//    Serial.print("BRAKEING           : ");
//    Serial.println(eTime);
    CurDir = BRAKE;      
  }
  LastDir = CurDir;  
  //=========================================================
  //  steer  
  //=========================================================
  int dMin = 50;
  int dMax = 100;
  int steerMax = 100;
  int dAngle;
  int dDir = CENTER;
  
  if   (s0 > dMax) s0 = dMax;
  else if (s0 < dMin) s0 = dMin;
  if (s2 > dMax) s2 = dMax;
  else if (s2 < dMin) s2 = dMin;
 
  if (s0 >= dMax && s2 >= dMax) {
    dDir = CENTER;
    dAngle = 0;
  } else if (s0 < dMax && s2 >= dMax) {
    dDir = RIGHT;
    dAngle = steerMax - (s0 - dMin) * dMax/dMin; 
  } else if (s0 >= dMax && s2 < dMax) {
    dDir = LEFT;
    dAngle = steerMax - (s2 - dMin) * dMax/dMin; 
  } else {
    dAngle = (s0 - s2) * 2;
    if (dAngle > 10) {
      dDir = LEFT;
    } else if (dAngle < -10) {
      dDir = RIGHT;
      dAngle = abs(dAngle);      
    } else {
      dDir = CENTER;
    }
  }
  if (CurDir == REVERSE) {
    if (dDir == RIGHT) {
      dDir = LEFT;
    } else if (dDir == LEFT) {
      dDir = RIGHT;
    }   
  }
  RC_steer(dDir, dAngle);
// /*  
//  Serial.print("dDir : ");
//  Serial.print(dDir);
//  Serial.print("  dAngle : ");
//  Serial.println(dAngle);
// */
}

//=========================================================
//  manual pilot function
//=========================================================
void manual_pilot()
{
  int sDrive;
  int angle;
  int K_OFF = 50;    // original 64
  
  if (Joy_Y > 10) {
    sDrive = constrain( Joy_Y + K_OFF, 0, Max_Speed);
    RC_drive(FORWARD, sDrive);
  } else if (Joy_Y < -10) {
    sDrive = constrain(-Joy_Y + K_OFF, 0, Max_Speed);
    RC_drive(REVERSE, sDrive);    
  } else {
    RC_drive(BRAKE, 0); 
  }
  angle = constrain(Joy_X + Trim, -100, 100);
  if (angle > 0) {
    RC_steer(RIGHT, angle);
  } else if (angle < 0) {
    RC_steer(LEFT, -angle);    
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
#ifdef USE_BLYNK
  Blynk.run();
#endif
  char buf[32];
    
  //=========================================================
  //  get VL53L0X TOF sensor value 
  //    S0: left S1: center S2: right
  //=========================================================
#ifdef SENSOR_VL53L1X
  //s0=sensor0.readRangeContinuousMillimeters();
  //s1=sensor1.readRangeContinuousMillimeters();
  //s2=sensor2.readRangeContinuousMillimeters();
  s0=sensor0.read();
  s1=sensor1.read();
  s2=sensor2.read();
#else
  s0=sensor0.readRangeSingleMillimeters();
  s1=sensor1.readRangeSingleMillimeters();
  s2=sensor2.readRangeSingleMillimeters();
#endif
#ifdef USE_BLYNK
  sprintf( buf, "%3dcm%3dcm%3dcm ", s0/10, s1/10, s2/10);
  lcd.print(0,0, " LEFT CENT.RIGHT");
  lcd.print(0, 1, buf);
#endif
/*
  Serial.print("Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor1:");
  Serial.print(s1);
  Serial.print("  Sensor2:");
  Serial.println(s2);
*/
  if (AutoPilot == 1) {
    auto_pilot();
  } else {
    manual_pilot();
  }
}

//=========================================================
//  Bylnk Callback functions
//=========================================================
// V0: Auto Pilot Switch
#ifdef USE_BLYNK
BLYNK_WRITE(V0) {
  AutoPilot = param[0].asInt();
//  Serial.print("AutoPilot = ");
//  Serial.println(AutoPilot);  
}
// V1: Joy Stick
BLYNK_WRITE(V1) {
  Joy_X = param[0].asInt();
  Joy_Y = param[1].asInt();
//  Serial.print("X = ");
//  Serial.print(Joy_X);
//  Serial.print("  Y = ");
//  Serial.println(Joy_Y);
}

// V2: Trim
BLYNK_WRITE(V2) {
  Trim = param[0].asInt();
//  Serial.print("Trim = ");
//  Serial.println(Trim);
}

// V4: Max Speed
BLYNK_WRITE(V4) {
  Max_Speed = param[0].asInt();
//  Serial.print("Max_Speed = ");
//  Serial.println(Max_Speed);
}
#endif
