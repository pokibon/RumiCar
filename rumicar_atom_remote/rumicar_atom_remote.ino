//=========================================================
//  rumicar_atom_remote.ino :  RumiCar ReMote Control
//  History     : V0.0  2020-06-19 Support newcreate
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
#define EXTERN extern
#include "RumiCar_atom.h"
/*
#ifdef  SENSOR_VL53L1X
VL53L1X sensor0;
VL53L1X sensor1;
VL53L1X sensor2;
#else
VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;
#endif
*/
//=========================================================
//  Bylnk BLE definition
//=========================================================
char auth[] = "mcJxlsL7QpjgwQ6Rr-OGJVhbWw9jWiSp";
#define DEVICE_NAME "ByRumiCarRM"
int16_t Joy_X = 0;
int16_t Joy_Y = 0;
int16_t AutoPilot = 1;
int16_t Trim = 0;
int16_t Max_Speed = 150;
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

  lcd.clear();              // Smart Phone LCD Display
#endif  
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

  Blynk.run();

  char buf[32];
    
  //=========================================================
  //  get VL53L0X TOF sensor value 
  //    S0: left S1: center S2: right
  //========================================================/
/*  
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
  if (loop_count == 0) {
    sprintf( buf, "%3dcm%3dcm%3dcm ", s0/10, s1/10, s2/10);
    lcd.print(0,0, " LEFT CENT.RIGHT");
    lcd.print(0, 1, buf);
  }
#endif
*/
/*
  Serial.print("  Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor1:");
  Serial.print(s1);
  Serial.print("  Sensor2:");
  Serial.println(s2);
*/
  manual_pilot();
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
