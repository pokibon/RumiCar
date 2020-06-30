//=========================================================
//  rumicar_atom_BT_V0.ino :  RumiCar test application 
//  History     : V2.0  2020-06-21 Support VL53L1X
//=========================================================
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include <Wire.h>
#define EXTERN extern
#include "RumiCar_atom.h"
#include "BluetoothSerial.h"

#define MAX_SPEED 0.7

VL53L1X sensor0;                  // create right sensor instanse
VL53L1X sensor1;                  // create front sensor instance
VL53L1X sensor2;                  // create left  sensor instance

//#define BT_ON

#ifdef BT_ON
BluetoothSerial SerialBT;
#endif
//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
//  uint16_t ROI_X, ROI_Y;
  Serial.begin(115200);  
  RC_setup();               //   RumiCar initial function
#ifdef BT_ON
  Serial.println("Start bluetooth!");
  SerialBT.begin("RumiCar_ESP32");
  Serial.println("The device started, now you can pair it with bluetooth!");
#endif
}
bool dirFlag = true;
float p;
float prep;
float d;
unsigned long preTime = 0;
//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
  int s0, s1, s2;
  int driveDir;
  int drivePower;

  M5.update();
  //=========================================================
  //  get VL53L0X TOF sensor value 
  //    S0: left S1: center S2: right
  //=========================================================
  s0=sensor0.read();        // read left  sensor
  s1=sensor1.read();        // read front sensor
  s2=sensor2.read();        // read right sensor
/*
  if (dirFlag) RC_steer(RIGHT, 100);
  else         RC_steer(LEFT,  100);
  dirFlag = !dirFlag;
  delay(2000);
*/


  if(s1<100){
    RC_drive(REVERSE,150);
  }else if (s1<150){
    RC_drive(FORWARD,150 * MAX_SPEED);
  }else if (s1<250){
    RC_drive(FORWARD,200 * MAX_SPEED);
  }else{
    RC_drive(FORWARD,255 * MAX_SPEED);
  }
  //=========================================================
  //  steer  
  //=========================================================
  #define Kp      0.3
  #define Kd      0.05
  int dAngle;
  int targetPos;
  int curPos;
  unsigned long dt;
  unsigned long t; 

  targetPos = (s0 + s2) / 2;
  curPos = s2;
  p = (targetPos - curPos) * Kp;

  t = millis();
  dt = t - preTime;
  preTime = t;

  d = (p - prep) / dt * 1000 * Kd;

  dAngle = constrain(p + d , -100, 100);
//  Serial.print("dAngle : ");
//  Serial.println(dAngle);
  if (dAngle > 0) {
    driveDir = LEFT;
    dAngle = abs(dAngle);
  } else if (dAngle < 0) {
    driveDir = RIGHT;
    dAngle = abs(dAngle);
  } else {
    driveDir = CENTER;
    dAngle = abs(dAngle);
  }
  RC_steer(driveDir, dAngle);
#ifdef BT_ON
  SerialBT.print("  S0:");
  SerialBT.print(s0);
  SerialBT.print("  S1:");
  SerialBT.print(s1);
  SerialBT.print("  S2:");
  SerialBT.print(s2);
  SerialBT.print("  D:");
  SerialBT.print(driveDir);
  SerialBT.print("  R:");
  SerialBT.print(dAngle);
  SerialBT.println();
#endif
///*
  Serial.print("  S0:");
  Serial.print(s0);
  Serial.print("\tS1:");
  Serial.print(s1);
  Serial.print("\tS2:");
  Serial.print(s2);
  Serial.print("\tp:");
  Serial.print(p);
  Serial.print("\tt:");
  Serial.print(t);
  Serial.print("\td:");
  Serial.print(d);
  Serial.print("\tdt:");
  Serial.print(dt);
  Serial.print("\tDIR:");
  Serial.print(driveDir);
  Serial.print("\tAngle:");
  Serial.print(dAngle);
  Serial.println();
//*/
}
