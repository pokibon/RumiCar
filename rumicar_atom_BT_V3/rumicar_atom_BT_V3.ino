//=========================================================
//  rumicar_atom_BT_V0.ino :  RumiCar test application 
//  History     : V2.0  2020-06-21 Support VL53L1X
//                V3.0  2020-07-03 Support PD Function
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

#define BT_ON

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

float p;                    // proportional control
float prep;                 // pre value of proportional control
float d;                    // differential control
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
  #define Kp      0.5       // Konstante p
  #define Kd      0.2       // Konstante d
  int dAngle;               // steering angle 0 - 100
  int targetPos;            // target position
  int curPos;               // current position
  unsigned long dt;         // diff time
  unsigned long t;          // current time

  targetPos = (s0 + s2) / 2;  // proportional 
  curPos = s2;                // standard wall is LEFT
  p = (targetPos - curPos) * Kp;  // P control

  t = millis();               // get current time
  dt = t - preTime;           // diff time
  preTime = t;                

  d = (p - prep) * 1000 / dt * Kd;  // calc differential
  prep = p;
  dAngle = constrain(p + d , -100, 100);  // normalize dAngle -100 to 100

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
  SerialBT.print("\tS1:");
  SerialBT.print(s1);
  SerialBT.print("\tS2:");
  SerialBT.print(s2);
  SerialBT.print("\tt:");
  SerialBT.print(t);
  SerialBT.print("\tdt:");
  SerialBT.print(dt);
  SerialBT.print("\tp:");
  SerialBT.print(p);
  SerialBT.print("\td:");
  SerialBT.print(d);
  SerialBT.print("\tDIR:");
  SerialBT.print(driveDir);
  SerialBT.print("\tAngle:");
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
  Serial.print("\tt:");
  Serial.print(t);
  Serial.print("\tdt:");
  Serial.print(dt);
  Serial.print("\tp:");
  Serial.print(p);
  Serial.print("\td:");
  Serial.print(d);
  Serial.print("\tDIR:");
  Serial.print(driveDir);
  Serial.print("\tAngle:");
  Serial.print(dAngle);
  Serial.println();
//*/
}
