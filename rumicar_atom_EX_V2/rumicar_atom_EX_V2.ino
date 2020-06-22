//=========================================================
//  rumicar_atom_EX_V2.ino :  RumiCar test application 
//  History     : V2.0  2020-06-21 Support VL53L1X
//=========================================================
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include <Wire.h>
#define EXTERN extern
#include "RumiCar_atom.h"

VL53L1X sensor0;                  // create right sensor instanse
VL53L1X sensor1;                  // create front sensor instance
VL53L1X sensor2;                  // create left  sensor instance

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
//  uint16_t ROI_X, ROI_Y;
  
  RC_setup();               //   RumiCar initial function
/*
  sensor1.VL53L1X_GetROI_XY(&ROI_X, &ROI_Y);
  Serial.print("  ROI_X :");
  Serial.print(ROI_X);
  Serial.print("  ROI_Y :");
  Serial.println(ROI_Y);
*/
}
bool dirFlag = true;
//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
  int s0, s1, s2;    
  M5.update();
  //=========================================================
  //  get VL53L0X TOF sensor value 
  //    S0: left S1: center S2: right
  //=========================================================
  s0=sensor0.read();        // read left  sensor
  s1=sensor1.read();        // read front sensor
  s2=sensor2.read();        // read right sensor

//  if (dirFlag) RC_steer(RIGHT, 100);
//  else         RC_steer(LEFT, 100);
//  dirFlag = !dirFlag;
//  delay(250);
///*
  Serial.print("  Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor1:");
  Serial.print(s1);
  Serial.print("  Sensor2:");
  Serial.println(s2);
//-*/
}
