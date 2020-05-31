//=========================================================
//  rumicar_atomV4.ino :  RumiCar application 
//  History     : V0.0  2020-05-29 New Create(K.Ohe)
//=========================================================
#include "M5Atom.h"
#include <Wire.h>
#include <VL53L0X.h>
#define EXTERN extern
#include "RumiCar_atom.h"

VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  RC_setup();
}
//=========================================================
//  Arduino Main function
//=========================================================
int iBuf = 0;
void loop()
{
int ibound =250;
int s0, s1, s2;
s0=sensor0.readRangeSingleMillimeters();
s1=sensor1.readRangeSingleMillimeters();
s2=sensor2.readRangeSingleMillimeters();

if(s1<100){
  RC_drive(REVERSE,150);
}else if (s1<150){
  RC_drive(FORWARD,150);
}else if (s1<250){
  RC_drive(FORWARD,200);
}else{
  RC_drive(FORWARD,255);
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
/*
 if(s0>s2){
  RC_steer(LEFT);
  }else{
    RC_steer(RIGHT);
  }
*/
}
