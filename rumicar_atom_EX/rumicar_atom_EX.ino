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
int ispeed = 180;
int idist1;
idist1=sensor1.readRangeSingleMillimeters();
if ( idist1 < 300 ){
  if ( idist1 > 120 ){
    RC_drive(FORWARD,ispeed);
  }else if (idist1 < 80){
    RC_drive(REVERSE,ispeed);;
    }else{
    RC_drive(BRAKE,ispeed);  
    }
}else{
    RC_drive(FREE,ispeed);  
  }
}
