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

BluetoothSerial SerialBT;

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
//  uint16_t ROI_X, ROI_Y;
  Serial.begin(115200);  
  RC_setup();               //   RumiCar initial function
  Serial.println("Start bluetooth!");
  SerialBT.begin("RumiCar_ESP32");
  Serial.println("The device started, now you can pair it with bluetooth!");
}
bool dirFlag = true;
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

//  if (abs(s0 - s2) < 10) {
//    RC_steer(CENTER);
//  } else if(s0>s2){
  if(s0>s2){
    driveDir    = LEFT;
    drivePower  = 45;
  }else{
    driveDir    = RIGHT;
    drivePower  = 45;
  }
  RC_steer(driveDir, drivePower);

///*
  SerialBT.print("  S0:");
  SerialBT.print(s0);
  SerialBT.print("  S1:");
  SerialBT.print(s1);
  SerialBT.print("  S2:");
  SerialBT.print(s2);
  SerialBT.print("  D:");
  SerialBT.print(driveDir);
  SerialBT.print("  P:");
  SerialBT.print(drivePower);
  SerialBT.println();
//*/
}
