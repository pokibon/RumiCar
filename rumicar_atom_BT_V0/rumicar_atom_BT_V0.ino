//=========================================================
//  rumicar_atom_BT_V0.ino :  RumiCar test application 
//  History     : V2.0  2020-06-21 Support VL53L1X
//=========================================================
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include <Wire.h>
#define EXTERN extern
#include "RumiCar_atom.h"
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

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
/*
  SerialBT.print("  Sensor0:");
  SerialBT.print(s0);
  SerialBT.print("  Sensor1:");
  SerialBT.print(s1);
  SerialBT.print("  Sensor2:");
  SerialBT.println(s2);
*/
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}
