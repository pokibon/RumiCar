//=========================================================
//  rumicar_atom_DS_V0.ino :  RumiCar test application 
//  History     : V0.0  2020-06-28 Daiso toy radicon
//=========================================================
#include "M5Atom.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "BluetoothSerial.h"

#define MAX_SPEED 1.0

VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;

BluetoothSerial SerialBT;

//操舵用の設定
#define LEFT   0
#define CENTER 1
#define RIGHT  2

//走行用の設定
#define FREE    0
#define REVERSE 1
#define FORWARD 2
#define BRAKE   3


#define RC_analogWrite ledcWrite
#define SHUT0 0
#define SHUT1 33
#define SHUT2 23

//Aが操舵、Bが走行
int AIN1 = 19;
int AIN2 = 21;
int BIN1 = 22;
int BIN2 = 25;

//#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY

extern const unsigned char img_rumicar[677];
enum LED_DIR {
  DR_ARROW,
  RIGHT_ARROW,
  UR_ARROW,
  DOWN_ARROW,
  STOP,
  UP_ARROW,
  DL_ARROW,
  LEFT_ARROW,
  UL_ARROW
};

class _DispLed {
private:
  int state_matrix[4][3] = {
    {RIGHT_ARROW, STOP,       LEFT_ARROW},  // Free
    {DR_ARROW,    DOWN_ARROW, DL_ARROW  },  // Reverse
    {UR_ARROW,    UP_ARROW,   UL_ARROW  },  // Forward
    {RIGHT_ARROW, STOP,       LEFT_ARROW}   // BREAK
  };
public:
  int x = 1;
  int y = 3;
  void show(void) {
    int dir = state_matrix[y][x];
    M5.dis.displaybuff((uint8_t *)img_rumicar, dir * 5, 0);
  }
} DispLed;

void setup()
{
  M5.begin(true, false, true);
  delay(50);
  M5.dis.clear();
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
//  pinMode(SHUT0, OUTPUT);
  pinMode(SHUT1, OUTPUT);
  pinMode(SHUT2, OUTPUT);

 // digitalWrite(SHUT0, LOW);
  digitalWrite(SHUT1, LOW);
  digitalWrite(SHUT2, LOW);
  delay(150);
  Wire.begin(32, 26);
//  pinMode(SHUT0, INPUT);
  delay(150);
  if (!sensor0.init(true))
  {
    Serial.println("Failed to detect and initialize sensor0!");
    while (1) {}
  }
  delay(100);
  sensor0.setAddress((uint8_t)20); // 20
  sensor0.setTimeout(500);      // 500 
  //seonsor1
  pinMode(SHUT1, INPUT);
  delay(150);
  
  if (!sensor1.init(true))
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }
  delay(100);
  sensor1.setAddress((uint8_t)21);
  sensor1.setTimeout(500);
  //seonsor2
  pinMode(SHUT2, INPUT);
  delay(150);
  
  if (!sensor2.init(true))
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  delay(100);
  sensor2.setAddress((uint8_t)22);
  sensor2.setTimeout(500);

  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor0.setMeasurementTimingBudget(20000);
  sensor1.setMeasurementTimingBudget(20000);
  sensor2.setMeasurementTimingBudget(20000);

  sensor0.startContinuous();
  sensor1.startContinuous();
  sensor2.startContinuous();

  //ESP32の場合はピン番号ではなくチャンネルでPWMを行うのでチャンネルとして再設定
#define PWM_level 8
  // 8の場合8bitの解像度でArduinoと同じESPは16bit迄行ける？
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(0, 490, PWM_level);  // 490
  ledcSetup(1, 490, PWM_level);  // 490
  ledcSetup(2, 960, PWM_level);  // 960
  ledcSetup(3, 960, PWM_level);  // 960

  //モータのピンとチャンネルの設定
  ledcAttachPin(AIN1, 0);
  ledcAttachPin(AIN2, 1);
  ledcAttachPin(BIN1, 2);
  ledcAttachPin(BIN2, 3);
  //pin番号をチャンネル番号に上書き
  AIN1 = 0;
  AIN2 = 1;
  BIN1 = 2;
  BIN2 = 3;

  RC_analogWrite(AIN1, 0);
  RC_analogWrite(AIN2, 0);
  RC_analogWrite(BIN1, 0);
  RC_analogWrite(BIN2, 0);

  Serial.println("Start bluetooth!");
  SerialBT.begin("RumiCar_Daiso");
  Serial.println("The device started, now you can pair it with bluetooth!");
}

//操舵の関数
int RC_steer (int direc )
{
  DispLed.x = direc;
  DispLed.show();
  if ( direc == RIGHT ){
    RC_analogWrite(AIN1,255); // 255
    RC_analogWrite(AIN2,0);
  }else if ( direc == LEFT ){
    RC_analogWrite(AIN1,0);
    RC_analogWrite(AIN2,255);
  }else if ( direc == CENTER ){
    RC_analogWrite(AIN1,0);
    RC_analogWrite(AIN2,0);
  }else{
    return 0;
  }
}

//走行の関数
int RC_drive(int direc, int ipwm){
  DispLed.y = direc;
  DispLed.show();
  if ( direc == FREE ){
    RC_analogWrite(BIN1,0);
    RC_analogWrite(BIN2,0);
  }else if ( direc == REVERSE ){
    RC_analogWrite(BIN1,0);
    RC_analogWrite(BIN2,ipwm * MAX_SPEED);
  }else if ( direc == FORWARD ){
    RC_analogWrite(BIN1,ipwm * MAX_SPEED);
    RC_analogWrite(BIN2,0);
  }else if ( direc == BRAKE ){
    RC_analogWrite(BIN1,ipwm * MAX_SPEED);
    RC_analogWrite(BIN2,ipwm * MAX_SPEED);
  }else{
    return 0;
  }
}
int iBuf = 0;
void loop()
{
  int s0, s1, s2;
  int driveDir;
  int drivePower;

  M5.update();

  s0=sensor0.readRangeContinuousMillimeters();
  s1=sensor1.readRangeContinuousMillimeters();
  s2=sensor2.readRangeContinuousMillimeters();
//  s0=sensor0.readRangeSingleMillimeters();
//  s1=sensor1.readRangeSingleMillimeters();
//  s2=sensor2.readRangeSingleMillimeters();

  if(s1<100){
    RC_drive(REVERSE,150);
  }else if (s1<150){
    RC_drive(FORWARD,150);
  }else if (s1<250){
    RC_drive(FORWARD,200);
  }else{
    RC_drive(FORWARD,255);
  }

  if(s0>s2){
    driveDir    = LEFT;
    drivePower  = 50;
  }else{
    driveDir    = RIGHT;
    drivePower  = 50;
  }
  RC_steer(driveDir);
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
