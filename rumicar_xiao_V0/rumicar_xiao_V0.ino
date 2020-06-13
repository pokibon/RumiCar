#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;

//操舵用の設定
#define LEFT   0
#define CENTER 1
#define RIGHT  2

//走行用の設定
#define FREE    0
#define REVERSE 1
#define FORWARD 2
#define BRAKE   3

#define RC_analogWrite analogWrite
//SHUTはディジタルピン,A0はD14,以降同じ、注意すること
#define SHUT0 0
#define SHUT1 1
#define SHUT2 2

//Aが操舵、Bが走行
int AIN1 = 7;
int AIN2 = 8;
int BIN1 = 9;
int BIN2 = 10;

#define HIGH_SPEED
void setup()
{
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SHUT0, OUTPUT);
  pinMode(SHUT1, OUTPUT);
  pinMode(SHUT2, OUTPUT);

  digitalWrite(SHUT0, LOW);
  digitalWrite(SHUT1, LOW);
  digitalWrite(SHUT2, LOW);
  delay(150);
  Wire.begin();
  //seonsor0
  pinMode(SHUT0, INPUT);
  delay(150);
  sensor0.init(true);
  delay(100);
  sensor0.setAddress((uint8_t)20); // 20
  sensor0.setTimeout(500);      // 500 
  //seonsor1
  pinMode(SHUT1, INPUT);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)21);
  sensor1.setTimeout(500);
  //seonsor2
  pinMode(SHUT2, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)22);
  sensor2.setTimeout(500);

  // increase timing budget to 200 ms
  sensor0.setMeasurementTimingBudget(200000);
  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);

  sensor0.startContinuous();
  sensor1.startContinuous();
  sensor2.startContinuous();

  RC_analogWrite(AIN1, 0);
  RC_analogWrite(AIN2, 0);
  RC_analogWrite(BIN1, 0);
  RC_analogWrite(BIN2, 0);

}
//操舵の関数
int RC_steer (int direc ){
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
  if ( direc == FREE ){
    RC_analogWrite(BIN1,0);
    RC_analogWrite(BIN2,0);
  }else if ( direc == REVERSE ){
    RC_analogWrite(BIN1,0);
    RC_analogWrite(BIN2,ipwm);
  }else if ( direc == FORWARD ){
    RC_analogWrite(BIN1,ipwm);
    RC_analogWrite(BIN2,0);
  }else if ( direc == BRAKE ){
    RC_analogWrite(BIN1,ipwm);
    RC_analogWrite(BIN2,ipwm);
  }else{
    return 0;
  }
}
int iBuf = 0;
void loop()
{
  int s0, s1, s2;
  int ispeed = 255;
  int idist1;
  int sDist = 100;
  int K_OFF = 64;
  int KP = 200;
  int KI = 20;
  int sP;
  long sI;
  int sDrive;
  int dDist;  
  s0=sensor0.readRangeContinuousMillimeters();
  s1=sensor1.readRangeContinuousMillimeters();
  s2=sensor2.readRangeContinuousMillimeters();
//s0=sensor0.readRangeSingleMillimeters();
//s1=sensor1.readRangeSingleMillimeters();
//s2=sensor2.readRangeSingleMillimeters();
/*
  Serial.print("Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor1:");
  Serial.print(s1);
  Serial.print("  Sensor2:");
  Serial.println(s2);
*/
//  idist1=sensor1.readRangeSingleMillimeters();    // 距離計測
  idist1 = s1;
  dDist = idist1 - sDist;                         // 差分

  if ( idist1 < 300 ){                            // 30cm 以内のみ動作
    sP = (dDist * KP) / 100;                      // P項
    iBuf += (sP * KI);                            // 積分
    sI = iBuf >> 8;                               // I項
    sDrive = sP + sI;                             // 指令値
    constrain(sDrive,-255,255);                   // Limit
//    sDrive = sP;
    if ( dDist > 10 ){                            // Direction
      RC_drive(FORWARD,(sDrive + K_OFF));
    }else if (dDist < -10){
      RC_drive(REVERSE,((-sDrive) + K_OFF));
      }else{                                      // +-10mm 以内は停止
        RC_drive(BRAKE,ispeed);                   // Brake
        iBuf = 0;                                 // 積分クリア
      }
  }else{
      RC_drive(FREE,ispeed);                      // Free
      iBuf = 0;                                   // 積分クリア
  }

  if      (s0 > 150 && s2 > 150) RC_steer(CENTER);
  else if (s0 > 150 && s2 <=150) RC_steer(LEFT);
  else if (s0 <=150 && s2 > 150) RC_steer(RIGHT);
  else {
    if (abs(s0 - s2) < 50) RC_steer(CENTER);
    else if (s0 > s2) RC_steer(LEFT);
    else RC_steer(RIGHT);
  }
}
