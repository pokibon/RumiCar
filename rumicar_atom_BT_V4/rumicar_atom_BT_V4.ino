//=========================================================
//  rumicar_atom_BT_V4.ino :  RumiCar application 
//  History     : V4.0   2020-07-04 Support PD Control steering
//=========================================================
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include <BlynkSimpleEsp32_BLE.h>
#include <Wire.h>
#define EXTERN extern
#include "RumiCar_atom.h"
#include "BluetoothSerial.h"

#define BT_ON
#ifdef BT_ON
BluetoothSerial SerialBT;
#endif
//=========================================================
//  RumiCar Default Parameter
//=========================================================
#define PILOT_MODE      1         // 1:Auto 2:Manual 
#define MAX_POWER       180       // 255 max
#define MAX_SPEED       1.0       // max speed factor
#define MID_SPEED       0.8       // mid speed factor
#define LOW_SPEED       1.0       // low speed need torque
#define BRAKE_TIME      1000      // coasting max time
#define MAX_DISTANCE_W  300       // max distance to wall
#define MID_DISTANCE_W  150       // keep distance from inside wall
#define MIN_DISTANCE_W  70        // min distance to wall
#define MAX_ANGLE       100       // max 100%
#define LIMIT_ANGLE      30       // max  30%
#define OVR_DISTANCE_F  800       // 800mm  detect straight 
#define MAX_DISTANCE_F  300       // 300mm  detect front wall
#define MID_DISTANCE_F  200       // 200mm  speed down distance
#define MIN_DISTANCE_F  100       // 100mm  reverse distance
#define STP_DISTANCE_F  0         //   0mm kiss to wall
#define REVERSE_TIME    100       // reverse time 500ms
#define OIO_OFFSET      0         // out in out offset 0=off
#define OIO_TIME        500       // continue 500ms to inside
#define DEVICE_NAME "RumiCar_ESP32" // BLE Device Name
#define Kp              0.6       // Konstante p
#define Kd              0.1       // Konstante d
#define DMODE           0         // Differential control mode
                                  // 1: normalize 0:active
#ifdef  SENSOR_VL53L1X            // use VL53L1X
VL53L1X sensor0;                  // create right sensor instanse
VL53L1X sensor1;                  // create front sensor instance
VL53L1X sensor2;                  // create left  sensor instance
#else                             // use VL53L0X
VL53L0X sensor0;                  // create right sensor instance
VL53L0X sensor1;                  // create front sensor instance
VL53L0X sensor2;                  // create left  sensor instance
#endif

//=========================================================
//  Bylnk BLE definition
//=========================================================
char auth[] = "mcJxlsL7QpjgwQ6Rr-OGJVhbWw9jWiSp";   // BLYNK Authentication code
int16_t Joy_X = 0;                // BLYNK JoyStick X
int16_t Joy_Y = 0;                // BLYNK JoyStick Y
int16_t AutoPilot = PILOT_MODE;   // Default Pilot Mode
int16_t Trim = 0;                 // Default Trimmer value
int16_t Max_Speed = MAX_POWER;    // Default Max Motor Power

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  RC_setup();               //   RumiCar initial function
#ifdef BT_ON
  Serial.println("Start bluetooth!");
  SerialBT.begin(DEVICE_NAME);
  Serial.println("The device started, now you can pair it with bluetooth!");
#endif  
}

//=========================================================
//  auto pilot variables difinition
//=========================================================
int s0, s1, s2;               // left, center, right censor value
int CurDir = BRAKE;           // current direction
int LastDir = BRAKE;          // last direction
float prep = 0;               // pre value of proportional control
float p;                      // proportional control
float d  = 0;                 // differential control
float d1 = 0;
float d2 = 0;
float kp = Kp;
float kd = Kd;                
unsigned long preTime = 0;
int dMode = DMODE;            // difference control mode 
//=========================================================
//  auto_driving
//=========================================================
void auto_driving()
{
  int sTime, eTime  = 0;      // fwd pass time
  int Brake_flag    = 0;      // 1:Brake on flag default 0
  int CurSpeed      = 0;      // current speed
  int LastDistance  = 0;      // last distance
  int CurDistance   = 0;      // current distance
  
  if(s1 < MIN_DISTANCE_F){                    // x < 100
    CurDir = REVERSE;
    CurSpeed = Max_Speed * MAX_SPEED;
    CurDistance = STP_DISTANCE_F;
  }else {
    if (s1 > MAX_DISTANCE_F) {                // 300 > x
      CurDistance = MAX_DISTANCE_F;
      CurDir = FORWARD;
      CurSpeed = Max_Speed * MAX_SPEED;
    } else {                      
      CurDir = FORWARD;
      if (s1 > MID_DISTANCE_F) {              // 300 > x > 200
        CurDistance = MID_DISTANCE_F;
        CurSpeed = Max_Speed * MID_SPEED;
      }else {                                 // 200 > x > 100
        CurDistance = MIN_DISTANCE_F;
//        if (LastDistance >= MIN_DISTANCE_F) { // Brake mode (experiment)
//          Brake_flag = 1;
//          Serial.println("#### BRAKE !!!");
//        } else {                            // Accel
          CurSpeed = Max_Speed * LOW_SPEED;
//        }
      }
    }
  }
  if (Brake_flag == 0) {                      // braking(experiment)
    RC_drive(CurDir, CurSpeed);
    if (CurDir == REVERSE) delay(REVERSE_TIME);
  } else {
    RC_drive(BRAKE, 255);
  }

  if (LastDir != FORWARD && CurDir == FORWARD) {  // change to FORWARD
    sTime = millis();
    eTime = 0;                                    // clear elapse time
  }
  if (LastDir == FORWARD && CurDir == FORWARD) {  // continue FORWARD
    eTime = millis() - sTime;                     // add elapse  time
    if (eTime > 10000) {
      eTime = 10000;
    }
  }
  if (eTime > BRAKE_TIME && CurDir == REVERSE) {  // coasting timeout 
    CurDir = BRAKE;                               // not REVERSE 
  }
  LastDistance = CurDistance;
}

//=========================================================
//  auto_steering
//=========================================================
void auto_steering()
{
  int steerMax;                     // limit steering angle
  int sCornerTime, eCornerTime = 0; // cornering time
  int last_dDir = CENTER;           // last direction
  int dMin = MIN_DISTANCE_W;        // min distance to wall     
  int dMax = MAX_DISTANCE_W;        // max distance to wall
  int dAngle;                       // steering angle
  int pos;                          // position between wall to wall
  int oioOffset = OIO_OFFSET;       // out in out offset
  int targetPos;                    // target position
  int curPos;                       // current position
  unsigned long dt;                 // diff time
  unsigned long t;                  // current time
  int driveDir;
  int drivePower;
  //=========================================================
  //  detect straight
  //=========================================================
  if (s1 > OVR_DISTANCE_F &&  // detect straight 
      s0 > MID_DISTANCE_W &&  // not near left wall
      s2 > MID_DISTANCE_W) {  // not near write wall
    steerMax = LIMIT_ANGLE ;  // limit steering
  } else {
    steerMax = MAX_ANGLE;
  }
  //=========================================================
  //  Limit wall distance
  //=========================================================
  if      (s0 > dMax) s0 = dMax;    // correct Max Value 
  else if (s0 < dMin) s0 = dMin;    // correct Min Value
  if      (s2 > dMax) s2 = dMax;    // correct Max Value
  else if (s2 < dMin) s2 = dMin;    // correct Min Value
  //=========================================================
  //  detect out in out mode
  //=========================================================
  if (eCornerTime > OIO_TIME)  oioOffset = OIO_OFFSET;
  else                         oioOffset = 0;
  pos = s0 - s2;                    // detect direction of travel 
  if (pos < 0)  {                   // turn to left
    s2 += oioOffset;                // turn to near left wall
    dMax = s2;
  } else {                          // turn to right
    s0 += oioOffset;                // turn to near right wall
    dMax = s0;
  }
/*
  Serial.print("  Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor2:");
  Serial.print(s2);
*/
  //=========================================================
  //  calc steering angle
  //=========================================================
  targetPos = (s0 + s2) / 2;        // proportional 
  curPos = s2;                      // standard wall is LEFT
  p = (targetPos - curPos) * kp;    // P control
  t = millis();                     // get current time
  dt = t - preTime;                 // diff time
  preTime = t;
  d = (p - prep) * 1000 / dt * kd;  // calc differential
  if (dMode == 1) {     
    d = (d + d1 + d2) / 3;
    d2 = d1;
    d1 = d;   
  }
  dAngle = constrain(p + d , -steerMax, steerMax);  // normalize dAngle -100 to 100
  prep = p;

  if (dAngle > 0) {
    driveDir = LEFT;
    dAngle = abs(dAngle);  
  } else if (dAngle < 0) {
    driveDir = RIGHT;
    dAngle = abs(dAngle);      
  } else {
    driveDir = CENTER;
    dAngle = 0;
  }
  //=========================================================
  //  kerikaeshi
  //=========================================================
  if (CurDir == REVERSE) {
    if (driveDir == RIGHT) {
      driveDir = LEFT;
    } else if (driveDir == LEFT) {
      driveDir = RIGHT;
    } else {                        // kirikaeshi
//      if (pos > 0) {
//        driveDir = RIGHT;
//      } else {
//        driveDir = LEFT;
//      }
      driveDir = CENTER;
      dAngle = steerMax * 0.7;
    }
  }
  RC_steer(driveDir, dAngle);           // steering
  //=========================================================
  //  calc corner time
  //=========================================================
  if (last_dDir != driveDir || LastDir != CurDir) { // change drive direction
    sCornerTime = millis();
    eCornerTime = 0;                            // clear elapse time
  }
  if (last_dDir == driveDir && LastDir == CurDir) { // continue
    eCornerTime = millis() - sCornerTime;       // calc elapse time
    if (eCornerTime > 10000) {
      eCornerTime = 10000;
    }
  }
  //=========================================================
  //  save old direction
  //=========================================================
  LastDir = CurDir;                 // save last value
  last_dDir = driveDir;

#ifdef BT_ON
  SerialBT.print("\tS0:");
  SerialBT.print(s0);
  SerialBT.print("\tS1:");
  SerialBT.print(s1);
  SerialBT.print("\tS2:");
  SerialBT.print(s2);
  SerialBT.print("\tt:");
  SerialBT.print(t);
  SerialBT.print("\tdt:");
  SerialBT.print(dt);
  SerialBT.print("\tp : ");
  SerialBT.print(p);
  SerialBT.print("\td : ");
  SerialBT.print(d);
  SerialBT.print("\tDIR:");
  SerialBT.print(driveDir);
  SerialBT.print("\tAngle : ");
  SerialBT.print(dAngle);
  SerialBT.print("\tKp : ");
  SerialBT.print(kp);
  SerialBT.print("\tKd : ");
  SerialBT.print(kd);
  SerialBT.print("\tSpeed:");
  SerialBT.print(Max_Speed);
  SerialBT.print("\tdMode : ");
  SerialBT.print(dMode);
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


//=========================================================
//  auto_pilot
//=========================================================
void auto_pilot()
{
  auto_driving();
  auto_steering();
}

//=========================================================
//  manual pilot function
//=========================================================
void manual_pilot()
{
  int sDrive;         // dirve power
  int angle;          // steering angle
  int K_OFF = 50;     // min power offset
                      // driving
  if (Joy_Y > 10) {   
    sDrive = constrain( Joy_Y + K_OFF, 0, Max_Speed);
    RC_drive(FORWARD, sDrive);
  } else if (Joy_Y < -10) {
    sDrive = constrain(-Joy_Y + K_OFF, 0, Max_Speed);
    RC_drive(REVERSE, sDrive);    
  } else {
    RC_drive(BRAKE, 0); 
  }
                      // steering
  angle = constrain(Joy_X + Trim, -100, 100);
  if (angle > 0) {
    RC_steer(RIGHT, angle);
  } else if (angle < 0) {
    RC_steer(LEFT, -angle);    
  } else {
    RC_steer(CENTER); 
  }
}

int loop_count = 0;
//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
  M5.update();
  //=========================================================
  //  get VL53L0X TOF sensor value 
  //    S0: left S1: center S2: right
  //=========================================================
#ifdef SENSOR_VL53L1X
  //s0=sensor0.readRangeContinuousMillimeters();
  //s1=sensor1.readRangeContinuousMillimeters();
  //s2=sensor2.readRangeContinuousMillimeters();
  s0=sensor0.read();        // read left  sensor
  s1=sensor1.read();        // read front sensor
  s2=sensor2.read();        // read right sensor
#else
  s0=sensor0.readRangeSingleMillimeters();
  s1=sensor1.readRangeSingleMillimeters();
  s2=sensor2.readRangeSingleMillimeters();
#endif
//=========================================================
//  Bluetooth Serial Control
//=========================================================
  if(SerialBT.available()){
    char action = SerialBT.read();
    if (action == 'a' ){
      if (AutoPilot == 0) AutoPilot = 1;
      else                AutoPilot = 0;
    } else if (action == 'm') {
      if (dMode == 0) dMode = 1;
      else            dMode = 0;
    } else if (action == 'S') {
      Max_Speed += 10;
      if (Max_Speed >= 255) Max_Speed = 255;
    } else if (action == 's') {
      Max_Speed -= 10;
      if (Max_Speed <= 100) Max_Speed = 100;
    } else if (action == 'P') {
      kp += 0.1;
    } else if (action == 'p') {
      kp -= 0.1;
      if (kp <= 0) kp = 0;
    } else if (action == 'D') {
      kd += 0.05;
    } else if (action == 'd') {
      kd -= 0.05;
      if (kd <= 0) kd = 0;
    }
  }
  if (AutoPilot == 1) {
    auto_pilot();
  } else {
    manual_pilot();
  }
}