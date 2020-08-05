//=========================================================
//  rumicar_atom_EX_V3.ino :  RumiCar application 
//  History     : V3.0  2020-07-17 Brake Test
//                V4.0  2020-08-03 Brake support
//                V4.1  2020-08-05 Debug kirikaeshi
//=========================================================
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include <Wire.h>
#define  EXTERN extern
#include "RumiCar_atom.h"
#include "BluetoothSerial.h"

#define BT_ON
#ifdef BT_ON
BluetoothSerial SerialBT;
#endif
//=========================================================
//  RumiCar Default Parameter
//=========================================================
#define DEVICE_NAME     "RumiCar_ESP32" // BLE Device Name
#define PILOT_MODE      1         // 1:Auto 2:Manual 
#define MAX_POWER       230       // 230 max
#define MIN_POWER       120       // 120 min
#define MAX_SPEED       1.0       // max speed factor
#define MID_SPEED       0.75      // mid speed factor
#define LOW_SPEED       0.5       // low speed need torque
#define BRAKE_TIME      1000      // coasting max time
#define MAX_DISTANCE_W  300       // max distance to wall
#define MID_DISTANCE_W  150       // keep distance from inside wall
#define MIN_DISTANCE_W  50        // min distance to wall
#define MAX_ANGLE       100       // max 100%
#define LIMIT_ANGLE      30       // max  30%
#define OVR_DISTANCE_F  800       // 800mm  detect straight 
#define MAX_DISTANCE_F  300       // 300mm  detect front wall
#define MID_DISTANCE_F  200       // 200mm  speed down distance
#define MIN_DISTANCE_F  100       // 100mm  reverse start distance
#define REVERSE_DISTANCE 100      // 100mm  reverse distance
#define STP_DISTANCE_F  0         // 0mm kiss to wall
#define REVERSE_TIME    200       // reverse time 500ms
#define OIO_OFFSET      50         // out in out offset 0=off
#define OIO_TIME        500       // continue 500ms to inside
#define Kp              0.8       // Konstante p
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
//  auto pilot variables difinition
//=========================================================
static int s0, s1, s2;                   // left, center, right censor value
static int curDriveDir = BRAKE;          // current direction
static int lastDriveDir = BRAKE;         // last drive direction
static int lastSteerDir = CENTER;        // last steer direction

static unsigned long steerCount[3];      // steer count
static int courseLayout = 1;             // 0:LEFT 2:RIHGT 1:unknown
static float prep = 0;                   // pre value of proportional control
static float p;                          // proportional control
static float d  = 0;                     // differential control
static float d1 = 0;
static float d2 = 0;
static float kp = Kp;
static float kd = Kd;                
static unsigned long preTime = 0;
static unsigned long dTime = 0;
static int dMode = DMODE;                // difference control mode 
static int16_t Joy_X = 0;                // JoyStick X
static int16_t Joy_Y = 0;                // JoyStick Y
static int16_t autoPilot = PILOT_MODE;   // Default Pilot Mode
static int16_t Trim = 0;                 // Default Trimmer value
static int16_t maxSpeed = MAX_POWER;     // Default Max Motor Power
static unsigned long sFwdTime = 0;       // fwd passage time
static unsigned long eFwdTime = 0;       // fwd passage time
static unsigned long sCornerTime = 0;    // cornering passage time
static unsigned long eCornerTime = 0;    // cornering passage time
static int preDistance    = 0;           // previous distance(S1)
static int reverseFlag = 0;              // accelation or brake
static int preSpeed       = 0;           // previous speed
static int targetSpeed    = 0;           // target speed
static int requestTorque  = 0;           // PWM Value
static int curSpeed       = 0;           // current speed
static int preS1          = 0;           // pre s1
static int reverseMode    = 0;           // reversing
static int steerDir;                     // steering direction
static int dAngle;                       // steering angle
//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  int i;
  for (i = 0; i < 3; i++) {
    steerCount[i] = 0;
  }
  RC_setup();               //   RumiCar initial function
#ifdef BT_ON
  Serial.println("Start bluetooth!");
  SerialBT.begin(DEVICE_NAME);
  Serial.println("The device started, now you can pair it with bluetooth!");
#endif  
}

//=========================================================
//  auto_driving
//=========================================================
void auto_driving()
{
  int dDistance      = 0;
  int minDistance    = 0;

  dDistance = preDistance - s1;
  curSpeed = (dDistance * 108) / (int)dTime;
  if (reverseMode == 1) {
    minDistance = MIN_DISTANCE_F + REVERSE_DISTANCE;
  } else {
    minDistance = MIN_DISTANCE_F;
  }
  if(s1 < minDistance){                    // x < 100
    curDriveDir = REVERSE;
    requestTorque = map(s1, STP_DISTANCE_F, MIN_DISTANCE_F, MIN_POWER + dAngle / 2, maxSpeed * MID_SPEED);
    RC_drive(curDriveDir, requestTorque);
    reverseMode = 1;
  } else {
    reverseMode = 0;
    if (s1 > OVR_DISTANCE_F) {                // 800 < x
      curDriveDir = FORWARD;
      requestTorque = maxSpeed * MAX_SPEED;   // full throttle
      RC_drive(curDriveDir, requestTorque);
    } else {                                  // 100 < x < 800
      curDriveDir = FORWARD;
      targetSpeed   = map(s1, MIN_DISTANCE_F, OVR_DISTANCE_F, 0 , maxSpeed * MAX_SPEED);
      requestTorque = map(s1, MIN_DISTANCE_F, OVR_DISTANCE_F, MIN_POWER + dAngle / 2, maxSpeed * MAX_SPEED);
      if (targetSpeed < curSpeed) {           // over speed
        curDriveDir   = BRAKE;
        requestTorque = 255;
      }
      RC_drive(curDriveDir, requestTorque);
    } 
  }
  //=========================================================
  //  detect kirikaeshi
  //=========================================================
  if (curDriveDir == REVERSE && curSpeed < 0) { // kirikeshi
    reverseFlag = 1;
  } else {
    reverseFlag = 0;
  }
  //=========================================================
  //  calc forward time
  //=========================================================
  if (lastDriveDir != FORWARD && curDriveDir == FORWARD) {  // change to FORWARD
    sFwdTime = millis();
    eFwdTime = 0;                                           // clear elapse time
  }
  if (lastDriveDir == FORWARD && curDriveDir == FORWARD) {  // continue FORWARD
    eFwdTime = millis() - sFwdTime;                         // add elapse  time
    if (eFwdTime > 10000) {
      eFwdTime = 10000;
    }
  }
  preDistance  = s1;
  preSpeed     = curSpeed;
  preS1        = s1;
}

//=========================================================
//  auto_steering
//=========================================================
void auto_steering()
{
  int steerMax;                     // limit steering angle
  int dMin = MIN_DISTANCE_W;        // min distance to wall     
  int dMax = MAX_DISTANCE_W;        // max distance to wall
  int pos;                          // position between wall to wall
  int oioOffset = OIO_OFFSET;       // out in out offset
  int targetPos;                    // target position
  int curPos;                       // current position

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
/*
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
*/
  oioOffset = 0;
  if (dMode > 0) {
    oioOffset = OIO_OFFSET;
    if (courseLayout == 0) {
      s0 += (oioOffset * dMode);
    } else if (courseLayout == 2) {
      s2 += (oioOffset * dMode);      
    }
  }
  //=========================================================
  //  calc steering angle
  //=========================================================
  targetPos = (s0 + s2) / 2;        // proportional 
  curPos = s2;                      // standard wall is LEFT
  p = (targetPos - curPos) * kp;    // P control
  d = (p - prep) * 1000 / dTime * kd;  // calc differential
//  if (dMode == 1) {     
//    d = (d + d1 + d2) / 3;
//    d2 = d1;
//    d1 = d;   
//  }
  dAngle = constrain(p + d , -steerMax, steerMax);  // normalize dAngle -100 to 100
  prep = p;

  if (dAngle > 0) {
    steerDir = LEFT;
    dAngle = abs(dAngle);  
  } else if (dAngle < 0) {
    steerDir = RIGHT;
    dAngle = abs(dAngle);      
  } else {
    steerDir = CENTER;
    dAngle = 0;
  }
  //=========================================================
  //  kerikaeshi
  //=========================================================
  if (reverseMode == 1 && reverseFlag == 1) {
    if (steerDir == RIGHT) {
      steerDir = LEFT;                  // counter steer
    } else if (steerDir == LEFT) {
      steerDir = RIGHT;                 // counter steer
    } else {                            // kirikaeshi
      steerDir = CENTER;
      dAngle = steerMax * 0.7;
    }
  }
  //=========================================================
  //  steering control
  //=========================================================
  RC_steer(steerDir, dAngle);           // steering
  //=========================================================
  //  calc corner time
  //=========================================================
  if (lastSteerDir != steerDir || lastDriveDir != curDriveDir) { // change drive direction
    sCornerTime = millis();
    eCornerTime = 0;                            // clear elapse time
  }
  if (lastSteerDir == steerDir && lastDriveDir == curDriveDir) { // continue
    eCornerTime = millis() - sCornerTime;       // calc elapse time
    if (eCornerTime > 10000) {
      eCornerTime = 10000;
    }
  }
  //=========================================================
  //  detect course layout
  //=========================================================
  steerCount[steerDir]++;
  if (steerCount[LEFT] > 100 || steerCount[RIGHT] > 100) {
    if (steerCount[LEFT] > steerCount[RIGHT]) {
      courseLayout = LEFT;
    } else {
      courseLayout = RIGHT;
    }
  } else {
    courseLayout = CENTER;                    // unknown
  }
  //=========================================================
  //  save old direction
  //=========================================================
  lastDriveDir = curDriveDir;                 // save last value
  lastSteerDir = steerDir;
}

//=========================================================
//  auto_pilot
//=========================================================
void auto_pilot()
{
  unsigned long t;                  // current time
  char buf[256];                    // serial pirnt buffer

  t = millis();                     // get current time
  dTime = t - preTime;                 // diff time
  preTime = t;
  auto_driving();
  auto_steering();
  //=========================================================
  //  Logging
  //=========================================================
#ifdef BT_ON
  sprintf(buf, "\t%8d\t%4d\t%4d\t%4d\t%5.2f\t%5.2f\t%1d\t%3d\t%4.2f\t%4.2f\t%3d\t%3d\t%1d\t%1d\t%1d", 
                t, s0, s1, s2, p, d, steerDir, dAngle, kp, kd, requestTorque, curSpeed, curDriveDir, courseLayout, dMode);
  SerialBT.println(buf);
#endif
//  SerialBT.println(buf);
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
    sDrive = constrain( Joy_Y + K_OFF, 0, maxSpeed);
    RC_drive(FORWARD, sDrive);
  } else if (Joy_Y < -10) {
    sDrive = constrain(-Joy_Y + K_OFF, 0, maxSpeed);
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
  s0 = sensor0.read();        // read left  sensor
  s1 = sensor1.read();        // read front sensor
  s2 = sensor2.read();        // read right sensor
///*
  Serial.print("\tSensor0:");
  Serial.print(s0);
  Serial.print("\tSensor1:");
  Serial.print(s1);
  Serial.print("\tSensor2:");
  Serial.println(s2);  
  if (s1 <= 0) s1 = OVR_DISTANCE_F; // over 2000mm return 0
  if (s0 <= 0) s0 = OVR_DISTANCE_F; // over 2000mm return 0
  if (s2 <= 0) s2 = OVR_DISTANCE_F; // over 2000mm return 0
//*/
#else
  //s0=sensor0.readRangeContinuousMillimeters();
  //s1=sensor1.readRangeContinuousMillimeters();
  //s2=sensor2.readRangeContinuousMillimeters();
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
      if (autoPilot == 0) {
        autoPilot = 1;
#ifdef BT_ON
        SerialBT.println("\tTime\tS0\tS1\tS2\tD\tP\tDIR\tAngle\tKp\tKd\trequestSpeed\tcurSpeed\tcurDriveDIr\tLayout\tdMode");
#endif
      } else {
        autoPilot = 0;
      }
    } else if (action == 'M') {
      dMode++;
      if (dMode > 3) dMode = 3;
    } else if (action == 'm') {
      dMode--;
      if (dMode < 0) dMode = 0;
    } else if (action == 'S') {
      maxSpeed += 10;
      if (maxSpeed >= 255) maxSpeed = 255;
    } else if (action == 's') {
      maxSpeed -= 10;
      if (maxSpeed <= 100) maxSpeed = 100;
    } else if (action == 'P') {
      kp += 0.05;
    } else if (action == 'p') {
      kp -= 0.05;
      if (kp <= 0) kp = 0;
    } else if (action == 'D') {
      kd += 0.05;
    } else if (action == 'd') {
      kd -= 0.05;
      if (kd <= 0) kd = 0;
    }
  }
  if (autoPilot == 1) {
    auto_pilot();
  } else {
    manual_pilot();
  }
}