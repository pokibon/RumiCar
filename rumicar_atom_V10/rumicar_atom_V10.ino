//=========================================================
//  rumicar_atom_V8.ino :  RumiCar application 
//  History     : V5.0  2020-05-29 Support Bylnk Remoto Control
//                V6.0  2020-06-05 Support VL53L1X distance sensor
//                V8.0  2020-06-11 Support reverse steering
//                V9.0  2020-06-14 Change Steering MEthod
//                V9.1  2020-06-19 tunning reverse mode
//                                 out - in -in
//                                 tunning strate runnin 
//                V10.0 2020-07-04 Support PD Control steering
//=========================================================
#define USE_BLYNK                 // use BLYNK Smartphone comminication interface
#ifdef USE_BLYNK
#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#endif
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <Wire.h>
#define EXTERN extern
#include "RumiCar_atom.h"
//=========================================================
//  RumiCar Default Parameter
//=========================================================
#define PILOT_MODE      1         // 1:Auto 2:Manual 
#define MAX_POWER       180       // 255 max
#define MAX_SPEED       1.0       // max speed factor
#define MID_SPEED       0.8       // mid speed factor
#define LOW_SPEED       1.0       // low speed need torque
#define BRAKE_TIME      1000      // coasting max time
#define MAX_DISTANCE_W  600       // max distance to wall
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
#define DEVICE_NAME "ByRumiCar"   // BLE Device Name
#define Kp              0.5       // Konstante p
#define Kd              0.2       // Konstante d
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
#ifdef USE_BLYNK
WidgetLCD lcd(V3);                // BLYNK Virtual LCD 16x2
#endif

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  RC_setup();               //   RumiCar initial function
#ifdef USE_BLYNK
  Blynk.setDeviceName(DEVICE_NAME); // start BLYNK
  Blynk.begin(auth);

  lcd.clear();              // cear BLYNK virtual LCD Display
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
unsigned long preTime = 0;
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
  p = (targetPos - curPos) * Kp;    // P control
  t = millis();                     // get current time
  dt = t - preTime;                 // diff time
  preTime = t;                
  d = (p - prep) * 1000 / dt * Kd;  // calc differential
  d = (d + d1 + d2) / 3;
  dAngle = constrain(p + d , -steerMax, steerMax);  // normalize dAngle -100 to 100
  prep = p;
  d2 = d1;
  d1 = d;
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
/*
  Serial.print("  Laset Direction : ");
  Serial.print(last_dDir);
  Serial.print("  Current Direction : ");
  Serial.print(driveDir);
  Serial.print("  dAngle : ");
  Serial.print(dAngle);
  Serial.print("  Elaps Time : ");
  Serial.print(eCornerTime);
*/
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
#ifdef USE_BLYNK
  loop_count++;
  if (loop_count >= 3) {    // load to 1/4
    loop_count = 0;
  }
  Blynk.run(); 
#endif
  char buf[32];
    
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
#ifdef USE_BLYNK
  if (loop_count == 0) {    // print virtual LCD
    sprintf( buf, "%3dcm%3dcm%3dcm ", s0/10, s1/10, s2/10);
    lcd.print(0,0, " LEFT CENT.RIGHT");
    lcd.print(0, 1, buf);
  }
#endif
///*
  Serial.print("  Sensor0:");
  Serial.print(s0);
  Serial.print("  Sensor1:");
  Serial.print(s1);
  Serial.print("  Sensor2:");
  Serial.println(s2);
//*/
  if (AutoPilot == 1) {
    auto_pilot();
  } else {
    manual_pilot();
  }
}

//=========================================================
//  Bylnk Callback functions
//=========================================================
// V0: Auto Pilot Switch
#ifdef USE_BLYNK
BLYNK_WRITE(V0) {
  AutoPilot = param[0].asInt();
//  Serial.print("AutoPilot = ");
//  Serial.println(AutoPilot);  
}
// V1: Joy Stick
BLYNK_WRITE(V1) {
  Joy_X = param[0].asInt();
  Joy_Y = param[1].asInt();
//  Serial.print("X = ");
//  Serial.print(Joy_X);
//  Serial.print("  Y = ");
//  Serial.println(Joy_Y);
}

// V2: Trim
BLYNK_WRITE(V2) {
  Trim = param[0].asInt();
//  Serial.print("Trim = ");
//  Serial.println(Trim);
}

// V4: Max Speed
BLYNK_WRITE(V4) {
  Max_Speed = param[0].asInt();
//  Serial.print("Max_Speed = ");
//  Serial.println(Max_Speed);
}
#endif
