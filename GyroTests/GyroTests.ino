#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "RoboClaw.h"


#define Telemetry 1

#define pred 0.01
#define KP_Angle_to_vel 180000
#define nominal 100
#define vel 0
#define bias -0.0436 // rad
#define alpha_avg_vel 0.95
#define max_velocity 50000

#define address 0x80
#define click_in_meter 47609
#define RADIANS_TO_DEGREES 57.29578

MPU6050 mpu;
RoboClaw roboclaw(&Serial1,10000);


unsigned long last_time,last_sent;
float wanted_angle = 0;
float pitch_rad,pitch_vel_rad_sec = 0;
float actual_angle = 0;
float wanted_velocity_m_s = 0;
int cycle_time_ms = 0;
uint8_t status1,status2,status3,status4;
bool valid1,valid2,valid3,valid4;
int32_t enc1;
int32_t enc2;
int32_t speed1;
int32_t speed2;
float average_vel;
float robot_vel_m_sec;
float robot_pos_m;

void setup() {
    Wire.begin();
    Serial.begin(115200);
    roboclaw.begin(115200);
}



void loop() {

  read_vel_pos();
  
  cycle_time_ms = millis() - last_time;
  last_time = millis();
        
  pitch_vel_rad_sec = 0;
  pitch_rad = 0;
  actual_angle = pitch_rad + bias - pitch_vel_rad_sec * cycle_time_ms * pred;
  average_vel = alpha_avg_vel*average_vel + (1-alpha_avg_vel)*(float(-1 * speed1 + speed2)) / 100000;
 
  wanted_velocity_m_s = 0; //-1*(tan(actual_angle - wanted_angle))*KP_Angle_to_vel;
  set_motors_vel (wanted_velocity_m_s);
}

void read_vel_pos(){
  enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  robot_vel_m_sec = float(speed2+speed1)/2/click_in_meter;
  robot_pos_m = float (enc2+enc1)/2/click_in_meter;
}

void set_motors_vel (float velocity){
  velocity = velocity * click_in_meter;
  if (velocity < 0) velocity -= nominal;
  if (velocity > 0) velocity += nominal;
  if (velocity < -max_velocity) velocity = -max_velocity;
  if (velocity >  max_velocity) velocity =  max_velocity;
  roboclaw.SpeedM1(address, velocity);
  roboclaw.SpeedM2(address, velocity); 
}

void send_telemetry(){
  if (Telemetry && millis()-last_sent>20){
    last_sent=millis();
//    Serial.print("vel"); Serial.print(robot_vel_m_sec);
//    Serial.print("pos"); Serial.print(robot_pos_m);
    Serial.print("pit"); Serial.print(pitch_rad);
    Serial.println("");
  }
}
