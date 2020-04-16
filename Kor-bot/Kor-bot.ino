#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <RoboClaw.h>
#include <EEPROM.h>

#define Telemetry 0
#define calib_gyro 0
#define alpha_gyro 0.995         // takes most of the pitch data from gyro .. very little noise to acc

#define KDD_Angle_to_acc 0.4     //  0.4
#define KD_Angle_to_acc 2        //  2 
#define KP_Angle_to_acc 6        //  6
#define KI_Angle_to_acc 3        //  3
#define KI_Angle_to_acc_res 5    //  5        integral that resets when error change dir
#define KP_vel_to_vel   0.015    //  0.015    adds to the velocity a factor of average velocity - do damp it down
#define KD_avgdError_to_acc 15   //  15       slows the robot when closing the error fast - like KD , but on average D to eliminate noise 
#define integral_limit 0.035

#define KP_pos_to_angle  0.04          // 0.04 to keep 0
#define KI_pos_to_angle  0.02          // 0.02 to keep 0
#define pos_error_integral_limit 0.4   // 0.4
 
#define KP_vel_to_angle  0.14          // 0.14
#define KI_vel_to_angle  0.02          // 0.02
#define vel_error_integral_limit 0.4   // 0.4

#define Gyro_bias 3.35             // deg   lower = move forward

#define alpha_avg_vel 0.9       // avergaring factor for the averaged velocity 
#define alpha_avg_dError 0.9    // avergaring factor for the averaged derivative of the error 
#define alpha_stick 0.95

#define address 0x80            // adress of the roboclaw 128
#define click_in_meter 47609    // encoder clics in one meter
#define max_velocity 3

#define PIN_SW1 3
#define PIN_SW2 4
#define PIN_LED1 5
#define PIN_LED2 6


#define RADIANS_TO_DEGREES 57.29578
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I  0x75 // R
#define MPU6050_I2C_ADDRESS 0x68 // R

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial2
#define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,3,0,8,0,61,0,10,24,1,
  5,32,8,41,48,48,177,26,31,2,
  0,43,91,20,9,177,26,8,1,79,
  78,0,0,68,34,-1,-2,64,34,186,
  50,137,87,97,110,116,101,100,32,40,
  68,101,103,41,0,65,99,116,117,97,
  108,32,40,68,101,103,41,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  uint8_t switch_1; // =1 if switch ON and =0 if OFF 

    // output variables
  float onlineGraph_1_var1;
  float onlineGraph_1_var2;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////



MPU6050 mpu;
RoboClaw roboclaw(&Serial1,10000);

typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

typedef struct {
  float yaw;
  float pitch;
  float roll;
  float yaw_vel;
  float pitch_vel;
  float roll_vel;
} orientation;

unsigned long last_read_time;
float last_x_angle;
float last_y_angle;
float last_z_angle;

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}

float    base_x_accel;
float    base_y_accel;
float    base_z_accel;
float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

void set_last_read_angle_data(unsigned long time, float x, float y, float z) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
}

unsigned long last_time,last_sent,last_cycle, lastUSRblink;
float wanted_angle = 0, prev_wanted_angle;
float pitch_rad,pitch_vel_rad_sec = 0;
float wanted_velocity_m_s = 0;
float wanted_acc_m_ss = 0;

uint8_t status1,status2,status3,status4;
bool valid1,valid2,valid3,valid4;
int32_t enc1;
int32_t enc2;
int32_t speed1;
int32_t speed2;
float average_robot_vel_m_s, average_dError;
float robot_vel_m_sec;
float robot_pos_m;
float base_acc, error,prev_error, dError,prev_dError, ddError, integral, integral_res;
float pos_error,prev_pos_error,pos_error_integral;
float vel_error,prev_vel_error,vel_error_integral;
float deltaT, velocity1, velocity2;
float wanted_velocity = 0;
float wanted_rotation = 0;
float wanted_position = 0;
uint8_t motion_enable = 0 , reset_encoders_when_stand = 0;
uint8_t want_to_stand = 1, prev_want_to_stand;
uint8_t SW1, prev_SW1, counter_SW1_ON, counter_SW1_OFF, SW1_pressed, SW1temp;
uint8_t SW2, prev_SW2, counter_SW2_ON, counter_SW2_OFF, SW2_pressed, SW2temp;
uint8_t SW1_motion_enable, main_LED, pitch_out_of_range=0; 

void setup() {
    pinMode (PIN_SW1,INPUT_PULLUP);
    pinMode (PIN_SW2,INPUT_PULLUP);
    pinMode (PIN_LED1,OUTPUT);
    pinMode (PIN_LED2,OUTPUT);

    RemoteXY_Init (); 
    Wire.begin();
    Serial.begin(115200);
    roboclaw.begin(115200);
    MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

    if (calib_gyro) calibrate_sensors();
    else { 
        EEPROM.get(4, base_x_gyro);      delay(20);
        EEPROM.get(10, base_y_gyro);     delay(20);
        EEPROM.get(16, base_z_gyro);     delay(20);
    }
    set_last_read_angle_data(millis(), 0, 0, 0);
}


void loop() {
  // Serial.println (millis()-last_cycle);
  while (millis()-last_cycle<10) {}  last_cycle = millis();
  deal_with_standing  ();
  get_pitch_and_vel   ();
  read_robot_vel_pos  ();
  calc_pos_vel_errors ();
  calc_PID_errors     ();
  calc_wanted_velocity();
  send_motors_commands(wanted_velocity_m_s, wanted_rotation);
  RemoteXY_Handler    ();      // read from bluetooth 
  get_user_commands   ();
  control_LEDs ();

  send_telemetry();
  RemoteXY.onlineGraph_1_var1 = wanted_angle * RADIANS_TO_DEGREES;
  RemoteXY.onlineGraph_1_var2 = pitch_rad * RADIANS_TO_DEGREES;
}

void  calc_PID_errors (){
  deltaT = float ((millis() - last_time))/1000;    last_time = millis();

  prev_error = error;
  prev_dError = dError;
  prev_wanted_angle = wanted_angle;  

  wanted_angle = KP_vel_to_angle * vel_error +  KI_vel_to_angle * vel_error_integral;
  if (want_to_stand == 1 && reset_encoders_when_stand == 0) {    // keep pos only after want to stand and encoders were reset
    wanted_angle  += KP_pos_to_angle * pos_error +  KI_pos_to_angle * pos_error_integral;
  }
  
  error = pitch_rad - wanted_angle;

  integral += error * deltaT;
  integral = constrainF (integral,-integral_limit,integral_limit); 
  if (motion_enable ==0) integral=0;
  
  integral_res += error * deltaT;
  integral_res = constrainF (integral,-integral_limit,integral_limit); 
  if (prev_error * error<0 || motion_enable==0) integral_res = 0;

  dError = pitch_vel_rad_sec - (wanted_angle - prev_wanted_angle) / deltaT;   // derivative of the error can be assumed to be the gyro rate
  average_dError = alpha_avg_dError * average_dError + (1-alpha_avg_dError) * dError;
  
  ddError = (dError - prev_dError) / deltaT;  
}

void calc_wanted_velocity (){
  base_acc = 9.8 * tan(pitch_rad);
  wanted_acc_m_ss = base_acc + KP_Angle_to_acc * error + KI_Angle_to_acc * integral + KD_Angle_to_acc * dError + KDD_Angle_to_acc * ddError + KI_Angle_to_acc_res*integral_res;
  wanted_acc_m_ss += KD_avgdError_to_acc * average_dError;
  
  wanted_velocity_m_s = wanted_velocity_m_s + wanted_acc_m_ss * deltaT + KP_vel_to_vel * (average_robot_vel_m_s - wanted_velocity);
  wanted_velocity_m_s = constrainF (wanted_velocity_m_s, -max_velocity,max_velocity);
  if (motion_enable ==0) wanted_velocity_m_s=0;
}

void deal_with_standing () {
  prev_want_to_stand = want_to_stand;
  if (abs(wanted_velocity)<=0.01) want_to_stand=1; else want_to_stand=0;
  if (want_to_stand ==1 && prev_want_to_stand==0)reset_encoders_when_stand = 1;
  if (reset_encoders_when_stand == 1 && abs(robot_vel_m_sec)<0.01 ) { 
      roboclaw.SetEncM1(address,0);
      roboclaw.SetEncM2(address,0);
      reset_encoders_when_stand = 0;
  }
}

void get_user_commands () {
  wanted_velocity = alpha_stick * wanted_velocity + (1-alpha_stick) * float(-RemoteXY.joystick_1_y)/75;
  wanted_rotation = alpha_stick * wanted_rotation + (1-alpha_stick) * float(RemoteXY.joystick_1_x)/200;
  motion_enable = ((RemoteXY.switch_1 || SW1_motion_enable) && pitch_out_of_range ==0);

  prev_SW1 = SW1; 
  SW1temp = (1-digitalRead (PIN_SW1));
  if (SW1temp==1) {counter_SW1_ON +=1;   if (counter_SW1_ON  >10) {SW1=1; counter_SW1_ON  = 11; }} else counter_SW1_ON  = 0;
  if (SW1temp==0) {counter_SW1_OFF+=1;   if (counter_SW1_OFF >10) {SW1=0; counter_SW1_OFF = 11; }} else counter_SW1_OFF = 0;
  if (SW1==0 && prev_SW1==1) SW1_pressed=1;  else SW1_pressed=0;

  prev_SW2 = SW2; 
  SW2temp = (1-digitalRead (PIN_SW2));
  if (SW2temp==1) {counter_SW2_ON +=1;   if (counter_SW2_ON  >10) {SW2=1; counter_SW2_ON  = 11; }} else counter_SW2_ON  = 0;
  if (SW2temp==0) {counter_SW2_OFF+=1;   if (counter_SW2_OFF >10) {SW2=0; counter_SW2_OFF = 11; }} else counter_SW2_OFF = 0;
  if (SW2==0 && prev_SW2==1) SW2_pressed=1;  else SW2_pressed=0;

  if (SW1_pressed) SW1_motion_enable = 1-SW1_motion_enable;
}

void control_LEDs ()
{
    if (main_LED)
    {
      if (millis()-lastUSRblink >10 + 380*motion_enable)
      {
        main_LED=0;
        lastUSRblink=millis();
        digitalWrite (PIN_LED1, 0); 
      }
    }
    else
    {
      if (millis()-lastUSRblink>390 - 370*motion_enable)
      {
        main_LED=1;
        lastUSRblink=millis();
        digitalWrite (PIN_LED1, 1); 
      }
    }
}

void get_pitch_and_vel () { 
  orientation a = get_orientation();
  pitch_vel_rad_sec = a.pitch_vel / RADIANS_TO_DEGREES;
  pitch_rad = (a.pitch + Gyro_bias) / RADIANS_TO_DEGREES;
  if (abs(pitch_rad)>0.4) pitch_out_of_range = 1; else pitch_out_of_range = 0;
}

void calc_pos_vel_errors(){
  prev_pos_error = pos_error;
  pos_error = wanted_position - robot_pos_m;
  if (want_to_stand ==1 && reset_encoders_when_stand ==0){    
    pos_error_integral += pos_error * deltaT;
  }   // accumulte position integrlar only when standing and after encoders were reset
  pos_error_integral = constrainF (pos_error_integral,-pos_error_integral_limit,pos_error_integral_limit); 
  if (motion_enable == 0) pos_error_integral = 0;

  prev_vel_error = vel_error;
  vel_error = wanted_velocity - average_robot_vel_m_s;
  if (want_to_stand ==1 && reset_encoders_when_stand ==0){    
    vel_error_integral += vel_error * deltaT;
  }   // accumulte position integrlar only when standing and after encoders were reset
  vel_error_integral = constrainF (vel_error_integral,-vel_error_integral_limit,vel_error_integral_limit); 
  if (motion_enable ==0)  vel_error_integral = 0;
}

float constrainF (float val, float minV, float maxV){
  if (val > maxV) val = maxV;
  if (val < minV) val = minV; 
  return(val);
}

void read_robot_vel_pos(){
  enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  robot_vel_m_sec = float(speed2+speed1)/2/click_in_meter;
  robot_pos_m = float (enc2+enc1)/2/click_in_meter;
  average_robot_vel_m_s = alpha_avg_vel* average_robot_vel_m_s + (1-alpha_avg_vel)*robot_vel_m_sec;
}

void send_motors_commands (float velocity ,float rotation ){
  if (motion_enable) {
    velocity1 = (velocity-rotation) * click_in_meter;
    velocity2 = (velocity+rotation) * click_in_meter;
  }
  else
  {
    velocity1 = 0;
    velocity2 = 0;
    roboclaw.SetEncM1(address,0);
    roboclaw.SetEncM2(address,0);
  }
  roboclaw.SpeedM1(address, velocity1);
  roboclaw.SpeedM2(address, velocity2); 
}

int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;

  int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  uint8_t swap;

  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;

  // Discard the first set of values read from the IMU
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);

  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(20);
  }

  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  EEPROM.put(4, base_x_gyro);      delay(20);
  EEPROM.put(10, base_y_gyro);     delay(20);
  EEPROM.put(16, base_z_gyro);     delay(20);
}

orientation get_orientation() {
  accel_t_gyro_union accel_t_gyro;

  int error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);

  unsigned long t_now = millis();

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;

  //float G_CONVERT = 16384;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;

  // Get angle values from accelerometer
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;

  float accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();

  float angle_x = alpha_gyro*gyro_angle_x + (1.0 - alpha_gyro)*accel_angle_x;
  float angle_y = alpha_gyro*gyro_angle_y + (1.0 - alpha_gyro)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z);

  return {
    angle_x,
    angle_y,
    angle_z,
    gyro_x,
    gyro_y,
    gyro_z
  };
}

int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

void send_telemetry(){
  if (Telemetry && millis()-last_sent>20){
    last_sent=millis();
//    Serial.print("vel"); Serial.print(robot_vel_m_sec);
//    Serial.print("pos"); Serial.print(robot_pos_m);
//    Serial.print(" pit "); Serial.print(pitch_rad);
//    Serial.print(" pit_v "); Serial.print(pitch_vel_rad_sec);
//    Serial.print(" wv "); Serial.print(wanted_velocity_m_s);
//    Serial.print(" CT "); Serial.print(deltaT);
//    Serial.print(" dd "); Serial.print(ddError);
//    Serial.print(" int "); Serial.print(1000*integral);
//    Serial.print(" wa "); Serial.print(57295*wanted_angle);
//    Serial.print(" ac "); Serial.print(57295*pitch_rad);
//    Serial.print(" avgdE "); Serial.print(1000*average_dError);
//    Serial.print(" posEI "); Serial.print(1000*pos_error_integral);
//    Serial.print(" velEI "); Serial.print(1000*vel_error_integral);
    Serial.println("");
  }
}
