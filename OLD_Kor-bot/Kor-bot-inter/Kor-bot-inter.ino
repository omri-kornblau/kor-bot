#include <Wire.h>
#include <I2Cdev.h>
#include <RoboClaw.h>
#include <EEPROM.h>

#define Telemetry   1
#define calib_gyro  0

#define alpha_gyro        0.99        // Takes most of the pitch data from gyro .. very little noise to acc
#define mix_yaw_pitch    -0.02        // in-orthogonality of the gyro axes
#define MPU  0x68                     // MPU6050 I2C address

#define KDD_Angle_to_acc    0.6       //  0.6
#define KD_Angle_to_acc     4         //  4
#define KP_Angle_to_acc     10        //  10
#define KI_Angle_to_acc     3         //  3
#define KI_Angle_to_acc_res 10        //  10       integral that resets when error change dir
#define KP_vel_to_vel       0.0       //  0    0.015    adds to the velocity a factor of average velocity - do damp it down
#define KD_avgdError_to_acc 15        //  15       slows the robot when closing the error fast - like KD , but on average D to eliminate noise 
#define integral_limit      0.035

#define KP_pos_to_angle     0.05      // 0.05 to keep 0
#define KI_pos_to_angle     0.015     // 0.015 to keep 0
#define Pos_integral_limit  0.4       // 0.4

#define KP_vel_to_angle     0.07       // 0.07
#define KP_avg_vel_to_angle 0.15      // 0.15
#define KI_vel_to_angle     0.02      // 0.02
#define Vel_integral_limit  0.4       // 0.4

#define Gyro_bias 0.75                // deg   lower value = move forward / add -1*offset displayed in phone
#define gain_to_bot_acc     1.0       // enlarge the effect of robot acceleration on the gyro

#define alpha_avg_vel       0.9       // 0.9 avergaring factor for the averaged velocity 
#define alpha_avg_dError    0.9       // 0.9 avergaring factor for the averaged derivative of the error 
#define alpha_stick         0.9       // 0.9 avergaring factor for the averaged user stick commands
#define alpha_avg_vel_err   0.95      // 0.95 avergaring factor for the averaged velocity error
#define alpha_yaw_average   0.9       // 0.9 

#define RoboClaw_address    0x80      // adress of the roboclaw 128
#define click_in_meter      47609     // encoder clics in one meter
#define max_velocity        3
#define max_jerk            7
#define max_acceleration    2

#define PIN_SW1   3
#define PIN_SW2   4
#define PIN_LED1  5
#define PIN_LED2  6

#define earth_G   9.8
#define RAD_TO_DEGs 57.295
#define MPU6050_ACCEL_XOUT_H 0x3B   // R
#define MPU6050_GYRO_DATA 0x43      // R 
#define MPU6050_PWR_MGMT_1 0x6B     // R/W
#define MPU6050_PWR_MGMT_2 0x6C     // R/W
#define MPU6050_WHO_AM_I  0x75      // R
#define MPU6050_I2C_ADDRESS 0x68    // R


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
  5,32,5,35,54,54,177,26,31,2,
  0,43,91,20,9,177,26,8,1,79,
  78,0,0,68,34,1,1,61,27,186,
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


RoboClaw roboclaw(&Serial1,10000);

typedef struct {
  float yaw;
  float pitch;
  float roll;
  float yaw_vel;
  float pitch_vel;
  float roll_vel;
} orientation;

unsigned long last_read_time;

float AccX, AccY, AccZ;
float accAngleX, accAngleY;
float gyro_Roll_vel,   gyro_Pitch_vel,   gyro_Yaw_vel;
float gyro_Roll_angle, gyro_Pitch_angle, gyro_Yaw_angle;
float Gyro_Roll_bias,  Gyro_Pitch_bias,  Gyro_Yaw_bias;
float elapsedTime, previousTime, time_of_MPU_request;

orientation robot_orientation;

unsigned long last_time,last_sent, last_cycle, last_main, lastUSRblink;
float wanted_angle = 0, prev_wanted_angle;
float pitch_rad;
float pitch_vel_rad_sec = 0;
float wanted_velocity_from_alg_m_s = 0;
float wanted_acc_m_ss = 0;

uint8_t status1 , status2 , status3 , status4 ;
bool    valid1  , valid2  , valid3  , valid4  ;
int32_t enc1    , enc2;
int32_t speed1  , speed2;

float robot_pos_m;
float robot_vel_m_sec;
float robot_acc_m_ss;
float average_robot_vel_m_s;
float error;
float dError;
float ddError;
float integral;
float integral_res;
float prev_error;
float prev_dError;
float average_dError;
float base_acceleration_to_cancel_moment;
float pos_error , prev_pos_error, pos_error_integral;
float vel_error , prev_vel_error, vel_error_integral;
float averaged_vel_error;
float deltaT = 0.01;
float velocity_Motor_1, velocity_Motor_2;
float started_rotation_angle;
float wanted_position = 0;
float wanted_velocity_from_user_m_s = 0;
float wanted_rotation_from_user = 0;
float filtered_stick_velocity, filtered_stick_rotation, max_allowed_acceleration;

uint8_t motion_enable = 0 , reset_encoders_when_stand = 0;
uint8_t want_to_stand = 1, prev_want_to_stand;
uint8_t SW1 , prev_SW1  , counter_SW1_ON  , counter_SW1_OFF , SW1_pressed , SW1temp;
uint8_t SW2 , prev_SW2  , counter_SW2_ON  , counter_SW2_OFF , SW2_pressed , SW2temp;
uint8_t SW1_motion_enable, last_motion_enable, main_LED;
uint8_t pitch_out_of_range=0; 
uint8_t RaspberryPi_index, dizzy , first_run=1;
byte temp;


void setup() 
{
    pinMode (PIN_SW1,INPUT_PULLUP);
    pinMode (PIN_SW2,INPUT_PULLUP);
    pinMode (PIN_LED1,OUTPUT);
    pinMode (PIN_LED2,OUTPUT);

    RemoteXY_Init (); 
    Wire.begin();
    
    if (Telemetry) Serial.begin(115200);    // Communication via USB to PC
                                            // Serial1 is connected to the Roboclaw  driver  and  is configured there
                                            // Serial2 is connected to the Bluetooth RemoteXY and is configured there
    Serial3.begin (115200);                 // Communincatioon with the Raspberry pi  
    roboclaw.begin(115200);
    reset_gyro ();

    if (calib_gyro) calibrate_gyro_vel();
    else 
      { 
        EEPROM.get(4,  Gyro_Roll_bias);    delay(20);
        EEPROM.get(10, Gyro_Pitch_bias);   delay(20);
        EEPROM.get(16, Gyro_Yaw_bias);     delay(20);
      }
    digitalWrite (PIN_LED2, 0);
      // initialize timer1 
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 20000;            // compare match register 16MHz/8/100Hz
  TCCR4B |= (1 << WGM12);   // CTC mode
  TCCR4B |= (1 << CS11);    // 8 prescaler 
  TIMSK4 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enab
}

ISR(TIMER4_COMPA_vect)          // timer compare interrupt service routine
{
    Serial.println (millis()-last_cycle);
    last_cycle = millis ();
      noInterrupts(); 
    deal_with_standing  ();
    get_pitch_and_vel   ();
    read_robot_vel_pos  ();
    calc_pos_vel_errors ();
    calc_PID_errors     ();
    calc_wanted_velocity();
    interrupts();  
}


void loop() 
{ 
    while (millis() - last_main<10) {interrupts();}
    last_main = millis  ();


    send_motors_commands(wanted_velocity_from_alg_m_s, wanted_rotation_from_user);
    
    RemoteXY_Handler    ();      // read from bluetooth 
    read_user_commands  ();
    control_LEDs        ();
    send_to_RaspberryPi ();
    send_telemetry      ();
    send_to_remoteXY    ();

        first_run = 0;
}


void  reset_gyro ()
{
    digitalWrite (PIN_LED2, 1);
    Wire.beginTransmission(MPU);            // Start communication with MPU6050 // MPU=0x68
    Wire.write(MPU6050_PWR_MGMT_1);         // Talk to the register 6B
    Wire.write(0x00);                       // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);             // End the transmission
}


void  send_to_remoteXY()
{
    RemoteXY.onlineGraph_1_var1 = wanted_angle * RAD_TO_DEGs;
    RemoteXY.onlineGraph_1_var2 = pitch_rad * RAD_TO_DEGs;
}

void alpha_beta (float &average,float value,float alpha)
{
   average = alpha * average + (1-alpha) * value;
}

void  send_to_RaspberryPi ()
{
    static float average_yaw , avg_stick_x;
    alpha_beta (average_yaw , robot_orientation.yaw , alpha_yaw_average);
    if (abs(RemoteXY.joystick_1_x) > abs(avg_stick_x)) alpha_beta (avg_stick_x, RemoteXY.joystick_1_x, alpha_yaw_average);
    else avg_stick_x = RemoteXY.joystick_1_x;
    
    if (RaspberryPi_index == 0)
        Serial3.write(0xff);
    if (RaspberryPi_index == 1)
        Serial3.write(0xff);
    if (RaspberryPi_index == 2)
        Serial3.write(byte( constrainF(573*pitch_rad+128,0,254) ));               // send pitch in 0.1 Deg 
    if (RaspberryPi_index == 3)
        Serial3.write(byte( constrainF((robot_orientation.yaw - average_yaw)*6 + (RemoteXY.joystick_1_x - avg_stick_x )*10 + 128,0,254)  ) );     // sends high byte of yaw in 0.1 deg
    if (RaspberryPi_index == 4)
        Serial3.write(byte( constrainF(50*robot_vel_m_sec + 128,0,254) ));        // send vel in 2cm/sec
    if (RaspberryPi_index == 5)
      {
        Serial3.write(byte( dizzy));  
        if (dizzy ==1) dizzy = 0; 
      }
    RaspberryPi_index += 1;
    if (RaspberryPi_index == 6) RaspberryPi_index = 0;
}


void  calc_PID_errors ()
{
    deltaT = float ((millis() - last_time))/1000;
    last_time = millis();

    prev_error = error;
    prev_dError = dError;
    prev_wanted_angle = wanted_angle;  

    wanted_angle = KP_vel_to_angle * vel_error +  KI_vel_to_angle * vel_error_integral + KP_avg_vel_to_angle * averaged_vel_error;
    if (want_to_stand == 1 && reset_encoders_when_stand == 0) // keep pos only after want to stand and encoders were reset
      {
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
    alpha_beta (average_dError, dError, alpha_avg_dError);

    ddError = (dError - prev_dError) / deltaT;  
}


void  calc_wanted_velocity ()
{
    base_acceleration_to_cancel_moment = earth_G * tan(pitch_rad);
    wanted_acc_m_ss  = base_acceleration_to_cancel_moment   + KP_Angle_to_acc * error    + KI_Angle_to_acc * integral;
    wanted_acc_m_ss += KD_Angle_to_acc * dError             + KDD_Angle_to_acc * ddError + KI_Angle_to_acc_res*integral_res;
    wanted_acc_m_ss += KD_avgdError_to_acc * average_dError;
    
    wanted_velocity_from_alg_m_s = wanted_velocity_from_alg_m_s + wanted_acc_m_ss * deltaT + KP_vel_to_vel * (average_robot_vel_m_s - wanted_velocity_from_user_m_s);
    wanted_velocity_from_alg_m_s = constrainF (wanted_velocity_from_alg_m_s, -max_velocity,max_velocity);
    if (motion_enable ==0) wanted_velocity_from_alg_m_s=0;
}


void  deal_with_standing () 
{
    prev_want_to_stand = want_to_stand;
    if (abs(wanted_velocity_from_user_m_s) <= 0.01 && abs(wanted_rotation_from_user) <= 0.02) want_to_stand = 1; 
    else want_to_stand = 0;
    if (want_to_stand == 1 && prev_want_to_stand == 0)reset_encoders_when_stand = 1;
    if (reset_encoders_when_stand == 1 && abs(robot_vel_m_sec) < 0.01 ) 
      { 
        roboclaw.SetEncM1(RoboClaw_address,0);
        roboclaw.SetEncM2(RoboClaw_address,0);
        reset_encoders_when_stand = 0;
      }
}


void  read_user_commands () 
{
    alpha_beta (filtered_stick_velocity, float(-RemoteXY.joystick_1_y)/50 , alpha_stick);
    alpha_beta (filtered_stick_rotation, float( RemoteXY.joystick_1_x)/150, alpha_stick);
    if (filtered_stick_velocity > wanted_velocity_from_user_m_s)
      {
        max_allowed_acceleration = max_allowed_acceleration + max_jerk * deltaT;
        if (max_allowed_acceleration > max_acceleration) max_allowed_acceleration = max_acceleration;
        wanted_velocity_from_user_m_s = min(wanted_velocity_from_user_m_s + max_allowed_acceleration * deltaT , filtered_stick_velocity);
      }
    if (filtered_stick_velocity < wanted_velocity_from_user_m_s)
      {
        max_allowed_acceleration = max_allowed_acceleration - max_jerk * deltaT;
        if (max_allowed_acceleration < max_acceleration) max_allowed_acceleration = max_acceleration;
        wanted_velocity_from_user_m_s = max(wanted_velocity_from_user_m_s - max_allowed_acceleration * deltaT , filtered_stick_velocity);
      }

    wanted_rotation_from_user = filtered_stick_rotation;
    
    last_motion_enable = motion_enable;
    motion_enable = ((RemoteXY.switch_1 || SW1_motion_enable) && pitch_out_of_range ==0);

    if (motion_enable == 1 && last_motion_enable == 0) reset_before_moving();
      
    if (abs(wanted_rotation_from_user) < 0.03)
      {
        if (abs(robot_orientation.yaw - started_rotation_angle) > 270) dizzy =1;
        started_rotation_angle = robot_orientation.yaw;
      }

    prev_SW1 = SW1; 
    SW1temp  = (1-digitalRead (PIN_SW1));
    count_switch (SW1temp, SW1 , 0);
    if (SW1==0 && prev_SW1==1) SW1_pressed=1;  
    else SW1_pressed=0;

    prev_SW2 = SW2; 
    SW2temp = (1-digitalRead (PIN_SW2));
    count_switch (SW2temp, SW2 , 1);
    if (SW2==0 && prev_SW2==1) SW2_pressed=1;  
    else SW2_pressed=0;

    if (SW1_pressed) SW1_motion_enable = 1-SW1_motion_enable;
}

void  reset_before_moving()
{
    roboclaw.SetEncM1(RoboClaw_address,0);
    roboclaw.SetEncM2(RoboClaw_address,0);
    wanted_velocity_from_user_m_s = 0;
    wanted_rotation_from_user = 0;
    filtered_stick_velocity = 0;
    filtered_stick_rotation = 0;
}

byte  count_switch (byte SW_status, byte &SWX , byte switch_num)
{ 
    static byte prev_SW_status[2], counter[2];
    counter[switch_num] +=1;   
    if (counter[switch_num] > 10) 
       {
         SWX=SW_status; 
         counter[switch_num] = 11; 
       }
    if (SW_status != prev_SW_status[switch_num]) counter[switch_num] = 0;
    prev_SW_status[switch_num] = SW_status;
}


void  control_LEDs ()
{   
    static byte LED2;
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
    if (SW2_pressed) LED2=1-LED2;
    //  digitalWrite (PIN_LED2, LED2);
}


void get_pitch_and_vel () 
{ 
    robot_orientation = get_orientation();
    pitch_vel_rad_sec = robot_orientation.pitch_vel / RAD_TO_DEGs;
    pitch_rad = (robot_orientation.pitch + Gyro_bias) / RAD_TO_DEGs;
    if (abs(pitch_rad)<0.4) pitch_out_of_range = 0; 
    else pitch_out_of_range = 1;
}


void calc_pos_vel_errors()
{
    prev_pos_error = pos_error;
    pos_error = wanted_position - robot_pos_m;
    if (want_to_stand ==1 && reset_encoders_when_stand ==0)
      {    
        pos_error_integral += pos_error * deltaT;
      }     // accumulte position integral only when standing and after encoders were reset
    pos_error_integral = constrainF (pos_error_integral,-Pos_integral_limit,Pos_integral_limit); 
    if (motion_enable == 0) pos_error_integral = 0;
  
    prev_vel_error = vel_error;
    vel_error = wanted_velocity_from_user_m_s - robot_vel_m_sec;
    alpha_beta (averaged_vel_error, vel_error, alpha_avg_vel_err);
    if (want_to_stand ==1 && reset_encoders_when_stand ==0)
      {    
        vel_error_integral += vel_error * deltaT;
      }     // accumulte position integrlar only when standing and after encoders were reset
    vel_error_integral = constrainF (vel_error_integral,-Vel_integral_limit,Vel_integral_limit); 
    if (motion_enable ==0)  vel_error_integral = 0;
}


float constrainF (float val, float minV, float maxV)
{
    if (val > maxV) val = maxV;
    if (val < minV) val = minV; 
    return(val);
}


void read_robot_vel_pos()
{
    float prev_robot_vel_m_sec;
    prev_robot_vel_m_sec = robot_vel_m_sec;
    enc1 = roboclaw.ReadEncM1(RoboClaw_address, &status1, &valid1);
    enc2 = roboclaw.ReadEncM2(RoboClaw_address, &status2, &valid2);
    speed1 = roboclaw.ReadSpeedM1(RoboClaw_address, &status3, &valid3);
    speed2 = roboclaw.ReadSpeedM2(RoboClaw_address, &status4, &valid4);
    robot_vel_m_sec = float (speed2 + speed1) / 2 / click_in_meter;
    robot_pos_m     = float (enc2   + enc1  ) / 2 / click_in_meter;
    robot_acc_m_ss  = (robot_vel_m_sec - prev_robot_vel_m_sec) / deltaT;
    alpha_beta (average_robot_vel_m_s, robot_vel_m_sec, alpha_avg_vel);
}


void send_motors_commands (float velocity ,float rotation )
{
    if (motion_enable) 
      {
        velocity_Motor_1 = (velocity - rotation) * click_in_meter;
        velocity_Motor_2 = (velocity + rotation) * click_in_meter;
      }
    else
      {
        velocity_Motor_1 = 0;
        velocity_Motor_2 = 0;
      }
    roboclaw.SpeedM1(RoboClaw_address, velocity_Motor_1);
    roboclaw.SpeedM2(RoboClaw_address, velocity_Motor_2); 
}


void calibrate_gyro_vel() 
{
    int c=0; 
    int num_reps = 1000;
    while (c < num_reps) 
      {
        request_from_MPU (MPU6050_GYRO_DATA, 6);
        gyro_Roll_vel  = Wire.read() << 8 | Wire.read();
        gyro_Pitch_vel = Wire.read() << 8 | Wire.read();
        gyro_Yaw_vel   = Wire.read() << 8 | Wire.read();
        Gyro_Roll_bias  = Gyro_Roll_bias  + (gyro_Roll_vel  / 131.0);
        Gyro_Pitch_bias = Gyro_Pitch_bias + (gyro_Pitch_vel / 131.0);
        Gyro_Yaw_bias   = Gyro_Yaw_bias   + (gyro_Yaw_vel   / 131.0);
        c++;
      }
    Gyro_Roll_bias  = Gyro_Roll_bias  / num_reps;
    Gyro_Pitch_bias = Gyro_Pitch_bias / num_reps;
    Gyro_Yaw_bias   = Gyro_Yaw_bias   / num_reps;
    EEPROM.put(4,  Gyro_Roll_bias);    delay(20);
    EEPROM.put(10, Gyro_Pitch_bias);   delay(20);
    EEPROM.put(16, Gyro_Yaw_bias);     delay(20);
}


void request_from_MPU (int register_start_address, int number_of_bytes)
{
    Wire.beginTransmission(MPU);
    Wire.write(register_start_address);                         // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, number_of_bytes, true);               // Read 6 registers total, each axis value is stored in 2 registers
}

orientation get_orientation() 
{
    static uint8_t timed_out;
    request_from_MPU (MPU6050_ACCEL_XOUT_H, 6);
    timed_out = 0; 
    time_of_MPU_request = millis();
    while (Wire.available() < 5 || timed_out) { if (millis() - time_of_MPU_request > 10) timed_out = 1; }
    if (timed_out) reset_gyro ();
    else
      {
        AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;        //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
        AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; 
        AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
      }
    float robot_acc_X = gain_to_bot_acc*robot_acc_m_ss / earth_G * cos(pitch_rad);
    float robot_acc_Z = gain_to_bot_acc*robot_acc_m_ss / earth_G * sin(pitch_rad);
    accAngleY = (atan( (robot_acc_X - AccX) / sqrt(pow(AccY, 2) + pow((AccZ/1.05+robot_acc_Z), 2))) * RAD_TO_DEGs) + 2.69;        // accel error y

    elapsedTime  = (millis() - previousTime) / 1000;          // Divide by 1000 to get seconds
    previousTime =  millis();                                 // Current time actual time read

    request_from_MPU (MPU6050_GYRO_DATA+2, 4);
    timed_out = 0; 
    time_of_MPU_request = millis();
    while (Wire.available() < 3 || timed_out) { if (millis() - time_of_MPU_request > 10) timed_out = 1; }
    if (timed_out) reset_gyro ();
    else
      {
        gyro_Pitch_vel = (Wire.read() << 8 | Wire.read()) / 131.0;
        gyro_Yaw_vel   = (Wire.read() << 8 | Wire.read()) / 131.0;
      }
    gyro_Pitch_vel = gyro_Pitch_vel - Gyro_Pitch_bias; 
    gyro_Yaw_vel   = gyro_Yaw_vel   - Gyro_Yaw_bias;
     
    gyro_Pitch_vel = gyro_Pitch_vel + mix_yaw_pitch * gyro_Yaw_vel;

    gyro_Pitch_angle = gyro_Pitch_angle + gyro_Pitch_vel  * elapsedTime;
    gyro_Yaw_angle   = gyro_Yaw_angle   + gyro_Yaw_vel    * elapsedTime;

    if (first_run) gyro_Pitch_angle = accAngleY;
    alpha_beta (gyro_Pitch_angle, accAngleY, alpha_gyro);
   
    return {
      gyro_Yaw_angle,
      gyro_Pitch_angle,
      gyro_Roll_angle,
      gyro_Yaw_vel,
      gyro_Pitch_vel,
      gyro_Roll_vel
    };
}


void send_telemetry()
{
  if (Telemetry && millis()-last_sent>20)
  {
      last_sent=millis();
//    Serial.print("vel"); Serial.print(robot_vel_m_sec);
//    Serial.print("pos"); Serial.print(robot_pos_m);
//    Serial.print(" pit "); Serial.print(pitch_rad);
//    Serial.print(" pit_v "); Serial.print(pitch_vel_rad_sec);
//    Serial.print(" wv "); Serial.print(wanted_velocity_from_alg_m_s);
//    Serial.print(" CT "); Serial.print(deltaT);
//    Serial.print(" dd "); Serial.print(ddError);
//    Serial.print(" int "); Serial.print(1000*integral);
//    Serial.print(" wa "); Serial.print(57295*wanted_angle);
//    Serial.print(" ac "); Serial.print(57295*pitch_rad);
//    Serial.print(" avgdE "); Serial.print(1000*average_dError);
//    Serial.print(" posEI "); Serial.print(1000*pos_error_integral);
//    Serial.print(" velEI "); Serial.print(1000*vel_error_integral);
//    Serial.println("");
  }
}
