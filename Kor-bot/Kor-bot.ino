#include <I2C.h>
#include <EEPROM.h>
#include <RoboClaw.h>

#define SEND_TELEMETRY    false
#define CALIBRATE_GYRO    false

// IMU & Gyro related definitions //
#define ALPHA_GYRO_ACCEL      0.99      // Takes most of the pitch data from gyro .. very little noise to acc
#define MIX_YAW_PITCH         -0.016    // in-orthogonality of the gyro axes //  more positive = right moves more when rotating 
#define MPU_ADDRESS           0x68      // MPU6050 I2C address
#define MPU6050_ACCEL_DATA    0x3B      // R
#define MPU6050_GYRO_DATA     0x43      // R 
#define MPU6050_PWR_MGMT_1    0x6B      // R/W
#define EARTH_GRAVITATION     9.8       // M/S2
#define RAD_TO_DEGs           57.295
#define GYRO_HEIGT_M          0.08      // m
#define IMU_PITCH_OFFSET_DEF  5         // deg   lower value = move forward / add -1*offset displayed in phone
#define VALID_ACC_ANGLE_DIFF  45        // if diff between the gyro angle and the acc angle is above this, the accelerometers are temporarily disabled
#define MAX_JUMP_IN_PITCH     90        // max jump in pitch or yaw velocities in one cycle (to delete comm errors)

// PARAMETERS OF ACCELERATION CALCULATION EXTENDED PID FILTER
#define KDD_PITCH_TO_ACC      0.6       //  0.6
#define KD_PITCH_TO_ACC       8         //  8
#define KP_PITCH_TO_ACC       15        //  15
#define KI_PITCH_TO_ACC       3         //  3
#define KI_PITCH_TO_ACC_RES   10        //  10    integral that resets when error change dir
#define KD_AVGdERROR_TO_ACC   2         //  2     slows the robot when closing the error fast - like KD , but on average D to eliminate noise 
#define INTEGRAL_LIMIT        0.035

// PARAMETERS OF WANTED PITCH CALCULATION  PID FILTER
#define KP_POS_TO_PITCH       0.05      //  0.05  
#define KI_POS_TO_PITCH       0.01      //  0.01  
#define POS_ERROR_LIMIT       0.2       //  max position error permitted for calculations 
#define POS_INTEGRAL_LIMIT    0.4       //  0.4
#define KP_VELOC_TO_PITCH     0.1       //  0.1
#define KP_AVG_VEL_TO_PITCH   0.1       //  0.1     
#define KI_VELOC_TO_PITCH     0.01      //  0.01
#define VEL_INTEGRAL_LIMIT    0.4       //  0.4

#define ALPHA_AVG_dERROR      0.9       //  0.9 avergaring factor for the averaged derivative of the error 
#define ALPHA_AVG_VEL_ERR     0.9       //  0.9 avergaring factor for the averaged velocity error
#define ALPHA_YAW_AVERAGE     0.95       //  0.9 
#define ALPHA_AVG_VELOCITY    0.9       //  0.9 avergaring factor for the averaged velocity 

#define JOYSTICK_DEAD_BAND    10        // % of joystick range 
#define ALPHA_JOYSTICK        0.8       //  0.9 avergaring factor for the averaged user stick commands
#define SPEED_TO_STAND_MS     0.03      // 2 cm/s

#define ROBOCLAW_ADDRESS      0x80      //  adress of the roboclaw 128
#define CLICKS_IN_METER       47609     //  encoder clics in one meter
#define MAX_JERK_USR_MSSS     4.0
#define MAX_ACCEL_USR_MSS     2.0       //  max acceleration that the user can request 
#define MAX_VELOCITY_ALG_MS   3.0       //  max velocity that the algorithms will send to the motors
#define MAX_VELOCITY_USR_MS   1.5       //  max velocity that the user can request
#define MAX_ROTATION_USR_MS   1.0
#define MAX_PITCH_FOR_MOTION  0.6       //  disable motion above this angle

#define PIN_SW1   3
#define PIN_SW2   4
#define PIN_LED1  5
#define PIN_LED2  6

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
  { 255,4,0,9,0,76,0,10,24,1,
  5,32,10,47,44,44,190,26,31,68,
  2,0,0,63,25,186,50,137,1,0,
  51,88,12,12,191,190,0,65,2,50,
  41,13,13,3,134,8,26,47,9,191,
  26,129,0,9,35,41,4,192,87,65,
  32,32,71,65,32,32,32,67,70,32,
  32,72,89,32,32,86,65,32,32,32,
  67,71,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  uint8_t button_1; // =1 if button pressed, else =0 
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ... 

    // output variables
  float onlineGraph_1_var1;
  float onlineGraph_1_var2;
  uint8_t led_1_g; // =0..255 LED Green brightness 

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
  float yaw_vel;
  float pitch_vel;
} orientation;

orientation robot_orientation;

uint32_t last_time,last_sent;
uint32_t last_cycle;
uint32_t lastUSRblink;
uint16_t acc_fail_events_counter;

float gyro_Pitch_vel,   gyro_Yaw_vel;
float gyro_Pitch_angle, gyro_Yaw_angle;
float gyro_Pitch_bias,  gyro_Yaw_bias;
float imu_pitch_offset, imu_pitch_offset_temp;
float gyro_Yaw_bias_temp = 0;
float gyro_Pitch_bias_temp = 0;
float accAngleY;
float wanted_pitch = 0, prev_wanted_pitch;
float pitch_rad;
float pitch_vel_rad_sec = 0;
float wanted_velocity_from_alg_m_s = 0;
float wanted_acc_m_ss = 0;
float robot_pos_m;
float robot_vel_m_sec;
float gyro_acc_m_ss;
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
float vel_error , vel_error_integral;
float averaged_vel_error;
float deltaT = 0.01;
float started_rotation_angle;
float wanted_position = 0;
float wanted_velocity_from_user_m_s = 0;
float wanted_rotation_from_user = 0;
float filtered_stick_velocity = 0;
float filtered_stick_rotation = 0;
float prev_wanted_acceleration;

bool motion_enable = false;
bool prev_motion_enable = false;
bool want_to_stand = true;
bool prev_want_to_stand = true;
bool reset_encoders_when_stand = false;
bool SW1 , prev_SW1 , SW1_pressed , SW1temp;
bool SW2 , prev_SW2 , SW2_pressed , SW2temp;
bool prev_button_1;
bool pitch_out_of_range = false; 
bool other_cycle = false; 
bool first_run = true;
bool main_LED = true;
bool dizzy = false;

uint8_t RaspberryPi_index;
uint8_t busy_cycle_time;
uint8_t GUI_selector, prev_GUI_selector;

void setup() 
{
    pinMode (PIN_SW1,INPUT_PULLUP);
    pinMode (PIN_SW2,INPUT_PULLUP);
    pinMode (PIN_LED1,OUTPUT);
    pinMode (PIN_LED2,OUTPUT);

    RemoteXY_Init (); 
    
    I2c.begin();
    I2c.timeOut(500);
    I2c.pullup(true);
        
    if (SEND_TELEMETRY) Serial.begin(115200);    // Communication via USB to PC
                                            // Serial1 is connected to the Roboclaw  driver  and  is configured there
                                            // Serial2 is connected to the Bluetooth RemoteXY and is configured there
    Serial3.begin (115200);                 // Communincatioon with the Raspberry pi  
    roboclaw.begin(115200);
    reset_gyro ();

    if (CALIBRATE_GYRO) calibrate_gyro_setup();
    else 
      { 
        EEPROM.get(10, gyro_Pitch_bias);   
        EEPROM.get(16, gyro_Yaw_bias);     
      }
    EEPROM.get(22, imu_pitch_offset);  
    EEPROM.get(28, imu_pitch_offset_temp);
    if (abs(imu_pitch_offset)>10 || imu_pitch_offset != imu_pitch_offset_temp || imu_pitch_offset ==0) imu_pitch_offset = IMU_PITCH_OFFSET_DEF;

    digitalWrite (PIN_LED2, 0);
}


void loop() 
{ 
    busy_cycle_time = millis()-last_cycle;
    while (millis()-last_cycle<10) {}  
    last_cycle = millis ();
    
    deal_with_standing  ();
    get_pitch_and_vel   ();
    read_robot_vel_pos  ();
    calc_pos_vel_errors ();
    calc_PID_errors     ();
    calc_wanted_velocity();
    send_motors_commands();
    send_to_remoteXY    ();
    RemoteXY_Handler    ();      // read from bluetooth 
    control_LEDs        ();
    send_to_RaspberryPi ();
    send_telemetry      ();
    read_user_commands  ();
    
    if (prev_GUI_selector == 5) calibrate_gyro_online(); 
    first_run = false;
    toggle (other_cycle);
}


void  reset_gyro ()
{
    I2c.write(MPU_ADDRESS,MPU6050_PWR_MGMT_1,0x00);
}


void  send_to_remoteXY()
{ 
  prev_GUI_selector = GUI_selector;
  GUI_selector = RemoteXY.select_1;
  switch (GUI_selector)
    {
      case 0:  // actual pitch vs wanted pitch    
        RemoteXY.onlineGraph_1_var1 = wanted_pitch * RAD_TO_DEGs;
        RemoteXY.onlineGraph_1_var2 = pitch_rad    * RAD_TO_DEGs;
        break;
  
      case 1:  // accelerometer based pitch vs gyro pitch
        RemoteXY.onlineGraph_1_var1 = accAngleY;
        RemoteXY.onlineGraph_1_var2 = pitch_rad    * RAD_TO_DEGs;
        break;
  
      case 2:  // busy time in cycle and accelerometer failures counter
        RemoteXY.onlineGraph_1_var1 = acc_fail_events_counter;
        RemoteXY.onlineGraph_1_var2 = busy_cycle_time;
        break;
  
      case 3:  // wanted heading vs yaw
        RemoteXY.onlineGraph_1_var1 = robot_orientation.yaw;
        RemoteXY.onlineGraph_1_var2 = 0;
        break;

      case 4:  // wanted heading vs yaw
        RemoteXY.onlineGraph_1_var1 = wanted_velocity_from_user_m_s;
        RemoteXY.onlineGraph_1_var2 = prev_wanted_acceleration;
        break;

      case 5:  // wanted heading vs yaw
        RemoteXY.onlineGraph_1_var1 = robot_orientation.yaw;
        RemoteXY.onlineGraph_1_var2 = gyro_Yaw_bias_temp;
        break;
  
      default:
        break;
    }
  if (motion_enable) RemoteXY.led_1_g = 255; else RemoteXY.led_1_g = 0;
}


void  calc_PID_errors ()
{
    deltaT = float ((millis() - last_time))/1000;
    last_time = millis();

    prev_error  = error;
    prev_dError = dError;
    prev_wanted_pitch = wanted_pitch;  

    wanted_pitch = KP_VELOC_TO_PITCH * vel_error +  KI_VELOC_TO_PITCH * vel_error_integral + KP_AVG_VEL_TO_PITCH * averaged_vel_error;
    if (want_to_stand && !reset_encoders_when_stand) // keep pos only after want to stand and encoders were reset
      {
        wanted_pitch  += KP_POS_TO_PITCH * pos_error +  KI_POS_TO_PITCH * pos_error_integral;
      }

    error = pitch_rad - wanted_pitch;

    integral += error * deltaT;
    constrainF (integral, -INTEGRAL_LIMIT , INTEGRAL_LIMIT); 
    if (!motion_enable) integral = 0;

    integral_res += error * deltaT;
    constrainF (integral_res, -INTEGRAL_LIMIT , INTEGRAL_LIMIT); 
    if (prev_error * error<0 || !motion_enable) integral_res = 0;

    dError = pitch_vel_rad_sec - (wanted_pitch - prev_wanted_pitch) / deltaT;   // derivative of the error can be assumed to be the gyro rate
    alpha_beta (average_dError, dError, ALPHA_AVG_dERROR);

    ddError = (dError - prev_dError) / deltaT;  
}


void  calc_wanted_velocity ()
{
    base_acceleration_to_cancel_moment = EARTH_GRAVITATION * tan(pitch_rad);
    wanted_acc_m_ss  = base_acceleration_to_cancel_moment   + KP_PITCH_TO_ACC  * error   + KI_PITCH_TO_ACC    * integral;
    wanted_acc_m_ss += KD_PITCH_TO_ACC     * dError         + KDD_PITCH_TO_ACC * ddError + KI_PITCH_TO_ACC_RES* integral_res;
    wanted_acc_m_ss += KD_AVGdERROR_TO_ACC * average_dError;
    
    wanted_velocity_from_alg_m_s = wanted_velocity_from_alg_m_s + wanted_acc_m_ss * deltaT;
    constrainF (wanted_velocity_from_alg_m_s, - MAX_VELOCITY_ALG_MS , MAX_VELOCITY_ALG_MS);
    if (!motion_enable) wanted_velocity_from_alg_m_s = 0;
}


void  deal_with_standing () 
{
    prev_want_to_stand = want_to_stand;
    if (abs(wanted_velocity_from_user_m_s) <= SPEED_TO_STAND_MS && abs(wanted_rotation_from_user) <= SPEED_TO_STAND_MS) want_to_stand = true; 
    else want_to_stand = false;
    if (want_to_stand && !prev_want_to_stand) reset_encoders_when_stand = true;
    if (reset_encoders_when_stand && abs(robot_vel_m_sec) < SPEED_TO_STAND_MS ) 
      { 
        roboclaw.SetEncM1(ROBOCLAW_ADDRESS,0);
        roboclaw.SetEncM2(ROBOCLAW_ADDRESS,0);
        reset_encoders_when_stand = false;
      }
}

void  filter_deadband_stick (float &filtered_stick_val, float raw_stick_val, float max_value)
{
    float stick_val_deadbanded;
    stick_val_deadbanded = 0;
    if (raw_stick_val >  JOYSTICK_DEAD_BAND) stick_val_deadbanded = raw_stick_val - JOYSTICK_DEAD_BAND;
    if (raw_stick_val < -JOYSTICK_DEAD_BAND) stick_val_deadbanded = raw_stick_val + JOYSTICK_DEAD_BAND;
    alpha_beta (filtered_stick_val, stick_val_deadbanded * max_value /100 , ALPHA_JOYSTICK);
}


void  read_user_commands () 
{
    static float prev_wanted_velocity;
    static float min_acceleration, max_acceleration;
    static float min_velocity, max_velocity;
    static float parabolic_extreme_velocity;
    
    filter_deadband_stick (filtered_stick_velocity, -RemoteXY.joystick_1_y, MAX_VELOCITY_USR_MS);
    filter_deadband_stick (filtered_stick_rotation,  RemoteXY.joystick_1_x, MAX_ROTATION_USR_MS);

    max_acceleration = min(prev_wanted_acceleration + MAX_JERK_USR_MSSS * deltaT , MAX_ACCEL_USR_MSS);
    min_acceleration = max(prev_wanted_acceleration - MAX_JERK_USR_MSSS * deltaT ,-MAX_ACCEL_USR_MSS);

    if (filtered_stick_velocity > prev_wanted_velocity && prev_wanted_acceleration > MAX_JERK_USR_MSSS* deltaT)
    {
      parabolic_extreme_velocity = prev_wanted_velocity + pow (prev_wanted_acceleration,2) /MAX_JERK_USR_MSSS /2;
      if ( parabolic_extreme_velocity >= filtered_stick_velocity) max_acceleration = prev_wanted_acceleration - MAX_JERK_USR_MSSS* deltaT;
    }
    
    if (filtered_stick_velocity<prev_wanted_velocity && prev_wanted_acceleration < -MAX_JERK_USR_MSSS* deltaT)
    {
      parabolic_extreme_velocity = prev_wanted_velocity - pow (prev_wanted_acceleration,2) /MAX_JERK_USR_MSSS /2;
      if ( parabolic_extreme_velocity <= filtered_stick_velocity) min_acceleration = prev_wanted_acceleration + MAX_JERK_USR_MSSS* deltaT;
    }

    max_velocity = min (prev_wanted_velocity + max_acceleration * deltaT , MAX_VELOCITY_USR_MS);
    min_velocity = max (prev_wanted_velocity + min_acceleration * deltaT ,-MAX_VELOCITY_USR_MS);

    wanted_velocity_from_user_m_s = filtered_stick_velocity;
    constrainF (wanted_velocity_from_user_m_s , min_velocity , max_velocity);
    
    prev_wanted_acceleration = (wanted_velocity_from_user_m_s - prev_wanted_velocity) / deltaT;
    prev_wanted_velocity = wanted_velocity_from_user_m_s;
    
    wanted_rotation_from_user = filtered_stick_rotation;
    if (abs(wanted_rotation_from_user) < SPEED_TO_STAND_MS)
      {
        if (abs(robot_orientation.yaw - started_rotation_angle) > 270) dizzy = true;
        started_rotation_angle = robot_orientation.yaw;
      }

    prev_SW1 = SW1; 
    SW1temp  = (1-digitalRead (PIN_SW1));
    filter_switch_transients (SW1temp, SW1 , 0);
    if (!SW1 && prev_SW1) SW1_pressed = true; else SW1_pressed = false;

    prev_SW2 = SW2; 
    SW2temp = (1-digitalRead (PIN_SW2));
    filter_switch_transients (SW2temp, SW2 , 1);
    if (!SW2 && prev_SW2) SW2_pressed = true; else SW2_pressed = false;

    prev_motion_enable = motion_enable;
    if (SW1_pressed) toggle (motion_enable);
    if (!RemoteXY.button_1 && prev_button_1) toggle(motion_enable);
    prev_button_1 = RemoteXY.button_1;
    if (motion_enable != prev_motion_enable) reset_before_moving();
}


void  reset_before_moving()
{   
    gyro_Yaw_angle = 0;
    roboclaw.SetEncM1(ROBOCLAW_ADDRESS,0);
    roboclaw.SetEncM2(ROBOCLAW_ADDRESS,0);
    wanted_velocity_from_user_m_s = 0;
    wanted_rotation_from_user = 0;
    filtered_stick_velocity = 0;
    filtered_stick_rotation = 0;
}

void filter_switch_transients (bool SW_status, bool &SWX , uint8_t switch_num)
{ 
    static bool prev_SW_status[2];
    static uint8_t counter[2];

    counter[switch_num] +=1;   
    if (counter[switch_num] > 10) 
       {
         SWX=SW_status; 
         counter[switch_num] = 11; 
       }
    if (SW_status != prev_SW_status[switch_num]) counter[switch_num] = 0;
    prev_SW_status[switch_num] = SW_status;
}

void toggle_mainLED (uint32_t delay_time)
{
    if (millis()- lastUSRblink > delay_time)
      {
        toggle (main_LED);
        lastUSRblink=millis();
        digitalWrite (PIN_LED1, main_LED); 
      }
}

void  control_LEDs ()
{   
    if (main_LED) toggle_mainLED (10  + 380*motion_enable);
    else          toggle_mainLED (390 - 370*motion_enable);
}


void get_pitch_and_vel () 
{ 
    robot_orientation = get_orientation();
    pitch_vel_rad_sec = robot_orientation.pitch_vel / RAD_TO_DEGs;
    pitch_rad = (robot_orientation.pitch) / RAD_TO_DEGs;
    if (abs(pitch_rad) < MAX_PITCH_FOR_MOTION) 
      {
        pitch_out_of_range = false; 
      }
    else 
      {
        pitch_out_of_range = true;
        motion_enable = false;
      }
}


void calc_pos_vel_errors()
{
    prev_pos_error = pos_error;
    pos_error = wanted_position - robot_pos_m;
    constrainF (pos_error, -POS_ERROR_LIMIT, POS_ERROR_LIMIT);
    if (want_to_stand && !reset_encoders_when_stand)
      {    
        pos_error_integral += pos_error * deltaT;
      }     // accumulte position integral only when standing and after encoders were reset
    constrainF (pos_error_integral,-POS_INTEGRAL_LIMIT,POS_INTEGRAL_LIMIT); 
  
    vel_error = wanted_velocity_from_user_m_s - robot_vel_m_sec;
    alpha_beta (averaged_vel_error , vel_error , ALPHA_AVG_VEL_ERR);
    vel_error_integral += vel_error * deltaT;
    constrainF (vel_error_integral , -VEL_INTEGRAL_LIMIT , VEL_INTEGRAL_LIMIT); 
    
    if (!motion_enable)
      {
        pos_error_integral = 0;
        vel_error_integral = 0;
      }
}


void read_robot_vel_pos()
{
    bool    valid1  , valid2  , valid3  , valid4  ;
    uint8_t status1 , status2 , status3 , status4 ;
    int32_t enc1    , enc2;
    int32_t speed1  , speed2;
    
    enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
    enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
    speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status3, &valid3);
    speed2 = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status4, &valid4);
    robot_vel_m_sec = float (speed2 + speed1) / 2 / CLICKS_IN_METER;
    robot_pos_m     = float (enc2   + enc1  ) / 2 / CLICKS_IN_METER;
    alpha_beta (average_robot_vel_m_s, robot_vel_m_sec, ALPHA_AVG_VELOCITY);
}


void send_motors_commands ()
{
    float velocity_Motor_1;
    float velocity_Motor_2;
    if (motion_enable) 
      {
        velocity_Motor_1 = (wanted_velocity_from_alg_m_s - wanted_rotation_from_user) * CLICKS_IN_METER;
        velocity_Motor_2 = (wanted_velocity_from_alg_m_s + wanted_rotation_from_user) * CLICKS_IN_METER;
      }
    else
      {
        velocity_Motor_1 = 0;
        velocity_Motor_2 = 0;
      }
    roboclaw.SpeedM1(ROBOCLAW_ADDRESS, velocity_Motor_1);
    roboclaw.SpeedM2(ROBOCLAW_ADDRESS, velocity_Motor_2); 
}


orientation get_orientation() 
{
    static float AccX, AccY, AccZ;
    static float gyro_vel_m_sec;
    static float prev_gyro_vel_m_sec;
    static float prev_gyro_Pitch_vel;
    static float prev_gyro_Yaw_vel;
    static float elapsedTime, previousTime;
    static uint16_t acc_fail_counter;
    float gyro_acc_X;
    float gyro_acc_Z;
    
    request_from_MPU (MPU6050_ACCEL_DATA, 6);
    AccX = (I2c.receive() << 8 | I2c.receive()) / 16384.0;        //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccY = (I2c.receive() << 8 | I2c.receive()) / 16384.0; 
    AccZ = (I2c.receive() << 8 | I2c.receive()) / 16384.0;

    prev_gyro_vel_m_sec = gyro_vel_m_sec;
    prev_gyro_Pitch_vel = gyro_Pitch_vel;
    prev_gyro_Yaw_vel = gyro_Yaw_vel;
    
    gyro_vel_m_sec = robot_vel_m_sec + GYRO_HEIGT_M * gyro_Pitch_vel / RAD_TO_DEGs;
    gyro_acc_m_ss  = (gyro_vel_m_sec - prev_gyro_vel_m_sec) / deltaT;

    gyro_acc_X = gyro_acc_m_ss / EARTH_GRAVITATION * cos(pitch_rad);
    gyro_acc_Z = gyro_acc_m_ss / EARTH_GRAVITATION * sin(pitch_rad);

    accAngleY = (atan( (gyro_acc_X - AccX) / sqrt(pow(AccY, 2) + pow((AccZ/1.05+gyro_acc_Z), 2))) * RAD_TO_DEGs) + imu_pitch_offset;        // accel error y

    elapsedTime  = (millis() - previousTime) / 1000;          // Divide by 1000 to get seconds
    previousTime =  millis();                                 // Current time actual time read

    request_from_MPU (MPU6050_GYRO_DATA+2, 4);
    gyro_Pitch_vel = (I2c.receive() << 8 | I2c.receive()) / 131.0 - gyro_Pitch_bias;
    gyro_Yaw_vel   = (I2c.receive() << 8 | I2c.receive()) / 131.0 - gyro_Yaw_bias;

    if (abs(gyro_Pitch_vel - prev_gyro_Pitch_vel) > MAX_JUMP_IN_PITCH) gyro_Pitch_vel = prev_gyro_Pitch_vel;
    if (abs(gyro_Yaw_vel   - prev_gyro_Yaw_vel  ) > 200) gyro_Yaw_vel   = prev_gyro_Yaw_vel  ;

    gyro_Pitch_vel = gyro_Pitch_vel + MIX_YAW_PITCH * gyro_Yaw_vel;
    
    gyro_Pitch_angle += gyro_Pitch_vel * elapsedTime;
    gyro_Yaw_angle   += gyro_Yaw_vel   * elapsedTime;

    if (first_run) gyro_Pitch_angle = accAngleY;
    if (abs(accAngleY - gyro_Pitch_angle) < VALID_ACC_ANGLE_DIFF) 
      {
        alpha_beta (gyro_Pitch_angle, accAngleY, ALPHA_GYRO_ACCEL);
        if (acc_fail_counter > 0 && abs(accAngleY - gyro_Pitch_angle) < VALID_ACC_ANGLE_DIFF / 4) 
          {
            acc_fail_counter = 0;
            acc_fail_events_counter ++;
          }
      }
    else
      {
        if (acc_fail_counter > 10) alpha_beta (gyro_Pitch_angle, accAngleY, ALPHA_GYRO_ACCEL);
        if (acc_fail_counter < 30) acc_fail_counter++;
      }
   
    return {
      gyro_Yaw_angle,
      gyro_Pitch_angle,
      gyro_Yaw_vel,
      gyro_Pitch_vel,
    };
}


void request_from_MPU (int register_start_address, int number_of_bytes)
{   
   I2c.read(MPU_ADDRESS,register_start_address,number_of_bytes);
}


void calibrate_gyro_setup() 
{
    int16_t c = 0; 
    int16_t num_reps = 500;
    gyro_Yaw_bias = 0;
    gyro_Pitch_bias = 0;
    while (c < num_reps) 
      {
        request_from_MPU (MPU6050_GYRO_DATA+2, 4);
        gyro_Pitch_vel = I2c.receive() << 8 | I2c.receive();
        gyro_Yaw_vel   = I2c.receive() << 8 | I2c.receive();
        gyro_Pitch_bias = gyro_Pitch_bias + (gyro_Pitch_vel / 131.0);
        gyro_Yaw_bias   = gyro_Yaw_bias   + (gyro_Yaw_vel   / 131.0);
        c++;
        delay (15);
      }
    gyro_Pitch_bias = gyro_Pitch_bias / num_reps;
    gyro_Yaw_bias   = gyro_Yaw_bias   / num_reps;
    EEPROM.put(10, gyro_Pitch_bias);   delay(20);
    EEPROM.put(16, gyro_Yaw_bias);     delay(20);
}

void calibrate_gyro_online() 
{
    static int32_t last_call_time; 
    static int16_t c; 
    if (millis() - last_call_time > 100)  // first call
      {
        c = 0;    
        gyro_Yaw_bias_temp = 0;
        gyro_Pitch_bias_temp = 0;
        imu_pitch_offset_temp = 0;
      }
    if(GUI_selector ==5)    // still calibrating
      {
        gyro_Yaw_bias_temp    += (gyro_Yaw_vel + gyro_Yaw_bias   );
        gyro_Pitch_bias_temp  += (gyro_Pitch_vel + gyro_Pitch_bias   );
        imu_pitch_offset_temp += (accAngleY - imu_pitch_offset);
        c++;
      }
    else if (c > 50)    // calib ended and had at least 50 cycles
      { 
        gyro_Yaw_bias = gyro_Yaw_bias_temp / c;
        EEPROM.put(16, gyro_Yaw_bias); 
        gyro_Pitch_bias = gyro_Pitch_bias_temp / c;
        EEPROM.put(10, gyro_Pitch_bias); 
        gyro_Yaw_angle = 0;

        imu_pitch_offset = -1* imu_pitch_offset_temp / c;
        EEPROM.put(22, imu_pitch_offset); 
        EEPROM.put(28, imu_pitch_offset); 
        c = 0;
      }
    last_call_time = millis();
}

void  toggle (bool &flag)
{
    flag = 1 - flag;
}


void constrainF (float &val, float minV, float maxV)
{
    if (val > maxV) val = maxV;
    if (val < minV) val = minV; 
}


void alpha_beta (float &average,float value,float ALPHA)
{
   average = ALPHA * average + (1-ALPHA) * value;
}


void  send_to_RaspberryPi ()
{
    static float average_yaw , avg_stick_x, eyes_pitch, eyes_yaw;
    alpha_beta (average_yaw , robot_orientation.yaw , ALPHA_YAW_AVERAGE);
    if (abs(RemoteXY.joystick_1_x) > abs(avg_stick_x)) alpha_beta (avg_stick_x, RemoteXY.joystick_1_x, ALPHA_YAW_AVERAGE);
    else avg_stick_x = RemoteXY.joystick_1_x;
    eyes_pitch = 1500*pitch_rad+128;
    eyes_yaw = (robot_orientation.yaw - average_yaw)*15 + (RemoteXY.joystick_1_x - avg_stick_x )*60 + 128;
    
    if (RaspberryPi_index == 0)
        Serial3.write(0xff);
        Serial3.write(0xff);
    if (RaspberryPi_index == 1)
        Serial3.write(byte(constrain (int (eyes_pitch),0,254) ));               // send pitch in 0.1 Deg 
        Serial3.write(byte(constrain (int (eyes_yaw  ),0,254) ));     // sends high byte of yaw in 0.1 deg
    if (RaspberryPi_index == 2)
        Serial3.write(byte(constrain (int (50*robot_vel_m_sec + 128),0,254) ));        // send vel in 2cm/sec
        Serial3.write(byte( dizzy));  
        if (dizzy) dizzy = false; 
    RaspberryPi_index += 1;
    if (RaspberryPi_index == 3) RaspberryPi_index = 0;
}


void send_telemetry()
{
  if (SEND_TELEMETRY  && millis()-last_sent>20)
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
//    Serial.print(" wa "); Serial.print(57295*wanted_pitch);
//    Serial.print(" ac "); Serial.print(57295*pitch_rad);
//    Serial.print(" avgdE "); Serial.print(1000*average_dError);
//    Serial.print(" posEI "); Serial.print(1000*pos_error_integral);
//    Serial.print(" velEI "); Serial.print(1000*vel_error_integral);
//    Serial.println("");
  }
}
