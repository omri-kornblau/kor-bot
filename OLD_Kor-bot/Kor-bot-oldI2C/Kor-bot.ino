#include <Wire.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <RoboClaw.h>

#define SEND_TELEMETRY    false
#define CALIBRATE_GYRO    false

// IMU & Gyro related definitions //
#define ALPHA_GYRO_ACCEL      0.99      // Takes most of the pitch data from gyro .. very little noise to acc
#define MIX_YAW_PITCH        -0.02      // in-orthogonality of the gyro axes
#define MPU_ADDRESS           0x68      // MPU6050 I2C address
#define MPU6050_ACCEL_DATA    0x3B      // R
#define MPU6050_GYRO_DATA     0x43      // R 
#define MPU6050_PWR_MGMT_1    0x6B      // R/W
#define EARTH_GRAVITATION     9.8       // M/S2
#define RAD_TO_DEGs           57.295
#define GYRO_HEIGTH_M         0.18      // m
#define IMU_PITCH_OFFSET      0.62      //  deg   lower value = move forward / add -1*offset displayed in phone

// PARAMETERS OF ACCELERATION CALCULATION EXTENDED PID FILTER
#define KDD_ANGLE_TO_ACC      0.6       //  0.6
#define KD_ANGLE_TO_ACC       8         //  8
#define KP_ANGLE_TO_ACC       15        //  15
#define KI_ANGLE_TO_ACC       3         //  3
#define KI_ANGLE_TO_ACC_RES   10        //  10    integral that resets when error change dir
#define KD_AVGdERROR_TO_ACC   2         //  2     slows the robot when closing the error fast - like KD , but on average D to eliminate noise 
#define INTEGRAL_LIMIT        0.035

// PARAMETERS OF WANTED ANGLE CALCULATION  PID FILTER
#define KP_POS_TO_ANGLE       0.1       //  0.1   to keep 0
#define KI_POS_TO_ANGLE       0.01      //  0.01  to keep 0
#define POS_ERROR_LIMIT       0.2       //  max position error permitted for calculations 
#define POS_INTEGRAL_LIMIT    0.4       //  0.4
#define KP_VEL_TO_ANGLE       0.1       //  0.1
#define KP_AVG_VEL_TO_ANGLE   0.07      //  0.07     
#define KI_VEL_TO_ANGLE       0.01      //  0.01
#define VEL_INTEGRAL_LIMIT    0.4       //  0.4

#define ALPHA_AVG_VEL         0.9       //  0.9 avergaring factor for the averaged velocity 
#define ALPHA_AVG_dERROR      0.9       //  0.9 avergaring factor for the averaged derivative of the error 
#define ALPHA_STICK           0.9       //  0.9 avergaring factor for the averaged user stick commands
#define ALPHA_AVG_VEL_ERR     0.9       //  0.9 avergaring factor for the averaged velocity error
#define ALPHA_YAW_AVERAGE     0.9       //  0.9 

#define JOYSTICK_DEAD_BAND    0.1       //
#define SPEED_TO_STAND_MS     0.02      // 0.2 m/s

#define ROBOCLAW_ADDRESS      0x80      //  adress of the roboclaw 128
#define CLICK_IN_METER        47609     //  encoder clics in one meter
#define MAX_VELOCITY_ALG_MS   4         //  max velocity that the algorithms will send to the motors
#define MAX_VELOCITY_USR_MS   2.5       //  max velocity that the user can request
#define MAX_ACCELERATION_MSS  2.5       //  max acceleration that the user can request 
#define MAX_JERK_MSSS         10
#define MAX_PITCH_FOR_MOTION  0.4       //  disable motion above this angle

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
  { 255,3,0,9,0,36,0,10,24,1,
  5,32,4,41,54,54,190,26,31,68,
  2,1,1,61,27,186,50,137,1,0,
  51,87,12,12,191,190,0,65,2,2,
  35,12,12 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  uint8_t button_1; // =1 if button pressed, else =0 

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

uint8_t status1 , status2 , status3 , status4 ;
bool    valid1  , valid2  , valid3  , valid4  ;
int32_t enc1    , enc2;
int32_t speed1  , speed2;

float AccX, AccY, AccZ;
float accAngleY;
float gyro_Pitch_vel,   gyro_Yaw_vel;
float gyro_Pitch_angle, gyro_Yaw_angle;
float gyro_Pitch_bias,  gyro_Yaw_bias;
float elapsedTime, previousTime, time_of_MPU_request;
float wanted_angle = 0, prev_wanted_angle;
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
float velocity_Motor_1, velocity_Motor_2;
float started_rotation_angle;
float wanted_position = 0;
float wanted_velocity_from_user_m_s = 0;
float wanted_rotation_from_user = 0;
float filtered_stick_velocity = 0;
float filtered_stick_rotation = 0;
float max_allowed_acceleration = 0;

bool motion_enable = false;
bool reset_encoders_when_stand = false;
bool want_to_stand = true;
bool prev_want_to_stand = true;
bool SW1 , prev_SW1 , SW1_pressed , SW1temp;
bool SW2 , prev_SW2 , SW2_pressed , SW2temp;
bool prev_button_1, prev_motion_enable;
bool main_LED = true;
bool pitch_out_of_range = false; 
bool other_cycle = false; 
bool first_run = true;
bool dizzy = false;

uint8_t RaspberryPi_index;

void setup() 
{
    pinMode (PIN_SW1,INPUT_PULLUP);
    pinMode (PIN_SW2,INPUT_PULLUP);
    pinMode (PIN_LED1,OUTPUT);
    pinMode (PIN_LED2,OUTPUT);

    RemoteXY_Init (); 
    Wire.begin();
    
    if (SEND_TELEMETRY) Serial.begin(115200);    // Communication via USB to PC
                                            // Serial1 is connected to the Roboclaw  driver  and  is configured there
                                            // Serial2 is connected to the Bluetooth RemoteXY and is configured there
    Serial3.begin (115200);                 // Communincatioon with the Raspberry pi  
    roboclaw.begin(115200);
    reset_gyro ();

    if (CALIBRATE_GYRO) calibrate_gyro_vel();
    else 
      { 
        EEPROM.get(10, gyro_Pitch_bias);   delay(20);
        EEPROM.get(16, gyro_Yaw_bias);     delay(20);
      }
    digitalWrite (PIN_LED2, 0);
}


void loop() 
{ 
    // Serial.println (millis()-last_cycle);
    while (millis()-last_cycle<10) {}  
    last_cycle = millis ();
    deal_with_standing  ();
       digitalWrite (PIN_LED2, 1);
    get_pitch_and_vel   ();
       digitalWrite (PIN_LED2, 0);
    read_robot_vel_pos  ();
    calc_pos_vel_errors ();
    calc_PID_errors     ();
    calc_wanted_velocity();
    send_motors_commands(wanted_velocity_from_alg_m_s, wanted_rotation_from_user);
    if (other_cycle) 
      {    
        send_to_remoteXY    ();
        RemoteXY_Handler    ();      // read from bluetooth 
      }
    else
      { 
        control_LEDs        ();
        send_to_RaspberryPi ();
        send_telemetry      ();
      }
    read_user_commands  ();
    
    first_run = false;
    toggle (other_cycle);
}


void  reset_gyro ()
{
    Wire.beginTransmission(MPU_ADDRESS);    // Start communication with MPU
    Wire.write(MPU6050_PWR_MGMT_1);         // Talk to the register 6B
    Wire.write(0x00);                       // reset - place a 0 into the 6B register
    Wire.endTransmission(true);             // End the transmission
}


void  send_to_remoteXY()
{
    RemoteXY.onlineGraph_1_var1 = wanted_angle  * RAD_TO_DEGs;
    RemoteXY.onlineGraph_1_var2 = pitch_rad     * RAD_TO_DEGs;
    if (motion_enable) RemoteXY.led_1_g = 255; else RemoteXY.led_1_g = 0;
}

void  calc_PID_errors ()
{
    deltaT = float ((millis() - last_time))/1000;
    last_time = millis();

    prev_error  = error;
    prev_dError = dError;
    prev_wanted_angle = wanted_angle;  

    wanted_angle = KP_VEL_TO_ANGLE * vel_error +  KI_VEL_TO_ANGLE * vel_error_integral + KP_AVG_VEL_TO_ANGLE * averaged_vel_error;
    if (want_to_stand && !reset_encoders_when_stand) // keep pos only after want to stand and encoders were reset
      {
        wanted_angle  += KP_POS_TO_ANGLE * pos_error +  KI_POS_TO_ANGLE * pos_error_integral;
      }

    error = pitch_rad - wanted_angle;

    integral += error * deltaT;
    constrainF (integral, -INTEGRAL_LIMIT , INTEGRAL_LIMIT); 
    if (!motion_enable) integral = 0;

    integral_res += error * deltaT;
    constrainF (integral_res, -INTEGRAL_LIMIT , INTEGRAL_LIMIT); 
    if (prev_error * error<0 || !motion_enable) integral_res = 0;

    dError = pitch_vel_rad_sec - (wanted_angle - prev_wanted_angle) / deltaT;   // derivative of the error can be assumed to be the gyro rate
    alpha_beta (average_dError, dError, ALPHA_AVG_dERROR);

    ddError = (dError - prev_dError) / deltaT;  
}


void  calc_wanted_velocity ()
{
    base_acceleration_to_cancel_moment = EARTH_GRAVITATION * tan(pitch_rad);
    wanted_acc_m_ss  = base_acceleration_to_cancel_moment   + KP_ANGLE_TO_ACC  * error   + KI_ANGLE_TO_ACC    * integral;
    wanted_acc_m_ss += KD_ANGLE_TO_ACC     * dError         + KDD_ANGLE_TO_ACC * ddError + KI_ANGLE_TO_ACC_RES* integral_res;
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


void  read_user_commands () 
{   float stick_velocity_deadbanded;
    stick_velocity_deadbanded = 0;
    if (RemoteXY.joystick_1_y >  JOYSTICK_DEAD_BAND) stick_velocity_deadbanded = RemoteXY.joystick_1_y - JOYSTICK_DEAD_BAND;
    if (RemoteXY.joystick_1_y < -JOYSTICK_DEAD_BAND) stick_velocity_deadbanded = RemoteXY.joystick_1_y + JOYSTICK_DEAD_BAND;

    alpha_beta (filtered_stick_velocity, float(-stick_velocity_deadbanded) * MAX_VELOCITY_USR_MS /100 , ALPHA_STICK);
    alpha_beta (filtered_stick_rotation, float( RemoteXY.joystick_1_x)/100, ALPHA_STICK);
    if (filtered_stick_velocity > wanted_velocity_from_user_m_s)
      {
        max_allowed_acceleration = max_allowed_acceleration + MAX_JERK_MSSS * deltaT;
        if (max_allowed_acceleration > MAX_ACCELERATION_MSS) max_allowed_acceleration = MAX_ACCELERATION_MSS;
        wanted_velocity_from_user_m_s = min(wanted_velocity_from_user_m_s + max_allowed_acceleration * deltaT , filtered_stick_velocity);
      }
    if (filtered_stick_velocity < wanted_velocity_from_user_m_s)
      {
        max_allowed_acceleration = max_allowed_acceleration - MAX_JERK_MSSS * deltaT;
        if (max_allowed_acceleration < MAX_ACCELERATION_MSS) max_allowed_acceleration = MAX_ACCELERATION_MSS;
        wanted_velocity_from_user_m_s = max(wanted_velocity_from_user_m_s - max_allowed_acceleration * deltaT , filtered_stick_velocity);
      }
    wanted_rotation_from_user = 0;
    if (filtered_stick_rotation >  JOYSTICK_DEAD_BAND) wanted_rotation_from_user = filtered_stick_rotation - JOYSTICK_DEAD_BAND;
    if (filtered_stick_rotation < -JOYSTICK_DEAD_BAND) wanted_rotation_from_user = filtered_stick_rotation + JOYSTICK_DEAD_BAND;
    
    if (abs(wanted_rotation_from_user) < SPEED_TO_STAND_MS)
      {
        if (abs(robot_orientation.yaw - started_rotation_angle) > 270) dizzy = true;
        started_rotation_angle = robot_orientation.yaw;
      }

    prev_SW1 = SW1; 
    SW1temp  = (1-digitalRead (PIN_SW1));
    filter_switch_transients (SW1temp, SW1 , 0);
    if (!SW1 && prev_SW1) SW1_pressed= true;  
    else SW1_pressed = false;

    prev_SW2 = SW2; 
    SW2temp = (1-digitalRead (PIN_SW2));
    filter_switch_transients (SW2temp, SW2 , 1);
    if (!SW2 && prev_SW2) SW2_pressed = true;  
    else SW2_pressed = false;

    prev_motion_enable = motion_enable;
    if (SW1_pressed) toggle (motion_enable);
    if (!RemoteXY.button_1 && prev_button_1) toggle(motion_enable);
    prev_button_1 = RemoteXY.button_1;
    if (motion_enable != prev_motion_enable) reset_before_moving();
}


void  reset_before_moving()
{
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


void  control_LEDs ()
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


void get_pitch_and_vel () 
{ 
    robot_orientation = get_orientation();
    pitch_vel_rad_sec = robot_orientation.pitch_vel / RAD_TO_DEGs;
    pitch_rad = (robot_orientation.pitch + IMU_PITCH_OFFSET) / RAD_TO_DEGs;
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
    enc1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status1, &valid1);
    enc2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
    speed1 = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status3, &valid3);
    speed2 = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status4, &valid4);
    robot_vel_m_sec = float (speed2 + speed1) / 2 / CLICK_IN_METER;
    robot_pos_m     = float (enc2   + enc1  ) / 2 / CLICK_IN_METER;
    alpha_beta (average_robot_vel_m_s, robot_vel_m_sec, ALPHA_AVG_VEL);
}


void send_motors_commands (float velocity ,float rotation )
{
    if (motion_enable) 
      {
        velocity_Motor_1 = (velocity - rotation) * CLICK_IN_METER;
        velocity_Motor_2 = (velocity + rotation) * CLICK_IN_METER;
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
    static uint8_t timed_out;
    static float gyro_vel_m_sec;
    float prev_gyro_vel_m_sec;
    request_from_MPU (MPU6050_ACCEL_DATA, 6);
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

    prev_gyro_vel_m_sec = gyro_vel_m_sec;
    gyro_vel_m_sec = robot_vel_m_sec + GYRO_HEIGTH_M * gyro_Pitch_vel / RAD_TO_DEGs;
    gyro_acc_m_ss  = (gyro_vel_m_sec - prev_gyro_vel_m_sec) / deltaT;

    float gyro_acc_X = gyro_acc_m_ss / EARTH_GRAVITATION * cos(pitch_rad);
    float gyro_acc_Z = gyro_acc_m_ss / EARTH_GRAVITATION * sin(pitch_rad);

    accAngleY = (atan( (gyro_acc_X - AccX) / sqrt(pow(AccY, 2) + pow((AccZ/1.05+gyro_acc_Z), 2))) * RAD_TO_DEGs) + 2.69;        // accel error y

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
    gyro_Pitch_vel = gyro_Pitch_vel - gyro_Pitch_bias; 
    gyro_Yaw_vel   = gyro_Yaw_vel   - gyro_Yaw_bias;
     
    gyro_Pitch_vel = gyro_Pitch_vel + MIX_YAW_PITCH * gyro_Yaw_vel;

    gyro_Pitch_angle = gyro_Pitch_angle + gyro_Pitch_vel  * elapsedTime;
    gyro_Yaw_angle   = gyro_Yaw_angle   + gyro_Yaw_vel    * elapsedTime;

    if (first_run) gyro_Pitch_angle = accAngleY;
    alpha_beta (gyro_Pitch_angle, accAngleY, ALPHA_GYRO_ACCEL);
   
    return {
      gyro_Yaw_angle,
      gyro_Pitch_angle,
      gyro_Yaw_vel,
      gyro_Pitch_vel,
    };
}


void request_from_MPU (int register_start_address, int number_of_bytes)
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(register_start_address);                         // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, number_of_bytes, true);               // Read 6 registers total, each axis value is stored in 2 registers
}


void calibrate_gyro_vel() 
{
    int32_t c = 0; 
    int32_t num_reps = 1000;
    while (c < num_reps) 
      {
        request_from_MPU (MPU6050_GYRO_DATA+2, 4);
        gyro_Pitch_vel = Wire.read() << 8 | Wire.read();
        gyro_Yaw_vel   = Wire.read() << 8 | Wire.read();
        gyro_Pitch_bias = gyro_Pitch_bias + (gyro_Pitch_vel / 131.0);
        gyro_Yaw_bias   = gyro_Yaw_bias   + (gyro_Yaw_vel   / 131.0);
        c++;
      }
    gyro_Pitch_bias = gyro_Pitch_bias / num_reps;
    gyro_Yaw_bias   = gyro_Yaw_bias   / num_reps;
    EEPROM.put(10, gyro_Pitch_bias);   delay(20);
    EEPROM.put(16, gyro_Yaw_bias);     delay(20);
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
    eyes_pitch = 573*pitch_rad+128;
    eyes_yaw = (robot_orientation.yaw - average_yaw)*6 + (RemoteXY.joystick_1_x - avg_stick_x )*10 + 128;
    
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
//    Serial.print(" wa "); Serial.print(57295*wanted_angle);
//    Serial.print(" ac "); Serial.print(57295*pitch_rad);
//    Serial.print(" avgdE "); Serial.print(1000*average_dError);
//    Serial.print(" posEI "); Serial.print(1000*pos_error_integral);
//    Serial.print(" velEI "); Serial.print(1000*vel_error_integral);
//    Serial.println("");
  }
}
