#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "RoboClaw.h"

#define address 0x80

#define RADIANS_TO_DEGREES 57.29578

MPU6050 mpu;
RoboClaw roboclaw(&Serial1,10000);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    Serial.begin(115200);
    roboclaw.begin(57600);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();
        
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
    }
    while (!dmpReady) {}
    packetSize = mpu.dmpGetFIFOPacketSize();
}

unsigned long last_time;

#define pred 0.06
#define acc 200000
#define nominal 1000
#define vel 0
#define bias 0.012217

float wanted_angle = 0;
float pitch_vel = 0;
float actual_angle = 0;
float ypr1 = 0;
float wanted_velocity = 0;
int cycle_time = 0;

void loop() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    while (!mpuInterrupt && fifoCount < packetSize) {
      
    }
    
    uint8_t status1,status2,status3,status4;
    bool valid1,valid2,valid3,valid4;
    
    int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
    int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
    int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
    int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) mpu.resetFIFO();
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
      cycle_time = millis() - last_time;
      last_time = millis();
            
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pitch_vel = float(gy) / 1024;
      actual_angle = ypr[1] + bias - pitch_vel*cycle_time*pred;
      wanted_angle = 0.95*wanted_angle + (1-0.95)*vel*(float(-1 * speed1 + speed2)) / 100000;
      Serial.print("angleL: "); Serial.println(ypr[1]);
      
      ypr1 = -1*(tan(actual_angle - wanted_angle))*acc;
      wanted_velocity += ypr1 * float(cycle_time) / 1000;
            
      if (wanted_velocity < 0) wanted_velocity -= nominal;
      if (wanted_velocity > 0) wanted_velocity += nominal;
      
      if (wanted_velocity < -30000) wanted_velocity = -30000;
      if (wanted_velocity > 30000) wanted_velocity = 30000;
      
      roboclaw.SpeedM1(address, wanted_velocity);
      roboclaw.SpeedM2(address, wanted_velocity);    
    }
}
