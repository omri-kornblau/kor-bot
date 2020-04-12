#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define RADIANS_TO_DEGREES 57.29578

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

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

void loop() {
    while (!mpuInterrupt && fifoCount < packetSize) {}

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) mpu.resetFIFO();
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print(millis() - last_time);
      Serial.print(": ");
      Serial.print("DMP:");
      Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
      Serial.print(":");
      Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
      Serial.print(":");
      Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
      last_time = millis();
    }
    delay(10);
}
