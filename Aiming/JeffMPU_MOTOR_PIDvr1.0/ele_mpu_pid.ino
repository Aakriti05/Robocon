#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int pin1 = 7;
int pin2 = 6;
int pin3 = 5;
int now = 0;
int current = 0;
int angle = 15;
int error = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    Serial.begin(115200);
    
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();

    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) 
    {
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print(F("DMP Initialization failed"));
    }
    pinMode(pin1,OUTPUT);
    pinMode(pin2,OUTPUT);
    pinMode(pin3,OUTPUT);
    current=millis();
}


void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
              
        motor_write(100);
        if (mpuInterrupt==true)
        {
         break;
        }
      
      // wait for MPU interrupt or extra packet(s) available
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    
    }
    else if (mpuIntStatus & 0x02) {
       
    while (fifoCount < packetSize)  fifoCount = mpu.getFIFOCount();
            
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
        
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    Serial.print("\n");
    Serial.print(ypr[1] * 180/M_PI);
    error = angle - ypr[1];
    
    if (error < 0) error = (-1)*error;
    
}

void motor_write(int PWM)
{
  if (PWM>0)
  {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);
    analogWrite(pin3,PWM);
  }
  else if (PWM<0)
  {
    digitalWrite(pin2,HIGH);
    digitalWrite(pin1,LOW);
    analogWrite(pin3,(-1)*PWM);
  }
  else
  {
    digitalWrite(pin2,HIGH);
    digitalWrite(pin1,HIGH);
    analogWrite(pin3,PWM);
  }
  
}

