#include<Wire.h>
#include <LiquidCrystal.h>
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.
int min_angle;
int max_angle;

int pin1 = 23;
int pin2 = 22;
int pin3 = 10;
int now = 0;
int current = 0;
double dt;
int present,past;
double kp = 70;
double ki = 0;
double kd = 0;
int ps3button = 3;
int pole;
double errorsum = 0;
double preverror = 0;
double error = 0;
int count = 0;



// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  lcd.begin(16, 2);
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  delay(100);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  double pitch = atan2(-AcX, AcZ)*degconvert;
  double gyroYangle = pitch;
  compAngleY = pitch;

  
  //start a timer
  timer=micros();
  pinMode(23,OUTPUT);
  pinMode(22,OUTPUT);
  pinMode(13,OUTPUT);
  past = millis();
  
}

void loop() {

  count++;
  get_angle();
  lcd.setCursor(0, 0);
  if (count == 40)
  {
    lcd.print(compAngleY);
    count = 0;
  }
}


void get_angle()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  double pitch = atan2(-AcX, AcZ)*degconvert;
  double gyroYrate = GyY/131.0;
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch;
  
  float polevalue = -pole_value();
  error = polevalue - compAngleY ;
  Serial.println(error);
  errorsum = errorsum + preverror;
  present = millis();
  dt = (present - past)*0.001;
  error = ((kp*error) + (ki*(error+errorsum)) + ((kd*(error - preverror))/dt));
  error=constrain(error,-255,255);
  
  
  past = present;
  preverror = error;
  motor_write(int(error));
}

int pole_value()
{
  //if (Serial.available())
    //ps3button = Serial.parseInt();

  if (ps3button == 1)
    pole = 5;
  else if (ps3button == 2)
    pole = 10;
  else if (ps3button == 3)
    pole = 15;
  else if (ps3button == 4)
    pole = 20;
  else if (ps3button == 5)
    pole = 25;

    return pole;
}   

 void motor_write(int pwm)
{
  if(pwm > 25)
  {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);
    analogWrite(pin3,pwm);
  }
  else if(pwm < (-25))
  {
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,HIGH);
    analogWrite(pin3,(-pwm));
  }
  else
  {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,HIGH);
    analogWrite(pin3,0);
  }
}
