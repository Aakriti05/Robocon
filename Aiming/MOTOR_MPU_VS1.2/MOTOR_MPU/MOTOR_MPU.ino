#include<Wire.h>
#include <LiquidCrystal.h>
#define degconvert 57.2957786 

const int MPU_addr = 0x68;
double acc_x, acc_z, gyro_y; 
double kp = 70;
double ki = 0;
double kd = 0;
double errorsum     = 0;
double preverror    = 0;
int pole, ps3button = 3;
int count    = 0;
int motdir1  = 23;
int motdir2  = 22;
int pwm      = 10;
double angle,dt,past,error;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  Wire.begin();
  
  if (ARDUINO >= 157) Wire.setClock(400000UL); 
  else TWBR = ((F_CPU / 400000UL) - 16) / 2; 
    
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  acc_x=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  acc_z=Wire.read()<<8|Wire.read();  
  gyro_y=Wire.read()<<8|Wire.read();  
  angle = atan2(-acc_x, acc_z)*degconvert;
  
  past = millis();
  pinMode(motdir1,OUTPUT);
  pinMode(motdir2,OUTPUT);
  pinMode(pwm,OUTPUT);
  
}



void loop() {
  get_angle();
  lcd.setCursor(0, 0);
  count++;
  if (count == 40){
    lcd.print(angle);
    count = 0;
  }
}



void get_angle()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); 
  acc_x  = Wire.read()<<8|Wire.read();      
  acc_z  = Wire.read()<<8|Wire.read();  
  gyro_y = Wire.read()<<8|Wire.read();  
  
  double pitch   = atan2(-acc_x, acc_z)*degconvert;
  double gy_rate = gyro_y/131.0;
  angle   = 0.99 * (angle + gy_rate * dt) + 0.01 * pitch;
  
  errorsum       = errorsum + preverror;
  double error   = (-1)*pole_value() - angle ;
  double present = millis();
  dt     = (present - past)*0.001;
  Serial.println(error);
  error = kp*error + ki*(error+errorsum) + kd*(error - preverror)/dt;
  error = constrain(error,-255,255);
  motor_write(int(error));
  
  past      = present;
  preverror = error;
}



int pole_value(){
  //if (Serial.available())    ps3button = Serial.parseInt();

  if      (ps3button == 1)   pole = 5;
  else if (ps3button == 2)   pole = 10;
  else if (ps3button == 3)   pole = 15;
  else if (ps3button == 4)   pole = 20;
  else if (ps3button == 5)   pole = 25;
  return pole;
} 

  
void motor_write(int Pwm){
  if(pwm > 25)  {
    digitalWrite(motdir1,HIGH);
    digitalWrite(motdir2,LOW);
    analogWrite(pwm,Pwm);
  }
  else if(pwm < (-25)){
    digitalWrite(motdir1,LOW);
    digitalWrite(motdir2,HIGH);
    analogWrite(pwm,(-Pwm));
  }
  else{
    digitalWrite(motdir1,HIGH);
    digitalWrite(motdir2,HIGH);
    analogWrite(pwm,0);
  }
}
