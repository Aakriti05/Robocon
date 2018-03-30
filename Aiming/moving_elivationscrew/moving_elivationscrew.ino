int pin1 = 22;
int pin2 = 23;
int pin3 = 10;
//int now = 0;
//int current = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(pin3,OUTPUT);
}

void loop() 
{
  // put your main code here, to run repeatedly:
//  current=millis();
//  
//  while((now-current)<2000)
//  {
//    motor_write(-150);
//    now=millis();
//  }
//  
//  while(((now-current)<4000)&&((now-current)>=2000))
//  {
//    motor_write(150);
//    now=millis();
//  }
//for(int i=30;i<255;i=i+5)
//{
//motor_write(i);
//Serial.println(i);
//delay(500);
//
//}
//if(int i=255)
motor_write(255);
//delay(4000);
//motor_write(-255);
//delay(4000);
  
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

