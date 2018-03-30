int dirPin1 = 10;
int dirPin2 = 12;
int pwmPin = 11;
int error,preverror;
int errorsum = 0;
double dt;
int present,past;
double kp = 1;
double ki = 1;
double kd = 1;

void setup() {
  // put your setup code here, to run once:
  pinMode(dirPin1,OUTPUT);
  pinMode(dirPin2,OUTPUT);
  pinMode(pwmPin,OUTPUT);
  Serial.begin(9600);
  
  if (Serial.available())
      preverror = Serial.parseInt();
  else 
      preverror = 0;

  past = millis();    
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {
    error = Serial.parseInt();
    errorsum = errorsum + preverror;
    present = millis();
    dt = (present - past)*0.001;
    error = ((kp*error) + (ki*(error+errorsum)) + ((kd*(error - preverror))/dt));
    past = present;
    preverror = error;
    motor_write(error);
  }
  else
    motor_write(0);
}

void motor_write(int pwm)
{
  if(pwm > 30)
  {
    digitalWrite(dirPin1,HIGH);
    digitalWrite(dirPin2,LOW);
    digitalWrite(pwmPin,pwm);
  }
  else if(pwm < (-30))
  {
    digitalWrite(dirPin1,LOW);
    digitalWrite(dirPin2,HIGH);
    digitalWrite(pwmPin,(-pwm));
  }
  else
  {
    digitalWrite(dirPin1,HIGH);
    digitalWrite(dirPin2,HIGH);
    digitalWrite(pwmPin,0);
  }
}
