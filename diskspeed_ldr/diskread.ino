int analogpin = A0;
int value;
unsigned long time0;
unsigned long time1;
unsigned long t;
int s;


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  value=analogRead(analogpin);
  //Serial.print("Value=");
  Serial.println(value);
  if (value >800)
  {
    time0=millis();
    //t=time1-time0;
    //Serial.print("time=");
    //Serial.println(t);
  } 
  if (value < 200)
  {
    time1=millis();
    t=time1-time0;
    if (t>30)
    {
    Serial.print("time=");
    Serial.println(t);
    } 
    time0=time1;
    
  }
  if (t>30)
  {
    s= (24*1000)/t; //in meter per sec
    Serial.print("speed=");
    Serial.println(s);
  }
delay(10);
}
