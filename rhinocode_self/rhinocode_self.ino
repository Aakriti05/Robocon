int led= 13;

void setup()
{
  Serial.begin(9600);
  digitalWrite(led,1);//led on
  Serial.println("M255");
  delay(1000);
  Serial.println("D100");
  delay(1000);
  digitalWrite(led,0);//
}

void loop()
{
  //Serial.println("P");
  //delay(1000);
  Serial.println("M-100");
  delay(100);
  Serial.println("G-600");
  digitalWrite(led,1);
  delay(5000);
  
  Serial.println("M100");
  Serial.println("G0");
  digitalWrite(led,0);
  delay(5000);
}
