int ledPin= 13;                                  
int number;

void setup() 
{
  pinMode(ledPin,OUTPUT); 
  Serial.begin(9600);
}

void loop() 
{
  Serial.println(analogRead(A0));
  number=analogRead(A0);
  
  if(number>400)
  {
  digitalWrite(ledPin,HIGH);
  }
  else if(number<=400)
  {
  digitalWrite(ledPin,LOW);
  }

}
