const int trig = 10;
const int echo = 11;
const int inter_time = 1000;
int time = 0;
int i;

byte data[ 10 ] = { B00111111 ,                       //利用陣列將七段顯示器顯示
                    B00000110 , B01011011 ,
                    B01001111 , B01100110 ,
                    B01101101 , B01111101 ,
                    B00100111 , B01111111 ,
                    B01101111
                  };

void setup() 
 {
  Serial.begin(9600);
  pinMode (trig, OUTPUT);
  pinMode (echo, INPUT);
  DDRD = B11111111;
}

void loop()
{
float duration, distance;
digitalWrite(trig, HIGH);
delayMicroseconds(1000);
digitalWrite(trig, LOW);
duration = pulseIn (echo, HIGH);
distance = (duration/2)/29;

if (distance < 5)
  {
    i = 0;
  }
  else if (distance <= 10 && distance >= 5)
  {
    i = 1;
  }
else if (distance <= 15 && distance >= 10)
{
  i = 2;
}
else if (distance <= 20 && distance >= 15)
{
  i = 3;
}
else if (distance <= 25 && distance >= 20)
{
  i = 4;
}
else
{
  i = 5;
}
PORTD = ~ data[ i ];
}
