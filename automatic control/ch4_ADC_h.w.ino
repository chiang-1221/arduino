int number;  //宣告變數
int i;
byte data[ 10 ] = { B00111111 ,B00000110 ,
                    B01011011 ,B01001111 ,
                    B01100110 ,B01101101 ,
                    B01111101 ,B00100111 ,
                    B01111111 ,B01101111 };


void setup() {
  Serial.begin(9600); //設定鮑率
  DDRD = B11111111; 
}

void loop() {
  number = analogRead(A0);  
  if (number < 100)
  {
    i = 0;
  }
  else if (number <= 200 && number >= 100)
  {
    i = 1;
  }
  else if (number <= 300 && number >= 200)
  {
    i = 2;
  }
  else if (number <= 400 && number >= 300)
  {
    i = 3;
  }
  else if (number <= 500 && number >= 400)
  {
    i = 4;
  }
  else if (number <= 600 && number >= 500)
  {
    i = 5;
  }
  else if (number <= 700 && number >= 600)
  {
    i = 6;
  }
  else if (number <= 800 && number >= 700)
  {
    i = 7;
  }
  else if (number <= 900 && number >= 800)
  {
    i = 8;
  }
  else 
  {
    i = 9;
  }

PORTD = data[ i ];

}
