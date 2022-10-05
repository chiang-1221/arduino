#include <Servo.h>
Servo myservo; // 建立Servo物件，控制伺服馬達
void setup() {  myservo.attach(3);}
void loop()
{   for (int i = 0; i <= 180; i += 1)
  {
        myservo.write(i);
    delay(20);
     
  }  
  for (int i = 180; i >= 0; i -= 1) 
  {
        myservo.write(i);
  }
}
