char incomingChar;//宣告一個變數來接收字元

void setup() {
  Serial.begin(38400);
  pinMode( 13, OUTPUT );  
}

void loop() {
  if (Serial.available())//不斷查詢(polling)看看Serial有沒有資料要進來
  {
    incomingChar = Serial.read();//從Ｓerial讀入一個字元，並指定給變數incomingChar
   if(incomingChar =='1' ){
    digitalWrite(13,HIGH);
  }
}
}
