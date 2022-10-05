byte data[ 10 ] = { B00111111 ,B00000110 ,B01011011 ,B01001111 , B01100110 , B01101101 , B01111101 ,B00100111 , B01111111 ,B01101111 };
int button = 8;   
int i=0 ;                     

void setup( ) 
{              
      DDRD = B11111111;  
      pinMode( button, INPUT );                         //設定PORTD全部為輸出腳模式
}

void loop( ) 
{
    if( digitalRead( button ) == HIGH )    
    { 
       if(i<9)
       {
       i=i+1;
       }
       else if(i>8)
       {
       i=0;
    }
    PORTD = data[ i ];  
    delay(300);
}                        
     
}
