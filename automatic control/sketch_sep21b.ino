byte data[ 10 ] = { B00111111 ,B00000110 ,B01011011 ,B01001111 , B01100110 , B01101101 , B01111101 ,B00100111 , B01111111 ,B01101111 };
int i=10;
void setup( )
{
  DDRD = B11111111;
  pinMode( 13, INPUT );                         
}

void loop( )
{
  for(i=10 ; i>=0 ; i-- )
      {
             PORTD = data[ i ]; 
             delay( 500 );
       } 
  if(i==0){
       pinMode(13, OUTPUT ); 
       i=10; 
       PORTD = data[ i ];                               
  }
}
