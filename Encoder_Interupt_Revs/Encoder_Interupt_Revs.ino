//PIN's definition
#define encoder0PinA  2
#define encoder0PinB  3


volatile int encoder0Pos = 0;
volatile boolean PastA0 = 0;
volatile boolean PastB0 = 0;
int revolutions0 = 0;


int lastEncode0 = 1;
int lastRevs0 = 1;
volatile unsigned int timeout = 0;

void setup() 
{

  pinMode(encoder0PinA, INPUT);
  //turn on pullup resistor
  //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder0PinB, INPUT); 
  //turn on pullup resistor
  //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  PastA0 = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB0 = (boolean)digitalRead(encoder0PinB); //and channel B

//To speed up even more, you may define manually the ISRs
// encoder A channel on interrupt 0 (arduino's pin 2)
  attachInterrupt(0, doEncoderA0, RISING);
// encoder B channel pin on interrupt 1 (arduino's pin 3)
  attachInterrupt(1, doEncoderB0, CHANGE); 
  Serial.begin(9600);
}


void loop()
{  
 //your staff....ENJOY! :D
 if(revolutions0 != lastRevs0 || encoder0Pos != lastEncode0 || timeout > 60000)  {
   timeout = 0;
   lastRevs0 = revolutions0;
   lastEncode0 = encoder0Pos;
   Serial.print(String(String(revolutions0, DEC) + String(",") + String(encoder0Pos, DEC) + String(";\n")));
   //Serial.print(",");
   //Serial.print(encoder0Pos, DEC);
   //Serial.print(";\n");
 }
 timeout++;
 
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 
void doEncoderA0()
{
     PastB0 ? encoder0Pos--:  encoder0Pos++;
     if (encoder0Pos > 464 || encoder0Pos < -464)  {
       revolutions0 += encoder0Pos / 464;
       encoder0Pos %= 464;
     }
     
     if (revolutions0 > 0 && encoder0Pos < 0 )  {
       revolutions0 --;
       encoder0Pos = 464 - encoder0Pos;
     }
     else if (revolutions0 < 0 && encoder0Pos > 0)  {
       revolutions0 ++;
       encoder0Pos = -464 - encoder0Pos;
     }
       
     
     
}

void doEncoderB0()
{
     PastB0 = !PastB0; 
}
