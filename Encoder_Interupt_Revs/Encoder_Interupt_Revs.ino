//PIN's definition
#define encoder0PinA  2
#define encoder0PinB  3


volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
int revolutions = 0;

int lastEncode = 1;
int lastRevs = 1;
volatile unsigned int timeout = 0;

void setup() 
{

  pinMode(encoder0PinA, INPUT);
  //turn on pullup resistor
  //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder0PinB, INPUT); 
  //turn on pullup resistor
  //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B

//To speed up even more, you may define manually the ISRs
// encoder A channel on interrupt 0 (arduino's pin 2)
  attachInterrupt(0, doEncoderA, RISING);
// encoder B channel pin on interrupt 1 (arduino's pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 
  Serial.begin(9600);
}


void loop()
{  
 //your staff....ENJOY! :D
 if(revolutions != lastRevs || encoder0Pos != lastEncode || timeout > 60000)  {
   timeout = 0;
   lastRevs = revolutions;
   lastEncode= encoder0Pos;
   Serial.print(String(String(revolutions, DEC) + String(",") + String(encoder0Pos, DEC) + String(";\n")));
   //Serial.print(",");
   //Serial.print(encoder0Pos, DEC);
   //Serial.print(";\n");
 }
 timeout++;
 
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 
void doEncoderA()
{
     PastB ? encoder0Pos--:  encoder0Pos++;
     if (encoder0Pos > 464 || encoder0Pos < -464)  {
       revolutions += encoder0Pos / 464;
       encoder0Pos %= 464;
     }
     
     if (revolutions > 0 && encoder0Pos < 0 )  {
       revolutions --;
       encoder0Pos = 464 - encoder0Pos;
     }
     else if (revolutions < 0 && encoder0Pos > 0)  {
       revolutions ++;
       encoder0Pos = -464 - encoder0Pos;
     }
       
     
     
}

void doEncoderB()
{
     PastB = !PastB; 
}
