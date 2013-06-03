#include <PinChangeInt.h>

//PIN's definition
#define encoder0PinA  2
#define encoder0PinB  3

#define encoder1PinA  10
#define encoder1PinB  12

volatile int encoder0Pos = 0;
volatile boolean PastA0 = 0;
volatile boolean PastB0 = 0;
int revolutions0 = 0;

volatile int encoder1Pos = 0;
volatile boolean PastA1 = 0;
volatile boolean PastB1 = 0;
int revolutions1 = 0;


int lastEncode0 = 1;
int lastRevs0 = 1;
int lastEncode1 = 1;
int lastRevs1 = 1;
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

  PCintPort::attachInterrupt(encoder1PinA, &doEncoderA1, RISING);

  PCintPort::attachInterrupt(encoder1PinB, &doEncoderB1, CHANGE);

  Serial.begin(9600);
}


void loop()
{  
 //your staff....ENJOY! :D
 if(revolutions0 != lastRevs0 || encoder0Pos != lastEncode0 || revolutions1 != lastRevs1 || encoder1Pos != lastEncode1 || timeout > 60000)  {
   timeout = 0;
   lastRevs0 = revolutions0;
   lastEncode0 = encoder0Pos;
   lastRevs1 = revolutions1;
   lastEncode1 = encoder1Pos;
   Serial.print(String(String(revolutions0, DEC) + String(",") + String(encoder0Pos, DEC) + String(";")));
   Serial.print(String(String(revolutions1, DEC) + String(",") + String(encoder1Pos, DEC) + String(";\n")));

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

void doEncoderA1()
{
     PastB1 ? encoder1Pos--:  encoder1Pos++;
     if (encoder1Pos > 464 || encoder1Pos < -464)  {
       revolutions1 += encoder1Pos / 464;
       encoder1Pos %= 464;
     }

     if (revolutions1 > 0 && encoder1Pos < 0 )  {
       revolutions0 --;
       encoder1Pos = 464 - encoder1Pos;
     }
     else if (revolutions1 < 0 && encoder1Pos > 0)  {
       revolutions1 ++;
       encoder1Pos = -464 - encoder1Pos;
     }
}

void doEncoderB1()
{
    PastB1 = !PastB1;
}
