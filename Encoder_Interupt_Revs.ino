 #define encoder0PinA 2

#define encoder0PinB 4

volatile int encoder0Pos = 0;
int revolutions = 0;

void setup() {

  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  pinMode(3, OUTPUT);

  analogWrite(3, 500);
// encoder pin on interrupt 0 (pin 2)

  attachInterrupt(0, doEncoderA, CHANGE);

// encoder pin on interrupt 1 (pin 3)

  attachInterrupt(1, doEncoderB, CHANGE);  

  Serial.begin (9600);

}

void loop(){ //Do stuff here 
  Serial.print(revolutions, DEC);
  Serial.print(" ");
  Serial.println(encoder0Pos, DEC); 
}

void doEncoderA(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
      if(encoder0Pos/1865 != 0)  {
        revolutions += encoder0Pos / 1865;
        encoder0Pos = encoder0Pos % 1865;
      }
    } 
    else {
      if(encoder0Pos/-1865 != 0)  {
        revolutions -= encoder0Pos / -1865;
        encoder0Pos = encoder0Pos/-1865;
      }
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      if(encoder0Pos/1865 != 0)  {
        revolutions += encoder0Pos / 1865;
        encoder0Pos = encoder0Pos % 1865;
      }
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      if(encoder0Pos/-1865 != 0)  {
        revolutions -= encoder0Pos / -1865;
        encoder0Pos = encoder0Pos % -1865;
      }
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  // use for debugging - remember to comment out

}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   

   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      if(encoder0Pos/1865 != 0)  {
        revolutions += encoder0Pos / 1865;
        encoder0Pos = encoder0Pos % 1865;
      }  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      if(encoder0Pos/-1865 != 0)  {
        revolutions -= encoder0Pos / -1865;
        encoder0Pos = encoder0Pos % -1865;
      }
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      if(encoder0Pos/1865 != 0)  {
        revolutions += encoder0Pos / 1865;
        encoder0Pos = encoder0Pos % 1865;
      }      
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      if(encoder0Pos/-1865 != 0)  {
        revolutions -= encoder0Pos / -1865;
        encoder0Pos = encoder0Pos % -1865;
      }
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }

} 
