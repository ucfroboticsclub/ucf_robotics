/*
Script to control relay connected to strobe.

States:

  Control Pin 11: HIGH    LOW
  Relay Pin 10  : SOLID   STROBE

*/


const int controlPin = 11;
const int relayPin = 10;

int controlState = 0;

void setup() 
{
 pinMode(relayPin, OUTPUT);
 pinMode(controlPin, INPUT_PULLUP);
}

void loop() 
{
  
  controlState = digitalRead(controlPin);
  if(controlState == LOW)
  {
    strobe();
  }
  
    digitalWrite(relayPin, HIGH);
    delay(300);
    
}



void strobe()
{
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(relayPin, LOW);
    delay(100);   
    digitalWrite(relayPin, HIGH);
    delay(100);
  }
}
