#include <math.h>

#define upButton 2
#define downButton 3
#define dir 4 
#define pwm 5


int upPressed = 1;
int downPressed = 1;
int pwmVal = 255;



void setup() 
{
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(dir, OUTPUT);
  pinMode(pwm, OUTPUT);

}

void loop() 
{
  upPressed = digitalRead(upButton);
  downPressed = digitalRead(downButton);

  if(upPressed == LOW)
  {
    digitalWrite(dir,HIGH);
    analogWrite(pwm,pwmVal);
  }
  else
  {
    digitalWrite(dir,LOW);
    analogWrite(pwm,pwmVal);
  }
  
}
