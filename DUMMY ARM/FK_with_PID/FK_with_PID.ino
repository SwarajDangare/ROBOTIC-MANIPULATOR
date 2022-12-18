#include <math.h>

#define upButton 2
#define downButton 3
#define dir 4 
#define pwm 5

int setAngle = 80;            // target
int currentAngle = 0;           // initieal velocity
int error = setAngle - crrentAngle;
int oldError = error;
float dt = 0.1                  // for each instant

//a = 60                      // loop for a minute

int upPressed = 1;
int downPressed = 1;
int pwmVal = 255;


float kp = 0.05
float ki = 0.3
float kd = 0.01


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


    float P = kp * error
    float I = I + ki * error * dt            // integration i.e addition of all the error
    float D = kd * (error - oldError) / dt   // derivative i.e  

    float pwm = P + I + D

    oldError = error
    error = setAngle - currentAngle
  
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
