#include <analogWrite.h>

#define ENCA 26
#define ENCB 25
#define PWM 27
#define DIR 14

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int dir1 = 0;
int pwm1 = 0;

float error = 0;
float prev_error = 0;
float I = 0;
float D = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  Serial.println("target pos");
}

void loop()
{
  // set target position
  int target = 1200;
  //target = 250*sin(prevT/1e6);

  PID(target, pos, &pwm1, &dir);

  // signal the motor
  setMotor(dir1, pwm1, PWM, DIR);



  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void PID(int target, int pitch, int* pwm, int* dir)
{
  // DEFINE PID CONSTANTS
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;


  //    // SAMPLING TIME
  //    float dt = 0.002;

  // time difference
  long currT = micros();
  float dt = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;


  // DEFINE ERROR
  error = target - pitch;
  //Serial.println(error);
  // INTEGRAL ERROR
  I = I + (error * dt);

  // DERIAVTE ERROR
  D = (error - prev_error) / dt;

  // CONTROL SIGNAL -> PWM
  *pwm = abs(kp * error + ki * I + kd * D);

  //CAP PWM


  // HOLD PAYLOAD
  if (abs(error) < 2)
  {
    *pwm = 0;

  }

  // REVERSE DIRECTION OF MOTOR WHEN CURRENT POSITION > DESIRED POSITION
  if (error > 0)
  {
    *dir = LOW;
  }

  if (error < 0)
  {
    *dir = HIGH;
  }
  prev_error = error;
}

void setMotor(int dir, int pwmVal, int pwm, int DIR) {
  analogWrite(pwm, pwmVal);
  digitalWrite(DIR, dir);
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos++;
  }
  else {
    pos--;
  }
}
