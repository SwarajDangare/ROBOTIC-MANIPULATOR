#define Left_EA 26
#define Left_EB 25 
#define Right_EA 33 
#define Right_EB 32 
#define Grip_EA 35
#define Grip_EB 34



#define Left_PWM 27
#define Left_DIR 14
#define Right_PWM 12
#define Right_DIR 13
#define Grip_PWM 16
#define Grip_DIR 17



int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(Left_EA,INPUT);
  pinMode(Left_EB,INPUT);
  pinMode(Right_EA,INPUT);
  pinMode(Right_EB,INPUT);
  pinMode(Grip_EA,INPUT);
  pinMode(Grip_EB,INPUT);
  
  pinMode(Left_PWM,OUTPUT);
  pinMode(Left_DIR,OUTPUT);
  pinMode(Right_PWM,OUTPUT);
  pinMode(Right_DIR,OUTPUT);
  pinMode(Grip_PWM,OUTPUT);
  pinMode(Grip_DIR,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(Left_EA),readEncoderL,RISING);
  attachInterrupt(digitalPinToInterrupt(Right_EA),readEncoderR,RISING);
  attachInterrupt(digitalPinToInterrupt(Grip_EA),readEncoderG,RISING);
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target = 1200;
  //target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = pos-target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(int ENCB){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}
