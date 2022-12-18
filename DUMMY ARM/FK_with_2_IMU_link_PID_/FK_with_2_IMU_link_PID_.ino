#include <Wire.h>

//---BNO055---
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLE_DELAY  100
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);

//---MPU6050---
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;

//---JOYSTICK--
const int joyLink1 = A0; 
const int joyLink2 = A1; 
const byte homeSW = 2;
const byte stow = 3;

//---ARDUINO_PIN_CONFIG---
#define dir_pin1 4 
#define pwm_pin1 5
#define dir_pin2 6 
#define pwm_pin2 7
int dir1 = 0;
int pwm1 = 0;
int dir2 = 0;
int pwm2 = 0;
int angLink1 = 0;
int angLink2 = 0;



// DEFINE TARGET ANGLE WRT EARTH  
// float target1 = 90 ;
// float target2 = 0 ;

// DEFINE TARGET JOINT ANGLES
float jointangle1 = 90;
float jointangle2 = 90;

volatile int target1 ; //jointangle1;
volatile int target2 ;  //jointangle1 - jointangle2;

void setup()
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

  //---BNO055---
  setBNO055(); 
  delay(500);

  //---MPU6050---
  setMPU(); 
  delay(100);
  
  target1 = yMPU();
  target2 = yBNO055();
 
  
  pinMode(dir_pin1, OUTPUT);
  pinMode(pwm_pin1, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  pinMode(pwm_pin2, OUTPUT);
  pinMode(homeSW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(homeSW), Home, LOW);
  pinMode(stow, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stow), Stow, LOW);

}

void loop()
{
  //---BNO055---
  float pitch2 = yBNO055();
  delay(10);
  
  //---MPU6059---
  float pitch1=yMPU();     
  delay(10);
  
//  angLink1 = analogRead(joyLink1);
//  angLink2 = analogRead(joyLink2);
//  int target1 = map(angLink1, 0, 1023,-45, 135);
//  int target2 = map(angLink2, 0, 1023, -105, 105);

  angLink1 = analogRead(joyLink1);
  angLink2 = analogRead(joyLink2);
  Serial.print(angLink1);
  Serial.print(" ");
  Serial.println(angLink2);
  angLink1 = map(angLink1+10, 100, 1023-150, -5, 5);
  angLink2 = map(angLink2+20, 100, 1023-150, -5, 5);
  Serial.print(angLink1);
  Serial.print(" ");
  Serial.println(angLink2);
  Serial.println();
  
  
//  angLink1 = map(angLink1, 0, (1022/2)-50,-3, 0);
//  angLink1 = map(angLink1, (1022/2)+50, 1023,0, 3);
//  angLink2 = map(angLink2, 0, (1022/2)-50,-3, 0);
//  angLink2 = map(angLink2, (1022/2)+50, 1023,0, 3);

  target1+=angLink1;
  
  target2+=angLink2;
  
  if(target1>=135)
  {
    target1=135;
  }
  if(target1<=-45)
  {
    target1=-45;
  }

  if(target2>=105)
  {
    target2=105;
  }
  if(target2<=-105)
  {
    target2=-105;
  }
  
  Serial.print("Link1 = ");
  Serial.print(target1);
  Serial.print("\t Link2 = ");
  Serial.println(target2);

  PID(target1,pitch1,&pwm1,&dir1);
  if(pwm1 > 125)
    {
     pwm1 = 125 ;
    }
  PID(target2,pitch2,&pwm2,&dir2);
  if(pwm2 > 125)
    {
     pwm2 = 125;
    }
    
    if(pwm1 < 40)
    {
     pwm1 = 0;
    }
    if(pwm2 < 25)
    {
     pwm2 = 0;
    }
  digitalWrite(dir_pin1, dir1);
//  if(pitch1==-134)
//  {
//    pwm1=0;
//  }
  analogWrite(pwm_pin1, pwm1);
  digitalWrite(dir_pin2, dir2);
  analogWrite(pwm_pin2, pwm2);
  
  //SerialDisplay(pitch1,pitch2,dir1,pwm1,dir2,pwm2);
  
    
}


void PID(int target, int pitch, int* pwm, int* dir)
{
    // DEFINE PID CONSTANTS
    float kp = 3;
    float ki = 0.0000;
    float kd = 0.0000;
    
    float error = 0;
    float prev_error = 0;
    float I = 0;
    float D = 0;
    
    // SAMPLING TIME
    float dt = 0.002;

//    unsigned int time = 0;
//  time = micros();
//   
//  int angle = 0;
//  while(angle < 180)
//    Serial.println(sin(angle++/57.295));
//
//   
//  time = micros() - time;
//   
//  Serial.println(time, DEC);
//  delay(1000);


    // DEFINE ERROR
    error = target - pitch; 
    //Serial.println(error);
    // INTEGRAL ERROR
    I = I + (error * dt);

    // DERIAVTE ERROR
    D = (error - prev_error) / dt;

    // CONTROL SIGNAL -> PWM
    *pwm = abs(kp*error + ki*I + kd*D);
    
    //CAP PWM
    
      
     // HOLD PAYLOAD
     if(abs(error)<3)
    {
      *pwm = 0;
      
    }
    
    // REVERSE DIRECTION OF MOTOR WHEN CURRENT POSITION > DESIRED POSITION
    if(error > 0)
    {
      *dir = LOW;
    }

    if(error < 0)
    {
      *dir = HIGH;
    }
  
}

void SerialDisplay(int pitch1,int pitch2,int dir1,int pwm1,int dir2,int pwm2)
{
  Serial.print("Pitch1= ");
  Serial.print(pitch1);

  Serial.print(" |\tPitch2= ");
  Serial.print(pitch2);
   
  Serial.print(" |\tDIR1 = ");
  Serial.print(dir1);
    
  Serial.print(" |\tPWM1 = ");
  Serial.print(pwm1);
  
  Serial.print(" |\tDIR2 = ");
  Serial.print(dir2);
    
  Serial.print(" |\tPWM2 = ");
  Serial.println(pwm2);
  Serial.println("");
}
void setMPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("MPU6050 detected!!");
}

void setBNO055()
{
  if (!bno.begin())
  {    
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
    Serial.println("BNO055 detected!!");
}

float yMPU()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
 
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);

  if(y>180)
  {
    y=y-360;
  }
  return y;
}

float yBNO055() 
{
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return(y);
}

void Home() {
  target1 = 135;
  target2 = 45;
}
void Stow() {
  target1 = 135;
  target2 = -45;
}
