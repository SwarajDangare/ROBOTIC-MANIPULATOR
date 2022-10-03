#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <vector.h>

#define pi 3.141527

//---BNO055---
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLE_DELAY  100
Adafruit_BNO055 bno2 = Adafruit_BNO055(55,0x28);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55,0x29);

//---INTERPOLATION---
 std::vector<double> linspace(double a, double b, std::size_t N)
    {
        double h = (b - a) / static_cast<double>(N-1);
        std::vector<double> xs(N);
        std::vector<double>::iterator x;
        double val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
            *x = val;
        }
        return xs;
    }
int k = 0;

//---JOYSTICK--
const int joyLink1 = A0; 
const int joyLink2 = A1; 
const byte homeSW = 2;
const byte stow = 3;
float moveX = 0;
float moveY = 0;

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

boolean loop1=true;

//---LENGTHS OF LINK---
float L1 = 44.5;
float L2 = 38.5;

//---DEFINE TARGET JOINT ANGLES---
float pitch1 = 0;
float pitch2 = 0;
float jointangle1 = 0;
float jointangle2 = 0;
float angle2 =0;
volatile float target1 ; //jointangle1;
volatile float target2 ;  //jointangle1 - jointangle2;
float b = 0;
float a = 0;

//---CORDINATES OF LINKS---
float x1;
float y1;
double x2;
double y2;
double X2;
double Y2;

//---DEFINE PID CONSTANTS---
float kp1 = 5.5;
float ki1 = 0.003;
float kd1 = 0.0001;
    
float error1 = 0;
float prev_error1 = 0;
float I1 = 0;
float D1 = 0;
float dt1 = 0.001;

float kp2 = 6;
float ki2 = 0.003;
float kd2 = 0.0001;
    
float error2 = 0;
float prev_error2 = 0;
float I2 = 0;
float D2 = 0;
float dt2 = 0.002;

// Circle
double *Theta_Traj = linspace(0, 2*pi, 101);

double Final_x[101];
double Final_y[101];


//==============================================================================================>

void setup()
{

 

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

   for(int j = 0; j < 101; j++)
  {
    Final_x[j] = ((L1+L2)/3)*cos(Theta_Traj[j]);
    Final_y[j] = ((L1+L2)/3)*sin(Theta_Traj[j]);
    Serial.println(Theta_Traj[j]);
  }

  //---BNO055-1---
  setBNO0551(); 
  delay(500);

  //---BNO055-2---
  setBNO0552(); 
  delay(500);
  

  //---BNO055-1---
  jointangle1 = radians(yBNO0551());
  delay(10);
  pitch1 = yBNO0551();
  delay(10);
  //---BNO055-2---
  angle2 = radians(yBNO0552());
  delay(10);
  pitch2 = yBNO0552();
  delay(10);
  jointangle2= jointangle1-angle2;

  getCorrdinates(jointangle1,jointangle2,L1,L2,&x1,&y1,&x2,&y2);
  Serial.print("XXXXX = ");
  Serial.print(x2);
  Serial.print(" |\tYYYYY = ");
  Serial.println(y2);
  delay(1000);
  pinMode(dir_pin1, OUTPUT);
  pinMode(pwm_pin1, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  pinMode(pwm_pin2, OUTPUT);
  pinMode(homeSW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(homeSW), Home, LOW);
  pinMode(stow, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stow), Stow, LOW);

  k = 0;

}

//==============================================================================================>

void loop()
{
  
  //---BNO055-1---
  jointangle1 = radians(yBNO0551());
  delay(10);
  //---BNO055-2---
  angle2 = radians(yBNO0552());
  delay(10);
  jointangle2= jointangle1-angle2;

  getCorrdinates(jointangle1,jointangle2,L1,L2,&x1,&y1,&X2,&Y2);
  Serial.print("currentX= ");
  Serial.print(X2);
  Serial.print(" |currentY= ");
  Serial.print(Y2);
//  Serial.println("");
  delay(10);

  //joy(&x2,&y2);
  x2 = Final_x[k];
  y2 = Final_y[k];

 

  Serial.print(" |X= ");
  Serial.print(x2);
  Serial.print(" |Y= ");
  Serial.print(y2);
  
//  jointangle1 = radians(pitch1);
//  angle2 = radians(pitch2);
//  jointangle2= jointangle1-angle2;

  jointangle2 = acos((x2*x2 + y2*y2 - (L1*L1 + L2*L2)) / (2 * L1 * L2));
  b = atan((L2 * sin(jointangle2)) / (L2 * cos(jointangle2) + L1));
  a = atan(y2 / x2);
  jointangle1 = a + b;

  target1=degrees(jointangle1);
  target2=target1-degrees(jointangle2);

  FK( target1, target2);

  k++;

  Serial.println("K = ");
  Serial.print(k);

  if (k == 101)
  {
    while(1);
  }
}


//==============================================================================================>
//==============================================================================================>



void joy(double* x2, double* y2)
{
  moveX = analogRead(joyLink2);
  moveY = analogRead(joyLink1);
//  moveX = map(moveX+10, 100, (1023-150), 0,1023*2);
//  moveY = map(moveY+20, 100, (1023-150), 0, 1023*2);
  moveX = map(moveX+10, 100, (1023-150), 1, -1);
  moveY = map(moveY+20, 100, (1023-150), -1, 1);

    *x2+=moveX;
    *y2+=moveY;
    
    if(*x2>70)
  {
    *x2=70;
  }
  if(*y2>60)
  {
    *y2=60;
  }
  if(*x2<10)
  {
    *x2=10;
  }
  if(*y2<-50)
  {
    *y2=-50;
  }
}

//==============================================================================================>

void FK(float target1, float target2)
{
  //PID of joint1
  if(loop1)
  {
    //---BNO055-1---
  pitch1 = yBNO0551();     
  delay(10);
  
//     if(target1>=135)
//    {
//      target1=135;
//    }
//    if(target1<=-45)
//    {
//      target1=-45;
//    }
    //PID of joint1 
    PID1(target1,pitch1,&pwm1,&dir1);
    if(pwm1 > 90)
    {
     pwm1 = 90 ;
    }
    if(pwm1 < 5)
    {
     pwm1 = 0;
    }
    digitalWrite(dir_pin1, dir1);
    analogWrite(pwm_pin1, pwm1);
    loop1=false;
  }

  
  else
  {
    //---BNO055-2---
  pitch2 = yBNO0552();
  delay(10);

//  if(target2>=105)
//  {
//    target2=105;
//  }
//  if(target2<=-105)
//  {
//    target2=-105;
//  }
  //PID of joint2  
  PID2(target2,pitch2,&pwm2,&dir2);
  if(pwm2 > 50)
    {
     pwm2 = 50;
    }
  if(pwm2 < 5)
    {
     pwm2 = 0;
    }
    
    digitalWrite(dir_pin2, dir2);
    analogWrite(pwm_pin2, pwm2);
    loop1=true;
  }
 

  Serial.print(" |target1= ");
  Serial.print(target1);

  Serial.print(" |target2= ");
  Serial.println(target2);
  
  SerialDisplay(pitch1,pitch2,dir1,pwm1,dir2,pwm2);
}

//==============================================================================================>

void PID1(int target, int pitch, int* pwm, int* dir)
{
  
//    // DEFINE PID CONSTANTS
//    kp1 = 5.5;
//    ki1 = 0.001;
//    kd1 = 0.0001;
//    
//    error1 = 0;
//    prev_error1 = 0;
//    I1 = 0;
//    D1 = 0;
//    
//    // SAMPLING TIME
//    dt1 = 0.002;

    // DEFINE ERROR
    error1 = target - pitch1; 
    //Serial.println(error);
    // INTEGRAL ERROR
    I1 = I1 + (error1 * dt1);

    // DERIAVTE ERROR
    D1 = (error1 - prev_error1) / dt1;

    // CONTROL SIGNAL -> PWM
    *pwm = abs(kp1*error1 + ki1*I1 + kd1*D1);
    
    //CAP PWM
    
      
     // HOLD PAYLOAD
//     if(abs(error)<3)
//    {
//      *pwm = 0;
//      
//    }
    
    // REVERSE DIRECTION OF MOTOR WHEN CURRENT POSITION > DESIRED POSITION
    if(error1 > 0)
    {
      *dir = LOW;
    }

    if(error1 < 0)
    {
      *dir = HIGH;
    }
    
  
}

//==============================================================================================>


void PID2(int target, int pitch, int* pwm, int* dir)
{
  
//    // DEFINE PID CONSTANTS
//    float kp = 3;
//    float ki = 0.002;
//    float kd = 0.00;
//    
//    float error = 0;
//    float prev_error = 0;
//    float I = 0;
//    float D = 0;
//    
//    // SAMPLING TIME
//    float dt = 0.002;

    // DEFINE ERROR
    error2 = target - pitch2; 
    //Serial.println(error);
    // INTEGRAL ERROR
    I2 = I2 + (error2* dt2);

    // DERIAVTE ERROR
    D2 = (error2 - prev_error2) / dt2;

    // CONTROL SIGNAL -> PWM
    *pwm = abs(kp2*error2 + ki2*I2 + kd2*D2);
//    *pwm = map(pwm2, 5, , 5, 60);
    //CAP PWM
    
      
     // HOLD PAYLOAD
//     if(abs(error)<3)
//    {
//      *pwm = 0;
//      
//    }
    
    // REVERSE DIRECTION OF MOTOR WHEN CURRENT POSITION > DESIRED POSITION
    if(error2 > 0)
    {
      *dir = LOW;
    }

    if(error2 < 0)
    {
      *dir = HIGH;
    }
    
  
}

//==============================================================================================>

void SerialDisplay(int pitch1,int pitch2,int dir1,int pwm1,int dir2,int pwm2)
{
//  Serial.print(" |Pitch1= ");
//  Serial.print(pitch1);
//
//  Serial.print(" |Pitch2= ");
//  Serial.println(pitch2);
  
   
//  Serial.print(" |\tDIR1 = ");
//  Serial.print(dir1);
//    
//  Serial.print(" |\tPWM1 = ");
//  Serial.print(pwm1);
//  
//  Serial.print(" |\tDIR2 = ");
//  Serial.print(dir2);
//    
//  Serial.print(" |\tPWM2 = ");
//  Serial.println(pwm2);
//  //Serial.println("");
}

//==============================================================================================>

void getCorrdinates(float jointangle1, float jointangle2, int L1, int L2, float* x1, float* y1, double* x2, double* y2)
{
  *x1 = L1 * cos(jointangle1);
  *y1 = L1 * sin(jointangle1);
  *x2 = L1 * cos(jointangle1) + L2 * cos(jointangle1 - jointangle2);
  *y2 = L1 * sin(jointangle1) + L2 * sin(jointangle1 - jointangle2);
}

//==============================================================================================>

void setBNO0551()
{
  if (!bno1.begin())
  {    
    Serial.print("Ooops, no BNO055-1 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
    Serial.println("BNO055-1 detected!!");
}

//==============================================================================================>

void setBNO0552()
{
  if (!bno2.begin())
  {    
    Serial.print("Ooops, no BNO055-2 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
    Serial.println("BNO055-2 detected!!");
}

//==============================================================================================>

float yBNO0551() 
{
  sensors_event_t orientationData;
  bno1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return(y);
}

//==============================================================================================>

float yBNO0552() 
{
  sensors_event_t orientationData;
  bno2.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return(y);
}

//==============================================================================================>

void Home() {
  target1 = 135;
  target2 = 45;
}
void Stow() {
  target1 = 135;
  target2 = -45;
}
