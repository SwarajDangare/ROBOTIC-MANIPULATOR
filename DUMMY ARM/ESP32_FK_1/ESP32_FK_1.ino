#include <WiFi.h>

const char *ssid = "heyhi";
const char *password = "allen1234";

WiFiServer wifiServer(27339);
//#include "Arduino.h"
//#include <ESP32AnalogRead.h>
//ESP32AnalogRead adc;
#include <analogWrite.h>
#include <Wire.h>

//---BNO055---
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLE_DELAY  100
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x29);


//---ARDUINO_PIN_CONFIG---
#define dir_pin1 26
#define pwm_pin1 27
#define dir_pin2 32
#define pwm_pin2 33
int dir1 = 0;
int pwm1 = 0;
int dir2 = 0;
int pwm2 = 0;
int angLink1 = 0;
int angLink2 = 0;
char rxData;


//---DEFINE TARGET JOINT ANGLES---
float pitch1 = 0;
float pitch2 = 0;
float jointangle1 = 0;
float jointangle2 = 0;
float angle2 = 0;
volatile float target1 ; //jointangle1;
volatile float target2 ;  //jointangle1 - jointangle2;
float b = 0;
float a = 0;

//---DEFINE PID CONSTANTS---
float kp1 = 5;
float ki1 = 0.002;
float kd1 = 0.002;

float error1 = 0;
float prev_error1 = 0;
float I1 = 0;
float D1 = 0;
float dt1 = 0.0001;

float kp2 = 4;
float ki2 = 0.001;
float kd2 = 0.00005;

float error2 = 0;
float prev_error2 = 0;
float I2 = 0;
float D2 = 0;
float dt2 = 0.0002;



void setup()
{

  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    //Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();

  Serial.println("Orientation Sensor Test");
  Serial.println("");
  delay(1000);



  //---BNO055-1---
  setBNO0551();
  delay(500);

  //---BNO055-2---
  setBNO0552();
  delay(500);

  target1 = yBNO0551();
  target2 = yBNO0552();


  pinMode(dir_pin1, OUTPUT);
  pinMode(pwm_pin1, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  pinMode(pwm_pin2, OUTPUT);
}

void loop()
{
  WiFiClient client = wifiServer.available();
  if (client)
  {
    //    Serial.println("Client Found");
    while (client.connected()){

      Serial.println("outter loop");
      //      Serial.println("Client Connected");
      while (client.available() > 0)
      {
        Serial.println("inner loop");
        //Serial.println("Client Available");
        rxData = client.read();
        Serial.println(rxData);
        client.write(rxData);

        if (rxData == 'w')
        {
          target1 -= 3;
          code();
        }
        else if (rxData == 's')
        {
          target1 += 3;
          code();
        }
        else if (rxData == 'a')
        {
          target2 -= 3;
          code();
        }
        else if (rxData == 'd')
        {
          target2 += 3;
          code();
        }
        code();
      }code();
      }
      code();
//    client.stop();
//    Serial.println("Client disconnected"); 
  }
  code();
}


void code(void){
  //---BNO055-2---
  float pitch2 = yBNO0552();
  delay(10);

  //---BNO055-1---uuuuu
  float pitch1 = yBNO0551();
  delay(10);

  Serial.print(angLink1);
  Serial.print(" ");
  Serial.println(angLink2);
  Serial.println();


  if (target1 >= 135)
  {
    target1 = 135;
  }
  if (target1 <= -45)
  {
    target1 = -45;
  }

  if (target2 >= 105)
  {
    target2 = 105;
  }
  if (target2 <= -105)
  {
    target2 = -105;
  }

  Serial.print("Target1 = ");
  Serial.print(target1);
  Serial.print("\t Target2 = ");
  Serial.println(target2);

  PID(target1, pitch1, &pwm1, &dir1);
  if (pwm1 > 150)
  {
    pwm1 = 150 ;
  }
  PID(target2, pitch2, &pwm2, &dir2);
  if (pwm2 > 125)
  {
    pwm2 = 125;
  }

  if (pwm1 < 40)
  {
    pwm1 = 0;
  }
  if (pwm2 < 25)
  {
    pwm2 = 0;
  }
  digitalWrite(dir_pin1, dir1);
  analogWrite(pwm_pin1, pwm1);
  digitalWrite(dir_pin2, dir2);
  analogWrite(pwm_pin2, pwm2);

  SerialDisplay(pitch1, pitch2, dir1, pwm1, dir2, pwm2);
}


void PID(int target, int pitch, int* pwm, int* dir)
{
  // DEFINE PID CONSTANTS
  float kp = 4;
  float ki = 0.001;
  float kd = 0.00005;

  float error = 0;
  float prev_error = 0;
  float I = 0;
  float D = 0;

  // SAMPLING TIME
  float dt = 0.0002;

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
    *dir = HIGH;
  }

  if (error < 0)
  {
    *dir = LOW;
  }

}

void SerialDisplay(int pitch1, int pitch2, int dir1, int pwm1, int dir2, int pwm2)
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

void setBNO0551()
{
  if (!bno1.begin())
  {
    Serial.print("Ooops, no BNO055-1 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BNO055-1 detected!!");
}

void setBNO0552()
{
  if (!bno2.begin())
  {
    Serial.print("Ooops, no BNO055-2 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BNO055-2 detected!!");
}

float yBNO0551()
{
  sensors_event_t orientationData;
  bno1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return (y);
}
float yBNO0552()
{
  sensors_event_t orientationData;
  bno2.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return (y);
}
