#include <Wire.h>

//---BNO055---
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLE_DELAY  100
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//---MPU6050---
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;



void setup()
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  //---BNO055---
  setBNO055();
  delay(500);

  //---MPU6050---
  setMPU();  
}

void loop() 
{
  //---BNO055---
  float pitch1 = yBNO055();
  
  delay(10);
  
  //---MPU6059---
  float pitch2=yMPU();
 

}


void setMPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
}
void setBNO055()
{
  if (!bno.begin()){    
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
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

  Serial.print(" |\tPitch2= ");
  Serial.println(y);

  return y;
}
float yBNO055() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.y;

  Serial.print("Pitch1= ");
  Serial.print(y);

  return(y);
  
}
