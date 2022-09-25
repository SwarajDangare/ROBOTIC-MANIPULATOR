#include <BNO055.h>
#include <BNO055_support.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLE_DELAY  100

Adafruit_BNO055 L1_IMU = Adafruit_BNO055();


void setup() {
  Serial.begin(115200);
  L1_IMU.begin();
  delay(100);
  L1_IMU.setExtCrystalUse(true);
    

}

void loop() {
//  imu::Vector<3> gyro = L1_IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  Serial.print(gyro.x());
//  Serial.print(",  ");
//  Serial.print(gyro.y());
//  Serial.print(",  ");
//  Serial.println(gyro.z());

  imu::Vector<3> acc = L1_IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(acc.x());
  Serial.print(",  ");
  Serial.print(acc.y());
  Serial.print(",  ");
  Serial.println(acc.z());

  delay(BNO055_SAMPLE_DELAY);

  

}
