#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLE_DELAY  100

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define dir_pin 4 
#define pwm_pin 5

int dir = 0;
int pwm = 0;


// DEFINE TARGET ANGLE  
float target =60 ;

// DEFINE PID CONSTANTS
float kp = 5;
float kd = 0;
float ki = 0;

float error = 0;
float prev_error = 0;
float e_int = 0;
float e_dot = 0;

// SAMPLING TIME
float dt = 0.001;



void setup()
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);

}

void loop()
{

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float pitch = printEvent(&orientationData);

  delay(10);

//    // CONVERT EULER ANGLES TO DEGREES
//    float roll = (myIMU.getRoll()) * 180.0 / PI;   
//    float pitch = (myIMU.getPitch()) * 180.0 / PI; 
//    float yaw = (myIMU.getYaw()) * 180.0 / PI;     


    // DEFINE ERROR
    error = target - pitch; 

    // INTEGRAL ERROR
    e_int = e_int + (error * dt);

    // DERIAVTE ERROR
    e_dot = (error - prev_error) / dt;

    // CONTROL SIGNAL -> PWM
    pwm = abs(kp*error + ki*e_int + kd*e_dot);
    
    // CAP PWM
//    if(pwm > 255)
//    {
//      pwm = 150;
//    }
      
    // HOLD PAYLOAD
//     if(abs(error)<3)
//    {
//      pwm = 0;
//      analogWrite(pwm_pin, abs(pwm));
//    }
    
    // GIVE MOTOR REQUIRED PWM
    analogWrite(pwm_pin, abs(pwm));
    
    // REVERSE DIRECTION OF MOTOR WHEN CURRENT POSITION > DESIRED POSITION
    if(error > 0)
    {
      dir = 1;
      digitalWrite(dir_pin, dir);
    }

    if(error < 0)
    {
      dir = 0;
      digitalWrite(dir_pin, dir);
    }

    Serial.println();
    
    Serial.print("DIRECTION PIN = ");
    Serial.print(digitalRead(dir_pin));

    Serial.println();
    
    Serial.print("PWM = ");
    Serial.print(pwm);
    
    Serial.println();
}


float printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }

  Serial.print(" |\ty= ");
  Serial.print(y);

  return(y);
  
}
